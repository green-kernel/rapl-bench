use std::{
	ffi,
	fs::File,
	io::{stdout, BufReader, ErrorKind, Read, Seek, Write},
	path::PathBuf,
	sync::{
		atomic::{AtomicBool, Ordering},
		mpsc, Arc,
	},
	thread,
};

use clap::Parser;
use serde::Serialize;

/// Initiates measurements by the rapl_bench kernel module and dumps the measurements to a csv
#[derive(Parser, Debug)]
#[command(version, about)]
struct Args {
	/// The minimum delay in microseconds rapl_bench should wait between measurements
	min_delay: u64,
	/// The maximum delay in microseconds rapl_bench should wait between measurements
	max_delay: u64,
	/// The file to write the csv to. If set to - the csv will be written to stdout
	#[arg(short, long, default_value = "-")]
	output: PathBuf,
}

/// Holds the settings for the kernel module
struct Settings {
	sample_delay_min: u64,
	sample_delay_max: u64,
	enable: bool,
}

impl Settings {
	/// Transforms the settings into a byte string that the kernel module can read
	fn to_bytes(&self) -> Vec<u8> {
		let sample_delay_min: ffi::c_ulong = self
			.sample_delay_min
			.try_into()
			.expect("The specified sample delay min is too high to fit into this platforms ulong");
		let sample_delay_max: ffi::c_ulong = self
			.sample_delay_max
			.try_into()
			.expect("The specified sample delay max is too long to fit into this platforms ulong");
		let enable: u8 = self.enable.into();

		let mut bytes = vec![];
		bytes.append(&mut sample_delay_min.to_ne_bytes().to_vec());
		bytes.append(&mut sample_delay_max.to_ne_bytes().to_vec());
		bytes.push(enable);
		bytes
	}
}

/// A single measurement as reported by the kernel module
#[derive(Serialize)]
struct DataPoint {
	is_overwritten: bool,
	timestamp: u64,
	pkg_energy: u64,
	pp0_energy: u64,
	dram_energy: u64,
	platform_energy: u64,
	unit: u64,
}

impl DataPoint {
	/// Parses a data point from an array of bytes. If the size of the data point struct changes in the kernel module,
	/// the size of the parameter has to be modified accordingly.
	fn from_bytes(bytes: &[u8; 64]) -> Self {
		let (is_overwritten, bytes) = bytes.split_at(1);
		let is_overwritten = match is_overwritten[0] {
				0 => false,
				1 => true,
				_ => unreachable!("The is_overwritten flag is supposed to be a bool and thus should only ever be 0 or 1")
			};
		let (timestamp, bytes) = Self::parse_u64(bytes);
		let (unit, bytes) = Self::parse_u64(bytes);
		let (pkg_energy, bytes) = Self::parse_u64(bytes);
		let (pp0_energy, bytes) = Self::parse_u64(bytes);
		let (dram_energy, bytes) = Self::parse_u64(bytes);
		let (platform_energy, _bytes) = Self::parse_u64(bytes);
		// The remaining bytes are padding
		Self {
			is_overwritten,
			timestamp,
			unit,
			pkg_energy,
			pp0_energy,
			dram_energy,
			platform_energy,
		}
	}

	/// Helper function that takes the first 8 bytes from a byte stream and converts them to a u64 in native byte order
	fn parse_u64(bytes: &[u8]) -> (u64, &[u8]) {
		let (data, bytes) = bytes.split_at(8);
		let data = u64::from_ne_bytes(data.try_into().unwrap());
		(data, bytes)
	}
}

/// Main function for the thread which continously reads measurements form the kernel
fn read_thread_loop(rapl_bench_path: &str, output_path: PathBuf, data_lost: Arc<AtomicBool>) {
	let mut data_channel = BufReader::new(File::open(rapl_bench_path).unwrap());
	let mut buf = [0; 64];

	let inner_writer: Box<dyn Write> = if output_path.as_os_str() == "-" {
		Box::new(stdout())
	} else {
		Box::new(File::create("out.csv").unwrap())
	};
	let mut csv_writer = csv::Writer::from_writer(inner_writer);

	loop {
		// We don't need to wait here as the kernel module will block if no data is available
		let result = data_channel.read_exact(&mut buf);
		if let Err(e) = result {
			if e.kind() == ErrorKind::UnexpectedEof {
				// If we've reached the end of the stream, we need to seek to byte 0 to issue the next read call
				// If no bytes have been read at all since the last seek the measurement has been stopped and we
				// can stop this thread.
				let seek_pos = data_channel.stream_position().unwrap();
				if seek_pos == 0 {
					break;
				} else if seek_pos % u64::try_from(buf.len()).unwrap() != 0 {
					panic!("Incomplete packets have been read");
				} else {
					data_channel.seek(std::io::SeekFrom::Start(0)).unwrap();
					continue;
				}
			}
			panic!("{}", e);
		}
		let data_point = DataPoint::from_bytes(&buf);
		if data_point.is_overwritten {
			data_lost.store(true, Ordering::Relaxed);
		}
		csv_writer.serialize(data_point).unwrap();
	}
}

fn main() {
	let probe_path = "/sys/module/rapl_bench/probe";

	// Read the command line arguments
	let args = Args::parse();

	// When crtl+c is pressed, send a signal through a channel to stop the measurement and write the remaining data
	let (sender, receiver) = mpsc::channel();
	ctrlc::set_handler(move || {
		sender.send(()).unwrap();
	})
	.unwrap();

	// Instruct the kernel module to start measuring
	let mut settings = Settings {
		sample_delay_min: args.min_delay.try_into().unwrap(),
		sample_delay_max: args.max_delay.try_into().unwrap(),
		enable: true,
	};
	let mut control_channel = File::create(probe_path).unwrap();
	control_channel
		.write_all_oneshot(&settings.to_bytes())
		.unwrap();

	// Start the read thread which will coninously read from the kernel module and dump the data into csv
	let data_lost = Arc::new(AtomicBool::new(false));
	let read_thread = {
		let data_lost = data_lost.clone();
		thread::spawn(|| read_thread_loop(probe_path, args.output, data_lost))
	};

	// Wait for Ctrl+C
	receiver.recv().unwrap();

	// Instruct the kernel module to stop measuring
	settings.enable = false;
	control_channel
		.write_all_oneshot(&settings.to_bytes())
		.unwrap();

	// Wait until measuring has concluded and all remaining data has been written
	read_thread.join().unwrap();

	// If any data loss occured because we couldn't keep up with reading it from the kernel module, inform the user.
	if data_lost.load(Ordering::Relaxed) {
		eprintln!("There was some data lost during this measurement");
	}
}

trait WriteAllOneshot {
	/// Issues a single `write` and checks if all data has been written in that `write`. If not, this function will panic.
	fn write_all_oneshot(&mut self, data: &[u8]) -> std::io::Result<()>;
}

impl<T: Write> WriteAllOneshot for T {
	fn write_all_oneshot(&mut self, data: &[u8]) -> std::io::Result<()> {
		let bytes_written = self.write(data)?;
		// If these numbers differs, this indicates that the kernel module and the user space program
		// disagree on how large a single packet is.
		assert_eq!(bytes_written, data.len());
		Ok(())
	}
}
