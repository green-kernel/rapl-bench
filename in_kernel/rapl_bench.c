// SPDX-License-Identifier: GPL-2.0

#include <asm/cpu_device_id.h>
#include <asm/intel-family.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/types.h>

static struct kobject *sysfs_kobj;
static struct task_struct *probe_thread = NULL;

// This struct stores re register ids for the rapl related MSRs
struct rapl_registers_t {
	uint32_t unit;
	uint32_t pkg_energy_status;
	uint32_t pp0_energy_status;
	uint32_t dram_energy_status;
	uint32_t platform_energy_status;
};

static struct rapl_registers_t registers_intel = {
	.unit = 0x606,
	.pkg_energy_status = 0x611,
	.pp0_energy_status = 0x639,
	.dram_energy_status = 0x619,
	.platform_energy_status = 0x64d,
};

static struct rapl_registers_t registers_amd = {
	.unit = 0xc0010299,
	.pkg_energy_status = 0xc001029b,
	.pp0_energy_status = 0xc001029a,
	.dram_energy_status = 0, // Unsupported or unknown register
	.platform_energy_status = 0, // Unsupported or unknown register
};

// This varialbe will hold the currenly confirgured register ids. This is set up on module init.
static struct rapl_registers_t active_registers;

// This struct will hold the settings that will be used for the current measurement run
// It will be received from user space
struct settings_t {
	unsigned long sample_delay_min;
	unsigned long sample_delay_max;
	uint8_t enable;
} __attribute__((packed));

// This variable will hold the currently active settings. It will be updated whenever a message is received form user space.
struct settings_t settings;

// This struct holds the data for a single measurement
struct data_point_t {
	// This uint8_t holds a boolean value. 0 if everything is going as expected, 1 if the ring buffer was filled up and we had to overwrite data
	uint8_t is_overwritten;
	// The time the measurement was recorded at. The time is in nanoseconds since system boot (without leap seconds)
	uint64_t time_ns;
	// The multiplier for rapl energy measurements. It may be 0 if the current CPU has a fixed multiplier.
	uint64_t unit;
	// The energy reported for the 'pkg' domain. It may be 0 if this domain doesn't exist on the current CPU.
	uint64_t pkg_energy;
	// The energy reported for the 'pp0' domain. It may be 0 if this domain doesn't exist on the current CPU.
	uint64_t pp0_energy;
	// The energy reported for the 'dram' domain. It may be 0 if this domain doesn't exist on the current CPU.
	uint64_t dram_energy;
	// The energy reported for the 'platform' domain. It may be 0 if this domain doesn't exist on the current CPU.
	uint64_t platform_energy;
	// By padding data_point_t to the next power of two (64 bytes) the compiler can emit a shirt instruction
	// instead of an mulitplication instruction. This likely reduces the energy impact of the probing (but reduces
	// the number of data points we can fit into the buffer).
	uint8_t padding[15];
} __attribute__((packed));

// The header for the ring buffer we use to buffer measurements until they are read by user space
struct buffer_header_t {
	size_t read_offset;
	size_t write_offset;
	bool is_empty;
} __attribute__((packed));

// This define determines how many measurements the ring buffer is able to hold.
// TODO Currently this equation gurantees that the entire ring buffer (including header) will fit into a page. This is likely too smal, as I could sometimes observe data loss in initial testing.
#define BUFFER_SIZE ((PAGE_SIZE - sizeof(struct buffer_header_t)) / sizeof(struct data_point_t))

// The ring buffer we use to buffer measurements until they are read by user space
struct buffer_t {
	struct buffer_header_t header;
	struct data_point_t data[BUFFER_SIZE];
};

// To minimize syncrhonization impact on the probe thread, this module uses a two buffer strategy.
// One buffer (the active buffer) is assigned to the probe thread. The probe thread will continously
// write measurements to the active buffer. If that buffer is full, it will start overwriting old data.
// If a read is requested from usersprace, the read thread will swap the active and passive buffers.
// The read thread can then read from the passive (previously active) buffer while the probe thread can
// already start writing to the new buffer.
static struct buffer_t buffer1, buffer2;
static struct buffer_t * active_buffer;

// This lock will be used to synchronize the probe thread and the thread communicating with user space
// TODO Would a raw spin lock be better for our use case?
static DEFINE_SPINLOCK(lock);

// Initialize / Reset a buffer.
static void buffer_init(struct buffer_t *buffer) {
	buffer->header.read_offset = 0;
	buffer->header.write_offset = 0;
	buffer->header.is_empty = true;
}

// This function reads the MSR indicated by register_no and returns it's value
static uint64_t read_msr(uint32_t register_no) {
	// rdmsr emits it's 64-value into two 32 bit registers: edx (high bits) and eax (low bits).
	// The variables are still declared as 64 bit types to signal to the c compiler that there is no need
	// to upcast the variables before combining them into a signle 64 bit type (which would emit an instruction).
	// rdmsr takes care of clearing the upper bits of rdx and rax, so skipping the upcast is safe.
	uint64_t high, low;
	asm(
		"rdmsr" // The rdmsr instruction reads a the msr with the ID indicated by ecx
		: "=d" (high), "=a" (low) // rdmsr outputs the contents of the MSR into edx:eax. This line causes the compiler to write the contents of rdx into 'high' and rax into 'low'
		: "c" (register_no) // Put register_no into ecx so it can be used by rdmsr
	);
	// Unify high and low in to a single uint64_t
	return (high << 32) | low;
}

// Helper function that reads a MSR and stores the result into a target location
// If the given register is not supported (indicated by setting register_no to 0), this function will store 0 instead.
static void store_msr_if_available(uint64_t *storage_location, uint32_t register_no) {
	if (register_no != 0) {
		*storage_location = read_msr(register_no);
	}
	else {
		*storage_location = 0;
	}
}

// This is the main loop for the thread that will repeatedly probe rapl
static int probe_thread_fn(void *data) {
	pr_info("RAPL bench: Starting RAPL measurement");
	// kthread_should_stop will be true once kthread_stop is being called by another thread
	while (!kthread_should_stop()) {
		spin_lock(&lock);

		// Create new entry in the ring buffer (or overwrite exsiting, if full)
		struct data_point_t *current_data_point = &active_buffer->data[active_buffer->header.write_offset];
		if (active_buffer->header.write_offset == active_buffer->header.read_offset && !active_buffer->header.is_empty) {
			current_data_point->is_overwritten = true;
		}
		else {
			current_data_point->is_overwritten = false;
		}
		active_buffer->header.is_empty = false;
		active_buffer->header.write_offset += 1;
		if (active_buffer->header.write_offset == BUFFER_SIZE) {
			active_buffer->header.write_offset = 0;
		}
		if (current_data_point->is_overwritten) {
			active_buffer->header.read_offset = active_buffer->header.write_offset;
		}

		// Measure rapl and write results to the buffer. For probes that are unavailable, 0 is written.
		store_msr_if_available(&current_data_point->unit, active_registers.unit);
		store_msr_if_available(&current_data_point->pkg_energy, active_registers.pkg_energy_status);
		store_msr_if_available(&current_data_point->pp0_energy, active_registers.pp0_energy_status);
		store_msr_if_available(&current_data_point->dram_energy, active_registers.dram_energy_status);
		store_msr_if_available(&current_data_point->platform_energy, active_registers.platform_energy_status);

		// This gives us the time since last boot. Opposed to timestamps this has the advantage of never introducing leap seconds into the measurements
		current_data_point->time_ns = ktime_get_ns();

		spin_unlock(&lock);
		// See https://github.com/torvalds/linux/blob/a351e9b9fc24e982ec2f0e76379a49826036da12/Documentation/timers/timers-howto.txt
		usleep_range(settings.sample_delay_min, settings.sample_delay_max);
	}
	pr_info("RAPL bench: Measurement stopped");
	return 0;
}

// This function is called when user space tries to read from the sysfs probe file
// It will swap the active and inactive buffer to minimize blocking time of the probe thread.
// It will dump all the data points of the acquired ring buffer to user space in binary format
// to maximize bandwidth. sysfs_buf is allocated by the kernel and limited to one page.
static ssize_t show(struct kobject *kobj, struct kobj_attribute *attr, char *sysfs_buf) {
	// Swap the active an passive buffer
	struct buffer_t * buffer = active_buffer;
	spin_lock(&lock);
	if (active_buffer == &buffer1) {
		active_buffer = &buffer2;
	} else {
		active_buffer = &buffer1;
	}
	spin_unlock(&lock);

	if (buffer->header.is_empty) {
		// The buffer can be empty at this point for two reasons:
		// Either probing has stopped due to a request form user space, which means there is no
		// more data to be read and we send an EOF as result...
		if (probe_thread == NULL) {
			return 0; // EOF
		}
		// ... or user space is calling 'read' so quickly that no new data points have been generated
		else {
			// Wait for the probe thread to gather some data and retry
			// 16 and 32 are chosen because:
			//   1. They both are powers of two, allowing the compiler to mulitply by bit shifting
			//   2. The buffer won't fit 64, so 32 is the highest possible power of two
			//   3. Having a large gap between the min and max delay minimizes the likelihood of generating an interrupt
			//   4. Having a large minimum delay reduces the number of system calls (as more data is transmitted per syscall)
			// -> Combining constraints 3 and 4 leads to 16 as a nice middle ground choice for the minimum
			usleep_range(settings.sample_delay_max * 16, settings.sample_delay_max * 32);
			return show(kobj, attr, sysfs_buf);
		}
	}

	ssize_t bytes_written = 0;
	// The ring buffer can be in two states
	if (buffer->header.write_offset > buffer->header.read_offset) {
		// If the data is continuous, we can get away with a single memcpy
		bytes_written = (buffer->header.write_offset - buffer->header.read_offset) * sizeof(*buffer->data);
		memcpy(sysfs_buf, &buffer->data[buffer->header.read_offset], bytes_written);
	}
	else {
		// If we have non continuous data, we need two memcpy.
		// One for the data in the back of the array
		ssize_t bytes_in_back = (BUFFER_SIZE - buffer->header.read_offset) * sizeof(*buffer->data);
		memcpy(sysfs_buf, &buffer->data[buffer->header.read_offset], bytes_in_back);
		// One for the data in the front of the array
		size_t bytes_in_front = buffer->header.write_offset * sizeof(*buffer->data);
		memcpy(sysfs_buf + bytes_in_back, &buffer->data[0], bytes_in_front);
		bytes_written = bytes_in_back + bytes_in_front;
	}
	// This resets the buffer, marking it as empty
	buffer_init(buffer);
	return bytes_written;
}

// Validates a settings_t structure for consistency
static bool validate_settings(struct settings_t *settings) {
	if (settings->enable != 1 && settings->enable != 0) {
		return false;
	}
	if (settings->sample_delay_min > settings->sample_delay_max) {
		return false;
	}
	return true;
}

// This is called when user space writes to the sysfs probe file
// The function will read a single settings_t structure and start/stop the measurement accordingly
static ssize_t store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count) {
	if (count < sizeof(struct settings_t)) {
		return 0;
	}

	struct settings_t *new_settings = (struct settings_t*) buf;
	if (!validate_settings(new_settings)) {
		pr_warn("RAPL bench: Ignoring request containing invalid settings");
		return sizeof(struct settings_t);
	}

	bool must_enable = new_settings->enable == 1 && settings.enable == 0;
	bool must_disable = new_settings->enable == 0 && settings.enable == 1;

	settings = *new_settings;
	if (must_enable) {
		buffer_init(&buffer1);
		buffer_init(&buffer2);
		active_buffer = &buffer1;
		probe_thread = kthread_run(probe_thread_fn, NULL, "rapl_bench_probe");
	} else if (must_disable) {
		kthread_stop(probe_thread);
		probe_thread = NULL;
	}

	return sizeof(struct settings_t);
}

// This strucutre tells the kernel which attributes the sysfs probe file will have.
static struct kobj_attribute bench_attr = {
	.attr = {
		.name = "probe", // The name of the sysfs file
		.mode = 0666 // The permissions of the sysfs file
	},
	.show = show, // This function will be called when user space reads from the sysfs file
	.store = store, // This funtion will be called when user space writes to the sysfs file
};

// By default the kernel module enables all RAPL features
// This function will disable features for processers where we know that they don't support them
static void intel_disable_unspported_features(struct cpuinfo_x86 *cpuinfo) {
	switch (cpuinfo->x86_vfm) {
	case INTEL_SANDYBRIDGE:
	case INTEL_IVYBRIDGE:
		active_registers.unit = 0;
		active_registers.dram_energy_status = 0;
		break;

		case INTEL_SANDYBRIDGE_X:
		case INTEL_IVYBRIDGE_X:
		case INTEL_HASWELL:
		case INTEL_HASWELL_L:
		case INTEL_HASWELL_G:
		case INTEL_BROADWELL:
		case INTEL_BROADWELL_G:
		case INTEL_ATOM_GOLDMONT:
		case INTEL_ATOM_GOLDMONT_PLUS:
		case INTEL_ATOM_GOLDMONT_D:
		case INTEL_SKYLAKE_L:
		case INTEL_SKYLAKE:
		case INTEL_KABYLAKE:
		case INTEL_KABYLAKE_L:
			active_registers.unit = 0;
			break;

		case INTEL_HASWELL_X:
		case INTEL_BROADWELL_X:
		case INTEL_SKYLAKE_X:
		case INTEL_XEON_PHI_KNL:
		case INTEL_XEON_PHI_KNM:
			// Everything is supported on these CPUs
			break;

		default:
			pr_warn("RAPL bench: Unknown CPU model (vendor %i, family %i, model %i). Assuming that all features are supported.", cpuinfo->x86_vendor, cpuinfo->x86, cpuinfo->x86_model);
			break;
	}
}

// This function is called when the kernel module is being loaded
// It will enumerate the features of the installed CPU and create the sysfs files required to control the measurements.
// Module loading will be aborted with the error "Device not found" if the current CPU is not supported by the module.
static int __init rapl_bench_init(void)
{
	// Determine processor capabilities
	// TODO If multiprocessor support is to be built, this line needs to be updated
	struct cpuinfo_x86 *cpuinfo = &cpu_data(smp_processor_id()); // &cpu_data looks odd, but is valid since cpu_data is not a function but a macro
	switch (cpuinfo->x86_vendor) {
		case X86_VENDOR_INTEL:
			pr_info("RAPL bench: Vendor: Intel");
			if (cpuinfo->x86 != 6) {
				pr_err("RAPL bench: Unknown Intel CPU family '%i'", cpuinfo->x86);
				return -ENODEV;
			}
			active_registers = registers_intel;
			intel_disable_unspported_features(cpuinfo);
			break;
		case X86_VENDOR_AMD:
			pr_info("RAPL bench: Vendor: AMD");
			if (cpuinfo->x86 != 23 && cpuinfo->x86 != 25) {
				pr_err("RAPL bench: Unknown AMD CPU family %i", cpuinfo->x86_vendor);
				return -ENODEV;
			}
			active_registers = registers_amd;
			break;
		default:
			pr_err("RAPL bench: Unsupported vendor: %i", cpuinfo->x86_vendor);
			return -ENODEV;
	}

	// Create the file /sys/module/rapl_bench/probe
	int error = sysfs_create_file(&THIS_MODULE->mkobj.kobj, &bench_attr.attr);
	if (error) {
		pr_debug("RAPL bench: Failed to create sysfs file");
		return error;
	}

	pr_info("RAPL bench: Ready");
	return 0;
}

// This function is being called when the kernel module is unloaded.
// It will stop any running measurement and clean up once once the measurement thread has stopped.
static void __exit rapl_bench_exit(void)
{
	if (probe_thread != NULL) {
		kthread_stop(probe_thread);
		probe_thread = NULL;
	}
	kobject_put(sysfs_kobj);
	pr_info("RAPL bench: Unloaded");
}

// These macros tell the linux kernel which functions should be called on module load/unload
module_init(rapl_bench_init);
module_exit(rapl_bench_exit);

// This macro is required to be able to access all functions of the kernel
MODULE_LICENSE("GPL");
