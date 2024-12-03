.PHONY: all check

SOURCE := rapl_bench.c

obj-m += $(patsubst %.c,%.o,$(SOURCE))

KDIR ?= /lib/modules/`uname -r`/build

all: rapl_bench

rapl_bench:
	make -C $(KDIR) M=$(PWD)

clean:
	make -C $(KDIR) M=$(PWD) clean
