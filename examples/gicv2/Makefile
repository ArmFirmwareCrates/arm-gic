target := aarch64-unknown-none
mode := release
kernel := target/$(target)/$(mode)/gicv2
kernel_qemu_bin := target/$(target)/$(mode)/gicv2.bin

sysroot := $(shell rustc --print sysroot)
objdump := $(shell find $(sysroot) -name llvm-objdump) --arch-name=aarch64
objcopy := $(shell find $(sysroot) -name llvm-objcopy)

BUILD_ARGS += --target $(target)
ifeq ($(mode), release)
	BUILD_ARGS += --release
endif

.PHONY: kernel clean qemu run env

env:
	rustup component add llvm-tools-preview rustfmt
	rustup target add $(target)

kernel_qemu:
	cargo build $(BUILD_ARGS)

$(kernel_qemu_bin): kernel_qemu
	$(objcopy) -O binary $(kernel) $(kernel_qemu_bin)

clean:
	cargo clean

qemu: $(kernel_qemu_bin)
	qemu-system-aarch64 \
	  $(QEMU_ARGS) \
		-machine virt \
		-cpu max \
		-serial mon:stdio \
		-kernel $(kernel_qemu_bin) \
		-vga none \
		-display none \
		-nic none \
		-d int,trace:'arm_timer*',trace:'gic*' -D qemu.log
