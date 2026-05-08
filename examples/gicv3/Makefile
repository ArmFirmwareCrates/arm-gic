root_dir := $(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))/../../
target := aarch64-unknown-none
mode := release
kernel := $(root_dir)/target/$(target)/$(mode)/gicv3-example
kernel_qemu_bin := $(root_dir)/target/$(target)/$(mode)/gicv3-example.bin

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
	cargo build --bin gicv3-example $(BUILD_ARGS)

$(kernel_qemu_bin): kernel_qemu
	$(objcopy) -O binary $(kernel) $(kernel_qemu_bin)

clean:
	cargo clean --manifest-path Cargo.toml

qemu: $(kernel_qemu_bin)
	qemu-system-aarch64 \
	  $(QEMU_ARGS) \
		-machine virt,gic-version=3 \
		-cpu max \
		-serial mon:stdio \
		-kernel $(kernel_qemu_bin) \
		-vga none \
		-display none \
		-nic none \
		-d int,trace:'arm_timer*',trace:'gic*' -D qemu.log
