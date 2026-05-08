// Copyright 2026 The arm-gic Authors.
// SPDX-License-Identifier: MIT OR Apache-2.0

fn main() {
    println!("cargo:rustc-link-arg=-Timage.ld");
    println!("cargo:rustc-link-arg=-Texamples/gicv3/qemu.ld");
    println!("cargo:rerun-if-changed=examples/gicv3/qemu.ld");
}
