// Copyright 2025 The arm-gic Authors.
// SPDX-License-Identifier: MIT OR Apache-2.0

//! Exception handlers.

use smccc::{psci::system_off, Hvc};

#[no_mangle]
extern "C" fn sync_exception_current(_elr: u64, _spsr: u64) {
    log::error!("sync_exception_current");
    system_off::<Hvc>().unwrap();
}

#[no_mangle]
extern "C" fn fiq_current(_elr: u64, _spsr: u64) {
    log::error!("fiq_current");
    system_off::<Hvc>().unwrap();
}

#[no_mangle]
extern "C" fn serr_current(_elr: u64, _spsr: u64) {
    log::error!("serr_current");
    system_off::<Hvc>().unwrap();
}

#[no_mangle]
extern "C" fn sync_lower(_elr: u64, _spsr: u64) {
    log::error!("sync_lower");
    system_off::<Hvc>().unwrap();
}

#[no_mangle]
extern "C" fn irq_lower(_elr: u64, _spsr: u64) {
    log::error!("irq_lower");
    system_off::<Hvc>().unwrap();
}

#[no_mangle]
extern "C" fn fiq_lower(_elr: u64, _spsr: u64) {
    log::error!("fiq_lower");
    system_off::<Hvc>().unwrap();
}

#[no_mangle]
extern "C" fn serr_lower(_elr: u64, _spsr: u64) {
    log::error!("serr_lower");
    system_off::<Hvc>().unwrap();
}
