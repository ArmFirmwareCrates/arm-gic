// Copyright 2026 The arm-gic Authors.
// SPDX-License-Identifier: MIT OR Apache-2.0

#![no_std]
#![no_main]

use aarch64_rt::{entry, exception_handlers, ExceptionHandlers, RegisterStateRef};
use arm_gic::{
    gicv3::{
        registers::{Gicd, GicrSgi},
        GicCpuInterface, GicV3,
    },
    irq_enable, wfi, IntId, InterruptGroup, Trigger, UniqueMmioPointer,
};
use arm_pl011_uart::{DataBits, LineConfig, PL011Registers, Parity, StopBits, Uart};
use core::{
    arch::asm,
    fmt::Write,
    panic::PanicInfo,
    ptr::NonNull,
    sync::atomic::{AtomicU8, Ordering},
};
use flat_device_tree::Fdt;
use log::{LevelFilter, Log, Metadata, Record};
use smccc::{psci::system_off, Hvc};
use spin::mutex::SpinMutex;

/// Base memory-mapped address of the primary PL011 UART device.
pub const UART_BASE_ADDRESS: NonNull<PL011Registers> = NonNull::new(0x900_0000 as *mut _).unwrap();

// Interrupt ID for physical EL1 timer is 30, and PPI interrupts start from 16:
const TIMER_IRQID: IntId = IntId::ppi(30 - 16);

// Count times the timer has fired.
static COUNTER: AtomicU8 = AtomicU8::new(0);

static LOGGER: Logger = Logger {
    uart: SpinMutex::new(None),
};

struct Logger {
    uart: SpinMutex<Option<Uart<'static>>>,
}

impl Log for Logger {
    fn enabled(&self, _metadata: &Metadata) -> bool {
        true
    }

    fn log(&self, record: &Record) {
        writeln!(
            LOGGER.uart.lock().as_mut().unwrap(),
            "[{}] {}",
            record.level(),
            record.args()
        )
        .unwrap();
    }

    fn flush(&self) {}
}

pub struct EL1PhysicalTimer;

impl EL1PhysicalTimer {
    pub fn enable() {
        unsafe {
            // SAFETY: this only accesses timer system registers
            asm!(
                // Set timer in a quarter of a second into the future.
                "mrs {tmp}, CNTFRQ_EL0",
                "lsr {tmp}, {tmp}, 2",
                "msr CNTP_TVAL_EL0, {tmp}",
                // Enable timer
                "mrs {tmp}, CNTP_CTL_EL0",
                "orr {tmp},{tmp}, #0x1",
                "msr CNTP_CTL_EL0, {tmp}",
                tmp = out(reg) _,
                options(nomem, nostack)
            );
        }
    }

    pub fn disable() {
        unsafe {
            // SAFETY: this only accesses timer system registers
            asm!(
                "mrs {tmp}, CNTP_CTL_EL0",
                // Set IMASK bit
                "orr {tmp}, {tmp}, #0x2",
                // Clear ENABLE bit
                "and {tmp}, {tmp}, #0xfffffffffffffffe",
                "msr CNTP_CTL_EL0, {tmp}",
                tmp = out(reg) _,
                options(nomem, nostack)
            );
        }
    }

    pub fn is_enabled() -> bool {
        let ctl: u64;
        unsafe {
            // SAFETY: this only accesses timer system registers
            asm!(
                "mrs {0}, CNTP_CTL_EL0",
                out(reg) ctl,
                options(nomem, nostack)
            );
        }
        ctl & 0x1 > 0
    }
}

struct Exceptions;

impl ExceptionHandlers for Exceptions {
    extern "C" fn irq_current(_: RegisterStateRef) {
        GicCpuInterface::handle_interrupt(InterruptGroup::Group1, |int_id| {
            if int_id == TIMER_IRQID {
                log::info!("tick");
                let counter = COUNTER.fetch_add(1, Ordering::SeqCst);
                if counter < 4 {
                    // Re-enable
                    EL1PhysicalTimer::enable();
                } else {
                    // Mask interrupt and disable timer
                    EL1PhysicalTimer::disable();
                }
            } else {
                log::error!("unexpected int_id: {int_id:?}");
            }
        });
    }
}

exception_handlers!(Exceptions);
entry!(main);
fn main(x0: u64, _x1: u64, _x2: u64, _x3: u64) -> ! {
    // Initialise logger uart
    {
        // SAFETY: BASE_ADDRESS is the base of the MMIO region for a UART and is mapped as device
        // memory.
        let uart_pointer = unsafe { UniqueMmioPointer::new(UART_BASE_ADDRESS) };

        let mut uart = Uart::new(uart_pointer);
        uart.enable(
            LineConfig {
                data_bits: DataBits::Bits8,
                parity: Parity::None,
                stop_bits: StopBits::One,
            },
            115_200,
            50_000_000,
        )
        .unwrap();
        LOGGER.uart.lock().replace(uart);

        log::set_logger(&LOGGER).unwrap();
        log::set_max_level(LevelFilter::Trace);
    }

    // Base addresses of the GICv3 distributor and redistributor.
    let gicd_base_address: *mut Gicd;
    let gicr_base_address: *mut GicrSgi;

    {
        // Extract the addresses from device tree:

        let fdt = unsafe { Fdt::from_ptr(x0 as *const u8) }
            .expect("Could not parse devicetree from x0 register");

        // Locate gicv3 node in devicetree
        // Documentation: https://www.kernel.org/doc/Documentation/devicetree/bindings/interrupt-controller/arm%2Cgic-v3.txt
        let gic_node = fdt
            .find_compatible(&["arm,gic-v3"])
            .expect("Could not find arm,gic-v3 node in device tree");

        let mut regs = gic_node.reg();

        gicd_base_address = regs
            .next()
            .expect("Expected GIC Distributor interface (GICD) register base")
            .starting_address
            .cast_mut()
            .cast();
        gicr_base_address = regs
            .next()
            .expect("Expected GIC Redistributors (GICR) register base")
            .starting_address
            .cast_mut()
            .cast();
    }

    let mut gic;

    // Initialise GIC
    {
        // SAFETY: These are the correct addresses for the QEMU virt machine as given by the passed
        // device tree and are mapped as device memory
        let gicd = unsafe { UniqueMmioPointer::new(NonNull::new(gicd_base_address).unwrap()) };
        let gicr = NonNull::new(gicr_base_address).unwrap();

        // Initialise the GIC.

        // SAFETY: QEMU virt machine bootloader passes DTB pointer via x0 register.
        gic = unsafe { GicV3::new(gicd, gicr, 1, false) };
        gic.setup(0);

        GicCpuInterface::set_priority_mask(0xff);

        gic.set_interrupt_priority(TIMER_IRQID, Some(0), 0x80)
            .unwrap();
        gic.enable_interrupt(TIMER_IRQID, Some(0), true).unwrap();
        gic.set_trigger(TIMER_IRQID, Some(0), Trigger::Level)
            .unwrap();
    }

    // Enable interrupts
    irq_enable();

    EL1PhysicalTimer::enable();

    // Wait for interrupt instead of busy-polling on COUNTER, it's simpler

    while COUNTER.load(Ordering::SeqCst) < 5 {
        wfi();
    }

    let counter = COUNTER.load(Ordering::SeqCst);
    assert_eq!(counter, 5);
    assert!(!EL1PhysicalTimer::is_enabled());

    log::info!("Shutting down.");
    system_off::<Hvc>().unwrap();
    loop {}
}

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    log::error!("{}", info);
    system_off::<Hvc>().unwrap();
    loop {}
}
