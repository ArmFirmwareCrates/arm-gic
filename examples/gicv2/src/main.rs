// Copyright 2025 The arm-gic Authors.
// SPDX-License-Identifier: MIT OR Apache-2.0

#![no_std]
#![no_main]

mod exceptions;

use aarch64_rt::{entry, exception_handlers, ExceptionHandlers, RegisterStateRef};
use arm_gic::{
    gicv2::{
        registers::{Gicc, Gicd},
        GicV2,
    },
    irq_enable, wfi, IntId, InterruptGroup,
};
use arm_pl011_uart::Uart;
use core::{
    arch::asm,
    fmt::Write,
    panic::PanicInfo,
    ptr::NonNull,
    sync::atomic::{AtomicU8, Ordering},
};
use log::{LevelFilter, Log, Metadata, Record};
use smccc::{psci::system_off, Hvc};
use spin::{mutex::SpinMutex, Once};

/// Base memory-mapped address of the primary PL011 UART device.
pub const UART_BASE_ADDRESS: *mut u32 = 0x900_0000 as _;

// Base addresses of the GICv2 distributor and redistributor.
const GICD_BASE_ADDRESS: *mut Gicd = 0x800_0000 as _;
const GICC_BASE_ADDRESS: *mut Gicc = 0x801_0000 as _;

// Interrupt ID for physical EL1 timer is 30, and PPI interrupts start from 16:
const TIMER_IRQID: IntId = IntId::ppi(30 - 16);

// Count times the timer has fired.
static COUNTER: AtomicU8 = AtomicU8::new(0);
static GIC: Once<SpinMutex<GicV2>> = Once::new();

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
        let mut gic = GIC.get().unwrap().lock();

        let int_id = gic
            .get_and_acknowledge_interrupt(InterruptGroup::Group0)
            .unwrap();
        gic.end_interrupt(int_id, InterruptGroup::Group0);
        if int_id == TIMER_IRQID {
            let counter = COUNTER.fetch_add(1, Ordering::SeqCst);
            if counter < 4 {
                log::info!("tick");
                // Re-enable
                EL1PhysicalTimer::enable();
            } else {
                // Mask interrupt and disable timer
                EL1PhysicalTimer::disable();
            }
        } else {
            log::error!("unexpected int_id: {int_id:?}");
        }
    }
}

exception_handlers!(Exceptions);
entry!(main);
fn main(_x0: u64, _x1: u64, _x2: u64, _x3: u64) -> ! {
    // Initialise logger uart
    {
        use arm_pl011_uart::{DataBits, LineConfig, Parity, StopBits, UniqueMmioPointer};

        // SAFETY: BASE_ADDRESS is the base of the MMIO region for a UART and is mapped as device
        // memory.
        let uart_pointer =
            unsafe { UniqueMmioPointer::new(NonNull::new(UART_BASE_ADDRESS.cast()).unwrap()) };

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

    // Initialise GIC
    {
        // SAFETY: These are the correct addresses for the QEMU virt machine and are mapped as device
        // memory
        let mut gic = unsafe { GicV2::new(GICD_BASE_ADDRESS, GICC_BASE_ADDRESS) };
        let gic_typer = gic.typer();
        log::info!(
            "GICv2 typer: lockable_spi_count {:?} has_security_extension: {:?} cpu_count: {:?}
        num_irqs: {:?}",
            gic_typer.lockable_spi_count(),
            gic_typer.has_security_extension(),
            gic_typer.cpu_count(),
            gic_typer.num_irqs(),
        );

        gic.setup();
        gic.set_priority_mask(0xff);

        gic.enable_group0(true);
        gic.set_group(TIMER_IRQID, InterruptGroup::Group0);
        gic.enable_interrupt(TIMER_IRQID, true).unwrap();
        gic.set_interrupt_priority(TIMER_IRQID, 0);
        gic.set_trigger(TIMER_IRQID, arm_gic::Trigger::Level);

        GIC.call_once(|| SpinMutex::new(gic));
    }

    // Enable interrupts
    irq_enable();

    EL1PhysicalTimer::enable();

    // Wait for interrupt instead of busy-polling on COUNTER, it's simpler
    for _ in 0..5 {
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
