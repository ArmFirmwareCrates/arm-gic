// Copyright The arm-gic Authors.
// SPDX-License-Identifier: MIT OR Apache-2.0

use crate::{
    IntId, Trigger,
    gicv3::{
        GicError, Group, HIGHEST_NS_PRIORITY, SecureIntGroup, clear_bit, register_count,
        registers::{GicrCtlr, GicrIidr, GicrPwrr, GicrSgi, GicrTyper, Sgi, Waker},
        set_bit, set_regs,
    },
};
use core::{hint::spin_loop, marker::PhantomData, ops::Range, ptr::NonNull, stringify};
use safe_mmio::{UniqueMmioPointer, field, field_shared};
use zerocopy::{transmute_mut, transmute_ref};

/// Reads a GICR register and stores it in a context structure.
macro_rules! save_reg {
    ($context:ident, $regs:expr, $reg:ident) => {
        $context.$reg = field_shared!($regs, $reg).read();
    };
    ($context:ident, $regs:ident, $reg:ident, $index:ident) => {
        $context.$reg[$index] = field_shared!($regs, $reg).get($index).unwrap().read();
    };
}

/// Restores a GICR register value from a context structure.
macro_rules! restore_reg {
    ($context:ident, $regs:expr, $reg:ident) => {
        field!($regs, $reg).write($context.$reg);
    };
    ($context:ident, $regs:ident, $reg:ident, $index:ident) => {
        field!($regs, $reg)
            .get($index)
            .unwrap()
            .write($context.$reg[$index]);
    };
}

/// Reads the SGI/(E)PPI registers and stores them in a context structure.
///
/// The macro iterates over a range of `$regs.$reg` and saves each register into `$context`. The
/// range is determined based on `$start_offset`, `$int_count`, `$bits_per_int` and the type of the
/// registers.
macro_rules! save_regs {
    ($context:expr, $regs:expr, $reg:ident, $int_count:expr, $bits_per_int:expr) => {
        let reg_count = register_count($int_count, $bits_per_int, &$context[0]);
        for i in 0..reg_count {
            $context[i] = field_shared!($regs, $reg).get(i).unwrap().read();
        }
    };
}

/// Restores the SGI/(E)PPI register values from a context structure.
///
/// The macro iterates over a range of `$regs.$reg` and restores each register from `$context`. The
/// range is determined based on `$start_offset`, `$int_count`, `$bits_per_int` and the type of the
/// registers.
macro_rules! restore_regs {
    ($context:expr, $regs:expr, $reg:ident, $int_count:expr, $bits_per_int:expr) => {
        let reg_count = register_count($int_count, $bits_per_int, &$context[0]);
        for i in 0..reg_count {
            field!($regs, $reg).get(i).unwrap().write($context[i]);
        }
    };
}

/// Iterator over the redistributor register blocks.
pub struct GicRedistributorIterator<'a> {
    pointer: Option<NonNull<GicrSgi>>,
    gic_v4: bool,
    phantom: PhantomData<&'a GicrSgi>,
}

impl GicRedistributorIterator<'_> {
    /// Creates a new iterator instance.
    ///
    /// # Safety
    ///
    /// The caller must ensure that `base` points to a continiously mapped GIC redistributor memory
    /// area that spans until the last redistributor block (N) where GICR_TYPER.Last is set and
    /// there must be no other references to this area.
    pub unsafe fn new(base: NonNull<GicrSgi>, gic_v4: bool) -> Self {
        Self {
            pointer: Some(base),
            gic_v4,
            phantom: PhantomData,
        }
    }
}

impl<'a> Iterator for GicRedistributorIterator<'a> {
    type Item = GicRedistributor<'a>;

    fn next(&mut self) -> Option<Self::Item> {
        let pointer = self.pointer?;

        // Safety: GicRedistributorIterator::new promises that base points to a valid GIC
        // redistributor block and pointer stops at the last redistributor.
        let redist = unsafe { UniqueMmioPointer::new(pointer) };

        let gicr = field_shared!(redist, gicr);
        let typer = field_shared!(gicr, typer).read();

        self.pointer = if !typer.last_redistributor() {
            // Step to next redistributor block
            let redistributor_size = if self.gic_v4 && typer.virtual_lpis_supported() {
                // In this case GicV4 adds 2 frames:
                // vlpi: 64KiB
                // reserved: 64KiB
                2
            } else {
                1
            };

            // Safety: GicRedistributorIterator::new promises that base points to a valid GIC
            // redistributor block and GICR_TYPER.Last was not set for this redistributor. It is
            // safe to step the pointer to the next redistributor block.
            unsafe { Some(pointer.add(redistributor_size)) }
        } else {
            // Clear pointer at the last redistributor block.
            None
        };

        Some(GicRedistributor::new(redist))
    }
}

/// Defines functions for accessing different parts of the `iregs` member.
///
/// Consumes an array of `(register name/immutable accessor name, mutable accessor name, type, bits
/// per interrupt)` tuples and generate a pair of immutable and mutable accessor functions.
macro_rules! define_context_registers {
    // Public variant.
    ($( ( $reg:ident, $reg_mut:ident, $type:ty, $bits:expr ) ),* $(,,)?) => {
        define_context_registers!(@step 0 ; $( ( $reg, $reg_mut, $type, $bits ) ),* );
    };

    // The stop condition output the sum of bits per interrupt constant.
    ( @step $n:expr ; ) => {
        const BITS_PER_INTERRUPT: usize = $n;
    };

    // Recursive step: generate a pair of functions, then munch the rest.
    ( @step $n:expr ;
      ( $reg:ident, $reg_mut:ident, $type:ty, $bits:expr ) $(, $rest:tt)*
    ) => {
        #[doc = "Autogenerated function to access "]
        #[doc = stringify!($reg)]
        #[doc = " registers."]
        pub fn $reg(&self) -> &[$type] {
            transmute_ref!(&self.iregs[Self::index($n..($n + $bits))])
        }

        #[doc = "Autogenerated function to mutable access "]
        #[doc = stringify!($reg)]
        #[doc = " registers."]
        pub fn $reg_mut(&mut self) -> &mut [$type] {
            transmute_mut!(&mut self.iregs[Self::index($n..($n + $bits))])
        }

        define_context_registers!(@step $n + $bits ; $( $rest ),*);
    };
}

/// Context of the GIC redistributor.
///
/// It contains a set of registers that has to be saved/restored on redistributor power off/on.
/// Use `GicRedistributorContext::ireg_count` to determine the const parameter based on the PPI
/// count.
///
/// ```
/// use arm_gic::gicv3::GicRedistributorContext;
///
/// let context = GicRedistributorContext::<{ GicRedistributorContext::ireg_count(96) }>::default();
/// ```
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct GicRedistributorContext<const IREG_COUNT: usize> {
    propbaser: u64,
    pendbaser: u64,
    ctlr: GicrCtlr,
    nsacr: u32,
    iregs: [u32; IREG_COUNT],
}

impl GicRedistributorContext<0> {
    /// Calculates `IREG_COUNT` const parameter of `GicRedistributorContext` based on the PPI count.
    pub const fn ireg_count(ppi_count: usize) -> usize {
        assert!(
            ppi_count.is_multiple_of(32),
            "ppi_count must be multiple of 32"
        );
        (ppi_count * GicRedistributorContext::<0>::BITS_PER_INTERRUPT).div_ceil(32)
    }
}

impl<const IREG_COUNT: usize> GicRedistributorContext<IREG_COUNT> {
    const PPI_COUNT: usize = (IREG_COUNT * 32).div_ceil(Self::BITS_PER_INTERRUPT);

    /// Creates a new empty instance of the redistributor context.
    pub const fn new() -> Self {
        Self {
            propbaser: 0,
            pendbaser: 0,
            ctlr: GicrCtlr::empty(),
            nsacr: 0,
            iregs: [0; IREG_COUNT],
        }
    }

    define_context_registers![
        (igroupr, igroupr_mut, u32, Sgi::IGROUPR_BITS),
        (isenabler, isenabler_mut, u32, Sgi::ISENABLER_BITS),
        (ispendr, ispendr_mut, u32, Sgi::ISPENDR_BITS),
        (isactiver, isactiver_mut, u32, Sgi::ISACTIVER_BITS),
        (igrpmodr, igrpmodr_mut, u32, Sgi::IGRPMODR_BITS),
        (icfgr, icfgr_mut, u32, Sgi::ICFGR_BITS),
        (ipriorityr, ipriorityr_mut, u8, Sgi::IPRIORITY_BITS)
    ];

    const fn index(bits: Range<usize>) -> Range<usize> {
        const {
            assert!(
                IREG_COUNT.is_multiple_of(Self::BITS_PER_INTERRUPT),
                "IREG_COUNT must be multiple of BITS_PER_INTERRUPT"
            )
        };
        (bits.start * Self::PPI_COUNT / 32)..(bits.end * Self::PPI_COUNT / 32)
    }
}

impl<const IREG_COUNT: usize> Default for GicRedistributorContext<IREG_COUNT> {
    fn default() -> Self {
        Self::new()
    }
}

/// Driver for a single GIC redistributor instance.
pub struct GicRedistributor<'a> {
    regs: UniqueMmioPointer<'a, GicrSgi>,
}

impl<'a> GicRedistributor<'a> {
    /// Creates a new driver instance for the GIC redistributor with the given registers.
    pub fn new(regs: UniqueMmioPointer<'a, GicrSgi>) -> Self {
        Self { regs }
    }

    /// Configures the redistributor to its default settings. All interrupts controlled by the
    /// redistributor are disabled, set to Group 1 non-secure, and level triggered.
    pub fn configure_default_settings(&mut self) {
        let ppi_count = self.ppi_count();

        // Disable all SGIs (imp. def.)/(E)PPIs before configuring them. This is a more scalable
        // approach as it avoids clearing the enable bits in the GICD_CTLR.
        let mut sgi = field!(self.regs, sgi);
        set_regs(
            field!(sgi, icenabler),
            0,
            ppi_count,
            Sgi::ICENABLER_BITS,
            0xffff_ffffu32,
        );

        self.wait_for_pending_write();

        // Treat all SGIs/(E)PPIs as G1NS by default
        let mut sgi = field!(self.regs, sgi);
        set_regs(
            field!(sgi, igroupr),
            0,
            ppi_count,
            Sgi::IGROUPR_BITS,
            0xffff_ffffu32,
        );

        // Setup the default (E)PPI/SGI priorities
        set_regs(
            field!(sgi, ipriorityr),
            0,
            ppi_count,
            Sgi::IPRIORITY_BITS,
            HIGHEST_NS_PRIORITY,
        );

        // Configure all (E)PPIs as level triggered by default
        set_regs(
            field!(sgi, icfgr),
            IntId::PPI_START as usize,
            ppi_count - IntId::PPI_START as usize,
            Sgi::ICFGR_BITS,
            0x0000_0000u32,
        );
    }

    /// Sets the interrupt priority of the given interrupt ID. This function will panic if invoked
    /// with a non-private IntId.
    pub fn set_interrupt_priority(&mut self, intid: IntId, priority: u8) -> Result<(), GicError> {
        let index = Self::private_index(intid)?;
        let mut sgi = field!(self.regs, sgi);
        field!(sgi, ipriorityr).get(index).unwrap().write(priority);
        Ok(())
    }

    /// Configures the trigger type for the interrupt with the given ID.
    ///
    /// This function will panic if invoked with a non-private IntId.
    pub fn set_trigger(&mut self, intid: IntId, trigger: Trigger) -> Result<(), GicError> {
        let index = Self::private_index(intid)?;

        const INT_PER_REGS: usize = 32 / Sgi::ICFGR_BITS;
        let reg_index = index / INT_PER_REGS;
        let bit = 1 << (((index % INT_PER_REGS) * Sgi::ICFGR_BITS) + 1);

        let mut sgi = field!(self.regs, sgi);
        let mut icfgr = field!(sgi, icfgr);
        let mut register = icfgr.get(reg_index).unwrap();
        let v = register.read();
        register.write(match trigger {
            Trigger::Edge => v | bit,
            Trigger::Level => v & !bit,
        });

        Ok(())
    }

    /// Assigns the interrupt with id `intid` to interrupt group `group`.
    ///
    /// This function will panic if invoked with a non-private IntId.
    pub fn set_group(&mut self, intid: IntId, group: Group) -> Result<(), GicError> {
        let index = Self::private_index(intid)?;

        let mut sgi = field!(self.regs, sgi);
        if let Group::Secure(sg) = group {
            clear_bit(field!(sgi, igroupr).into(), index);
            let igrpmodr = field!(sgi, igrpmodr).into();
            match sg {
                SecureIntGroup::Group1S => set_bit(igrpmodr, index),
                SecureIntGroup::Group0 => clear_bit(igrpmodr, index),
            }
        } else {
            set_bit(field!(sgi, igroupr).into(), index);
            clear_bit(field!(sgi, igrpmodr).into(), index);
        }

        Ok(())
    }

    /// Enables or disables the interrupt with the given ID.
    ///
    /// This function will panic if invoked with a non-private IntId.
    pub fn enable_interrupt(&mut self, intid: IntId, enable: bool) -> Result<(), GicError> {
        let index = Self::private_index(intid)?;
        let mut sgi = field!(self.regs, sgi);

        if enable {
            set_bit(field!(sgi, isenabler).into(), index);
        } else {
            set_bit(field!(sgi, icenabler).into(), index);
        }

        Ok(())
    }

    /// Enables or disables all PPI/SGI interrupts on the redistributor.
    pub fn enable_all_interrupts(&mut self, enable: bool) {
        let ppi_count = self.ppi_count();

        let mut sgi = field!(self.regs, sgi);
        let mut regs = if enable {
            field!(sgi, isenabler)
        } else {
            field!(sgi, icenabler)
        };

        assert_eq!(Sgi::ISENABLER_BITS, Sgi::ICENABLER_BITS);
        let bits = 0xffff_ffff;
        for i in 0..register_count(ppi_count, Sgi::ISENABLER_BITS, &bits) {
            regs.get(i).unwrap().write(bits);
        }
    }

    /// Restores the given GIC Redistributor register context.
    ///
    /// This disables LPI and per-cpu interrupts before it starts to restore the Redistributor. This
    /// function must be invoked after Distributor restore but prior to CPU interface enable. The
    /// pending and active interrupts are restored after the interrupts are fully configured and
    /// enabled.
    pub fn restore<const N: usize>(
        &mut self,
        context: &GicRedistributorContext<N>,
    ) -> Result<(), GicError> {
        let ppi_count = Self::min_count(self.ppi_count(), GicRedistributorContext::<N>::PPI_COUNT)?;

        // Call the post-restore hook that implements the IMP DEF sequence that may be required on
        // some GIC implementations. As this may need to access the Redistributor registers, we pass
        // it proc_num.
        // TODO: handle GIC500/600 gicv3_distif_post_restore(proc_num);

        // Disable all SGIs (imp. def.)/(E)PPIs before configuring them. This is a more scalable
        // approach as it avoids clearing the enable bits in the GICD_CTLR.
        let mut sgi = field!(self.regs, sgi);
        set_regs(
            field!(sgi, icenabler),
            0,
            ppi_count,
            Sgi::ISENABLER_BITS,
            0xffff_ffffu32,
        );

        self.wait_for_pending_write();

        // Disable the LPIs to avoid unpredictable behavior when writing to GICR_PROPBASER and
        // GICR_PENDBASER.
        let mut gicr = field!(self.regs, gicr);
        field!(gicr, ctlr).write(context.ctlr - GicrCtlr::EnableLPIs);

        self.wait_for_pending_write();

        // Restore GICR registers
        let mut gicr = field!(self.regs, gicr);
        restore_reg!(context, gicr, propbaser);
        restore_reg!(context, gicr, pendbaser);

        // Restore SGI registers
        let mut sgi = field!(self.regs, sgi);

        restore_regs!(
            context.igroupr(),
            sgi,
            igroupr,
            ppi_count,
            Sgi::IGROUPR_BITS
        );
        restore_regs!(
            context.igrpmodr(),
            sgi,
            igrpmodr,
            ppi_count,
            Sgi::IGRPMODR_BITS
        );
        restore_regs!(
            context.ipriorityr(),
            sgi,
            ipriorityr,
            ppi_count,
            Sgi::IPRIORITY_BITS
        );
        restore_regs!(context.icfgr(), sgi, icfgr, ppi_count, Sgi::ICFGR_BITS);
        restore_reg!(context, sgi, nsacr);

        // Restore after group and priorities are set.
        restore_regs!(
            context.ispendr(),
            sgi,
            ispendr,
            ppi_count,
            Sgi::ISPENDR_BITS
        );
        restore_regs!(
            context.isactiver(),
            sgi,
            isactiver,
            ppi_count,
            Sgi::ISACTIVER_BITS
        );

        // Wait for all writes to the Distributor to complete before enabling the SGI and (E)PPIs.
        self.wait_for_upstream_pending_write();

        let mut sgi = field!(self.regs, sgi);
        restore_regs!(
            context.isenabler(),
            sgi,
            isenabler,
            ppi_count,
            Sgi::ISENABLER_BITS
        );

        // Restore GICR_CTLR.Enable_LPIs bit and wait for pending writes in case the first write to
        // GICR_CTLR was still in flight (this write only restores GICR_CTLR.Enable_LPIs and no
        // waiting is required for this bit.
        let mut gicr = field!(self.regs, gicr);
        restore_reg!(context, gicr, ctlr);
        self.wait_for_pending_write();

        Ok(())
    }

    /// Saves the current states of the GIC redistributor instance into a context structure.
    pub fn save<const N: usize>(
        &self,
        context: &mut GicRedistributorContext<N>,
    ) -> Result<(), GicError> {
        // Wait for any write to GICR_CTLR to complete before trying to save any state.
        self.wait_for_pending_write();

        // Save GICR registers
        let gicr = field_shared!(self.regs, gicr);

        save_reg!(context, gicr, propbaser);
        save_reg!(context, gicr, pendbaser);
        save_reg!(context, gicr, ctlr);

        // Save SGI registers
        let ppi_count = Self::min_count(self.ppi_count(), GicRedistributorContext::<N>::PPI_COUNT)?;
        let sgi = field_shared!(self.regs, sgi);

        save_reg!(context, sgi, nsacr);
        save_regs!(
            context.igroupr_mut(),
            sgi,
            igroupr,
            ppi_count,
            Sgi::IGROUPR_BITS
        );
        save_regs!(
            context.isenabler_mut(),
            sgi,
            isenabler,
            ppi_count,
            Sgi::ISENABLER_BITS
        );
        save_regs!(
            context.ispendr_mut(),
            sgi,
            ispendr,
            ppi_count,
            Sgi::ISPENDR_BITS
        );
        save_regs!(
            context.isactiver_mut(),
            sgi,
            isactiver,
            ppi_count,
            Sgi::ISACTIVER_BITS
        );
        save_regs!(
            context.igrpmodr_mut(),
            sgi,
            igrpmodr,
            ppi_count,
            Sgi::IGRPMODR_BITS
        );
        save_regs!(context.icfgr_mut(), sgi, icfgr, ppi_count, Sgi::ICFGR_BITS);
        save_regs!(
            context.ipriorityr_mut(),
            sgi,
            ipriorityr,
            ppi_count,
            Sgi::IPRIORITY_BITS
        );

        // Call the pre-save hook that implements the IMP DEF sequence that may be required on some
        // GIC implementations..
        // TODO: handle GIC500/600 gicv3_distif_pre_save(proc_num);

        Ok(())
    }

    /// Powers on GIC-600 or GIC-700 redistributor (if detected).
    pub fn power_on(&mut self) {
        if self.is_gic600_700() {
            self.gic600_700_power_on();
        }
    }

    /// Powers off GIC-600 or GIC-700 redistributor (if detected).
    pub fn power_off(&mut self) {
        if self.is_gic600_700() {
            self.gic600_700_power_off();
        }
    }

    fn wait_until_group_not_in_transit(&self) {
        let gicr = field_shared!(self.regs, gicr);
        let pwrr = field_shared!(gicr, pwrr).read();

        // Check group not transitioning
        while pwrr.contains(GicrPwrr::RedistributorGroupPowerDown)
            != pwrr.contains(GicrPwrr::RedistributorGroupPoweredOff)
        {
            spin_loop();
        }
    }

    /// Checks whether the GIC model is GIC600 or GIC700.
    fn is_gic600_700(&self) -> bool {
        let gicr = field_shared!(self.regs, gicr);
        let iidr = field_shared!(gicr, iidr).read();

        iidr.model_id() == GicrIidr::MODEL_ID_ARM_GIC_600
            || iidr.model_id() == GicrIidr::MODEL_ID_ARM_GIC_600AE
            || iidr.model_id() == GicrIidr::MODEL_ID_ARM_GIC_700
    }

    /// Performs the GIC600/700 specific redistributor power on sequence.
    fn gic600_700_power_on(&mut self) {
        loop {
            // Wait until group not transitioning.
            self.wait_until_group_not_in_transit();

            let mut gicr = field!(self.regs, gicr);
            let mut pwrr = field!(gicr, pwrr);

            // Power on the redistributor.
            pwrr.write(GicrPwrr::empty());

            // Wait until the power on state is reflected.
            // If RDPD == 0 then powered on.
            if !pwrr.read().contains(GicrPwrr::RedistributorPowerDown) {
                break;
            }
        }
    }

    /// Performs the GIC600/700 specific redistributor power off sequence.
    fn gic600_700_power_off(&mut self) {
        // Wait until group not transitioning.
        self.wait_until_group_not_in_transit();

        let mut gicr = field!(self.regs, gicr);
        let mut pwrr = field!(gicr, pwrr);

        // Power off the redistributor.
        pwrr.write(GicrPwrr::RedistributorPowerDown);

        // If this is the last man, turning this redistributor frame off will
        // result in the group itself being powered off and RDGPD = 1.
        // In that case, wait as long as it's in transition, or has aborted
        // the transition altogether for any reason.
        if pwrr.read().contains(GicrPwrr::RedistributorGroupPowerDown) {
            self.wait_until_group_not_in_transit();
        }
    }

    /// Informs the GIC redistributor that the core has awakened.
    ///
    /// Blocks until `GICR_WAKER.ChildrenAsleep` is cleared.
    pub fn mark_core_awake(&mut self) -> Result<(), GicError> {
        let mut gicr = field!(self.regs, gicr);
        let mut waker = field!(gicr, waker);
        let mut gicr_waker = waker.read();

        // The WAKER_PS_BIT should be changed to 0 only when WAKER_CA_BIT is 1.
        if !gicr_waker.contains(Waker::CHILDREN_ASLEEP) {
            return Err(GicError::AlreadyAwake);
        }

        // Mark the connected core as awake.
        gicr_waker -= Waker::PROCESSOR_SLEEP;
        waker.write(gicr_waker);

        // Wait till the WAKER_CA_BIT changes to 0.
        while waker.read().contains(Waker::CHILDREN_ASLEEP) {
            spin_loop();
        }

        Ok(())
    }

    /// Informs the GIC redistributor that the core is asleep.
    ///
    /// Blocks until `GICR_WAKER.ChildrenAsleep` is set.
    pub fn mark_core_asleep(&mut self) -> Result<(), GicError> {
        let mut gicr = field!(self.regs, gicr);
        let mut waker = field!(gicr, waker);
        let mut gicr_waker = waker.read();

        // The WAKER_PS_BIT should be changed to 1 only when WAKER_CA_BIT is 0.
        if gicr_waker.contains(Waker::CHILDREN_ASLEEP) {
            return Err(GicError::AlreadyAsleep);
        }

        // Mark the connected core as asleep.
        gicr_waker |= Waker::PROCESSOR_SLEEP;
        waker.write(gicr_waker);

        // Wait till the WAKER_CA_BIT changes to 1.
        while !waker.read().contains(Waker::CHILDREN_ASLEEP) {
            spin_loop();
        }

        Ok(())
    }

    /// Waits until a register write for the current Security state is in progress.
    pub fn wait_for_pending_write(&self) {
        self.wait_ctlr(GicrCtlr::RWP);
    }

    /// Waits for all upstream writes to be communicated to the Distributor.
    pub fn wait_for_upstream_pending_write(&self) {
        self.wait_ctlr(GicrCtlr::UWP);
    }

    /// Reads redistributor type register, that provides information about the configuration of this
    /// redistributor.
    pub fn typer(&self) -> GicrTyper {
        let gicr = field_shared!(self.regs, gicr);
        field_shared!(gicr, typer).read()
    }

    /// Waits until the given flag is set in GICR_CTRL.
    fn wait_ctlr(&self, flag: GicrCtlr) {
        let gicr = field_shared!(self.regs, gicr);
        let ctlr = field_shared!(gicr, ctlr);
        while ctlr.read().contains(flag) {
            spin_loop();
        }
    }

    /// Returns the PPI count based on GICR_TYPER.PPI_NUM.
    fn ppi_count(&self) -> usize {
        let gicr = field_shared!(self.regs, gicr);
        let typer = field_shared!(gicr, typer).read();
        let ppi_num = typer.max_eppi_count();

        ppi_num as usize + 32
    }

    /// Returns the index of a private interrupt ID or an error.
    fn private_index(intid: IntId) -> Result<usize, GicError> {
        match intid.private_index() {
            Some(index) => Ok(index),
            None => Err(GicError::InvalidGicrIntid(intid)),
        }
    }

    /// Returns `min(implemented, stored)` or returns an error if `implemented > stored`.
    fn min_count(implemented: usize, stored: usize) -> Result<usize, GicError> {
        if implemented <= stored {
            Ok(implemented)
        } else {
            Err(GicError::InvalidContextSize(implemented, stored))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use zerocopy::{FromBytes, Immutable, IntoBytes, KnownLayout, transmute_mut};

    #[derive(Clone, Eq, FromBytes, Immutable, IntoBytes, KnownLayout, PartialEq)]
    #[repr(C, align(8))]
    struct FakeRegisters {
        regs: [u32; Self::REG_COUNT],
    }

    impl FakeRegisters {
        // Two 64k blocks as an u32 array
        const REG_COUNT: usize = 2 * 64 * 1024 / 4;

        pub fn new() -> Self {
            Self {
                regs: [0u32; Self::REG_COUNT],
            }
        }
    }

    struct FakeRedistributor {
        regs: FakeRegisters,
    }

    impl FakeRedistributor {
        pub fn new() -> Self {
            Self {
                regs: FakeRegisters::new(),
            }
        }

        pub fn clear(&mut self) {
            self.regs.regs.fill(0);
        }

        pub fn regs_write(&mut self, offset: usize, value: u32) {
            self.regs.regs[offset / 4] = value;
        }

        pub fn reg_read(&self, offset: usize) -> u32 {
            self.regs.regs[offset / 4]
        }

        fn get(&mut self) -> UniqueMmioPointer<'_, GicrSgi> {
            UniqueMmioPointer::from(transmute_mut!(&mut self.regs))
        }

        pub fn redistributor_for_test(&mut self) -> GicRedistributor<'_> {
            GicRedistributor::new(self.get())
        }
    }

    #[derive(Debug)]
    struct TestCase<T> {
        intid: IntId,
        offset: usize,
        expected_value: u32,
        param: T,
    }

    impl<T> TestCase<T> {
        pub const fn new(intid: IntId, offset: usize, expected_value: u32, param: T) -> Self {
            Self {
                intid,
                offset,
                expected_value,
                param,
            }
        }
    }

    #[test]
    fn configure_default_settings() {
        let mut regs = FakeRedistributor::new();

        // GICR_TYPER.PPInum = 1
        regs.regs_write(0x0_008, 1 << 27);

        let mut redistributor = regs.redistributor_for_test();
        redistributor.configure_default_settings();

        // IGROUPR
        assert_eq!(0xffff_ffff, regs.reg_read(0x1_0080));
        assert_eq!(0xffff_ffff, regs.reg_read(0x1_0084));
        assert_eq!(0x0000_0000, regs.reg_read(0x1_0088));

        // ICENABLER
        assert_eq!(0xffff_ffff, regs.reg_read(0x1_0180));
        assert_eq!(0xffff_ffff, regs.reg_read(0x1_0184));
        assert_eq!(0x0000_0000, regs.reg_read(0x1_0188));

        // IPRIORITYR
        assert_eq!(0x8080_8080, regs.reg_read(0x1_0400));
        assert_eq!(0x8080_8080, regs.reg_read(0x1_0404));
        assert_eq!(0x8080_8080, regs.reg_read(0x1_0408));
        assert_eq!(0x8080_8080, regs.reg_read(0x1_040c));
        assert_eq!(0x8080_8080, regs.reg_read(0x1_0410));
        assert_eq!(0x8080_8080, regs.reg_read(0x1_0414));
        assert_eq!(0x8080_8080, regs.reg_read(0x1_0418));
        assert_eq!(0x8080_8080, regs.reg_read(0x1_041c));
        assert_eq!(0x8080_8080, regs.reg_read(0x1_0420));
        assert_eq!(0x8080_8080, regs.reg_read(0x1_0424));
        assert_eq!(0x8080_8080, regs.reg_read(0x1_0428));
        assert_eq!(0x8080_8080, regs.reg_read(0x1_042c));
        assert_eq!(0x8080_8080, regs.reg_read(0x1_0430));
        assert_eq!(0x8080_8080, regs.reg_read(0x1_0434));
        assert_eq!(0x8080_8080, regs.reg_read(0x1_0438));
        assert_eq!(0x8080_8080, regs.reg_read(0x1_043c));
        assert_eq!(0x0000_0000, regs.reg_read(0x1_0440));
        assert_eq!(0x0000_0000, regs.reg_read(0x1_0444));
        assert_eq!(0x0000_0000, regs.reg_read(0x1_0448));
        assert_eq!(0x0000_0000, regs.reg_read(0x1_044c));
        assert_eq!(0x0000_0000, regs.reg_read(0x1_0450));
        assert_eq!(0x0000_0000, regs.reg_read(0x1_0454));
        assert_eq!(0x0000_0000, regs.reg_read(0x1_0458));
        assert_eq!(0x0000_0000, regs.reg_read(0x1_045c));

        // ICFGR
        assert_eq!(0x0000_0000, regs.reg_read(0x1_0c00));
        assert_eq!(0x0000_0000, regs.reg_read(0x1_0c04));
        assert_eq!(0x0000_0000, regs.reg_read(0x1_0c08));
    }

    #[test]
    fn set_interrupt_priority() {
        let tests = [
            TestCase::new(IntId::sgi(0), 0x1_0400, 0x0000_00ab, 0xab),
            TestCase::new(IntId::sgi(15), 0x1_040c, 0xcd00_0000, 0xcd),
            TestCase::new(IntId::ppi(0), 0x1_0410, 0x0000_0012, 0x12),
            TestCase::new(IntId::ppi(15), 0x1_041c, 0x3400_0000, 0x34),
            TestCase::new(IntId::eppi(0), 0x1_0420, 0x0000_0056, 0x56),
            TestCase::new(IntId::eppi(63), 0x1_045c, 0x7800_0000, 0x78),
        ];

        let mut regs = FakeRedistributor::new();

        for test in tests {
            let mut redistributor = regs.redistributor_for_test();
            assert_eq!(
                Ok(()),
                redistributor.set_interrupt_priority(test.intid, test.param)
            );
            assert_eq!(
                test.expected_value,
                regs.reg_read(test.offset),
                "test case: {test:x?}",
            );
            regs.clear();
        }

        let mut redistributor = regs.redistributor_for_test();
        assert_eq!(
            Ok(()),
            redistributor.set_interrupt_priority(IntId::sgi(0), 0xcd)
        );
        assert_eq!(
            Ok(()),
            redistributor.set_interrupt_priority(IntId::sgi(1), 0xab)
        );
        assert_eq!(
            Ok(()),
            redistributor.set_interrupt_priority(IntId::sgi(3), 0x12)
        );
        assert_eq!(
            Err(GicError::InvalidGicrIntid(IntId::spi(0))),
            redistributor.set_interrupt_priority(IntId::spi(0), 0x12)
        );
        assert_eq!(0x1200_abcd, regs.reg_read(0x1_0400));
    }

    #[test]
    fn set_trigger() {
        let tests = [
            TestCase::new(IntId::sgi(0), 0x1_0c00, 0x0000_0002, Trigger::Edge),
            TestCase::new(IntId::sgi(15), 0x1_0c00, 0x8000_0000, Trigger::Edge),
            TestCase::new(IntId::ppi(0), 0x1_0c04, 0x0000_0002, Trigger::Edge),
            TestCase::new(IntId::ppi(15), 0x1_0c04, 0x8000_0000, Trigger::Edge),
            TestCase::new(IntId::eppi(0), 0x1_0c08, 0x0000_0002, Trigger::Edge),
            TestCase::new(IntId::eppi(63), 0x1_0c14, 0x8000_0000, Trigger::Edge),
        ];

        let mut regs = FakeRedistributor::new();

        for test in tests {
            let mut redistributor = regs.redistributor_for_test();
            assert_eq!(Ok(()), redistributor.set_trigger(test.intid, test.param));
            assert_eq!(
                test.expected_value,
                regs.reg_read(test.offset),
                "test case: {test:x?}",
            );
            regs.clear();
        }

        let mut redistributor = regs.redistributor_for_test();
        assert_eq!(
            Ok(()),
            redistributor.set_trigger(IntId::sgi(0), Trigger::Edge)
        );
        assert_eq!(
            Ok(()),
            redistributor.set_trigger(IntId::sgi(1), Trigger::Edge)
        );
        assert_eq!(
            Ok(()),
            redistributor.set_trigger(IntId::sgi(3), Trigger::Edge)
        );
        assert_eq!(
            Ok(()),
            redistributor.set_trigger(IntId::sgi(3), Trigger::Level)
        );
        assert_eq!(
            Err(GicError::InvalidGicrIntid(IntId::spi(0))),
            redistributor.set_trigger(IntId::spi(0), Trigger::Level)
        );
        assert_eq!(0x0000_000a, regs.reg_read(0x1_0c00));
    }

    #[test]
    fn set_group() {
        let tests = [
            TestCase::new(IntId::sgi(0), 0, 1, Group::Group1NS),
            TestCase::new(IntId::sgi(15), 15, 0, Group::Secure(SecureIntGroup::Group0)),
            TestCase::new(IntId::ppi(0), 16, 2, Group::Secure(SecureIntGroup::Group1S)),
            TestCase::new(IntId::ppi(15), 31, 1, Group::Group1NS),
            TestCase::new(IntId::eppi(0), 32, 0, Group::Secure(SecureIntGroup::Group0)),
            TestCase::new(
                IntId::eppi(63),
                95,
                2,
                Group::Secure(SecureIntGroup::Group1S),
            ),
        ];

        let mut regs = FakeRedistributor::new();

        for test in tests {
            let mut redistributor = regs.redistributor_for_test();
            assert_eq!(Ok(()), redistributor.set_group(test.intid, test.param));

            let expected_igroupr = (test.expected_value & 1) << (test.offset % 32);
            let exptected_igrpmodr = ((test.expected_value & 2) >> 1) << (test.offset % 32);

            assert_eq!(
                expected_igroupr,
                regs.reg_read(0x1_0080 + test.offset / 32 * 4),
                "test case: {test:x?}",
            );

            assert_eq!(
                exptected_igrpmodr,
                regs.reg_read(0x1_0d00 + test.offset / 32 * 4),
                "test case: {test:x?}",
            );
            regs.clear();
        }

        let mut redistributor = regs.redistributor_for_test();
        assert_eq!(
            Ok(()),
            redistributor.set_group(IntId::sgi(0), Group::Group1NS)
        );
        assert_eq!(
            Ok(()),
            redistributor.set_group(IntId::sgi(1), Group::Secure(SecureIntGroup::Group0))
        );
        assert_eq!(
            Ok(()),
            redistributor.set_group(IntId::sgi(3), Group::Secure(SecureIntGroup::Group1S))
        );
        assert_eq!(
            Ok(()),
            redistributor.set_group(IntId::sgi(3), Group::Group1NS)
        );
        assert_eq!(
            Err(GicError::InvalidGicrIntid(IntId::spi(0))),
            redistributor.set_group(IntId::spi(0), Group::Group1NS)
        );
        assert_eq!(0x0000_0009, regs.reg_read(0x1_0080));
        assert_eq!(0x0000_0000, regs.reg_read(0x1_0d00));
    }

    #[test]
    fn enable_interrupt() {
        let tests = [
            TestCase::new(IntId::sgi(0), 0x1_0100, 0x0000_0001, true),
            TestCase::new(IntId::sgi(15), 0x1_0180, 0x0000_8000, false),
            TestCase::new(IntId::ppi(0), 0x1_0100, 0x0001_0000, true),
            TestCase::new(IntId::ppi(15), 0x1_0180, 0x8000_0000, false),
            TestCase::new(IntId::eppi(0), 0x1_0104, 0x0000_0001, true),
            TestCase::new(IntId::eppi(63), 0x1_0188, 0x8000_0000, false),
        ];

        let mut regs = FakeRedistributor::new();

        for test in tests {
            let mut redistributor = regs.redistributor_for_test();
            assert_eq!(
                Ok(()),
                redistributor.enable_interrupt(test.intid, test.param)
            );
            assert_eq!(
                test.expected_value,
                regs.reg_read(test.offset),
                "test case: {test:x?}",
            );
            regs.clear();
        }

        let mut redistributor = regs.redistributor_for_test();
        assert_eq!(Ok(()), redistributor.enable_interrupt(IntId::sgi(0), true));
        assert_eq!(Ok(()), redistributor.enable_interrupt(IntId::sgi(1), true));
        assert_eq!(Ok(()), redistributor.enable_interrupt(IntId::sgi(3), true));
        assert_eq!(Ok(()), redistributor.enable_interrupt(IntId::sgi(3), false));
        assert_eq!(
            Err(GicError::InvalidGicrIntid(IntId::spi(0))),
            redistributor.enable_interrupt(IntId::spi(0), false)
        );
        assert_eq!(0x0000_000b, regs.reg_read(0x1_0100));
        assert_eq!(0x0000_0008, regs.reg_read(0x1_0180));
    }

    #[test]
    fn enable_all_interrupts() {
        let mut regs = FakeRedistributor::new();

        // GICR_TYPER.PPInum = 0
        {
            let mut redistributor = regs.redistributor_for_test();
            redistributor.enable_all_interrupts(true);
        }

        assert_eq!(0xffff_ffff, regs.reg_read(0x1_0100));
        assert_eq!(0x0000_0000, regs.reg_read(0x1_0104));
        assert_eq!(0x0000_0000, regs.reg_read(0x1_0108));

        regs.clear();

        // GICR_TYPER.PPInum = 1
        regs.regs_write(0x0_008, 1 << 27);
        {
            let mut redistributor = regs.redistributor_for_test();
            redistributor.enable_all_interrupts(false);
        }

        assert_eq!(0xffff_ffff, regs.reg_read(0x1_0180));
        assert_eq!(0xffff_ffff, regs.reg_read(0x1_0184));
        assert_eq!(0x0000_0000, regs.reg_read(0x1_0188));

        regs.clear();

        // GICR_TYPER.PPInum = 2
        regs.regs_write(0x0_008, 2 << 27);
        {
            let mut redistributor = regs.redistributor_for_test();
            redistributor.enable_all_interrupts(true);
        }

        assert_eq!(0xffff_ffff, regs.reg_read(0x1_0100));
        assert_eq!(0xffff_ffff, regs.reg_read(0x1_0104));
        assert_eq!(0xffff_ffff, regs.reg_read(0x1_0108));
    }

    #[test]
    fn context_size() {
        assert_eq!(
            (24usize + 32 * 15 / 8).next_multiple_of(size_of::<usize>()),
            size_of::<GicRedistributorContext::<{ GicRedistributorContext::ireg_count(32) }>>()
        );

        assert_eq!(
            (24usize + 64 * 15 / 8).next_multiple_of(size_of::<usize>()),
            size_of::<GicRedistributorContext::<{ GicRedistributorContext::ireg_count(64) }>>()
        );

        assert_eq!(
            (24usize + 96 * 15 / 8).next_multiple_of(size_of::<usize>()),
            size_of::<GicRedistributorContext::<{ GicRedistributorContext::ireg_count(96) }>>()
        );
    }

    #[test]
    fn save_restore() {
        let mut context =
            GicRedistributorContext::<{ GicRedistributorContext::ireg_count(32) }>::new();

        let saved_offsets = [
            0x0_0000..=0x0_0000, // GICR_CTLR
            0x0_0070..=0x0_0070, // GICR_PROPBASER
            0x0_0078..=0x0_0078, // GICR_PENDBASER
            0x1_0080..=0x1_0080, // GICR_IGROUPR
            0x1_0100..=0x1_0100, // GICR_ISENABLER
            0x1_0200..=0x1_0200, // GICR_ISPENR
            0x1_0300..=0x1_0300, // GICR_ISACTIVER
            0x1_0400..=0x1_041c, // GICR_IPRIORITYR
            0x1_0c00..=0x1_0c04, // GICR_ICFGR
            0x1_0d00..=0x1_0d00, // GICR_IGRPMODR
            0x1_0e00..=0x1_0e00, // GICR_NSACR
        ];

        fn generate_value(offset: usize) -> u32 {
            (offset * 2 + 1) as u32
        }

        let mut regs = FakeRedistributor::new();

        // GICR_TYPER.PPInum = 0

        // Fill registers with values
        for offset_range in &saved_offsets {
            for offset in offset_range.clone().step_by(4) {
                regs.regs_write(offset, generate_value(offset));
            }
        }

        // Save redistributor registers
        {
            let redistributor = regs.redistributor_for_test();
            assert_eq!(Ok(()), redistributor.save(&mut context));
        }

        // Create clean redistributor and restore registers
        regs.clear();

        // GICR_TYPER.PPInum = 0

        {
            let mut redistributor = regs.redistributor_for_test();
            assert_eq!(Ok(()), redistributor.restore(&context));
        }

        // Validate registers
        for offset_range in saved_offsets {
            for offset in offset_range.step_by(4) {
                assert_eq!(
                    generate_value(offset),
                    regs.reg_read(offset),
                    "offset {offset:#x}",
                );
            }
        }
    }

    #[test]
    fn save_restore_extended() {
        let mut context =
            GicRedistributorContext::<{ GicRedistributorContext::ireg_count(96) }>::new();

        let saved_offsets = [
            0x0_0000..=0x0_0000, // GICR_CTLR
            0x0_0070..=0x0_0070, // GICR_PROPBASER
            0x0_0078..=0x0_0078, // GICR_PENDBASER
            0x1_0080..=0x1_0088, // GICR_IGROUPR
            0x1_0100..=0x1_0108, // GICR_ISENABLER
            0x1_0200..=0x1_0208, // GICR_ISPENR
            0x1_0300..=0x1_0308, // GICR_ISACTIVER
            0x1_0400..=0x1_045c, // GICR_IPRIORITYR
            0x1_0c00..=0x1_0c14, // GICR_ICFGR
            0x1_0d00..=0x1_0d08, // GICR_IGRPMODR
            0x1_0e00..=0x1_0e00, // GICR_NSACR
        ];

        fn generate_value(offset: usize) -> u32 {
            (offset * 2 + 1) as u32
        }

        let mut regs = FakeRedistributor::new();

        // GICR_TYPER.PPInum = 2
        regs.regs_write(0x0_008, 2 << 27);

        // Fill registers with values
        for offset_range in &saved_offsets {
            for offset in offset_range.clone().step_by(4) {
                regs.regs_write(offset, generate_value(offset));
            }
        }

        // Save redistributor registers
        {
            let redistributor = regs.redistributor_for_test();
            assert_eq!(Ok(()), redistributor.save(&mut context));
        }

        // Create clean redistributor and restore registers
        regs.clear();

        // GICR_TYPER.PPInum = 2
        regs.regs_write(0x0_008, 2 << 27);

        {
            let mut redistributor = regs.redistributor_for_test();
            assert_eq!(Ok(()), redistributor.restore(&context));
        }

        // Validate registers
        for offset_range in saved_offsets {
            for offset in offset_range.step_by(4) {
                assert_eq!(
                    generate_value(offset),
                    regs.reg_read(offset),
                    "offset {offset:#x}",
                );
            }
        }
    }

    #[test]
    fn mark_core_awake() {
        let mut regs = FakeRedistributor::new();

        // GICR_WAKER.ChildrenAsleep = 0
        let mut redistributor = regs.redistributor_for_test();
        assert_eq!(Err(GicError::AlreadyAwake), redistributor.mark_core_awake());

        // Cannot test further without waiting in infinite loop
    }

    #[test]
    fn mark_core_asleep() {
        let mut regs = FakeRedistributor::new();

        // GICR_WAKER.ChildrenAsleep = 1
        regs.regs_write(0x14, 0x0000_0004);

        let mut redistributor = regs.redistributor_for_test();
        assert_eq!(
            Err(GicError::AlreadyAsleep),
            redistributor.mark_core_asleep()
        );

        // Cannot test further without waiting in infinite loop
    }

    #[test]
    fn wait_for_pending_write() {
        let mut regs = FakeRedistributor::new();
        let redistributor = regs.redistributor_for_test();
        redistributor.wait_for_pending_write();
    }

    #[test]
    fn wait_for_upstream_pending_write() {
        let mut regs = FakeRedistributor::new();
        let redistributor = regs.redistributor_for_test();
        redistributor.wait_for_upstream_pending_write();
    }

    #[test]
    fn typer() {
        let mut regs = FakeRedistributor::new();

        regs.regs_write(0x0_0008, 0xabcd_0123);

        let redistributor = regs.redistributor_for_test();
        let typer = redistributor.typer();
        assert_eq!(0xcd01, typer.processor_number());
    }
}
