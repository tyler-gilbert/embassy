#[allow(unused)]
#[cfg(stm32h7)]
use pac::adc::vals::{Adcaldif, Difsel, Exten};
#[allow(unused)]
#[cfg(stm32g4)]
use pac::adc::vals::{Adcaldif, Difsel, Exten, Rovsm, Trovs};
use pac::adccommon::vals::Presc;

use super::{blocking_delay_us, Adc, AdcChannel, Instance, Resolution, SampleTime};
use crate::time::Hertz;
use crate::{pac, rcc, Peripheral};

/// Default VREF voltage used for sample conversion to millivolts.
pub const VREF_DEFAULT_MV: u32 = 3300;
/// VREF voltage used for factory calibration of VREFINTCAL register.
pub const VREF_CALIB_MV: u32 = 3300;

/// Max single ADC operation clock frequency
#[cfg(stm32g4)]
const MAX_ADC_CLK_FREQ: Hertz = Hertz::mhz(60);
#[cfg(stm32h7)]
const MAX_ADC_CLK_FREQ: Hertz = Hertz::mhz(50);

#[cfg(stm32g4)]
const VREF_CHANNEL: u8 = 18;
#[cfg(stm32g4)]
const TEMP_CHANNEL: u8 = 16;

#[cfg(stm32h7)]
const VREF_CHANNEL: u8 = 19;
#[cfg(stm32h7)]
const TEMP_CHANNEL: u8 = 18;

// TODO this should be 14 for H7a/b/35
const VBAT_CHANNEL: u8 = 17;

// NOTE: Vrefint/Temperature/Vbat are not available on all ADCs, this currently cannot be modeled with stm32-data, so these are available from the software on all ADCs
/// Internal voltage reference channel.
pub struct VrefInt;
impl<T: Instance> AdcChannel<T> for VrefInt {}
impl<T: Instance> super::SealedAdcChannel<T> for VrefInt {
    fn channel(&self) -> u8 {
        VREF_CHANNEL
    }
}

/// Internal temperature channel.
pub struct Temperature;
impl<T: Instance> AdcChannel<T> for Temperature {}
impl<T: Instance> super::SealedAdcChannel<T> for Temperature {
    fn channel(&self) -> u8 {
        TEMP_CHANNEL
    }
}

/// Internal battery voltage channel.
pub struct Vbat;
impl<T: Instance> AdcChannel<T> for Vbat {}
impl<T: Instance> super::SealedAdcChannel<T> for Vbat {
    fn channel(&self) -> u8 {
        VBAT_CHANNEL
    }
}

// NOTE (unused): The prescaler enum closely copies the hardware capabilities,
// but high prescaling doesn't make a lot of sense in the current implementation and is ommited.
#[allow(unused)]
enum Prescaler {
    NotDivided,
    DividedBy2,
    DividedBy4,
    DividedBy6,
    DividedBy8,
    DividedBy10,
    DividedBy12,
    DividedBy16,
    DividedBy32,
    DividedBy64,
    DividedBy128,
    DividedBy256,
}

impl Prescaler {
    fn from_ker_ck(frequency: Hertz) -> Self {
        let raw_prescaler = frequency.0 / MAX_ADC_CLK_FREQ.0;
        match raw_prescaler {
            0 => Self::NotDivided,
            1 => Self::DividedBy2,
            2..=3 => Self::DividedBy4,
            4..=5 => Self::DividedBy6,
            6..=7 => Self::DividedBy8,
            8..=9 => Self::DividedBy10,
            10..=11 => Self::DividedBy12,
            _ => unimplemented!(),
        }
    }

    fn divisor(&self) -> u32 {
        match self {
            Prescaler::NotDivided => 1,
            Prescaler::DividedBy2 => 2,
            Prescaler::DividedBy4 => 4,
            Prescaler::DividedBy6 => 6,
            Prescaler::DividedBy8 => 8,
            Prescaler::DividedBy10 => 10,
            Prescaler::DividedBy12 => 12,
            Prescaler::DividedBy16 => 16,
            Prescaler::DividedBy32 => 32,
            Prescaler::DividedBy64 => 64,
            Prescaler::DividedBy128 => 128,
            Prescaler::DividedBy256 => 256,
        }
    }

    fn presc(&self) -> Presc {
        match self {
            Prescaler::NotDivided => Presc::DIV1,
            Prescaler::DividedBy2 => Presc::DIV2,
            Prescaler::DividedBy4 => Presc::DIV4,
            Prescaler::DividedBy6 => Presc::DIV6,
            Prescaler::DividedBy8 => Presc::DIV8,
            Prescaler::DividedBy10 => Presc::DIV10,
            Prescaler::DividedBy12 => Presc::DIV12,
            Prescaler::DividedBy16 => Presc::DIV16,
            Prescaler::DividedBy32 => Presc::DIV32,
            Prescaler::DividedBy64 => Presc::DIV64,
            Prescaler::DividedBy128 => Presc::DIV128,
            Prescaler::DividedBy256 => Presc::DIV256,
        }
    }
}

impl<'d, T: Instance> Adc<'d, T> {
    /// Create a new ADC driver.
    pub fn new(adc: impl Peripheral<P = T> + 'd) -> Self {
        embassy_hal_internal::into_ref!(adc);
        rcc::enable_and_reset::<T>();

        let prescaler = Prescaler::from_ker_ck(T::frequency());

        T::common_regs().ccr().modify(|w| w.set_presc(prescaler.presc()));

        let frequency = Hertz(T::frequency().0 / prescaler.divisor());
        info!("ADC frequency set to {} Hz", frequency.0);

        if frequency > MAX_ADC_CLK_FREQ {
            panic!("Maximal allowed frequency for the ADC is {} MHz and it varies with different packages, refer to ST docs for more information.", MAX_ADC_CLK_FREQ.0 /  1_000_000 );
        }

        let mut s = Self {
            adc,
            sample_time: SampleTime::from_bits(0),
        };
        s.power_up();
        s.configure_differential_inputs();

        s.calibrate();
        blocking_delay_us(1);

        s.enable();
        s.configure();

        s
    }

    fn power_up(&mut self) {
        T::regs().cr().modify(|reg| {
            reg.set_deeppwd(false);
            reg.set_advregen(true);
        });

        blocking_delay_us(10);
    }

    fn configure_differential_inputs(&mut self) {
        T::regs().difsel().modify(|w| {
            for n in 0..18 {
                w.set_difsel(n, Difsel::SINGLEENDED);
            }
        });
    }

    fn calibrate(&mut self) {
        T::regs().cr().modify(|w| {
            w.set_adcaldif(Adcaldif::SINGLEENDED);
        });

        T::regs().cr().modify(|w| w.set_adcal(true));

        while T::regs().cr().read().adcal() {}
    }

    fn enable(&mut self) {
        T::regs().isr().write(|w| w.set_adrdy(true));
        T::regs().cr().modify(|w| w.set_aden(true));
        while !T::regs().isr().read().adrdy() {}
        T::regs().isr().write(|w| w.set_adrdy(true));
    }

    fn configure(&mut self) {
        // single conversion mode, software trigger
        T::regs().cfgr().modify(|w| {
            w.set_cont(false);
            w.set_exten(Exten::DISABLED);
        });
    }

    /// Enable reading the voltage reference internal channel.
    pub fn enable_vrefint(&self) -> VrefInt {
        T::common_regs().ccr().modify(|reg| {
            reg.set_vrefen(true);
        });

        VrefInt {}
    }

    /// Enable reading the temperature internal channel.
    pub fn enable_temperature(&self) -> Temperature {
        T::common_regs().ccr().modify(|reg| {
            reg.set_vsenseen(true);
        });

        Temperature {}
    }

    /// Enable reading the vbat internal channel.
    pub fn enable_vbat(&self) -> Vbat {
        T::common_regs().ccr().modify(|reg| {
            reg.set_vbaten(true);
        });

        Vbat {}
    }

    /// Enable differential channel.
    /// Caution:
    /// : When configuring the channel “i” in differential input mode, its negative input voltage VINN[i]
    /// is connected to another channel. As a consequence, this channel is no longer usable in
    /// single-ended mode or in differential mode and must never be configured to be converted.
    /// Some channels are shared between ADC1/ADC2/ADC3/ADC4/ADC5: this can make the
    /// channel on the other ADC unusable. The only exception is when ADC master and the slave
    /// operate in interleaved mode.
    #[cfg(stm32g4)]
    pub fn set_differential_channel(&mut self, ch: usize, enable: bool) {
        T::regs().cr().modify(|w| w.set_aden(false)); // disable adc
        T::regs().difsel().modify(|w| {
            w.set_difsel(
                ch,
                if enable {
                    Difsel::DIFFERENTIAL
                } else {
                    Difsel::SINGLEENDED
                },
            );
        });
        T::regs().cr().modify(|w| w.set_aden(true));
    }

    #[cfg(stm32g4)]
    pub fn set_differential(&mut self, channel: &mut impl AdcChannel<T>, enable: bool) {
        self.set_differential_channel(channel.channel() as usize, enable);
    }

    /// Set oversampling shift.
    #[cfg(stm32g4)]
    pub fn set_oversampling_shift(&mut self, shift: u8) {
        T::regs().cfgr2().modify(|reg| reg.set_ovss(shift));
    }

    /// Set oversampling ratio.
    #[cfg(stm32g4)]
    pub fn set_oversampling_ratio(&mut self, ratio: u8) {
        T::regs().cfgr2().modify(|reg| reg.set_ovsr(ratio));
    }

    /// Enable oversampling in regular mode.
    #[cfg(stm32g4)]
    pub fn enable_regular_oversampling_mode(&mut self, mode: Rovsm, trig_mode: Trovs, enable: bool) {
        T::regs().cfgr2().modify(|reg| reg.set_trovs(trig_mode));
        T::regs().cfgr2().modify(|reg| reg.set_rovsm(mode));
        T::regs().cfgr2().modify(|reg| reg.set_rovse(enable));
    }

    // Reads that are not implemented as INJECTED in "blocking_read"
    // #[cfg(stm32g4)]
    // pub fn enalble_injected_oversampling_mode(&mut self, enable: bool) {
    //     T::regs().cfgr2().modify(|reg| reg.set_jovse(enable));
    // }

    // #[cfg(stm32g4)]
    // pub fn enable_oversampling_regular_injected_mode(&mut self, enable: bool) {
    //     // the regularoversampling mode is forced to resumed mode (ROVSM bit ignored),
    //     T::regs().cfgr2().modify(|reg| reg.set_rovse(enable));
    //     T::regs().cfgr2().modify(|reg| reg.set_jovse(enable));
    // }

    /// Set the ADC sample time.
    pub fn set_sample_time(&mut self, sample_time: SampleTime) {
        self.sample_time = sample_time;
    }

    /// Set the ADC resolution.
    pub fn set_resolution(&mut self, resolution: Resolution) {
        T::regs().cfgr().modify(|reg| reg.set_res(resolution.into()));
    }

    /// Perform a single conversion.
    fn convert(&mut self) -> u16 {
        T::regs().isr().modify(|reg| {
            reg.set_eos(true);
            reg.set_eoc(true);
        });

        // Start conversion
        T::regs().cr().modify(|reg| {
            reg.set_adstart(true);
        });

        while !T::regs().isr().read().eos() {
            // spin
        }

        T::regs().dr().read().0 as u16
    }

    /// Read an ADC pin.
    pub fn blocking_read(&mut self, channel: &mut impl AdcChannel<T>) -> u16 {
        channel.setup();

        self.read_channel(channel.channel())
    }

    fn read_channel(&mut self, channel: u8) -> u16 {
        // Configure channel
        Self::set_channel_sample_time(channel, self.sample_time);

        #[cfg(stm32h7)]
        {
            T::regs().cfgr2().modify(|w| w.set_lshift(0));
            T::regs()
                .pcsel()
                .write(|w| w.set_pcsel(channel as _, Pcsel::PRESELECTED));
        }

        T::regs().sqr1().write(|reg| {
            reg.set_sq(0, channel);
            reg.set_l(0);
        });

        self.convert()
    }

    fn set_channel_sample_time(ch: u8, sample_time: SampleTime) {
        let sample_time = sample_time.into();
        if ch <= 9 {
            T::regs().smpr().modify(|reg| reg.set_smp(ch as _, sample_time));
        } else {
            T::regs().smpr2().modify(|reg| reg.set_smp((ch - 10) as _, sample_time));
        }
    }
}
