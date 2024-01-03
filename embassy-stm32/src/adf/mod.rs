#![macro_use]

use embassy_embedded_hal::SetConfig;
use embassy_hal_internal::{into_ref, PeripheralRef};

pub use crate::dma::word;
use crate::dma::{ringbuffer, Channel, ReadableRingBuffer, Request, TransferOptions, WritableRingBuffer};
use crate::gpio::sealed::{AFType, Pin as _};
use crate::gpio::AnyPin;
use crate::pac::adf::{vals, Adf as Regs};
use crate::rcc::RccPeripheral;
use crate::{peripherals, Peripheral};

#[derive(Debug, PartialEq, Eq, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    NotATransmitter,
    NotAReceiver,
    OverrunError,
}

#[derive(Copy, Clone)]
pub enum AcquisitionMode {
    AsynchronousContinuous,
    AsynchronousSingleShot,
    SynchronousContinuous,
    SynchonousSingleShot,
    WindowContinuous,
}

impl AcquisitionMode {
    fn val(&self) -> vals::Acqmod {
        match self {
            AcquisitionMode::AsynchronousContinuous => vals::Acqmod::ASYNCHRONOUSCONTINUOUS,
            AcquisitionMode::AsynchronousSingleShot => vals::Acqmod::ASYNCHRONOUSSINGLESHOT,
            AcquisitionMode::SynchronousContinuous => vals::Acqmod::SYNCRONOUSCONTINUOUS,
            AcquisitionMode::SynchonousSingleShot => vals::Acqmod::SYNCRONOUSSINGLESHOT,
            AcquisitionMode::WindowContinuous => vals::Acqmod::WINDOWCONTINUOUS,
        }
    }
}

#[derive(Copy, Clone)]
pub enum ReceiveFifoThreshold {
    NotEmpty,
    HalfFull,
}

impl ReceiveFifoThreshold {
    fn val(&self) -> vals::Rxfifo {
        match self {
            ReceiveFifoThreshold::NotEmpty => vals::Rxfifo::NOTEMPTY,
            ReceiveFifoThreshold::HalfFull => vals::Rxfifo::HALFFULL,
        }
    }
}

#[derive(Copy, Clone)]
pub enum TriggerSensitivitySelection {
    RisingEdge,
    FallingEdge,
}

impl TriggerSensitivitySelection {
    fn val(&self) -> vals::Trgsens {
        match self {
            TriggerSensitivitySelection::RisingEdge => vals::Trgsens::RISINGEDGE,
            TriggerSensitivitySelection::FallingEdge => vals::Trgsens::FALLINGEDGE,
        }
    }
}

#[derive(Copy, Clone)]
pub enum Mode {
    Master,
    Slave,
}

#[derive(Copy, Clone)]
pub enum ClockDirection {
    Input,
    Output,
}

impl ClockDirection {
    fn val(&self) -> vals::Cckdir {
        match self {
            ClockDirection::Input => vals::Cckdir::INPUT,
            ClockDirection::Output => vals::Cckdir::OUTPUT,
        }
    }
}

#[derive(Copy, Clone)]
pub enum ClockGeneratorMode {
    Immediate,
    Trigger,
}

impl ClockGeneratorMode {
    fn val(&self) -> vals::Ckgmod {
        match self {
            ClockGeneratorMode::Immediate => vals::Ckgmod::IMMEDIATE,
            ClockGeneratorMode::Trigger => vals::Ckgmod::TRIGGER,
        }
    }
}

#[derive(Copy, Clone)]
pub enum HangoverTimeWindow {
    Frames4,
    Frames8,
    Frames16,
    Frames32,
    Frames64,
    Frames128,
    Frames256,
    Frames512,
}

impl HangoverTimeWindow {
    fn val(&self) -> vals::Hgovr {
        match self {
            HangoverTimeWindow::Frames4 => vals::Hgovr::FRAMES4,
            HangoverTimeWindow::Frames8 => vals::Hgovr::FRAMES8,
            HangoverTimeWindow::Frames16 => vals::Hgovr::FRAMES16,
            HangoverTimeWindow::Frames32 => vals::Hgovr::FRAMES32,
            HangoverTimeWindow::Frames64 => vals::Hgovr::FRAMES64,
            HangoverTimeWindow::Frames128 => vals::Hgovr::FRAMES128,
            HangoverTimeWindow::Frames256 => vals::Hgovr::FRAMES256,
            HangoverTimeWindow::Frames512 => vals::Hgovr::FRAMES512,
        }
    }
}

#[derive(Copy, Clone)]
pub enum NoiseLearningFrames {
    Frames2,
    Frames4,
    Frames8,
    Frames16,
    Frames32,
}

impl NoiseLearningFrames {
    fn val(&self) -> vals::Lfrnb {
        match self {
            NoiseLearningFrames::Frames2 => vals::Lfrnb::FRAMES2,
            NoiseLearningFrames::Frames4 => vals::Lfrnb::FRAMES4,
            NoiseLearningFrames::Frames8 => vals::Lfrnb::FRAMES8,
            NoiseLearningFrames::Frames16 => vals::Lfrnb::FRAMES16,
            NoiseLearningFrames::Frames32 => vals::Lfrnb::FRAMES32,
        }
    }
}

#[derive(Copy, Clone)]
pub enum SignalToNoiseThreshold {
    LevelPlus3dot5dB,
    LevelPlus6dot0dB,
    LevelPlus9dot5dB,
    LevelPlus12dot0dB,
    LevelPlus15dot6dB,
    LevelPlus18dot0dB,
    LevelPlus21dot6dB,
    LevelPlus24dot1dB,
    LevelPlus27dot6dB,
    LevelPlus30dot1dB,
}

impl SignalToNoiseThreshold {
    fn val(&self) -> vals::Snthr {
        match self {
            SignalToNoiseThreshold::LevelPlus3dot5dB => vals::Snthr::NOISEPLUS3_5,
            SignalToNoiseThreshold::LevelPlus6dot0dB => vals::Snthr::NOISEPLUS6_0,
            SignalToNoiseThreshold::LevelPlus9dot5dB => vals::Snthr::NOISEPLUS9_5,
            SignalToNoiseThreshold::LevelPlus12dot0dB => vals::Snthr::NOISEPLUS12,
            SignalToNoiseThreshold::LevelPlus15dot6dB => vals::Snthr::NOISEPLUS15_6,
            SignalToNoiseThreshold::LevelPlus18dot0dB => vals::Snthr::NOISEPLUS18,
            SignalToNoiseThreshold::LevelPlus21dot6dB => vals::Snthr::NOISEPLUS21_6,
            SignalToNoiseThreshold::LevelPlus24dot1dB => vals::Snthr::NOISEPLUS24_1,
            SignalToNoiseThreshold::LevelPlus27dot6dB => vals::Snthr::NOISEPLUS27_6,
            SignalToNoiseThreshold::LevelPlus30dot1dB => vals::Snthr::NOISEPLUS30_1,
        }
    }
}

pub mod sound_activity_detector {
    use super::*;

    #[derive(Copy, Clone)]
    pub enum WorkingMode {
        VoiceActivityDetector,
        SoundDetector,
        AmbientNoiseEstimator,
    }

    impl WorkingMode {
        fn val(&self) -> vals::Sadmod {
            match self {
                WorkingMode::VoiceActivityDetector => vals::Sadmod::THRESHOLDESTIMATEDAMBIENTNOISE,
                WorkingMode::SoundDetector => vals::Sadmod::THRESHOLDMINIMUMNOISELEVEL,
                WorkingMode::AmbientNoiseEstimator => vals::Sadmod::THRESHOLDMINIMUMNOISELEVELX4,
            }
        }
    }

    #[derive(Copy, Clone)]
    pub enum FrameSize {
        Samples8,
        Samples16,
        Samples32,
        Samples64,
        Samples128,
        Samples256,
        Samples512,
    }

    impl FrameSize {
        fn val(&self) -> vals::Frsize {
            match self {
                FrameSize::Samples8 => vals::Frsize::SAMPLES8,
                FrameSize::Samples16 => vals::Frsize::SAMPLES16,
                FrameSize::Samples32 => vals::Frsize::SAMPLES32,
                FrameSize::Samples64 => vals::Frsize::SAMPLES64,
                FrameSize::Samples128 => vals::Frsize::SAMPLES128,
                FrameSize::Samples256 => vals::Frsize::SAMPLES256,
                FrameSize::Samples512 => vals::Frsize::SAMPLES512,
            }
        }
    }

    #[derive(Copy, Clone)]
    pub enum TriggerEventConfiguration {
        EnterMonitor,
        EnerExitDetect,
    }

    impl TriggerEventConfiguration {
        fn val(&self) -> vals::Detcfg {
            match self {
                TriggerEventConfiguration::EnterMonitor => vals::Detcfg::MONITOR,
                TriggerEventConfiguration::EnerExitDetect => vals::Detcfg::DETECT,
            }
        }
    }

    #[derive(Copy, Clone)]
    pub enum DataCaptureMode {
        Disabled,
        OnDetected,
        Enabled,
    }

    impl DataCaptureMode {
        fn val(&self) -> vals::Datcap {
            match self {
                DataCaptureMode::Disabled => vals::Datcap::DISABLED,
                DataCaptureMode::OnDetected => vals::Datcap::ONDETECTED,
                DataCaptureMode::Enabled => vals::Datcap::ENABLED,
            }
        }
    }

    #[derive(Copy, Clone)]
    pub struct Config {
        pub working_mode: WorkingMode,
        pub minimum_noise_level: word::U13,
        pub hangover_time_window: HangoverTimeWindow,
        pub noise_learning_frames: NoiseLearningFrames,
        pub ambient_noise_slope_control: word::U3,
        pub signal_to_noise_threshold: SignalToNoiseThreshold,
        pub frame_size: FrameSize,
        pub trigger_event_configuration: TriggerEventConfiguration,
        pub data_capture_mode: DataCaptureMode,
    }
}

/// [`ADF`] configuration.
#[non_exhaustive]
#[derive(Copy, Clone)]
pub struct Config {
    pub mode: Mode,
    // The number of samples discard when DFLT0 is restarted
    pub number_discarded: u8,
    pub clock0_direction: ClockDirection,
    pub clock1_direction: ClockDirection,
    pub clock_generator_dividers: bool,
    pub sound_activity_detection: Option<sound_activity_detector::Config>,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            mode: Mode::Master,
            number_discarded: 0,
            clock0_direction: ClockDirection::Input,
            clock1_direction: ClockDirection::Input,
            clock_generator_dividers: false,
            sound_activity_detection: None,
        }
    }
}

impl Config {}

pub struct Adf<'d, T: Instance, C: Channel, W: word::Word> {
    _peri: PeripheralRef<'d, T>,
    cck0: Option<PeripheralRef<'d, AnyPin>>,
    cck1: Option<PeripheralRef<'d, AnyPin>>,
    sdi0: Option<PeripheralRef<'d, AnyPin>>,
    ring_buffer: Option<ReadableRingBuffer<'d, C, W>>,
}

impl<'d, T: Instance, C: Channel, W: word::Word> Adf<'d, T, C, W> {
    pub fn new(peri: impl Peripheral<P = T> + 'd) -> Self {
        T::enable_and_reset();

        Self {
            _peri: unsafe { peri.clone_unchecked().into_ref() },
            cck0: None,
            cck1: None,
            sdi0: None,
            ring_buffer: None,
        }
    }

    fn new_inner(
        peri: impl Peripheral<P = T> + 'd,
        cck0: Option<PeripheralRef<'d, AnyPin>>,
        cck1: Option<PeripheralRef<'d, AnyPin>>,
        sdi0: Option<PeripheralRef<'d, AnyPin>>,
        ring_buffer: ReadableRingBuffer<'d, C, W>,
        config: Config,
    ) -> Self {
        let mut adf = Self::new(peri);

        let regs = T::REGS;

        regs.ckgcr().modify(|w| {
            w.set_cck0dir(config.clock0_direction.val());
            w.set_cck1dir(config.clock1_direction.val());
        });

        //set the pins
        adf.cck0 = cck0;
        adf.cck1 = cck1;
        adf.sdi0 = sdi0;
        adf.ring_buffer = Some(ring_buffer);

        adf
    }
}

impl<'d, T: Instance, C: Channel, W: word::Word> Drop for Adf<'d, T, C, W> {
    fn drop(&mut self) {
        //let ch = T::REGS.ch(self.sub_block as usize);

        //hit the master disable

        self.cck0.as_ref().map(|x| x.set_as_disconnected());
        self.cck1.as_ref().map(|x| x.set_as_disconnected());
        self.sdi0.as_ref().map(|x| x.set_as_disconnected());
    }
}

pub(crate) mod sealed {
    use super::*;

    pub trait Instance {
        const REGS: Regs;
    }
}

pub trait Word: word::Word {}

pub trait Instance: Peripheral<P = Self> + sealed::Instance + RccPeripheral {}
pin_trait!(Cck0, Instance);
pin_trait!(Cck1, Instance);
pin_trait!(Sdi0, Instance);

dma_trait!(RxDma, Instance);

foreach_peripheral!(
    (adf, $inst:ident) => {
        impl sealed::Instance for peripherals::$inst {
            const REGS: Regs = crate::pac::$inst;
        }
        impl Instance for peripherals::$inst {}
    };
);

impl<'d, T: Instance, C: Channel, W: word::Word> SetConfig for Adf<'d, T, C, W> {
    type Config = Config;
    type ConfigError = ();
    fn set_config(&mut self, _config: &Self::Config) -> Result<(), ()> {
        // self.reconfigure(*config);

        Ok(())
    }
}
