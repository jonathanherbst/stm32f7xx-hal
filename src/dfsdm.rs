use core::marker::PhantomData;
use core::mem;

use crate::pac as p;

struct Config {

}

trait Instance {
    type Error;

    fn configure(&self, config: Config)
}

enum FilterId {
    Filter1,
    Filter2,
    Filter3,
    Filter4,
}

impl Instance for p::DFSDM {
    type Error = u32;

    fn configure(&self) {

    }

    fn read_filter(filter: FilterId) -> nb::Result<s32, Error> {

    }
}

#[repr(u8)]
enum ChannelClockSamplePoint {
    SPIRisingEdge = 0b00,
    SPIFallingEdge = 0b01,
    ManchesterCoded = 0b10,
    ManchesterCodedInverted = 0b11,
}

enum ChannelClockSelect {
    External(ChannelClockSamplePoint),
    Internal(ChannelClockSamplePoint),
    InternalFallingDiv2,
    InternalRisingDiv2,
}

impl Into<u8> for ChannelClockSelect {
    fn into(self) -> u8 {
        match self {
            Self::External(_) => 0b00,
            Self::Internal(_) => 0b01,
            Self::InternalFallingDiv2 => 0b10,
            Self::InternalRisingDiv2 => 0b11,
        }
    }
}

impl ChannelClockSelect {
    fn sample_point(self) -> u8 {
        match self {
            Self::External(sp) => sp as u8,
            Self::Internal(sp) => sp as u8,
            _ => 0b00,
        }
    }
}

#[repr(u8)]
pub enum ChannelDataPacking {
    Standard = 0x00,
    Interleaved = 0x01,
    Dual = 0x10,
}

pub struct ExternalChannelConfig {
    pub data_packing: ChannelDataPacking,
    pub use_prev_input_pin: bool,
    pub clock_select: ChannelClockSelect,
    pub offset: u32,
    pub shift: u8,
}

pub struct InternalChannelConfig {
    pub data_packing: ChannelDataPacking,
}

trait Channel {
    /// Configure the channel to use an external pin
    fn configure_external(reg: &p::dfsdm::RegisterBlock, config: &ExternalChannelConfig);

    /// Configure the channel as an internal data source
    fn configure_internal(reg: &p::dfsdm::RegisterBlock, config: &ExternalChannelConfig);

    /// Read data from the channel
    fn read_indata(reg: &p::dfsdm::RegisterBlock) -> (i16, i16);

    /// Write data to the channel when used as an internal data source
    fn write_indata(reg: &p::dfsdm::RegisterBlock, indata0: i16, indata1: i16);
}

macro_rules! impl_channel {
    (
        $(
            $name:ident,
            $name_lower:ident,
            ($cfgr1:ident, $cfgr2:ident, $awscdr:ident, $wdatr:ident, $datainr:ident);
        )*
    ) => {
        pub struct Channels<I> {
            $(pub $name_lower: $name<I>,)*
        }

        impl<I> Channels<I> {
            fn new() -> Self {
                Self {
                    $($name_lower: $name(PhantomData),)*
                }
            }
        }

        $(
            pub struct $name<I>(PhantomData<I>);

            /// We always need to modify instead of write cfgr1 because cfgr1 for channel 1 contains configuration for the whole dfsdm peripheral
            impl<I> Channel for $name<I> {
                fn configure_external(reg: &p::dfsdm::RegisterBlock, config: &ExternalChannelConfig) {
                    // reg writes are unsafe, but the stm32f7 reg doesn't have enums for these registers so no unsafe option.
                    
                    // make sure the channel is off first.
                    reg.$cfgr1.modify(|r, w| {w.chen().clear_bit()});

                    reg.$cfgr2.write(|w| unsafe {
                        w
                            .offset().bits(config.offset & 0xFFFFFF)
                            .dtrbs().bits(config.shift & 0b11111)
                    });

                    reg.$awscdr.reset();

                    reg.$cfgr1.modify(|r, w| unsafe {
                        w
                            .datmpx().bits(0b00)
                            .datpack().bits(config.data_packing as u8)
                            .chinsel().bit(config.use_prev_input_pin)
                            .spicksel().bits(config.clock_select.into())
                            .sitp().bits(config.clock_select.sample_point())
                            .chen().set_bit()
                    });
                }

                fn configure_internal(reg: &p::dfsdm::RegisterBlock, config: &ExternalChannelConfig) {
                    
                    // make sure the channel is off first.
                    reg.$cfgr1.modify(|r, w| {w.chen().clear_bit()});

                    reg.$cfgr2.reset();
                    reg.$awscdr.reset();
                    
                    reg.$cfgr1.modify(|r, w| unsafe {
                        w
                            .datmpx().bits(0b10)
                            .datpack().bits(config.data_packing as u8)
                            .chen().set_bit()
                    });
                }

                fn read_indata(reg: &p::dfsdm::RegisterBlock) -> (i16, i16) {
                    let r = reg.$datainr.read();
                    unsafe {
                        (
                            mem::transmute::<u16, i16>((*r.indat0()).bits()),
                            mem::transmute::<u16, i16>((*r.indat1()).bits()),
                        )
                    }
                }

                fn write_indata(reg: &p::dfsdm::RegisterBlock, indata0: i16, indata1: i16) {
                    // the in data is supposed to be signed, but the stm32f7 reg has them unsigned
                    reg.$datainr.write(|w| unsafe {
                        w
                            .indat0().bits(mem::transmute::<i16, u16>(indata0))
                            .indat1().bits(mem::transmute::<i16, u16>(indata1))
                    })
                }
            }
        )*
    }
}

impl_channel!(
    Channel0, channel0, (dfsdm_chcfg0r1, dfsdm_chcfg0r2, dfsdm_awscd0r, dfsdm_chwdat0r, dfsdm_chdatin0r);
    Channel1, channel1, (dfsdm_chcfg1r1, dfsdm_chcfg1r2, dfsdm_awscd1r, dfsdm_chwdat1r, dfsdm_chdatin1r);
    Channel2, channel2, (dfsdm_chcfg2r1, dfsdm_chcfg2r2, dfsdm_awscd2r, dfsdm_chwdat2r, dfsdm_chdatin2r);
    Channel3, channel3, (dfsdm_chcfg3r1, dfsdm_chcfg3r2, dfsdm_awscd3r, dfsdm_chwdat3r, dfsdm_chdatin3r);
    Channel4, channel4, (dfsdm_chcfg4r1, dfsdm_chcfg4r2, dfsdm_awscd4r, dfsdm_chwdat4r, dfsdm_chdatin4r);
    Channel5, channel5, (dfsdm_chcfg5r1, dfsdm_chcfg5r2, dfsdm_awscd5r, dfsdm_chwdat5r, dfsdm_chdatin5r);
    Channel6, channel6, (dfsdm_chcfg6r1, dfsdm_chcfg6r2, dfsdm_awscd6r, dfsdm_chwdat6r, dfsdm_chdatin6r);
    Channel7, channel7, (dfsdm_chcfg7r1, dfsdm_chcfg7r2, dfsdm_awscd7r, dfsdm_chwdat7r, dfsdm_chdatin7r);
);

#[repr(u8)]
pub enum FilterOrder {
    FastSinc = 0,
    Sinc1 = 1,
    Sinc2 = 2,
    Sinc3 = 3,
    Sinc4 = 4,
    Sinc5 = 5,
}

pub struct FilterConfig {
    pub fast_conversion: bool,
    pub channel: u8,
    pub enable_dma: bool,
    pub synchronous: bool,
    pub continuous: bool,
    pub order: FilterOrder,
    pub decimation_rate: u16,
    pub averaging_length: u8,
}

trait Filter {
    /// Configure the filter to use an external channel.
    fn configure_external<C: Channel>(reg: &p::dfsdm::RegisterBlock, config: &FilterConfig, channel_config: &ExternalChannelConfig);

    /// Read conversion data from the filter
    /// 
    /// # Returns
    /// 
    /// (The converted data, true if the data was delayed from an injected conversion, the channel that was converted)
    fn read_data(reg: &p::dfsdm::RegisterBlock) -> (u32, bool, u8);

    /// Clock cycles since the last conversion started.
    fn read_conversion_timer(reg: &p::dfsdm::RegisterBlock) -> u32;
}

macro_rules! impl_filters {
    (
        $(
            $name:ident,
            $name_lower:ident,
            ($cr1:ident, $cr2:ident, $isr:ident, $icr:ident, $fcr:ident, $rdatar:ident, $cnvtimr:ident);
        )*
    ) => {
        pub struct Filters<I> {
            $(pub $name_lower: $name<I>,)*
        }

        impl<I> Filters<I> {
            fn new() -> Self {
                Self {
                    $($name_lower: $name(PhantomData),)*
                }
            }
        }

        $(
            pub struct $name<I>(PhantomData<I>);

            /// We always need to modify instead of write cfgr1 because cfgr1 for channel 1 contains configuration for the whole dfsdm peripheral
            impl<I> Filter for $name<I> {
                fn configure_external<C: Channel>(reg: &p::dfsdm::RegisterBlock, config: &FilterConfig, channel_config: &ExternalChannelConfig) {
                    C::configure_external(reg, channel_config);
                    reg.$cr1.reset();
                    reg.$cr2.reset();

                    reg.$fcr.write(|w| unsafe {
                        w
                            .ford().bits(config.order as u8)
                            .fosr().bits(config.decimation_rate)
                            .iosr().bits(config.averaging_length)
                    });
                    reg.$cr1.write(|w| unsafe {
                        w
                            .fast().bit(config.fast_conversion)
                            .rch().bits(0) // todo: change to actual channel
                            .rdmaen().bit(config.enable_dma)
                            .rsync().bit(config.synchronous)
                            .rcont().bit(config.continuous)
                            .dfen().set_bit()
                    });
                }

                fn read_data(reg: &p::dfsdm::RegisterBlock) -> (u32, bool, u8) {
                    let r = reg.$rdatar.read();
                    (
                        r.rdata().bits(),
                        r.rpend().bit(),
                        r.rdatach().bits(),
                    )
                }

                fn read_conversion_timer(reg: &p::dfsdm::RegisterBlock) -> u32 {
                    reg.$cnvtimr.read().cnvcnt().bits()
                }
            }

        )*
    }
}

impl_filters!(
    Filter0, filter0, (dfsdm0_cr1, dfsdm0_cr2, dfsdm0_isr, dfsdm0_icr, dfsdm0_fcr, dfsdm0_rdatar, dfsdm0_cnvtimr);
);