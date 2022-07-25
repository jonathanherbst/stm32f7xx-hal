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


enum ChannelClockSamplePoint {
    SPIRisingEdge,
    SPIFallingEdge,
    ManchesterCoded,
    ManchesterCodedInverted,
}

enum ChannelClockSelect {
    External(ChannelClockSamplePoint),
    Internal(ChannelClockSamplePoint),
    InternalFallingDiv2,
    InternalRisingDiv2,
}

pub struct ChannelConfig {
    pub usePrevInputPin: bool,
    pub clock_select: ChannelClockSelect,
    pub offset: u32,
    pub shift: u8,
}

trait Channel {
    fn configure(reg: &p::dfsdm::RegisterBlock, config: &ChannelConfig);
}

pub struct FilterConfig {
    pub fast_conversion: bool,
    pub channel: u8,
    pub enable_dma: bool,
    pub synchronous: bool,
    pub continuous: bool,
}

trait Filter {
    fn configure(&mut self, config: &FilterConfig);
}

macro_rules! impl_channel {
    (
        $(
            $name:ident,
            $name_lower:ident;
        )*
    ) => {
        use core::marker::PhantomData;

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

            impl<I> Channel for $name<I> {
                fn configure(reg: &p::dfsdm::RegisterBlock, config: &ChannelConfig) {
                    reg.
                }
            }
        )*
    }
}

impl_channel!(
    Channel0, channel0;
    Channel1, channel1;
);