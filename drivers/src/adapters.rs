macro_rules! register {
    ($($module:ident),+) => {
        $(
            pub mod $module;
        )+

        paste::paste! {
            pub enum Adapter {
                $(
                    [<$module:camel>]($module::Adapter),
                )+
            }

            impl Adapter {
                pub fn state(&self) -> State {
                    match self {
                        $(
                            Self::[<$module:camel>](adapter) => State::[<$module:camel>](adapter.state().clone()),
                        )+
                    }
                }

                pub fn events_lengths(&self, slice: &[u8]) -> EventsLengths {
                    match self {
                        $(
                            Self::[<$module:camel>](adapter) => EventsLengths::[<$module:camel>](adapter.events_lengths(slice)),
                        )+
                    }
                }

                pub fn events_lengths_until(&mut self, slice: &[u8], threshold_t: u64) -> (EventsLengths, usize) {
                    match self {
                        $(
                            Self::[<$module:camel>](adapter) => {
                                let (events_lengths, position) = adapter.events_lengths_until(slice, threshold_t);
                                (EventsLengths::[<$module:camel>](events_lengths), position)
                            }
                        )+
                    }
                }

                pub fn current_t(&self) -> u64 {
                    match self {
                        $(
                            Self::[<$module:camel>](adapter) => adapter.current_t(),
                        )+
                    }
                }
            }

            $(
                impl From<$module::Adapter> for Adapter {
                    fn from(adapter: $module::Adapter) -> Self {
                        Self::[<$module:camel>](adapter)
                    }
                }
            )+

            #[derive(Clone, Copy)]
            #[allow(clippy::large_enum_variant)]
            pub enum State {
                $(
                    [<$module:camel>]($module::State),
                )+
            }

            impl State {
                pub fn current_t(&self) -> u64 {
                    match self {
                        $(
                            Self::[<$module:camel>](state) => state.t,
                        )+
                    }
                }
            }

            pub enum EventsLengths {
                $(
                    [<$module:camel>]($module::EventsLengths),
                )+
            }
        }
    }
}

register! { davis346, dvxplorer, evt3 }
