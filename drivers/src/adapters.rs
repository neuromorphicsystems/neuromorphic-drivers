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

            $(
                impl From<$module::Adapter> for Adapter {
                    fn from(adapter: $module::Adapter) -> Self {
                        Self::[<$module:camel>](adapter)
                    }
                }
            )+
        }
    }
}

register! { davis346, evt3 }
