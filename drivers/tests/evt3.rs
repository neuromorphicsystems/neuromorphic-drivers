#[test]
fn convert() {
    let bytes = std::fs::read("tests/test.raw").unwrap();

    for _ in 0..10 {
        {
            let mut adapter =
                neuromorphic_drivers::adapters::evt3::Adapter::from_dimensions(1280, 720);
            let start = std::time::Instant::now();
            let dvs_events_length = {
                let events_lengths = adapter.events_lengths(&bytes);
                events_lengths.on + events_lengths.off
            };
            let mut events = Vec::new();
            events.reserve(dvs_events_length);
            unsafe {
                events.set_len(dvs_events_length);
            }
            let events_slice = &mut events[..];
            let mut index = 0;
            adapter.convert(
                &bytes,
                |event| {
                    events_slice[index] = event;
                    index += 1;
                },
                |_| {},
            );
            events.truncate(index);
            println!(
                "convert (calc. size + single allocation): {} µs, t = {}, dvs={}",
                start.elapsed().as_micros(),
                adapter.state().t,
                events.len(),
            );
            std::fs::write("tests/test.events", unsafe {
                std::slice::from_raw_parts(
                    events.as_ptr() as *const u8,
                    events.len()
                        * core::mem::size_of::<neuromorphic_types::PolarityEvent<u64, u16, u16>>(),
                )
            })
            .unwrap();
        }
        {
            let mut events = Vec::new();
            let mut adapter =
                neuromorphic_drivers::adapters::evt3::Adapter::from_dimensions(1280, 720);
            let start = std::time::Instant::now();
            adapter.convert(&bytes, |event| events.push(event), |_| {});
            println!(
                "convert (dynamic allocation): {} µs, t = {}, dvs={}",
                start.elapsed().as_micros(),
                adapter.state().t,
                events.len(),
            );
            std::fs::write("tests/test.events", unsafe {
                std::slice::from_raw_parts(
                    events.as_ptr() as *const u8,
                    events.len()
                        * core::mem::size_of::<neuromorphic_types::PolarityEvent<u64, u16, u16>>(),
                )
            })
            .unwrap();
        }
        {
            let mut adapter =
                neuromorphic_drivers::adapters::evt3::Adapter::from_dimensions(1280, 720);
            let start = std::time::Instant::now();
            adapter.convert(&bytes, |_| {}, |_| {});
            println!(
                "consume: {} µs, t = {}",
                start.elapsed().as_micros(),
                adapter.state().t,
            );
        }
    }
}
