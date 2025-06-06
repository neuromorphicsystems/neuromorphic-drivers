use neuromorphic_drivers::UsbDevice;

#[test]
fn read() -> Result<(), neuromorphic_drivers::Error> {
    let (flag, event_loop) = neuromorphic_drivers::flag_and_event_loop()?;
    let device = neuromorphic_drivers::prophesee_evk4::open(
        neuromorphic_drivers::SerialOrBusNumberAndAddress::None,
        neuromorphic_drivers::prophesee_evk4::DEFAULT_CONFIGURATION,
        &neuromorphic_drivers::prophesee_evk4::DEFAULT_USB_CONFIGURATION,
        event_loop,
        flag.clone(),
    )?;
    let start = std::time::Instant::now();
    let mut previous = std::time::Instant::now();
    while start.elapsed() < std::time::Duration::from_secs(10) {
        let buffer_view = device.next_with_timeout(&std::time::Duration::from_millis(100));
        if let Some(buffer_view) = buffer_view {
            let now = std::time::Instant::now();
            println!(
                "{} B (backlog: {} packets, delay: {} µs, data rate: {:.3} MB/s)",
                buffer_view.slice.len(),
                buffer_view.backlog(),
                buffer_view.delay().as_micros(),
                (buffer_view.slice.len() as f64 / 1e6) / now.duration_since(previous).as_secs_f64()
            );
            previous = now;
        }
        flag.load_error()?;
        if flag.load_warning().is_some() {
            eprintln!("USB circular buffer overflow");
        }
    }
    Ok(())
}
