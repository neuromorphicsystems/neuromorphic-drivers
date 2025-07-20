import neuromorphic_drivers as nd

nd.print_device_list()

with nd.open() as device:
    print(device.serial(), device.properties())
    for status, packet in device:
        if packet.polarity_events_overflow_indices is not None:
            print(status, packet.polarity_events_overflow_indices)
        else:
            print(status)
