import neuromorphic_drivers as nd

nd.print_device_list()

with nd.open() as device:
    print(device.serial(), device.properties())
    for status, packet in device:
        if "dvs_events_overflow_indices" in packet:
            print(status, packet["dvs_events_overflow_indices"])
        else:
            print(status)
