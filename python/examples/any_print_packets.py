import neuromorphic_drivers as nd

nd.print_device_list()

with nd.open() as device:
    print(device.serial(), device.properties())
    for status, packet in device:
        print(packet)
