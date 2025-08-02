import neuromorphic_drivers as nd

nd.print_device_list()

with nd.open() as device:
    print(device.serial(), device.properties())
    for status, packet in device:
        print(f"{round(status.delay() * 1e6)} µs, backlog: {status.ring.backlog}")
        if status.ring.backlog > 1000:
            device.clear_backlog(until=0)
