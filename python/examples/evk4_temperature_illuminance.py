import time

import neuromorphic_drivers as nd

nd.print_device_list()

# The following values were estimated from a measurement curve provided by Sony
# The curve is given as an example and varies from sensor to sensor
# Users interested in accurate measurements must estimate these values for each sensor
# We use a very basic regression here, better models may exist
EVK4_ILLUMINANCE_ALPHA = 0.000000920554835579854387356562
EVK4_ILLUMINANCE_BETA = -1.009776663165910859376594999048

with nd.open(nd.prophesee_evk4.Configuration(), raw=True) as device:
    print(device.serial(), device.properties())
    next = time.monotonic() + 1.0
    for status, packet in device:
        if time.monotonic() >= next:
            temperature_celsius = device.temperature_celsius()
            illuminance = (
                EVK4_ILLUMINANCE_ALPHA * device.illuminance()
            ) ** EVK4_ILLUMINANCE_BETA
            print(f"{temperature_celsius}ºC, {illuminance} lux")
            next += 1.0
