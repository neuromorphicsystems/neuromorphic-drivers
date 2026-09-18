import enum


class Connection(enum.Enum):
    USB_UNKNOWN = "USB Unknown speed"
    USB_LOW = "USB 1.0 Low Speed (1.5 Mb/s)"
    USB_FULL = "USB 1.1 Full Speed (12 Mb/s)"
    USB_HIGH = "USB 2.0 High Speed (480 Mb/s)"
    USB_SUPER = "USB 3.0 SuperSpeed (5.0 Gb/s)"
    USB_SUPER_PLUS = "USB 3.1 SuperSpeed+ (10.0 Gb/s)"
    ETHERNET = "Ethernet"


class Name(enum.Enum):
    INIVATION_DAVIS346 = "iniVation DAVIS 346"
    INIVATION_DVXPLORER = "iniVation DVXplorer"
    PROPHESEE_EVK3_HD = "Prophesee EVK3 HD"
    PROPHESEE_EVK4 = "Prophesee EVK4"
    CENTURYARKS_VGA = "CenturyArks VGA"
    LUCID_TRITON = "Lucid Triton"
