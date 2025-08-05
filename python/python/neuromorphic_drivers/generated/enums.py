import enum


class Speed(enum.Enum):
    UNKNOWN = "USB Unknown speed"
    LOW = "USB 1.0 Low Speed (1.5 Mb/s)"
    FULL = "USB 1.1 Full Speed (12 Mb/s)"
    HIGH = "USB 2.0 High Speed (480 Mb/s)"
    SUPER = "USB 3.0 SuperSpeed (5.0 Gb/s)"
    SUPER_PLUS = "USB 3.1 SuperSpeed+ (10.0 Gb/s)"


class Name(enum.Enum):
    INIVATION_DAVIS346 = "iniVation DAVIS 346"
    INIVATION_DVXPLORER = "iniVation DVXplorer"
    PROPHESEE_EVK3_HD = "Prophesee EVK3 HD"
    PROPHESEE_EVK4 = "Prophesee EVK4"
