import typing

from . import enums
from .. import serde
from .devices import inivation_davis346
from .devices import inivation_dvxplorer
from .devices import prophesee_evk3_hd
from .devices import prophesee_evk4


Properties = typing.Union[
    inivation_davis346.Properties,
    inivation_dvxplorer.Properties,
    prophesee_evk3_hd.Properties,
    prophesee_evk4.Properties,
]

Configuration = typing.Union[
    inivation_davis346.Configuration,
    inivation_dvxplorer.Configuration,
    prophesee_evk3_hd.Configuration,
    prophesee_evk4.Configuration,
]

UsbConfiguration = typing.Union[
    inivation_davis346.UsbConfiguration,
    inivation_dvxplorer.UsbConfiguration,
    prophesee_evk3_hd.UsbConfiguration,
    prophesee_evk4.UsbConfiguration,
]


def name_to_properties(name: enums.Name) -> Properties:
    if name == enums.Name.INIVATION_DAVIS346:
        return inivation_davis346.Properties()
    if name == enums.Name.INIVATION_DVXPLORER:
        return inivation_dvxplorer.Properties()
    if name == enums.Name.PROPHESEE_EVK3_HD:
        return prophesee_evk3_hd.Properties()
    if name == enums.Name.PROPHESEE_EVK4:
        return prophesee_evk4.Properties()
    raise Exception(f"unknown name {name}")


def deserialize_configuration(name: enums.Name, data: bytes) -> Configuration:
    if name == enums.Name.INIVATION_DAVIS346:
        return serde.bincode.deserialize(data, inivation_davis346.Configuration)[0]
    if name == enums.Name.INIVATION_DVXPLORER:
        return serde.bincode.deserialize(data, inivation_dvxplorer.Configuration)[0]
    if name == enums.Name.PROPHESEE_EVK3_HD:
        return serde.bincode.deserialize(data, prophesee_evk3_hd.Configuration)[0]
    if name == enums.Name.PROPHESEE_EVK4:
        return serde.bincode.deserialize(data, prophesee_evk4.Configuration)[0]
    raise Exception(f"unknown name {name}")
