import typing

from . import enums
from .. import serde
from .devices import inivation_davis346
from .devices import inivation_dvxplorer
from .devices import prophesee_evk3_hd
from .devices import prophesee_evk4
from .devices import centuryarks_vga
from .devices import lucid_triton


Properties = typing.Union[
    inivation_davis346.Properties,
    inivation_dvxplorer.Properties,
    prophesee_evk3_hd.Properties,
    prophesee_evk4.Properties,
    centuryarks_vga.Properties,
    lucid_triton.Properties,
]

Configuration = typing.Union[
    inivation_davis346.Configuration,
    inivation_dvxplorer.Configuration,
    prophesee_evk3_hd.Configuration,
    prophesee_evk4.Configuration,
    centuryarks_vga.Configuration,
    lucid_triton.Configuration,
]

BiasesBounds = typing.Union[
    inivation_davis346.BiasesBounds,
    inivation_dvxplorer.BiasesBounds,
    prophesee_evk3_hd.BiasesBounds,
    prophesee_evk4.BiasesBounds,
    centuryarks_vga.BiasesBounds,
    lucid_triton.BiasesBounds,
]

RingConfiguration = typing.Union[
    inivation_davis346.RingConfiguration,
    inivation_dvxplorer.RingConfiguration,
    prophesee_evk3_hd.RingConfiguration,
    prophesee_evk4.RingConfiguration,
    centuryarks_vga.RingConfiguration,
    lucid_triton.RingConfiguration,
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
    if name == enums.Name.CENTURYARKS_VGA:
        return centuryarks_vga.Properties()
    if name == enums.Name.LUCID_TRITON:
        return lucid_triton.Properties()
    raise Exception(f"unknown name {name}")


def deserialize_biases_bounds(name: enums.Name, data: bytes) -> BiasesBounds:
    if name == enums.Name.INIVATION_DAVIS346:
        return serde.bincode.deserialize(data, inivation_davis346.BiasesBounds)[0]
    if name == enums.Name.INIVATION_DVXPLORER:
        return serde.bincode.deserialize(data, inivation_dvxplorer.BiasesBounds)[0]
    if name == enums.Name.PROPHESEE_EVK3_HD:
        return serde.bincode.deserialize(data, prophesee_evk3_hd.BiasesBounds)[0]
    if name == enums.Name.PROPHESEE_EVK4:
        return serde.bincode.deserialize(data, prophesee_evk4.BiasesBounds)[0]
    if name == enums.Name.CENTURYARKS_VGA:
        return serde.bincode.deserialize(data, centuryarks_vga.BiasesBounds)[0]
    if name == enums.Name.LUCID_TRITON:
        return serde.bincode.deserialize(data, lucid_triton.BiasesBounds)[0]
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
    if name == enums.Name.CENTURYARKS_VGA:
        return serde.bincode.deserialize(data, centuryarks_vga.Configuration)[0]
    if name == enums.Name.LUCID_TRITON:
        return serde.bincode.deserialize(data, lucid_triton.Configuration)[0]
    raise Exception(f"unknown name {name}")
