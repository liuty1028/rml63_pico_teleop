from enum import Enum


class TeleopState(str, Enum):
    READY = "READY"
    ACTIVE = "ACTIVE"
    HOLD = "HOLD"
    FAULT = "FAULT"
