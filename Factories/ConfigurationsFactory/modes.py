from enum import Enum
class MPCModes(Enum):
    UNCONSTRAINED = "unconst"
    UNCONSTRAINED_WITH_SOLVER = "unconst_solv"
    CONSTRAINED = "const"