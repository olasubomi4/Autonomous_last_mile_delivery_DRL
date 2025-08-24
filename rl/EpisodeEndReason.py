from enum import Enum

class EpisodeEndReason(str, Enum):
    SUCCESS = "success"
    COLLISION = "collision"
    STAGNATION = "stagnation"