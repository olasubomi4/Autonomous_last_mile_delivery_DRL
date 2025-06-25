from enum import Enum
class DeliveryStates(Enum):
    PREPARING = "preparing"
    PENDING = "pending"
    IN_PROGRESS = "in progress"
    COMPLETED = "completed"