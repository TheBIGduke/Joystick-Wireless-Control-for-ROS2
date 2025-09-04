from delivery_bridge.utils import EnumWithDescription as Enum

"""
class PathMode(Enum):
    LOOP = "loop", "Trayección en bucle"
    REVERSE_LOOP = "reverse_loop", "Trayección en bucle inverso"
    ONCE = "once", "Trayección una vez"
    STOP = "stop", "Detener trayección"
    PAUSE = "pause", "Pausar trayección"
    RESUME = "resume", "Reanudar trayección"
    CANCEL_GOAL = "cancel_goal", "Cancelar destino actual"
"""

class WpOption(Enum):
    PAUSE = "pause", "Pausar objetivo"
    RESUME = "resume", "Reanudar wp objetivo"
    CANCEL = "cancel", "Cancelar wp actual"
    STOP = "stop", "Cancelar wp actual"

class NavigationState(Enum):
    READY = 0, "Navigation is ready to receive a goal"
    ACTIVE = 1, "Navigation is processing a goal"
    SUCCEEDED = 3, "Navigation successfully completed a goal"
    CANCELED = 10, "Navigation successfully canceled a goal"
    FAILED = 11, "Navigation failed to complete a goal"
    UNKNOWN = 12, "Navigation is in an unknown state"
    PAUSED = 13, "Navigation is paused"
    REJECTED = 5, "Navigation rejected a goal"

    PREEMPTED = 2, "Navigation received a new goal and was preempted"
    ABORTED = 4, "Navigation failed to complete a goal"
    PREEMPTING = 6, "Navigation received a new goal and is preempting"
    RECALLING = 7, "Navigation received a cancel request and is recalling a goal"
    RECALLED = 8, "Navigation successfully recalled a goal"
    LOST = 9, "Navigation lost connection to the action server"
