import uuid

vertices = (
        (3, -0.1, -0.75),
        (3, 0.1, -0.75),
        (-3, 0.1, -0.75),
        (-3, -0.1, -0.75),
        (3, -0.1, 0.75),
        (3, 0.1, 0.75),
        (-3, -0.1, 0.75),
        (-3, 0.1, 0.75)
        )

edges = (
        (0,1),
        (0,3),
        (0,4),
        (2,1),
        (2,3),
        (2,7),
        (6,3),
        (6,4),
        (6,7),
        (5,1),
        (5,4),
        (5,7)
        )

surfaces = (
        (0,1,2,3),
        (3,2,7,6),
        (6,7,5,4),
        (4,5,1,0),
        (1,5,7,2),
        (4,0,3,6)
        )

colors = (
        (1,0,0),
        (1,0,0),
        (1,0,0),
        (1,0,0),
        (1,0,0),
        (1,0,0)
        )

SERVICE_UUID    = uuid.UUID("42200001-5520-5820-4920-53204d204f20")
FORCE_UUID      = uuid.UUID("42201111-5520-5820-4920-53204d204f20")
ACCEL_UUID      = uuid.UUID("42202222-5520-5820-4920-53204d204f20")
GYRO_UUID       = uuid.UUID("42203333-5520-5820-4920-53204d204f20")

MPU_ACCEL_READINGSCALE_2G       = 16384.0
MPU_GYRO_READINGSCALE_250DEG    = 131.0
BRUXISM_UUIDS   = [FORCE_UUID, ACCEL_UUID, GYRO_UUID]

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    END = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'