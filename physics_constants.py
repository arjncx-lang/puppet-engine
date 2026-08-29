# physics_constants.py
# Pure physics, character tuning, and safety clamp constants

# World gravity
GRAVITY               = -20.0

# Anatomical Spaz Dimensions
HEAD_RADIUS           = 0.22
TORSO_RADIUS          = 0.15
TORSO_HEIGHT          = 0.32
PELVIS_RADIUS         = 0.12
PELVIS_HEIGHT         = 0.16

# Limbs
ARM_RADIUS            = 0.065
ARM_LENGTH            = 0.32
LEG_RADIUS            = 0.080
LEG_LENGTH            = 0.36
HAND_RADIUS           = 0.075
FOOT_RADIUS           = 0.085

# Eye parameters
EYE_OFFSET_X          = 0.070
EYE_OFFSET_Y          = 0.170
EYE_OFFSET_Z          = 0.045
EYE_RADIUS            = 0.042
PUPIL_RADIUS          = 0.022

# Densities
PELVIS_DENSITY        = 5.0
LEG_DENSITY           = 2.0
ARM_DENSITY           = 2.0

# Locomotion & Snappy Responsiveness
MOVE_SPEED            = 7.5    # Normal running speed (m/s)
SPRINT_SPEED          = 12.5   # Sprint speed with Shift (m/s)
ACCEL_RATE            = 32.0   # Snappy instant direction response (m/s^2)
JUMP_VELOCITY         = 10.5   # Upward jump velocity (m/s)

# Stability & Safety Clamps
MAX_LINEAR_VELOCITY   = 24.0   # Prevents collision impulse explosions / tunneling
MAX_ANGULAR_VELOCITY  = 22.0   # Prevents physics glitch spins
MAX_BALANCE           = 255    # Balance points when standing on stable footing

# Combat & Pickups
PUNCH_DURATION        = 0.30   # Exact BombSquad punch duration (s)
PUNCH_IMPULSE         = 22.0   # Normal punch impact force
SPIN_PUNCH_MULT       = 2.4    # Multiplier for 360 Tornado Spin Punch
PICKUP_RADIUS         = 1.55   # Search distance to lift objects / guns
PICKUP_LIFT_TIME      = 0.20   # Time for smooth lift from floor to overhead (s)
THROW_VELOCITY        = 13.5   # Forward launch speed for thrown objects
THROW_UP_VELOCITY     = 5.2    # Upward arc for thrown objects

# Camera (Over-the-shoulder TPS free look)
CAM_DISTANCE          = 7.0
CAM_SENSITIVITY       = 0.16
