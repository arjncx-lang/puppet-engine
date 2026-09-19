# physics_constants.py
# Pure physics, character tuning, and safety clamp constants

# World gravity
GRAVITY               = -20.0

# Anatomical Puppet Dimensions
HEAD_RADIUS           = 0.22
TORSO_RADIUS          = 0.15
TORSO_HEIGHT          = 0.32
PELVIS_RADIUS         = 0.12
PELVIS_HEIGHT         = 0.16

# Limbs & Articulated 2-Bone Geometry
ARM_RADIUS            = 0.065
ARM_LENGTH            = 0.32
ARM_L1                = 0.20
ARM_L2                = 0.18
LEG_RADIUS            = 0.080
LEG_LENGTH            = 0.36
THIGH_LENGTH          = 0.19
SHIN_LENGTH           = 0.17
KNEE_RADIUS           = 0.076
HAND_RADIUS           = 0.075
FOOT_RADIUS           = 0.085

# Bullet Physics Collision Groups (Filter Algorithm: groups-mask)
COLLISION_GROUP_GROUND   = 0
COLLISION_GROUP_CHAR     = 1
COLLISION_GROUP_PROP     = 2
COLLISION_GROUP_WEAPON   = 3
COLLISION_GROUP_GRENADE  = 4
COLLISION_GROUP_HELD     = 5
COLLISION_GROUP_DEBRIS   = 6

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
MAX_KINETIC_ENERGY    = 2800.0 # Prevents singularity energy explosions

# Exact 2nd-Order Spring-Damper Oscillator Frequencies (rad/s) and Damping Ratios
SPRING_OMEGA_TORSO    = 18.0   # Torso orientation response speed
SPRING_ZETA_TORSO     = 0.95   # Subtle organic bounce (0.95 ~ critical)
SPRING_OMEGA_HEAD     = 22.0   # Head gaze tracking speed
SPRING_ZETA_HEAD      = 1.00   # Critically damped (zero overshoot)
SPRING_OMEGA_WEAPON   = 24.0   # Weapon sway and recoil settle speed
SPRING_ZETA_WEAPON    = 0.88   # Snappy organic recoil recovery
SPRING_OMEGA_CAM      = 14.0   # Camera spring-arm tracking
SPRING_ZETA_CAM       = 1.00   # Critically damped camera follow

# Biomechanical Inverted Pendulum Lean & Bank Limits
BANK_MAX_DEG          = 25.0   # Max inward roll angle on cornering
PITCH_MAX_DEG         = 20.0   # Max longitudinal pitch angle under acceleration/braking

# Virtual Pneumatic Ground Suspension (Critically Damped 2nd-Order Harmonic Oscillator)
SUSPENSION_REST_DIST  = 0.48   # Target ride height above ground
SUSPENSION_K          = 480.0  # Spring stiffness (N/m)
SUSPENSION_C          = 96.0   # Damping coefficient (N*s/m, zeta ~ 0.90 critically damped)
MAX_WALKABLE_SLOPE    = 48.0   # Maximum walkable slope in degrees

# Cycloidal Gait Kinematics (Zero Foot Slip)
GAIT_STRIDE_BASE      = 0.65   # Neutral stride length (m)
GAIT_STRIDE_SPRINT    = 0.95   # Sprint stride length (m)
GAIT_STEP_HEIGHT      = 0.12   # Peak foot lift during swing phase (m)

# Aerodynamic Drag & Rotational Damping
AERO_DRAG_COEFF       = 0.040  # Quadratic air resistance on thrown props
ROT_DRAG_COEFF        = 0.050  # Rotational air resistance on tumbling props

# Camera (Over-the-shoulder TPS free look)
CAM_DISTANCE          = 7.0
CAM_SENSITIVITY       = 0.16
CAM_FOV_BASE          = 60.0   # Normal field of view (deg)
CAM_FOV_SPRINT        = 72.0   # High-speed sprint field of view (deg)
CAM_COLLISION_MARGIN  = 0.25   # Distance buffer when camera hits walls

# Lissajous Weapon Sway
SWAY_FREQ             = 1.6    # Breathing frequency (rad/s)
SWAY_AMP_X            = 0.012  # Horizontal breathing drift (m)
SWAY_AMP_Z            = 0.008  # Vertical breathing drift (m)

# Combat & Pickups
PUNCH_DURATION        = 0.30   # 3-Phase spring punch duration (s)
PUNCH_IMPULSE         = 22.0   # Normal punch impact force
SPIN_PUNCH_MULT       = 2.4    # Multiplier for 360 Tornado Spin Punch
PICKUP_RADIUS         = 1.55   # Search distance to lift objects / guns
PICKUP_LIFT_TIME      = 0.20   # Time for smooth lift from floor to overhead (s)
THROW_VELOCITY        = 13.5   # Forward launch speed for thrown objects
THROW_UP_VELOCITY     = 5.2    # Upward arc for thrown objects
HOLD_CLEARANCE_FWD    = 0.50   # Safe forward distance for held objects to prevent clipping
HOLD_CLEARANCE_Z      = 0.44   # Safe height for carried objects
THROW_GRACE_TIME      = 0.30   # Duration to ignore player-prop collision upon release (s)

# Memory Efficiency & Particle/VFX Pool Caps
MAX_ACTIVE_CASINGS    = 24     # Maximum simultaneous physical spent shell casings
MAX_ACTIVE_SPARKS     = 64     # Pre-allocated pooled kinetic spark nodes
MAX_ACTIVE_DUST       = 24     # Pre-allocated pooled dust puff sphere nodes
MAX_ACTIVE_TRACERS    = 16     # Pre-allocated pooled bullet tracer line nodes
MAX_ACTIVE_FLASHES    = 8      # Pre-allocated pooled muzzle flash sphere nodes
MAX_ACTIVE_FIREBALLS  = 6      # Pre-allocated pooled explosion fireball nodes

# Bullet Physics Solver & Sub-stepping Configuration
PHYSICS_SUBSTEPS      = 8      # Maximum internal simulation substeps per frame
PHYSICS_FIXED_DT      = 1.0 / 120.0  # Internal 120 Hz deterministic physics tick (s)
PHYSICS_MAX_DT        = 0.0333 # Maximum allowed external delta time clamp (s)
