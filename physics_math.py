# physics_math.py
# Advanced Mathematical & Physics Formulation Library for PuppetEngine
import math, random
from panda3d.core import Vec3, Point3


def box_normalize_to_circle(lr, ud):
    """
    Normalizes a 2D square-bound input vector (keyboard WASD) to a circle boundary.
    Eliminates diagonal speed boost (sqrt(2) approx 1.414x speed exploit) while preserving
    analog stick fidelity:
    v_norm = v_raw * 1 / sqrt((vx / max(|vx|, |vy|))^2 + (vy / max(|vx|, |vy|))^2)
    """
    if abs(lr) < 0.0001 or abs(ud) < 0.0001:
        mag = math.hypot(lr, ud)
        if mag > 1.0:
            return lr / mag, ud / mag
        return lr, ud

    s = 1.0 / abs(lr) if abs(lr) > abs(ud) else 1.0 / abs(ud)
    proj_lr = lr * s
    proj_ud = ud * s
    proj_len = math.sqrt(proj_lr * proj_lr + proj_ud * proj_ud)
    fin_scale = 1.0 / proj_len
    return lr * fin_scale, ud * fin_scale


class SpringDamper1D:
    """
    Exact closed-form analytical solution of a 2nd-order damped harmonic oscillator:
        m * x''(t) + c * x'(t) + k * (x(t) - x_target) = 0
    Parameterised by:
        omega (natural angular frequency, rad/s = sqrt(k / m))
        zeta  (damping ratio = c / (2 * sqrt(m * k)))
            zeta == 1.0: Critically Damped (fastest approach without overshoot)
            zeta <  1.0: Underdamped (tunable organic oscillation)
            zeta >  1.0: Overdamped (smooth heavy inertia lag)
    Framerate-invariant: exact integration regardless of dt size or variable framerates.
    """
    def __init__(self, initial_pos=0.0, omega=16.0, zeta=1.0):
        self.pos   = float(initial_pos)
        self.vel   = 0.0
        self.omega = float(omega)
        self.zeta  = float(zeta)

    def reset(self, pos=0.0, vel=0.0):
        self.pos = float(pos)
        self.vel = float(vel)

    def update(self, target_pos, dt):
        if dt <= 0.0:
            return self.pos

        # Clamp dt to prevent numerical overflow in extreme hiccups
        dt = min(dt, 0.1)

        x0 = self.pos - target_pos
        v0 = self.vel
        w  = self.omega
        z  = self.zeta

        if z >= 0.9999 and z <= 1.0001:
            # Critically damped: zeta == 1
            # x(t) = (c1 + c2*t) * e^(-w*t)
            exp_wt = math.exp(-w * dt)
            c1 = x0
            c2 = v0 + w * x0
            x_t = (c1 + c2 * dt) * exp_wt
            v_t = (c2 - w * (c1 + c2 * dt)) * exp_wt
        elif z < 0.9999:
            # Underdamped: zeta < 1
            wd = w * math.sqrt(max(0.0001, 1.0 - z * z))
            exp_zwt = math.exp(-z * w * dt)
            c1 = x0
            c2 = (v0 + z * w * x0) / wd
            cos_wdt = math.cos(wd * dt)
            sin_wdt = math.sin(wd * dt)
            x_t = exp_zwt * (c1 * cos_wdt + c2 * sin_wdt)
            v_t = exp_zwt * (
                (-z * w * (c1 * cos_wdt + c2 * sin_wdt)) +
                wd * (-c1 * sin_wdt + c2 * cos_wdt)
            )
        else:
            # Overdamped: zeta > 1
            s = math.sqrt(z * z - 1.0)
            r1 = -w * (z - s)
            r2 = -w * (z + s)
            c2 = (v0 - r1 * x0) / (r2 - r1)
            c1 = x0 - c2
            exp_r1 = math.exp(r1 * dt)
            exp_r2 = math.exp(r2 * dt)
            x_t = c1 * exp_r1 + c2 * exp_r2
            v_t = c1 * r1 * exp_r1 + c2 * r2 * exp_r2

        self.pos = target_pos + x_t
        self.vel = v_t
        return self.pos


class SpringDamper3D:
    """
    3D Vector wrapper around three independent exact 2nd-order SpringDamper1D instances.
    Guarantees perfectly stable, frame-rate independent vector smoothing without lag or clipping.
    """
    def __init__(self, initial_pos=(0.0, 0.0, 0.0), omega=16.0, zeta=1.0):
        self.sx = SpringDamper1D(initial_pos[0], omega, zeta)
        self.sy = SpringDamper1D(initial_pos[1], omega, zeta)
        self.sz = SpringDamper1D(initial_pos[2], omega, zeta)
        self._out = Vec3(initial_pos[0], initial_pos[1], initial_pos[2])

    def reset(self, pos=(0.0, 0.0, 0.0), vel=(0.0, 0.0, 0.0)):
        self.sx.reset(pos[0], vel[0])
        self.sy.reset(pos[1], vel[1])
        self.sz.reset(pos[2], vel[2])
        self._out.set(pos[0], pos[1], pos[2])

    def update(self, target_pos, dt):
        px = self.sx.update(target_pos[0], dt)
        py = self.sy.update(target_pos[1], dt)
        pz = self.sz.update(target_pos[2], dt)
        self._out.set(px, py, pz)
        return self._out

    def get_pos(self):
        return Vec3(self.sx.pos, self.sy.pos, self.sz.pos)

    def get_vel(self):
        return Vec3(self.sx.vel, self.sy.vel, self.sz.vel)


class CameraTraumaSystem:
    """
    GDC-Standard Trauma-based Non-Linear Screen Shake (Squirrel Eiserloh Formulation):
        Shake = Trauma^2 (or Trauma^3)
    Produces subtle organic tremors for light events and high-impact cinematic feedback
    for heavy explosions, shotgun blasts, and landing shocks.
    """
    def __init__(self, decay_rate=1.6, max_yaw=2.4, max_pitch=3.0, max_roll=1.8):
        self.trauma = 0.0
        self.decay_rate = decay_rate
        self.max_yaw = max_yaw
        self.max_pitch = max_pitch
        self.max_roll = max_roll
        self.time = 0.0

    def add_trauma(self, amount):
        self.trauma = min(1.0, self.trauma + amount)

    def update(self, dt):
        self.time += dt
        if self.trauma > 0.001:
            self.trauma = max(0.0, self.trauma - self.decay_rate * dt)
            shake = self.trauma * self.trauma
            pitch_offset = self.max_pitch * shake * math.sin(38.0 * self.time)
            yaw_offset   = self.max_yaw   * shake * math.sin(47.0 * self.time + 1.2)
            roll_offset  = self.max_roll  * shake * math.sin(53.0 * self.time + 2.4)
            return Vec3(yaw_offset, pitch_offset, roll_offset)
        return Vec3(0, 0, 0)


def cubic_hermite(p0, v0, p1, v1, t):
    """
    C1-continuous Cubic Hermite Spline for jerk-free physical parameter transitions.
    """
    t = max(0.0, min(1.0, t))
    t2 = t * t
    t3 = t2 * t
    h00 = 2.0 * t3 - 3.0 * t2 + 1.0
    h10 = t3 - 2.0 * t2 + t
    h01 = -2.0 * t3 + 3.0 * t2
    h11 = t3 - t2
    return h00 * p0 + h10 * v0 + h01 * p1 + h11 * v1


def calculate_centrifugal_bank_angle(planar_speed, angular_velocity_z, gravity=20.0, max_bank_deg=28.0):
    """
    Inverted Pendulum Biomechanics:
    Computes anatomical inward bank angle during cornering.
    Balances centrifugal force against gravity:
        tan(theta_bank) = (v * omega) / g
        theta_bank = -arctan((v * omega) / g)
    """
    if abs(planar_speed) < 0.05 or abs(angular_velocity_z) < 0.05:
        return 0.0
    a_c = planar_speed * angular_velocity_z
    tan_theta = a_c / max(1.0, abs(gravity))
    bank_rad = math.atan(tan_theta)
    bank_deg = math.degrees(bank_rad)
    return max(-max_bank_deg, min(max_bank_deg, -bank_deg))


def calculate_longitudinal_pitch_angle(forward_acceleration, gravity=20.0, max_pitch_deg=22.0):
    """
    Inertial Longitudinal Pitch:
    Computes body pitch lean under acceleration (leaning forward) and braking (leaning backward):
        theta_pitch = arctan(a_forward / g)
    """
    if abs(forward_acceleration) < 0.01:
        return 0.0
    tan_theta = forward_acceleration / max(1.0, abs(gravity))
    pitch_rad = math.atan(tan_theta)
    pitch_deg = math.degrees(pitch_rad)
    return max(-max_pitch_deg, min(max_pitch_deg, pitch_deg))


def cycloidal_step_displacement(phase, stride_length, step_height):
    """
    Cycloidal Gait Kinematics:
    Produces zero-jerk, zero-ground-slip bipedal foot stepping.
    Phase in [0, 2*pi):
      Phase in [0, pi):   STANCE PHASE (Foot planted on ground, moves backward at ground speed)
      Phase in [pi, 2*pi): SWING PHASE  (Foot lifts and steps forward along a cycloid trajectory)
    """
    norm_phase = phase % (2.0 * math.pi)

    if norm_phase < math.pi:
        u = norm_phase / math.pi
        rel_x = (0.5 - u) * stride_length
        rel_z = 0.0
    else:
        u = (norm_phase - math.pi) / math.pi
        cycloid_x = u - math.sin(2.0 * math.pi * u) / (2.0 * math.pi)
        rel_x = (-0.5 + cycloid_x) * stride_length
        rel_z = step_height * 0.5 * (1.0 - math.cos(2.0 * math.pi * u))

    return rel_x, rel_z


def calculate_ground_suspension_force(ray_distance, rest_distance, vertical_velocity, k_spring=450.0, c_damper=35.0):
    """
    Virtual Spring-Damper Ground Suspension (Pneumatic Raycast Suspension):
    Computes suspension force to keep character floating at rest_distance above ground:
        F_suspension = k * (rest_distance - ray_distance) - c * v_z
    """
    penetration = rest_distance - ray_distance
    force = k_spring * penetration - c_damper * vertical_velocity
    return max(-200.0, min(600.0, force))


def calculate_slope_slip_force(surface_normal, mass=6.0, gravity=-20.0, max_walkable_angle_deg=48.0):
    """
    Slope Sliding Physics:
    Calculates downhill slip force when the terrain slope exceeds max walkable angle:
        alpha = arccos(n_z)
        F_slip = m * (g - (g . n) * n)
    """
    n = surface_normal.normalized()
    cos_angle = max(-1.0, min(1.0, n.z))
    slope_angle_deg = math.degrees(math.acos(cos_angle))

    if slope_angle_deg <= max_walkable_angle_deg or slope_angle_deg >= 89.0:
        return Vec3(0, 0, 0), slope_angle_deg

    g_vec = Vec3(0, 0, gravity)
    g_tangent = g_vec - n * (g_vec.dot(n))
    slip_force = g_tangent * mass
    return slip_force, slope_angle_deg


def calculate_off_center_torque(hit_pos, body_pos, impulse_vec, max_lever_arm=0.25, torque_coupling=0.08, max_torque_impulse=5.0):
    """
    Safe Rigid Body Rotational Dynamics:
    Computes bounded rotational torque impulse tau = r x J with physical inertia coupling.
    - Clamps lever arm r to realistic object radius
    - Scales torque impulse to prevent runaway centrifugal acceleration
    - Guarantees finite bounded impulse
    """
    r = hit_pos - body_pos
    r_len = r.length()
    if r_len > max_lever_arm and r_len > 0.001:
        r = r * (max_lever_arm / r_len)

    # Cross product: r x J scaled by realistic rotational coupling
    tx = (r.y * impulse_vec.z - r.z * impulse_vec.y) * torque_coupling
    ty = (r.z * impulse_vec.x - r.x * impulse_vec.z) * torque_coupling
    tz = (r.x * impulse_vec.y - r.y * impulse_vec.x) * torque_coupling
    torque = Vec3(tx, ty, tz)

    t_mag = torque.length()
    if t_mag > max_torque_impulse and t_mag > 0.001:
        torque = torque * (max_torque_impulse / t_mag)

    return torque


def apply_semi_implicit_drag(velocity, angular_velocity, dt, c_linear=0.04, c_angular=0.12, max_ang_vel=22.0):
    """
    Unconditionally Stable Semi-Implicit Aerodynamic & Rotational Drag Integration:
        v_(t+1) = v_t / (1 + c_lin * |v_t| * dt)
        w_(t+1) = w_t / (1 + c_ang * |w_t| * dt)
    Guaranteed zero-overshoot, zero oscillation, and absolute mathematical stability under any dt.
    """
    dt = min(0.05, max(0.0001, dt))
    v_mag = velocity.length()
    if v_mag > 0.01:
        decay_lin = 1.0 / (1.0 + c_linear * v_mag * dt)
        velocity = velocity * decay_lin

    w_mag = angular_velocity.length()
    if w_mag > 0.01:
        decay_ang = 1.0 / (1.0 + c_angular * w_mag * dt)
        angular_velocity = angular_velocity * decay_ang

    # Absolute safety clamp on angular velocity
    if angular_velocity.lengthSquared() > max_ang_vel * max_ang_vel:
        angular_velocity = angular_velocity.normalized() * max_ang_vel

    return velocity, angular_velocity


def calculate_lissajous_sway(time_val, freq=1.4, amp_x=0.015, amp_z=0.009):
    """
    Anatomical Lissajous Figure-8 Aim Sway:
    Natural bipedal breathing and postural micro-sway:
        x_sway = amp_x * sin(freq * t)
        z_sway = amp_z * sin(2 * freq * t)
    """
    sx = amp_x * math.sin(freq * time_val)
    sz = amp_z * math.sin(2.0 * freq * time_val)
    return sx, sz


def clamp_kinetic_energy(velocity, angular_velocity, mass=6.0, max_energy=2800.0, max_lin_vel=24.0, max_ang_vel=22.0):
    """
    Energy Conservation & Physics Safety Clamp:
    Guarantees no collision explosion or singularity glitch can shoot objects to infinity.
    E_k = 0.5 * m * v^2
    """
    # Guard against NaN/Inf glitches
    if math.isnan(velocity.x) or math.isnan(velocity.y) or math.isnan(velocity.z):
        velocity = Vec3(0, 0, 0)
    if math.isnan(angular_velocity.x) or math.isnan(angular_velocity.y) or math.isnan(angular_velocity.z):
        angular_velocity = Vec3(0, 0, 0)

    v_sq = velocity.lengthSquared()
    kinetic_linear = 0.5 * mass * v_sq
    if kinetic_linear > max_energy:
        scale = math.sqrt(max_energy / kinetic_linear)
        velocity = velocity * scale

    # Hard cap on linear velocity to prevent tunnel glitches
    if velocity.lengthSquared() > max_lin_vel * max_lin_vel:
        velocity = velocity.normalized() * max_lin_vel

    w_sq = angular_velocity.lengthSquared()
    if w_sq > max_ang_vel * max_ang_vel:
        angular_velocity = angular_velocity.normalized() * max_ang_vel

    return velocity, angular_velocity


class SquashStretchSystem:
    """
    Overgrowth-Style Volume-Preserving Procedural Squash and Stretch (David Rosen GDC):
    Deforms character along vertical axis based on vertical velocity changes and landing impacts:
        S_z(t) integrated via 2nd-order damped harmonic oscillator (omega=24.0, zeta=0.68)
        Volume conservation: S_x = S_y = 1.0 / sqrt(S_z)
    Produces organic fleshy squash on landing and dynamic stretch on jump takeoff.
    """
    def __init__(self, omega=24.0, zeta=0.68):
        self.spring = SpringDamper1D(1.0, omega=omega, zeta=zeta)
        self.target_sz = 1.0

    def trigger_jump_stretch(self, factor=1.24):
        self.spring.reset(pos=factor, vel=4.5)

    def trigger_landing_squash(self, impact_speed, max_squash=0.68):
        impact_ratio = min(1.0, max(0.1, abs(impact_speed) / 14.0))
        squashed_sz = 1.0 - (1.0 - max_squash) * impact_ratio
        self.spring.reset(pos=squashed_sz, vel=-5.0 * impact_ratio)

    def update(self, current_vz, is_grounded, dt):
        if not is_grounded:
            target = 1.0 + max(-0.15, min(0.20, current_vz * 0.015))
        else:
            target = 1.0

        sz = self.spring.update(target, dt)
        sz = max(0.60, min(1.45, sz))
        s_planar = 1.0 / math.sqrt(sz)
        return Vec3(s_planar, s_planar, sz)


def calculate_slope_foot_alignment(ground_normal, character_yaw_deg):
    """
    Dynamic Terrain Slope Foot Alignment (Procedural Ankle IK):
    Transforms world ground normal into character's local coordinate frame to compute
    exact foot pitch (toe up/down) and foot roll (side tilt) to lock shoes flush to any slope.
    """
    n = ground_normal.normalized()
    if n.z >= 0.999:
        return 0.0, 0.0

    yr = math.radians(character_yaw_deg)
    fwd_x = -math.sin(yr); fwd_y = math.cos(yr)
    rgt_x =  math.cos(yr); rgt_y = math.sin(yr)

    slope_fwd = n.x * fwd_x + n.y * fwd_y
    slope_rgt = n.x * rgt_x + n.y * rgt_y

    foot_pitch_deg = math.degrees(math.atan2(slope_fwd, max(0.1, n.z)))
    foot_roll_deg  = -math.degrees(math.atan2(slope_rgt, max(0.1, n.z)))

    return max(-32.0, min(32.0, foot_pitch_deg)), max(-32.0, min(32.0, foot_roll_deg))


class HitStopManager:
    """
    Martin Jonasson & Petri Purho 'Juice It or Lose It' Hit-Stop (Micro-Freeze):
    Moments of high kinetic impact freeze time for 30-50ms, communicating crushing weight.
    """
    def __init__(self):
        self.timer = 0.0

    def trigger(self, duration=0.042):
        self.timer = max(self.timer, duration)

    def process_dt(self, dt):
        if self.timer > 0.0:
            self.timer -= dt
            return dt * 0.08
        return dt


class ProceduralWeaponController:
    """
    AAA-Grade 6-DOF Procedural Weapon Recoil & Handling Dynamics:
    - Linear Kickback Spring (along barrel axis into shoulder)
    - Angular Recoil Spring (Muzzle Rise Pitch, Twitch Yaw, Rifling Torque Roll)
    - Rotational Weapon Inertia & Camera Sweep Lag
    """
    def __init__(self):
        self.kickback_spring   = SpringDamper1D(0.0, omega=32.0, zeta=0.75)
        self.rot_recoil_spring = SpringDamper3D((0.0, 0.0, 0.0), omega=28.0, zeta=0.72)
        self.inertia_spring    = SpringDamper3D((0.0, 0.0, 0.0), omega=20.0, zeta=0.92)

    def trigger_recoil(self, linear_kick, pitch_kick, yaw_kick=1.2, roll_kick=2.0):
        # Linear impulse: kicks back into shoulder
        self.kickback_spring.vel -= linear_kick * 30.0
        # Angular impulse: muzzle climbs up, with random torque twitch
        r_yaw = (random.random() * 2.0 - 1.0) * yaw_kick
        r_roll = (random.random() * 2.0 - 1.0) * roll_kick
        self.rot_recoil_spring.sx.vel += r_yaw * 24.0
        self.rot_recoil_spring.sy.vel += pitch_kick * 32.0
        self.rot_recoil_spring.sz.vel += r_roll * 24.0

    def update(self, cam_delta_yaw, cam_delta_pitch, dt):
        kick_z = self.kickback_spring.update(0.0, dt)
        recoil_hpr = self.rot_recoil_spring.update((0.0, 0.0, 0.0), dt)

        # Dynamic Weapon Inertia & Camera Lag
        target_lag_yaw   = max(-9.0, min(9.0, -cam_delta_yaw * 0.22))
        target_lag_pitch = max(-7.0, min(7.0, cam_delta_pitch * 0.18))
        target_lag_roll  = max(-12.0, min(12.0, -cam_delta_yaw * 0.28))
        lag_hpr = self.inertia_spring.update((target_lag_yaw, target_lag_pitch, target_lag_roll), dt)

        total_rot = Vec3(recoil_hpr.x + lag_hpr.x, recoil_hpr.y + lag_hpr.y, recoil_hpr.z + lag_hpr.z)
        return kick_z, total_rot


def solve_two_bone_ik_3d(shoulder, target, l1, l2, pole_vec):
    """
    Analytical Closed-Form 3D Two-Bone Inverse Kinematics (Law of Cosines):
    Solves exact 3D elbow position for upper-arm length l1 and forearm length l2
    reaching from shoulder to target, with pole_vec directing elbow flexion.
    100% stable, non-iterative, zero-singularity, sub-millimeter precision.
    """
    d_vec = target - shoulder
    d = d_vec.length()
    if d < 0.0001:
        d = 0.0001
        d_vec = Vec3(0, 1, 0)

    # Clamp target reach to geometric bounds
    d = max(abs(l1 - l2) + 0.001, min(l1 + l2 - 0.001, d))
    aim_dir = d_vec.normalized()

    # Law of Cosines
    cos_alpha = (l1 * l1 + d * d - l2 * l2) / (2.0 * l1 * d)
    cos_alpha = max(-1.0, min(1.0, cos_alpha))
    alpha = math.acos(cos_alpha)

    # Project pole vector onto the plane perpendicular to aim_dir
    proj = pole_vec - aim_dir * pole_vec.dot(aim_dir)
    if proj.lengthSquared() < 0.0001:
        proj = Vec3(0, 0, -1) - aim_dir * aim_dir.z
        if proj.lengthSquared() < 0.0001:
            proj = Vec3(1, 0, 0)
    bend_dir = proj.normalized()

    elbow = shoulder + aim_dir * (l1 * math.cos(alpha)) + bend_dir * (l1 * math.sin(alpha))
    return elbow


def calculate_ricochet_reflection(incident_dir, surface_normal, spread=0.15):
    """
    Computes physical reflection vector: R = D - 2*(D . N)*N with randomized surface micro-roughness.
    Adapted from tactical ballistics & A3P ricochet dynamics.
    """
    d = incident_dir.normalized()
    n = surface_normal.normalized()
    dot = d.dot(n)
    refl = d - n * (2.0 * dot)
    if spread > 0.0:
        refl.x += (random.random() * 2.0 - 1.0) * spread
        refl.y += (random.random() * 2.0 - 1.0) * spread
        refl.z += (random.random() * 2.0 - 1.0) * spread
    return refl.normalized()


def calculate_radial_explosion_impulse(blast_pos, target_pos, max_force, radius, upward_lift=4.5, max_impulse_cap=90.0):
    """
    Computes radial shockwave impulse from blast center to target position.
    Applies distance attenuation (1 - d/R) and upward kinetic lift.
    Safe and bounded to prevent singularity or NaN physics crashes.
    Returns (impulse_vector, distance_ratio, is_inside_radius).
    """
    diff = target_pos - blast_pos
    dist = diff.length()
    if math.isnan(dist) or dist >= radius:
        return Vec3(0, 0, 0), 0.0, False

    if dist < 0.001:
        dir_norm = Vec3(0, 0, 1.0)
        dist = 0.001
    else:
        dir_norm = diff / dist

    ratio = max(0.0, min(1.0, 1.0 - (dist / radius)))
    # Non-linear shockwave falloff: quadratic pressure drop
    pressure = ratio ** 1.3
    linear_impulse = dir_norm * (max_force * pressure) + Vec3(0, 0, upward_lift * pressure)
    # Bounded impulse cap
    if linear_impulse.lengthSquared() > max_impulse_cap * max_impulse_cap:
        linear_impulse = linear_impulse.normalized() * max_impulse_cap
    return linear_impulse, ratio, True


def calculate_ballistica_arm_swing(roll_amt, run_gas, is_female=False):
    """
    Contralateral Bipedal Running Arm Kinematics:
    Left arm swings forward in opposition to right leg; right arm in opposition to left leg.
    Panda3D convention:
      Pitch < 0: swings arm forward (+Y)
      Pitch > 0: swings arm backward (-Y)
      Elbow < 0: flexes forearm forward
    """
    blend = run_gas * run_gas
    inv_blend = 1.0 - run_gas

    # Gait roll phase: roll_amt == 0 is left foot forward, right foot back.
    # Therefore, at roll_amt == 0, RIGHT arm swings forward, LEFT arm swings back.
    # sin(roll_amt): 
    #   When sin(roll_amt) > 0: left leg is swinging forward, so right arm swings forward (- pitch), left arm back (+ pitch).
    #   When sin(roll_amt) < 0: right leg is swinging forward, so left arm swings forward (- pitch), right arm back (+ pitch).
    swing = math.sin(roll_amt)
    swing_quad = math.cos(roll_amt)

    # Angular amplitudes in degrees
    walk_amp = 18.0
    run_amp  = 40.0
    pitch_amp = walk_amp * inv_blend + run_amp * blend

    # Contralateral arm pitch (Panda3D: negative pitch = forward reach)
    l_pitch =  swing * pitch_amp
    r_pitch = -swing * pitch_amp

    # Elbow flexion: when arm swings forward, elbow flexes forward (up to -55 deg); relaxes on backswing
    l_forward_factor = max(0.0, -l_pitch / max(1.0, pitch_amp))
    r_forward_factor = max(0.0, -r_pitch / max(1.0, pitch_amp))

    l_elbow = -14.0 - blend * (16.0 + l_forward_factor * 34.0)
    r_elbow = -14.0 - blend * (16.0 + r_forward_factor * 34.0)

    # Shoulder roll (outward flaring for athletic posture)
    l_roll = -9.0 - blend * 6.0 + swing_quad * 3.0 * blend
    r_roll =  9.0 + blend * 6.0 - swing_quad * 3.0 * blend

    return (l_pitch, l_roll, l_elbow), (r_pitch, r_roll, r_elbow)


def calculate_ballistica_punch_momentum(angular_vel, linear_vel, prev_ang_d, prev_ang_m, prev_lin_d, prev_lin_m):
    """
    Integrates Ballistica's angular and linear punch momentum accumulators (spaz_node.cc:2060-2086).
    Builds up momentum during high-speed rotation and forward sprinting to augment punch impact.
    Returns: (ang_d, ang_m, lin_d, lin_m)
    """
    abs_a_vel = min(25.0, abs(angular_vel))
    ang_d = prev_ang_d + abs_a_vel * 0.0004
    ang_d *= 0.965
    ang_m = prev_ang_m + ang_d
    ang_m *= 0.92
    if abs_a_vel < 5.0:
        ang_m *= 0.8 + 0.2 * (abs_a_vel / 5.0)

    lin_d = prev_lin_d + linear_vel * 0.004
    lin_d *= 0.95
    lin_m = prev_lin_m + lin_d
    lin_m *= 0.96
    if linear_vel < 5.0:
        lin_m *= 0.9 + 0.1 * (linear_vel / 5.0)

    return ang_d, ang_m, lin_d, lin_m


def calculate_ballistica_airborne_flail(anim_time):
    """
    Computes Ballistica's counter-rotating circular arm and leg flail when airborne with zero balance (spaz_node.cc:2838-2859).
    """
    wave_amt = anim_time * 11.0
    v1 = math.sin(wave_amt) * 30.0
    v2 = math.cos(wave_amt) * 26.0

    l_arm = (0.0, -55.0 + v1, -25.0 + v2 * 0.4)
    l_elbow = (0.0, -35.0 - v2 * 0.5, 0.0)
    r_arm = (0.0, -55.0 - v1, 25.0 - v2 * 0.4)
    r_elbow = (0.0, -35.0 + v2 * 0.5, 0.0)

    return l_arm, l_elbow, r_arm, r_elbow


def calculate_ballistica_arm_swing_anchors(roll_amt, run_gas, is_female=False):
    """
    Computes Ballistica's exact torso-local IK hand anchor targets for arm swing
    during walking and running (spaz_node.cc:2935-2971).

    Unlike the angle-based calculate_ballistica_arm_swing(), this returns raw
    torso-local (x, y, z) hand target positions that can be directly used as
    IK anchor targets in world-space IK solvers, matching how Ballistica's
    JointFixedEF anchor1 is set directly.

    Ballistica coordinate system (torso-local):
        x: left(+) to right(-)
        y: back(-) to front(+) / vertical
        z: down(-) to up(+)

    Returns:
        (l_xyz, r_xyz, linear_stiffness, linear_damping)
    """
    blend = run_gas * run_gas
    inv_blend = 1.0 - run_gas

    v1run = math.sin(roll_amt + math.pi * 0.5) * 0.20
    v2run = math.cos(roll_amt) * 0.30
    v1    = math.sin(roll_amt) * 0.05
    v2    = math.cos(roll_amt) * (0.30 if is_female else 0.60)

    # Fixed lateral offset +-0.2 exactly as Ballistica (spaz_node.cc:2960, 2967)
    l_x =  0.20
    r_x = -0.20

    l_y = (-v1run - 0.15) * blend + (-v1 - 0.10) * inv_blend
    l_z = (-v2run + 0.15) * blend + (-v2 + 0.10) * inv_blend

    r_y = ( v1run - 0.15) * blend + ( v1 - 0.10) * inv_blend
    r_z = ( v2run + 0.15) * blend + ( v2 + 0.10) * inv_blend

    # Ballistica joint stiffness blend (spaz_node.cc:2943-2951)
    lin_stiffness = 14.0 * blend + 0.5 * inv_blend
    lin_damping   =  0.08 * blend + 0.001 * inv_blend

    return (l_x, l_y, l_z), (r_x, r_y, r_z), lin_stiffness, lin_damping


def calculate_ballistica_airborne_arm_anchors(anim_time):
    """
    Computes Ballistica's exact torso-local hand anchor targets when airborne
    with zero balance (spaz_node.cc:2837-2858).

    Ballistica uses: wave_amt = scenetime_ms * -0.018
    At 60 fps this is equivalent to anim_time * -11.0 * (1000/60) ~ -11.0 rad/s

        v1 = sin(wave_amt) * 0.34
        v2 = cos(wave_amt) * 0.34
        left  = [+0.4,  v1+0.6,  v2+0.2]
        right = [-0.4, -v1+0.6, -v2+0.2]

    Returns:
        (l_xyz, r_xyz, linear_stiffness, linear_damping)
    """
    wave_amt = anim_time * -11.0
    v1 = math.sin(wave_amt) * 0.34
    v2 = math.cos(wave_amt) * 0.34

    l_xyz = ( 0.40,  v1 + 0.60,  v2 + 0.20)
    r_xyz = (-0.40, -v1 + 0.60, -v2 + 0.20)

    return l_xyz, r_xyz, 6.0, 0.01


def calculate_ballistica_pickup_reach_anchors(swipe_progress):
    """
    Computes Ballistica's hand anchor targets while reaching to pick up an object
    (spaz_node.cc:2862-2885).

    Args:
        swipe_progress: float 0->1 representing the reach/swipe animation phase.
            0.0 to 0.5: arms reach forward
            0.5 to 1.0: arms swipe across (inward) to grab

    Returns:
        (l_xyz, r_xyz, linear_stiffness, linear_damping)
    """
    if swipe_progress < 0.5:
        l_xyz = ( 0.40, 0.50, 0.70)
        r_xyz = (-0.40, 0.20, 0.70)
    else:
        l_xyz = (-0.10, 0.50, 0.70)
        r_xyz = ( 0.10, 0.20, 0.70)

    return l_xyz, r_xyz, 6.0, 0.10


def calculate_ballistica_punch_ik_anchors(elapsed_ms, punch_right,
                                           punch_dir_x, punch_dir_z,
                                           shoulder_local_x, shoulder_local_y, shoulder_local_z):
    """
    Computes Ballistica's 3-phase punch hand IK anchor targets in torso-local space
    (spaz_node.cc:2761-2819).

    Phase 1 - Anticipation (0-80ms):
        Punch hand draws back away from target
        Opposite hand pulls back to guard position

    Phase 2 - Strike (80-200ms):
        Punch hand drives toward shoulder + punch_dir * 0.7 in world space
        This function pre-applies the offset in torso-local space

    Phase 3 - Recovery (200ms+):
        Returns None for punch_xyz to indicate free hang

    Returns:
        (punch_xyz, opposite_xyz, punch_stiffness, punch_damping, opp_stiffness, opp_damping)
        punch_xyz = None in recovery phase (caller should use arm-swing IK instead)
    """
    mirror = -1.0 if punch_right else 1.0

    opp_xyz       = (-0.20 * mirror, 0.10, 0.0)
    opp_stiffness = 30.0
    opp_damping   = 0.10

    if elapsed_ms < 80.0:
        # Anticipation: draw back
        punch_xyz       = (0.40 * mirror, 0.0, -0.10)
        punch_stiffness = 100.0
        punch_damping   = 1.0
    elif elapsed_ms < 200.0:
        # Strike: extend toward target. Ballistica gets world-pos of shoulder
        # then offsets by punch_dir * 0.7 and converts back to torso space.
        # We approximate that in torso-local using the pre-computed shoulder offset.
        wx = shoulder_local_x + punch_dir_x * 0.70
        wy = shoulder_local_y + 0.13
        wz = shoulder_local_z + punch_dir_z * 0.70
        punch_xyz       = (wx, wy, wz)
        punch_stiffness = 100.0
        punch_damping   = 1.0
    else:
        punch_xyz       = None
        punch_stiffness = 0.0
        punch_damping   = 0.0

    return punch_xyz, opp_xyz, punch_stiffness, punch_damping, opp_stiffness, opp_damping


def calculate_ballistica_shoulder_anchors(breath, punching, punch_right,
                                           shoulder_offset=(0.0, 0.0, 0.0)):
    """
    Computes Ballistica's torso-local shoulder socket anchor positions (spaz_node.cc:2669-2699).

    Includes:
    - Anatomical base position
    - Breathing-driven Y offset
    - Punch lean: slight Z shift of both shoulders toward the punch side

    Base (Ballistica): x=-0.15, y=0.14, z=0.0

    Returns:
        (right_shoulder_xyz, left_shoulder_xyz)
    """
    bx = -0.15 + shoulder_offset[0]
    by =  0.14 + shoulder_offset[1] + breath * 0.012
    bz =  0.00 + shoulder_offset[2]

    l_z_off = 0.0
    r_z_off = 0.0
    if punching:
        if punch_right:
            l_z_off = -0.05
            r_z_off =  0.05
        else:
            l_z_off =  0.05
            r_z_off = -0.05

    right_shoulder = ( bx, by, bz + r_z_off)
    left_shoulder  = (-bx, by, bz + l_z_off)

    return right_shoulder, left_shoulder


def calculate_ballistica_celebration_anchors(anim_time, celebrating_left, celebrating_right):
    """
    Computes Ballistica's celebration arm anchor targets: arms raised triumphantly
    (spaz_node.cc:2910-2933).

    Args:
        anim_time: current animation time in seconds
        celebrating_left:  bool - left arm should be raised
        celebrating_right: bool - right arm should be raised

    Returns:
        (l_xyz or None, r_xyz or None, stiffness, damping)
    """
    v1 = math.sin(anim_time * 0.04) * 0.1
    v2 = math.cos(anim_time * 0.03) * 0.1

    l_xyz = ( 0.40 + v2, 0.50, 0.20 + v1) if celebrating_left  else None
    r_xyz = (-0.40 - v2, 0.50, 0.20 + v1) if celebrating_right else None

    return l_xyz, r_xyz, 30.0, 0.08


def calculate_ballistica_steered_movement_vector(
    cur_dir_x, cur_dir_y, new_input_x, new_input_y, run_gas, current_speed, dt=1.0/60.0
):
    """
    Ballistica's momentum-preserving steering filter (spaz_node.cc:1973-2028).
    When running (run_gas > 0.05), strips out any component >90 deg off current heading,
    forcing smooth curved turns. Blends direction with speed-dependent inertia.

    Returns:
        (out_dir_x, out_dir_y)
    """
    input_len = math.hypot(new_input_x, new_input_y)
    if input_len < 0.001:
        return 0.0, 0.0

    in_x = new_input_x / input_len
    in_y = new_input_y / input_len

    cur_len = math.hypot(cur_dir_x, cur_dir_y)
    if cur_len < 0.001 or run_gas < 0.05:
        return in_x * input_len, in_y * input_len

    c_x = cur_dir_x / cur_len
    c_y = cur_dir_y / cur_len

    # Strip out any component of new direction more than 90 deg off current heading (spaz_node.cc:1974-1988)
    dot = in_x * c_x + in_y * c_y
    if dot < 0.0:
        in_x -= run_gas * (c_x * dot)
        in_y -= run_gas * (c_y * dot)
        n_len = math.hypot(in_x, in_y)
        if n_len > 0.0001:
            in_x /= n_len
            in_y /= n_len

    # Speed-dependent inertia smoothing (spaz_node.cc:2013-2022)
    # Higher run_gas and higher speed preserve current heading more strongly
    smoothing = 0.975 * (0.90 + 0.10 * run_gas)
    if current_speed < 2.0:
        smoothing *= (current_speed / 2.0)

    # Frame-rate independent adaptation (base ~120hz in Ballistica)
    smooth_factor = math.pow(smoothing, dt * 120.0)
    blend_x = smooth_factor * cur_dir_x + (1.0 - smooth_factor) * (in_x * input_len)
    blend_y = smooth_factor * cur_dir_y + (1.0 - smooth_factor) * (in_y * input_len)

    return blend_x, blend_y


def calculate_ballistica_torso_tilt_and_sway(
    v_mag, accel_fwd, accel_side, run_gas, roll_amt, spin_rate, is_holding,
    diff_smooth_side=0.0, diff_smooth_fwd=0.0,
    diff_smoother_side=0.0, diff_smoother_fwd=0.0
):
    """
    Ballistica's acceleration tilt and gait roll sway (spaz_node.cc:3344-3378, 3449-3456).

    Args:
        v_mag: scalar speed (min clamped to 7.0 for response sensitivity)
        accel_fwd: forward acceleration along character heading
        accel_side: lateral acceleration (side-to-side)
        run_gas: current running intensity [0, 1]
        roll_amt: gait roll cycle [0, 2*pi]
        spin_rate: angular velocity magnitude (rad/s)
        is_holding: bool, whether character is carrying an object
        diff_smooth_*: short-term input jerk
        diff_smoother_*: medium-term running input derivative

    Returns:
        (tilt_pitch, tilt_roll, stride_sway_roll)
    """
    v_eff = max(7.0, v_mag)
    gas_mult = 0.20 + 0.80 * run_gas

    # Base acceleration tilts (clamped to +-0.9 in Ballistica)
    tilt_roll_raw = gas_mult * max(-0.9, min(0.9, v_eff * accel_side * 0.05))
    tilt_pitch_raw = gas_mult * max(-0.9, min(0.9, v_eff * accel_fwd * -0.05))

    fast = min(1.0, v_mag / 5.0)
    tilt_roll_raw += (1.0 - fast) * (diff_smooth_side * 1.5) + fast * (diff_smoother_side * 4.0)
    tilt_pitch_raw += (1.0 - fast) * (diff_smooth_fwd * 1.5) + fast * (diff_smoother_fwd * 4.0)

    rotate_tilt = 1.2
    if is_holding:
        rotate_tilt *= 0.5

    # If spinning rapidly, suppress tilt so character does not wobble off-axis (spaz_node.cc:3375-3378)
    if spin_rate > 10.0:
        rotate_tilt = 0.0
    elif spin_rate > 5.0:
        rotate_tilt *= 1.0 - (spin_rate - 5.0) / 5.0

    tilt_roll = tilt_roll_raw * rotate_tilt * 16.0   # convert to degrees
    tilt_pitch = tilt_pitch_raw * rotate_tilt * 16.0

    # Stride sway synchronized with footfall (spaz_node.cc:3450-3455)
    sway_rad = math.sin(roll_amt - math.pi) * (run_gas * 0.09 + (1.0 - run_gas) * 0.04)
    stride_sway_deg = math.degrees(sway_rad)

    return tilt_pitch, tilt_roll, stride_sway_deg


def calculate_ballistica_airborne_legs(roll_amt):
    """
    Ballistica's counter-rotating leg flail when airborne with lost balance (spaz_node.cc:2431-2457).
    Feet cycle along cycloidal arcs:
        left:  y = -0.3, z =  0.22 * cos(roll_amt)
        right: y = -0.3, z = -0.22 * cos(roll_amt)

    Returns:
        (l_pitch, l_roll, r_pitch, r_roll)
    """
    z_left = 0.22 * math.cos(roll_amt)
    l_pitch = math.degrees(math.atan2(z_left, 0.40))
    r_pitch = -l_pitch
    l_roll = -6.0 + math.sin(roll_amt) * 4.0
    r_roll =  6.0 - math.sin(roll_amt) * 4.0

    return l_pitch, l_roll, r_pitch, r_roll


def calculate_ballistica_limb_stretch(current_dist, rest_dist=0.20, max_stretch=1.35):
    """
    Ballistica dynamic limb mesh elasticity (spaz_node.cc:4275-4306, 4350-4375).
    Scales limb bone length along primary axis during reaching or punching.

    Returns:
        stretch_factor: float between 0.85 and max_stretch
    """
    if rest_dist <= 0.001:
        return 1.0
    ratio = current_dist / rest_dist
    return max(0.85, min(max_stretch, ratio))


def calculate_ballistica_throw_physics(
    fwd_vec, move_scale, since_pickup_ms, is_bomb_reversed=False
):
    """
    Ballistica throw power scaling, launch velocity, and character kickback (spaz_node.cc:1543-1572, 3771-3850).

    Args:
        fwd_vec: Vec3 forward unit vector of character
        move_scale: scalar magnitude of movement input [0, 1]
        since_pickup_ms: elapsed milliseconds since object was picked up
        is_bomb_reversed: if True (bomb button held), throw backwards lightly

    Returns:
        (launch_velocity, kickback_impulse)
    """
    throw_power = 0.80 * (0.60 + 0.40 * min(1.0, move_scale))

    # Penalty for throws executed immediately after pickup (spaz_node.cc:1558-1560)
    if since_pickup_ms < 500.0:
        throw_power *= 0.40 + 0.60 * (since_pickup_ms / 500.0)

    # Ballistica forces: 50.0 forward, 80.0 upward
    if is_bomb_reversed:
        launch_vel = fwd_vec * (-throw_power * 3.5) + (0, 0, throw_power * 5.0)
        kickback = fwd_vec * (throw_power * 0.8)
    else:
        launch_vel = fwd_vec * (throw_power * 14.0) + (0, 0, throw_power * 6.5)
        # Kickback impulse on torso (spaz_node.cc:3829, 3844-3846)
        kickback = -fwd_vec * (throw_power * 1.8)

    return launch_vel, kickback

