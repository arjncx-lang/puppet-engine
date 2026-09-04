# character.py
# Complete PuppetCharacter Engine with Procedural IK, Exact Spring-Dampers & Physics Biomechanics
import math, random
from panda3d.core import Vec3, TransformState, Point3
from panda3d.bullet import (BulletRigidBodyNode, BulletCapsuleShape, ZUp)
from physics_constants import *
from physics_math import (
    box_normalize_to_circle,
    SpringDamper1D,
    SpringDamper3D,
    calculate_centrifugal_bank_angle,
    calculate_longitudinal_pitch_angle,
    calculate_ground_suspension_force,
    calculate_slope_slip_force,
    cycloidal_step_displacement,
    calculate_lissajous_sway,
    clamp_kinetic_energy,
    SquashStretchSystem,
    calculate_slope_foot_alignment,
    ProceduralWeaponController,
    solve_two_bone_ik_3d,
    calculate_ballistica_arm_swing,
    calculate_ballistica_punch_momentum,
    calculate_ballistica_airborne_flail,
)
from props import make_sharp_box


class PuppetCharacter:

    def __init__(self, world, render, loader, start_pos=(0, 0, 0)):
        self.world         = world
        self.render        = render
        self.loader        = loader
        self.facing        = 0.0
        self.jump_ready    = True
        self.roll_amt      = 0.0
        self.anim_time     = 0.0
        self.run_gas       = 0.0
        self.turn_diff     = 0.0
        self.angular_vel_y = 0.0
        self.aim_target_3d = Point3(0, 10, 1)

        # Ice mode
        self.ice_mode      = False

        # Balance, Grounding & Raycast Suspension
        self.balance        = MAX_BALANCE
        self.footing        = True
        self.ground_normal  = Vec3(0, 0, 1)
        self.ground_dist    = SUSPENSION_REST_DIST

        # Biomechanical Acceleration & Bank Tracking
        self.prev_planar_vel = Vec3(0, 0, 0)
        self.forward_accel   = 0.0
        self.gait_phase      = 0.0

        # Exact 2nd-Order Spring-Damper Stabilizers
        self.torso_hpr_spring = SpringDamper3D((0.0, 0.0, 0.0), omega=SPRING_OMEGA_TORSO, zeta=SPRING_ZETA_TORSO)
        self.torso_pos_spring = SpringDamper3D((0.0, 0.0, 0.0), omega=SPRING_OMEGA_TORSO, zeta=SPRING_ZETA_TORSO)
        self.head_hpr_spring  = SpringDamper3D((0.0, 0.0, 0.0), omega=SPRING_OMEGA_HEAD, zeta=SPRING_ZETA_HEAD)
        self.gun_recoil_spring = SpringDamper1D(0.0, omega=SPRING_OMEGA_WEAPON, zeta=SPRING_ZETA_WEAPON)
        self.gun_sway_spring  = SpringDamper3D((0.0, 0.0, 0.0), omega=SPRING_OMEGA_WEAPON, zeta=0.92)

        # Procedural Squash & Stretch + Terrain Slope Foot Alignment (Overgrowth Techniques)
        self.squash_system     = SquashStretchSystem(omega=24.0, zeta=0.68)
        self.foot_pitch_spring = SpringDamper1D(0.0, omega=18.0, zeta=1.0)
        self.foot_roll_spring  = SpringDamper1D(0.0, omega=18.0, zeta=1.0)
        self.spine_flex_spring = SpringDamper1D(0.0, omega=16.0, zeta=1.0)

        # 6-DOF Procedural Weapon Recoil & Sweep Inertia
        self.weapon_dynamics   = ProceduralWeaponController()
        self.cam_prev_yaw      = 180.0

        # Exact 3-Phase Spring Punch state
        self.punch_timer     = 0.0
        self.punch_right     = False
        self.punch_cooldown  = 0.0
        self.is_spin_punch   = False

        # Ballistica Angular & Linear Punch Momentum Accumulators (spaz_node.cc:2060-2086)
        self.punch_mom_ang_d = 0.0
        self.punch_mom_ang_m = 0.0
        self.punch_mom_lin_d = 0.0
        self.punch_mom_lin_m = 0.0

        # PUBG Multi-Weapon Inventory & Back Holsters
        self.weapons_inventory = {}     # { "pistol": gun_obj, "rifle": gun_obj, "shotgun": gun_obj }
        self.active_gun_slot   = None   # "pistol" / "rifle" / "shotgun" or None
        self.held_prop         = None   # Generic crate or pin held overhead
        self.lift_progress     = 0.0
        self.throw_timer       = 0.0

        # Gun Fire & Recoil states
        self.gun_fire_timer    = 0.0
        self.gun_recoil_pitch  = 0.0
        self.is_reloading      = False
        self.reload_timer      = 0.0

        # Knockout & Dizziness
        self.knockout_timer  = 0.0
        self.continuous_spin_time = 0.0

        # Head Jolt
        self.prev_vel_z      = 0.0
        self.head_jolt_pitch = 0.0

        # Dynamic Expression Eyelid Angle
        self.eyelid_angle    = 0.0
        self.next_blink_time = 2.5
        self.blink_val       = 0.0

        # Callbacks
        self.dust_callback   = None
        self.sfx_callback    = None
        self.shoot_callback  = None
        self.casing_callback = None
        self.grenade_callback = None
        self.grenades_count   = 4
        self.grenade_cooldown = 0.0

        self._build(start_pos)

    def _build(self, s):
        x, y, z = s
        W, R, L = self.world, self.render, self.loader

        total_height = TORSO_HEIGHT + PELVIS_HEIGHT + LEG_LENGTH
        capsule_shape = BulletCapsuleShape(TORSO_RADIUS * 1.05, total_height * 0.5, ZUp)

        node = BulletRigidBodyNode("puppet_main")
        node.setMass(6.0)
        node.addShape(capsule_shape)
        node.setLinearDamping(0.15)
        node.setAngularDamping(0.35)
        node.setFriction(0.35)
        node.setDeactivationEnabled(False)
        node.setAngularFactor(Vec3(0, 0, 1))

        self.root_np = R.attachNewNode(node)
        spawn_z = z + total_height * 0.5 + 0.1
        self.root_np.setPos(x, y, spawn_z)
        W.attachRigidBody(node)
        self.physics_body = node

        # ── SEAMLESS ANATOMICAL VISUAL HIERARCHY ──
        self.torso_pivot = self.root_np.attachNewNode("torso_pivot")
        self.head_pivot = self.torso_pivot.attachNewNode("head_pivot")
        self.head_pivot.setPos(0, 0, TORSO_HEIGHT * 0.48 + HEAD_RADIUS * 0.85)
        self.ik_helper_np = R.attachNewNode("ik_helper")

        # Tactical Holster Sockets (Spine Mounts & Duty Hip Holster)
        self.back_holster_sockets = {
            1: self.torso_pivot.attachNewNode("holster_slot1"),
            2: self.torso_pivot.attachNewNode("holster_slot2"),
            3: self.torso_pivot.attachNewNode("holster_slot3"),
        }
        # Slot 1: Tactical Right Hip Holster (Muzzle down along thigh, grip back for quick draw)
        self.back_holster_sockets[1].setPos(0.19, -0.02, -0.07)
        self.back_holster_sockets[1].setHpr(0, -90, 0)

        # Slot 2: Tactical Rifle Spine Scabbard (Flat on side across back, barrel up diagonally)
        self.back_holster_sockets[2].setPos(0.06, -0.19, 0.04)
        self.back_holster_sockets[2].setHpr(90.0, 75.5, 0.0)

        # Slot 3: Tactical Shotgun Spine Scabbard (Flat on side across back, opposite diagonal)
        self.back_holster_sockets[3].setPos(-0.06, -0.21, 0.04)
        self.back_holster_sockets[3].setHpr(-90.0, 75.5, 0.0)

        self.arm_pivots = {}
        for side, sx in [("left", -(TORSO_RADIUS + 0.01)), ("right", (TORSO_RADIUS + 0.01))]:
            socket = self.torso_pivot.attachNewNode(f"{side}_shoulder_socket")
            socket.setPos(sx, 0, TORSO_HEIGHT * 0.18)
            pivot = socket.attachNewNode(f"{side}_arm_pivot")
            self.arm_pivots[side] = pivot

        self.leg_pivots = {}
        for side, sx in [("left", -0.095), ("right", 0.095)]:
            socket = self.torso_pivot.attachNewNode(f"{side}_hip_socket")
            socket.setPos(sx, 0, -(TORSO_HEIGHT * 0.35))
            pivot = socket.attachNewNode(f"{side}_leg_pivot")
            self.leg_pivots[side] = pivot

        self.eyes = []
        self.skin_nodes = {}
        self.gloves = []

        if L:
            # Torso & Pelvis
            self.torso_mesh = L.loadModel("models/misc/sphere")
            self.torso_mesh.setScale(TORSO_RADIUS, TORSO_RADIUS, TORSO_HEIGHT * 0.55)
            self.torso_mesh.setColor(0.22, 0.48, 0.92, 1)
            self.torso_mesh.reparentTo(self.torso_pivot)
            self.skin_nodes["torso"] = self.torso_mesh

            self.pelvis_mesh = L.loadModel("models/misc/sphere")
            self.pelvis_mesh.setScale(PELVIS_RADIUS * 1.05, PELVIS_RADIUS * 1.05, PELVIS_HEIGHT * 0.6)
            self.pelvis_mesh.setColor(0.18, 0.40, 0.82, 1)
            self.pelvis_mesh.setPos(0, 0, -(TORSO_HEIGHT * 0.42))
            self.pelvis_mesh.reparentTo(self.torso_pivot)
            self.skin_nodes["pelvis"] = self.pelvis_mesh

            # Head
            self.head_mesh = L.loadModel("models/misc/sphere")
            self.head_mesh.setScale(HEAD_RADIUS, HEAD_RADIUS * 0.98, HEAD_RADIUS * 1.05)
            self.head_mesh.setColor(0.96, 0.78, 0.58, 1)
            self.head_mesh.reparentTo(self.head_pivot)

            # Eyes
            for side, sx in [("left", -EYE_OFFSET_X), ("right", EYE_OFFSET_X)]:
                eye_socket = self.head_pivot.attachNewNode(f"{side}_eye")
                eye_socket.setPos(sx, EYE_OFFSET_Y, EYE_OFFSET_Z)

                white = L.loadModel("models/misc/sphere")
                white.setScale(EYE_RADIUS)
                white.setColor(0.98, 0.98, 0.98, 1)
                white.reparentTo(eye_socket)

                pupil = L.loadModel("models/misc/sphere")
                pupil.setScale(PUPIL_RADIUS, PUPIL_RADIUS * 0.6, PUPIL_RADIUS)
                pupil.setColor(0.10, 0.12, 0.20, 1)
                pupil.setPos(0, EYE_RADIUS * 0.65, 0)
                pupil.reparentTo(eye_socket)

                eyelid = L.loadModel("models/misc/sphere")
                eyelid.setScale(EYE_RADIUS * 1.10, EYE_RADIUS * 1.05, EYE_RADIUS * 0.5)
                eyelid.setColor(0.96, 0.78, 0.58, 1)
                eyelid.setPos(0, 0, EYE_RADIUS * 0.5)
                eyelid.reparentTo(eye_socket)
                eyelid.hide()

                self.eyes.append({"socket": eye_socket, "eyelid": eyelid, "pupil": pupil, "side": side})

            # ── SEAMLESS UNBREAKABLE 2-BONE LIMBS & ANATOMICAL TACTICAL GLOVES ──
            self.upper_arms = {}
            self.elbow_pivots = {}
            self.elbow_caps = {}
            self.forearms = {}
            self.hand_nodes = {}

            L1 = 0.20  # Upper arm length
            L2 = 0.18  # Forearm length
            self.arm_l1 = L1
            self.arm_l2 = L2

            for side in ("left", "right"):
                shoulder_pivot = self.arm_pivots[side]

                # 1. Deltoid Knuckle Cap (smooth overlapping ball joint at shoulder)
                sh_cap = L.loadModel("models/misc/sphere")
                sh_cap.setScale(ARM_RADIUS * 1.05)
                sh_cap.setColor(0.88, 0.56, 0.18, 1)
                sh_cap.reparentTo(shoulder_pivot)

                # 2. Upper arm bicep sleeve extending along -Z from 0 to -L1
                upper = L.loadModel("models/misc/sphere")
                upper.setScale(ARM_RADIUS * 0.95, ARM_RADIUS * 0.95, L1 * 0.5)
                upper.setColor(0.88, 0.56, 0.18, 1)
                upper.setPos(0, 0, -(L1 * 0.5))
                upper.reparentTo(shoulder_pivot)
                self.upper_arms[side] = upper

                # 3. Articulated Elbow Pivot (strictly parented at (0, 0, -L1))
                elbow = shoulder_pivot.attachNewNode(f"{side}_elbow_pivot")
                elbow.setPos(0, 0, -L1)
                self.elbow_pivots[side] = elbow

                # 4. Seamless Overlapping Elbow Knuckle Cap (fills joint at (0, 0, 0))
                el_cap = L.loadModel("models/misc/sphere")
                el_cap.setScale(ARM_RADIUS * 0.92)
                el_cap.setColor(0.88, 0.56, 0.18, 1)
                el_cap.reparentTo(elbow)
                self.elbow_caps[side] = el_cap

                # 5. Forearm sleeve extending along -Z from 0 to -L2
                forearm = L.loadModel("models/misc/sphere")
                forearm.setScale(ARM_RADIUS * 0.88, ARM_RADIUS * 0.88, L2 * 0.5)
                forearm.setColor(0.88, 0.56, 0.18, 1)
                forearm.setPos(0, 0, -(L2 * 0.5))
                forearm.reparentTo(elbow)
                self.forearms[side] = forearm

                # 6. Hand Node strictly parented at (0, 0, -L2)
                hand = elbow.attachNewNode(f"{side}_hand_node")
                hand.setPos(0, 0, -L2)
                self.hand_nodes[side] = hand

                # 7. Anatomical Tactical Combat Hand
                sign = 1.0 if side == "right" else -1.0
                glove_root = hand.attachNewNode(f"{side}_glove_root")

                # Wrist Gauntlet Cuff (dark neoprene sealing forearm to glove)
                make_sharp_box(L, glove_root, (0.076, 0.076, 0.032), (0, 0, 0.015), color=(0.14, 0.14, 0.16, 1))

                # Contoured Palm & Dorsal Plate (deep tactical red)
                make_sharp_box(L, glove_root, (0.070, 0.065, 0.070), (0, 0.012, -0.028), color=(0.92, 0.20, 0.18, 1))

                # Carbon Knuckle Armor Guard
                make_sharp_box(L, glove_root, (0.068, 0.024, 0.022), (0, 0.042, -0.024), color=(0.16, 0.16, 0.18, 1))

                # Ergonomic Angled Thumb
                make_sharp_box(L, glove_root, (0.026, 0.034, 0.026), (sign * 0.038, 0.022, -0.026), hpr=(sign * 25, -15, 0), color=(0.82, 0.18, 0.16, 1))

                # Gripping Fingers Pad
                make_sharp_box(L, glove_root, (0.062, 0.034, 0.032), (0, 0.022, -0.068), hpr=(0, 22, 0), color=(0.82, 0.18, 0.16, 1))

                self.gloves.append(glove_root)

            # Legs & Shoes
            for side in ("left", "right"):
                pivot = self.leg_pivots[side]
                leg = L.loadModel("models/misc/sphere")
                leg.setScale(LEG_RADIUS, LEG_RADIUS, LEG_LENGTH * 0.5)
                leg.setColor(0.18, 0.40, 0.82, 1)
                leg.setPos(0, 0, -(LEG_LENGTH * 0.38))
                leg.reparentTo(pivot)

                shoe = L.loadModel("models/misc/sphere")
                shoe.setScale(FOOT_RADIUS * 0.95, FOOT_RADIUS * 1.3, FOOT_RADIUS * 0.7)
                shoe.setColor(0.20, 0.20, 0.24, 1)
                shoe.setPos(0, FOOT_RADIUS * 0.35, -(LEG_LENGTH * 0.80))
                shoe.reparentTo(pivot)

    def set_dust_callback(self, cb): self.dust_callback = cb
    def set_sfx_callback(self, cb): self.sfx_callback = cb
    def set_shoot_callback(self, cb): self.shoot_callback = cb
    def set_casing_callback(self, cb): self.casing_callback = cb
    def set_grenade_callback(self, cb): self.grenade_callback = cb

    def set_skin(self, body_color, glove_color):
        if "torso" in self.skin_nodes:
            self.skin_nodes["torso"].setColor(*body_color)
            self.skin_nodes["pelvis"].setColor(body_color[0]*0.8, body_color[1]*0.8, body_color[2]*0.8, 1)
        for g in self.gloves:
            g.setColor(*glove_color)

    def toggle_ice_mode(self):
        self.ice_mode = not self.ice_mode
        self.physics_body.setFriction(0.04 if self.ice_mode else 0.30)
        return self.ice_mode

    def is_holding_gun(self):
        return self.active_gun_slot is not None and self.active_gun_slot in self.weapons_inventory

    def get_active_gun(self):
        if self.is_holding_gun():
            return self.weapons_inventory[self.active_gun_slot]
        return None

    def cycle_weapon(self, direction=1):
        """Cycles forward/backward through owned weapons (via Mouse Scroll or +/- keys)."""
        if not self.weapons_inventory:
            return False

        order = ["pistol", "rifle", "shotgun"]
        available = [w for w in order if w in self.weapons_inventory]
        if not available:
            return False

        if self.active_gun_slot not in available:
            self.active_gun_slot = available[0] if direction > 0 else available[-1]
        else:
            cur_idx = available.index(self.active_gun_slot)
            new_idx = (cur_idx + direction) % len(available)
            self.active_gun_slot = available[new_idx]

        if self.sfx_callback:
            self.sfx_callback("gun_pickup")
        return True

    def switch_weapon_slot(self, slot_key):
        slot_map = {1: "pistol", 2: "rifle", 3: "shotgun"}
        gun_type = slot_map.get(slot_key, None)

        if not gun_type or gun_type not in self.weapons_inventory:
            return False

        if self.active_gun_slot == gun_type:
            self.active_gun_slot = None
            if self.sfx_callback:
                self.sfx_callback("gun_pickup")
            return True

        self.active_gun_slot = gun_type
        if self.sfx_callback:
            self.sfx_callback("gun_pickup")
        return True

    def trigger_reload(self):
        gun = self.get_active_gun()
        if gun and not self.is_reloading:
            if gun.ammo_mag < gun.cfg["mag_size"] and gun.ammo_reserve > 0:
                self.is_reloading = True
                self.reload_timer = 0.50
                if self.sfx_callback:
                    self.sfx_callback("reload")
                return True
        return False

    def trigger_fire(self, target_3d_point):
        if self.knockout_timer > 0.0 or self.is_reloading:
            return False

        gun = self.get_active_gun()
        if not gun:
            return False

        if self.gun_fire_timer > 0.0:
            return False

        if gun.ammo_mag <= 0:
            self.trigger_reload()
            return False

        gun.ammo_mag -= 1
        self.gun_fire_timer = gun.cfg["fire_rate"]

        # 6-DOF Procedural Weapon Recoil (Linear Kickback + Muzzle Rise + Rifling Torque Twist)
        w_type = gun.weapon_type
        if w_type == "pistol":
            self.weapon_dynamics.trigger_recoil(linear_kick=0.055, pitch_kick=8.0, yaw_kick=1.2, roll_kick=1.6)
        elif w_type == "rifle":
            self.weapon_dynamics.trigger_recoil(linear_kick=0.080, pitch_kick=11.5, yaw_kick=1.5, roll_kick=2.2)
        elif w_type == "shotgun":
            self.weapon_dynamics.trigger_recoil(linear_kick=0.140, pitch_kick=18.5, yaw_kick=2.6, roll_kick=4.0)

        recoil_kick = 16.0 if gun.weapon_type != "shotgun" else 26.0
        self.gun_recoil_spring.reset(self.gun_recoil_pitch + recoil_kick, -90.0)
        self.gun_recoil_pitch = self.gun_recoil_spring.pos

        if self.sfx_callback:
            self.sfx_callback(gun.cfg["sound"])

        torso_pos = self.root_np.getPos()
        fwd = self.get_forward_vector()
        muzzle_pos = torso_pos + fwd * 0.50 + Vec3(0, 0, 0.38)

        # Eject 1 physical spent shell casing per trigger pull
        if self.casing_callback:
            self.casing_callback(muzzle_pos, fwd, gun.weapon_type == "shotgun")

        if self.shoot_callback:
            pellets = gun.cfg["pellets"]
            spread  = gun.cfg["spread"]
            force   = gun.cfg["force"]

            for _ in range(pellets):
                if spread > 0.0:
                    offset_x = random.uniform(-spread, spread) * 20.0
                    offset_y = random.uniform(-spread, spread) * 20.0
                    offset_z = random.uniform(-spread, spread) * 20.0
                    pellet_target = target_3d_point + Vec3(offset_x, offset_y, offset_z)
                else:
                    pellet_target = target_3d_point

                self.shoot_callback(muzzle_pos, pellet_target, force)
        return True

    def trigger_primary_action(self, targets_list, target_3d_point):
        if self.knockout_timer > 0.0:
            return False

        if self.is_holding_gun():
            return self.trigger_fire(target_3d_point)

        if self.held_prop is not None:
            self.throw_held_object()
            return True

        return self.trigger_punch(targets_list)

    def trigger_punch(self, targets_list):
        if self.punch_timer > 0.0 or self.punch_cooldown > 0.0:
            return False

        self.punch_timer = PUNCH_DURATION
        self.punch_cooldown = PUNCH_DURATION * 0.7

        # Ballistica dynamic hand selection: couple punch hand to spin direction if turning fast
        if abs(self.angular_vel_y) < 0.35:
            self.punch_right = not self.punch_right
        else:
            self.punch_right = (self.angular_vel_y > 0.0)

        if self.sfx_callback:
            self.sfx_callback("punch_whoosh")

        self.is_spin_punch = (abs(self.angular_vel_y) > 4.5 or abs(self.turn_diff) > 45.0)

        torso_pos = self.root_np.getPos()
        rad = math.radians(self.facing)
        fwd = Vec3(-math.sin(rad), math.cos(rad), 0)

        hit_reach = 1.35 if self.is_spin_punch else 0.95
        punch_point = torso_pos + fwd * (0.68 if not self.is_spin_punch else 0.0)

        # Ballistica momentum scaling: angular and linear sprint velocity augment impact force
        momentum_mult = 1.0 + min(1.5, self.punch_mom_ang_m * 0.45 + self.punch_mom_lin_m * 0.35)
        impulse_mag = PUNCH_IMPULSE * (SPIN_PUNCH_MULT if self.is_spin_punch else 1.0) * momentum_mult

        hit_any = False
        for target in targets_list:
            if target == self.held_prop:
                continue
            t_pos = target.get_pos()
            dist = (t_pos - punch_point).length()
            if dist < hit_reach:
                impact_dir = (t_pos - torso_pos).normalized()
                impact = (impact_dir + Vec3(0, 0, 0.45)) * impulse_mag
                target.apply_impulse(impact)
                hit_any = True

        if hit_any and self.sfx_callback:
            self.sfx_callback("punch_hit")

        return True


    def trigger_pickup(self, targets_list):
        if self.knockout_timer > 0.0:
            return False

        if self.held_prop is not None:
            self.throw_held_object()
            return True

        torso_pos = self.root_np.getPos()
        fwd = self.get_forward_vector()
        search_point = torso_pos + fwd * 0.65

        closest_target = None
        closest_dist = PICKUP_RADIUS

        for target in targets_list:
            dist = (target.get_pos() - search_point).length()
            if dist < closest_dist:
                closest_dist = dist
                closest_target = target

        if closest_target:
            if getattr(closest_target, "is_gun", False):
                g_type = closest_target.weapon_type
                if g_type in self.weapons_inventory:
                    self.weapons_inventory[g_type].add_ammo(closest_target.cfg["ammo_pickup"])
                    if self.sfx_callback:
                        self.sfx_callback("reload")
                    return ("ammo", closest_target.cfg["ammo_pickup"], closest_target.name)
                else:
                    self.weapons_inventory[g_type] = closest_target
                    self.active_gun_slot = g_type
                    closest_target.set_held(True)
                    if self.sfx_callback:
                        self.sfx_callback("gun_pickup")
                    return ("weapon", closest_target.name)
            else:
                self.held_prop = closest_target
                self.lift_progress = 0.0
                closest_target.set_held(True)
                return ("prop", closest_target.name)

        return None

    def throw_held_object(self):
        if self.held_prop is None:
            return
        obj = self.held_prop
        self.held_prop = None
        obj.set_held(False)

        if self.sfx_callback:
            self.sfx_callback("throw")

        torso_pos = self.root_np.getPos()
        cur_v = self.physics_body.getLinearVelocity()
        fwd = self.get_forward_vector()

        fwd_speed = max(0.0, fwd.dot(cur_v))
        launch_vel = fwd * (THROW_VELOCITY + fwd_speed * 0.6) + Vec3(0, 0, THROW_UP_VELOCITY)
        obj.set_pos(torso_pos + fwd * 0.45 + Vec3(0, 0, 0.65))
        obj.set_linear_velocity(launch_vel)
        self.throw_timer = 0.22

    def trigger_knockout(self, duration=1.5):
        self.knockout_timer = duration
        self.balance = 0
        if self.sfx_callback:
            self.sfx_callback("stun")
        if self.held_prop:
            self.throw_held_object()

    def trigger_grenade_throw(self, target_3d_point):
        if self.grenades_count <= 0 or self.grenade_cooldown > 0.0 or self.knockout_timer > 0.0:
            return False

        self.grenades_count -= 1
        self.grenade_cooldown = 0.75
        self.throw_timer = 0.28

        torso_pos = self.root_np.getPos()
        fwd = self.get_forward_vector()
        spawn_pos = torso_pos + fwd * 0.45 + Vec3(0, 0, 0.55)

        aim_diff = target_3d_point - spawn_pos
        dist = aim_diff.length()
        if dist > 0.001:
            aim_dir = aim_diff / dist
        else:
            aim_dir = fwd

        toss_speed = max(11.0, min(22.0, dist * 1.30))
        throw_vel = aim_dir * toss_speed + Vec3(0, 0, 3.8)

        if self.sfx_callback:
            self.sfx_callback("throw")

        if self.grenade_callback:
            self.grenade_callback(spawn_pos, throw_vel)

        return True

    def take_blast_impact(self, impulse_vec, blast_dist_ratio):
        self.physics_body.setActive(True)
        self.physics_body.applyCentralImpulse(impulse_vec)
        balance_loss = int(180 * blast_dist_ratio)
        self.balance = max(0, self.balance - balance_loss)
        if self.balance < 40:
            self.footing = False
            if self.sfx_callback:
                self.sfx_callback("stun")
            if self.held_prop:
                self.throw_held_object()

    def _orient_downward_joint(self, joint_np, target_world, origin_pos=None):
        p = origin_pos if origin_pos is not None else joint_np.getPos(self.render)
        self.ik_helper_np.setPos(p)
        self.ik_helper_np.lookAt(self.render, target_world)
        self.ik_helper_np.setP(self.ik_helper_np.getP() + 90.0)
        joint_np.setHpr(self.render, self.ik_helper_np.getHpr(self.render))

    def get_forward_vector(self):
        rad = math.radians(self.facing)
        return Vec3(-math.sin(rad), math.cos(rad), 0)

    def apply_movement(self, world_mx, world_my, do_jump, is_sprinting, is_shooting_held, dt, cam_yaw, target_3d_point):
        self.anim_time += dt
        self.aim_target_3d = target_3d_point

        if self.knockout_timer > 0.0:
            self.knockout_timer = max(0.0, self.knockout_timer - dt)
            self.torso_pivot.setHpr(0, 45.0, 30.0)
            self.arm_pivots["left"].setHpr(20, -20, -10)
            self.arm_pivots["right"].setHpr(-20, -20, 10)
            return

        if self.punch_timer > 0:
            self.punch_timer = max(0.0, self.punch_timer - dt)
        if self.punch_cooldown > 0:
            self.punch_cooldown = max(0.0, self.punch_cooldown - dt)
        if self.throw_timer > 0:
            self.throw_timer = max(0.0, self.throw_timer - dt)
        if self.gun_fire_timer > 0:
            self.gun_fire_timer = max(0.0, self.gun_fire_timer - dt)
        if self.grenade_cooldown > 0:
            self.grenade_cooldown = max(0.0, self.grenade_cooldown - dt)

        # Exact 2nd-order damped spring recoil decay
        self.gun_recoil_pitch = self.gun_recoil_spring.update(0.0, dt)

        if self.is_reloading:
            self.reload_timer -= dt
            if self.reload_timer <= 0:
                self.is_reloading = False
                gun = self.get_active_gun()
                if gun:
                    gun.reload()

        if is_shooting_held and self.is_holding_gun():
            active_gun = self.get_active_gun()
            if active_gun and active_gun.cfg["is_auto"]:
                self.trigger_fire(target_3d_point)

        if self.held_prop and self.lift_progress < 1.0:
            self.lift_progress = min(1.0, self.lift_progress + dt / PICKUP_LIFT_TIME)

        body = self.physics_body
        cur_v = body.getLinearVelocity()
        was_grounded = self.footing

        # Raycast Virtual Pneumatic Ground Suspension Probe
        torso_pos = self.root_np.getPos()
        p_from = Point3(torso_pos.x, torso_pos.y, torso_pos.z + 0.1)
        p_to   = Point3(torso_pos.x, torso_pos.y, torso_pos.z - 1.3)
        ray_res = self.world.rayTestClosest(p_from, p_to)

        if ray_res.hasHit() and ray_res.getNode() != self.physics_body:
            hit_p = ray_res.getHitPos()
            hit_dist = p_from.z - hit_p.z - 0.1
            self.ground_dist = hit_dist
            self.ground_normal = ray_res.getHitNormal()
            self.footing = (hit_dist <= SUSPENSION_REST_DIST + 0.18 and abs(cur_v.z) < 4.0)

            if self.footing and not do_jump:
                f_susp = calculate_ground_suspension_force(hit_dist, SUSPENSION_REST_DIST, cur_v.z, SUSPENSION_K, SUSPENSION_C)
                f_susp = max(-60.0, min(180.0, f_susp))
                body.applyCentralForce(Vec3(0, 0, f_susp))

                # Dynamic slope slip force
                slip_force, slope_deg = calculate_slope_slip_force(self.ground_normal, mass=6.0, gravity=GRAVITY, max_walkable_angle_deg=MAX_WALKABLE_SLOPE)
                if slip_force.lengthSquared() > 0.05:
                    body.applyCentralForce(slip_force)
        else:
            self.footing = False
            self.ground_normal = Vec3(0, 0, 1)

        # Procedural Landing Squash Impact (Overgrowth Dynamic Deformation)
        if not was_grounded and self.footing:
            self.squash_system.trigger_landing_squash(cur_v.z)

        # Dynamic footing & balance meter update
        if self.footing:
            if self.balance < 100: self.balance += 20
            elif self.balance < 235: self.balance += 20
            elif self.balance < MAX_BALANCE: self.balance += 1
        else:
            if self.balance > 100: self.balance -= 20
            elif self.balance > 0: self.balance -= 5

        # Biomechanical acceleration & forward load tracking
        cur_planar_vel = Vec3(cur_v.x, cur_v.y, 0)
        accel_vec = (cur_planar_vel - self.prev_planar_vel) / max(0.001, dt)
        fwd = self.get_forward_vector()
        self.forward_accel = accel_vec.dot(fwd)
        self.prev_planar_vel = cur_planar_vel

        delta_vz = cur_v.z - self.prev_vel_z
        self.prev_vel_z = cur_v.z
        if delta_vz > 5.0:
            self.head_jolt_pitch = 18.0
            if self.dust_callback:
                t_pos = self.root_np.getPos()
                self.dust_callback((t_pos.x, t_pos.y, 0.02))
        else:
            self.head_jolt_pitch *= max(0.0, 1.0 - dt * 8.0)

        is_moving = (abs(world_mx) > 0.01 or abs(world_my) > 0.01)
        horiz_speed = math.hypot(cur_v.x, cur_v.y) if is_moving else 0.0

        speed_limit = SPRINT_SPEED if is_sprinting else MOVE_SPEED
        if self.held_prop:
            speed_limit *= 0.90

        target_gas = (horiz_speed / speed_limit) if is_moving else 0.0
        smoothing = 0.95 if target_gas > self.run_gas else 0.65
        self.run_gas = smoothing * self.run_gas + (1.0 - smoothing) * target_gas
        if not self.footing:
            self.run_gas = max(0.0, self.run_gas - dt * 2.0)


        if self.is_holding_gun():
            cur_h = self.root_np.getH()
            self.turn_diff = (cam_yaw - cur_h + 180.0) % 360.0 - 180.0
            turn_w = math.radians(self.turn_diff) * 20.0
            body.setAngularVelocity(Vec3(0, 0, turn_w))
            self.angular_vel_y = turn_w
            self.facing = cur_h

            target_vx = world_mx * speed_limit
            target_vy = world_my * speed_limit
        elif is_moving:
            target_deg = math.degrees(math.atan2(-world_mx, world_my))
            cur_h = self.root_np.getH()
            self.turn_diff = (target_deg - cur_h + 180.0) % 360.0 - 180.0
            turn_w = math.radians(self.turn_diff) * (8.0 if self.ice_mode else 16.0)
            body.setAngularVelocity(Vec3(0, 0, turn_w))
            self.angular_vel_y = turn_w
            self.facing = cur_h

            target_vx = world_mx * speed_limit
            target_vy = world_my * speed_limit
        else:
            self.turn_diff = 0.0
            self.angular_vel_y = 0.0
            if not self.ice_mode:
                body.setAngularVelocity(Vec3(0, 0, 0))
            target_vx = 0.0
            target_vy = 0.0

        if is_moving:
            accel = 6.0 if self.ice_mode else ACCEL_RATE
            new_vx = cur_v.x + (target_vx - cur_v.x) * min(1.0, dt * accel)
            new_vy = cur_v.y + (target_vy - cur_v.y) * min(1.0, dt * accel)
        else:
            stop_rate = 3.0 if self.ice_mode else 28.0
            new_vx = cur_v.x * max(0.0, 1.0 - dt * stop_rate)
            new_vy = cur_v.y * max(0.0, 1.0 - dt * stop_rate)
            if abs(new_vx) < 0.05: new_vx = 0.0
            if abs(new_vy) < 0.05: new_vy = 0.0

        body.setLinearVelocity(Vec3(new_vx, new_vy, cur_v.z))

        # Kinetic Energy & Velocity Safety Clamp
        safe_v, safe_w = clamp_kinetic_energy(body.getLinearVelocity(), body.getAngularVelocity(), 6.0, MAX_KINETIC_ENERGY)
        body.setLinearVelocity(safe_v)
        body.setAngularVelocity(safe_w)

        # Ballistica cyclical gait roll amount & momentum accumulator integration
        if self.footing and horiz_speed > 0.1:
            self.roll_amt += horiz_speed * dt * 9.5
            if self.roll_amt > 2.0 * math.pi:
                self.roll_amt -= 2.0 * math.pi
        else:
            self.roll_amt *= max(0.0, 1.0 - dt * 4.0)

        self.punch_mom_ang_d, self.punch_mom_ang_m, self.punch_mom_lin_d, self.punch_mom_lin_m = (
            calculate_ballistica_punch_momentum(
                angular_vel=self.angular_vel_y,
                linear_vel=cur_v.length(),
                prev_ang_d=self.punch_mom_ang_d,
                prev_ang_m=self.punch_mom_ang_m,
                prev_lin_d=self.punch_mom_lin_d,
                prev_lin_m=self.punch_mom_lin_m,
            )
        )


        if do_jump and self.jump_ready and self.footing:
            body.setLinearVelocity(Vec3(new_vx, new_vy, JUMP_VELOCITY))
            self.jump_ready = False
            self.squash_system.trigger_jump_stretch(1.24)
            if self.sfx_callback:
                self.sfx_callback("jump")
            if self.dust_callback:
                t_pos = self.root_np.getPos()
                self.dust_callback((t_pos.x, t_pos.y, 0.02))

        if not do_jump:
            self.jump_ready = True

        torso_pos = self.root_np.getPos()
        fwd = self.get_forward_vector()

        # Anatomical Lissajous Figure-8 Weapon Sway
        sway_x, sway_z = calculate_lissajous_sway(self.anim_time, SWAY_FREQ, SWAY_AMP_X, SWAY_AMP_Z)
        target_sway = Vec3(sway_x, 0, sway_z) if self.is_holding_gun() else Vec3(0, 0, 0)
        curr_sway = self.gun_sway_spring.update(target_sway, dt)

        # 6-DOF Procedural Weapon Dynamics (Linear Kickback + Angular Recoil + Sweep Inertia)
        cam_dyaw = (cam_yaw - self.cam_prev_yaw) / max(0.001, dt)
        self.cam_prev_yaw = cam_yaw
        kick_z, wep_rot = self.weapon_dynamics.update(cam_dyaw, 0.0, dt)

        rgt = Vec3(fwd.y, -fwd.x, 0)
        up  = Vec3(0, 0, 1)

        for g_type, gun in self.weapons_inventory.items():
            slot_idx = gun.cfg["slot"]
            if g_type == self.active_gun_slot:
                if gun.np.getParent() != self.render:
                    gun.np.reparentTo(self.render)
                if g_type == "pistol":
                    # Tactical Two-Handed Weaver Combat Stance (Pushed forward, centered)
                    stance_offset = fwd * (0.24 + kick_z) + rgt * 0.08 + up * 0.30
                else:
                    # Rifle / Shotgun: Buttstock firmly seated in Right Shoulder Pocket!
                    stance_offset = fwd * (0.12 + kick_z) + rgt * 0.14 + up * 0.28

                gun_pos = torso_pos + stance_offset + curr_sway
                gun.set_pos(gun_pos)
                gun.look_at(target_3d_point)
                gun.set_hpr(gun.np.getHpr() + wep_rot)
            else:
                holster_socket = self.back_holster_sockets[slot_idx]
                if gun.np.getParent() != holster_socket:
                    gun.np.reparentTo(holster_socket)
                    gun.np.setPos(0, 0, 0)
                    gun.np.setHpr(0, 0, 0)

        if self.held_prop:
            lift_s = math.sin(self.lift_progress * math.pi * 0.5)
            stride_bob  = math.sin(self.roll_amt * 2.0) * 0.015 * self.run_gas
            stride_sway = math.cos(self.roll_amt) * 0.02 * self.run_gas
            # Realistic Chest-Braced Carry: rests securely against lower chest / upper abdomen
            cur_z = 0.18 * (1.0 - lift_s) + (0.34 + stride_bob) * lift_s
            fwd_offset = (0.46 * (1.0 - lift_s)) + (0.36 * lift_s)
            pos = torso_pos + fwd * fwd_offset + Vec3(stride_sway, 0, cur_z)
            self.held_prop.set_pos(pos)

        if (horiz_speed >= 5.0 or self.punch_timer > 0.0 or self.is_holding_gun()):
            target_eyelid_angle = 28.0
        elif not self.footing:
            target_eyelid_angle = -12.0
        else:
            target_eyelid_angle = 0.0

        self.eyelid_angle += (target_eyelid_angle - self.eyelid_angle) * min(1.0, dt * 8.0)

        if self.anim_time >= self.next_blink_time:
            self.blink_val = 1.0
            self.next_blink_time = self.anim_time + random.uniform(2.5, 5.0)

        if self.blink_val > 0.0:
            self.blink_val = max(0.0, self.blink_val - dt * 6.0)
            is_blink = (self.blink_val > 0.2)
            for eye in self.eyes:
                if is_blink:
                    eye["eyelid"].show()
                    eye["pupil"].hide()
                else:
                    eye["eyelid"].hide()
                    eye["pupil"].show()
        else:
            for eye in self.eyes:
                angle = self.eyelid_angle if eye["side"] == "left" else -self.eyelid_angle
                eye["socket"].setR(angle)

        self._update_layered_animation(dt, horiz_speed, is_sprinting, target_3d_point)

    def _update_layered_animation(self, dt, speed, is_sprinting, target_3d_point):
        # ── PROCEDURAL ANKLE IK (Terrain Slope Alignment) ──
        raw_fp, raw_fr = calculate_slope_foot_alignment(self.ground_normal, self.facing)
        target_fp = raw_fp if self.footing else 0.0
        target_fr = raw_fr if self.footing else 0.0
        foot_p = self.foot_pitch_spring.update(target_fp, dt)
        foot_r = self.foot_roll_spring.update(target_fr, dt)

        # ── CYCLOIDAL GAIT KINEMATICS (Zero Ground Slip with Ballistica Stride Extension) ──
        if self.footing:
            if speed > 0.25:
                # Ballistica stride reach and step lift scaling
                stride_reach_mult = 1.0 + 0.38 * self.run_gas
                step_lift_mult = 1.0 + 0.35 * self.run_gas
                stride_len = (GAIT_STRIDE_BASE + (GAIT_STRIDE_SPRINT - GAIT_STRIDE_BASE) * min(1.0, speed / SPRINT_SPEED)) * stride_reach_mult
                effective_step_height = GAIT_STEP_HEIGHT * step_lift_mult

                stride_mult = (1.2 if self.ice_mode else (1.6 if is_sprinting else 1.3))
                self.gait_phase += (speed / max(0.1, stride_len)) * dt * (2.0 * math.pi) * stride_mult
                if self.gait_phase > 2.0 * math.pi:
                    self.gait_phase -= 2.0 * math.pi
                    if is_sprinting and self.dust_callback and not self.ice_mode:
                        t_pos = self.root_np.getPos()
                        self.dust_callback((t_pos.x, t_pos.y, 0.02))

                # Exact cycloidal displacement for left and right feet
                l_x, l_z = cycloidal_step_displacement(self.gait_phase, stride_len, effective_step_height)
                r_x, r_z = cycloidal_step_displacement(self.gait_phase + math.pi, stride_len, effective_step_height)

                l_pitch = math.degrees(math.atan2(l_x, LEG_LENGTH * 1.1))
                r_pitch = math.degrees(math.atan2(r_x, LEG_LENGTH * 1.1))
                l_lift = (l_z / effective_step_height) * 15.0
                r_lift = (r_z / effective_step_height) * 15.0

                self.leg_pivots["left"].setHpr(0, l_pitch + l_lift + foot_p, foot_r)
                self.leg_pivots["right"].setHpr(0, r_pitch + r_lift + foot_p, foot_r)
            else:
                self.leg_pivots["left"].setHpr(0, foot_p, foot_r)
                self.leg_pivots["right"].setHpr(0, foot_p, foot_r)
        else:
            self.gait_phase -= dt * 10.0
            kick = math.sin(self.gait_phase) * 25.0
            self.leg_pivots["left"].setHpr(0, kick, -8.0)
            self.leg_pivots["right"].setHpr(0, -kick, 8.0)

        # ── INVERTED PENDULUM BIOMECHANICAL BANKING & ACCELERATION PITCH ──
        bank_roll = calculate_centrifugal_bank_angle(speed, self.angular_vel_y, abs(GRAVITY), BANK_MAX_DEG)
        accel_pitch = calculate_longitudinal_pitch_angle(self.forward_accel, abs(GRAVITY), PITCH_MAX_DEG)

        # ── TORSO ORIENTATION & POSITION WITH EXACT 2ND-ORDER SPRING-DAMPER ──
        target_torso_hpr = Vec3(0, 0, 0)
        target_torso_pos = Vec3(0, 0, 0)

        if self.is_holding_gun():
            torso_pos = self.root_np.getPos()
            aim_vec = (target_3d_point - (torso_pos + Vec3(0, 0, 0.38))).normalized()
            pitch_deg = math.degrees(math.asin(max(-0.95, min(0.95, aim_vec.z))))
            active_gun = self.get_active_gun()
            torso_blade_yaw = -16.0 if active_gun and active_gun.weapon_type != "pistol" else -8.0
            target_torso_hpr = Vec3(torso_blade_yaw, -pitch_deg * 0.35 + accel_pitch * 0.5, bank_roll)
        elif self.punch_timer > 0.0:
            if self.is_spin_punch:
                progress = 1.0 - (self.punch_timer / PUNCH_DURATION)
                spin_h = progress * 360.0
                target_torso_hpr = Vec3(spin_h, 8.0, 0)
            else:
                elapsed_ms = (PUNCH_DURATION - self.punch_timer) * 1000.0
                mirror = 1.0 if self.punch_right else -1.0
                if elapsed_ms < 65.0:
                    prog = elapsed_ms / 65.0
                    torso_twist = -16.0 * prog * mirror
                    target_torso_pos = Vec3(0, 0, 0)
                elif elapsed_ms < 145.0:
                    prog = (elapsed_ms - 65.0) / 80.0
                    thrust = math.sin(prog * math.pi * 0.5)
                    torso_twist = (-16.0 + thrust * 42.0) * mirror
                    target_torso_pos = Vec3(0, 0.10 * thrust, 0)
                else:
                    prog = (elapsed_ms - 145.0) / 135.0
                    ret_s = 1.0 - prog
                    torso_twist = 26.0 * ret_s * mirror
                    target_torso_pos = Vec3(0, 0.10 * ret_s, 0)
                target_torso_hpr = Vec3(torso_twist, 4.0, 0)
        elif self.throw_timer > 0.0:
            throw_prog = 1.0 - (self.throw_timer / 0.22)
            snap_lean = math.sin(throw_prog * math.pi) * 22.0
            target_torso_hpr = Vec3(0, snap_lean, 0)
        elif self.held_prop:
            lift_s = math.sin(self.lift_progress * math.pi * 0.5)
            torso_squat = -0.10 * math.sin(self.lift_progress * math.pi)
            torso_lean = 14.0 * (1.0 - lift_s) - 8.0 * lift_s
            target_torso_hpr = Vec3(0, torso_lean + accel_pitch * 0.5, bank_roll)
            target_torso_pos = Vec3(0, 0, torso_squat)
        elif self.footing:
            if speed > 0.3:
                gas  = self.run_gas
                stride_phase = self.gait_phase
                torso_bob   = abs(math.sin(stride_phase)) * (0.05 if is_sprinting else 0.04) * gas
                torso_pitch = gas * (14.0 if is_sprinting else 9.0) + accel_pitch
                torso_roll  = bank_roll - self.turn_diff * (0.35 if self.ice_mode else 0.20) * gas
                target_torso_pos = Vec3(0, 0, torso_bob)
                target_torso_hpr = Vec3(0, torso_pitch, torso_roll)
            else:
                breath = math.sin(self.anim_time * 3.6)
                sway   = math.cos(self.anim_time * 1.8)
                target_torso_pos = Vec3(0, 0, breath * 0.012)
                target_torso_hpr = Vec3(sway * 1.2, breath * 1.5, 0)
        else:
            target_torso_hpr = Vec3(0, -8.0, 0)

        # ── PROCEDURAL SQUASH & STRETCH (David Rosen Overgrowth Formulation) ──
        cur_vz = self.physics_body.getLinearVelocity().z
        squash_scale = self.squash_system.update(cur_vz, self.footing, dt)
        self.torso_pivot.setScale(squash_scale)

        # ── PROCEDURAL SPINE CURVATURE FLEXION ──
        spine_flex = self.spine_flex_spring.update(self.turn_diff * 0.22, dt)
        target_torso_hpr = Vec3(target_torso_hpr.x + spine_flex, target_torso_hpr.y, target_torso_hpr.z)

        smoothed_torso_hpr = self.torso_hpr_spring.update(target_torso_hpr, dt)
        smoothed_torso_pos = self.torso_pos_spring.update(target_torso_pos, dt)
        self.torso_pivot.setHpr(smoothed_torso_hpr)
        self.torso_pivot.setPos(smoothed_torso_pos)

        # ── HEAD TRACKING WITH EXACT 2ND-ORDER SPRING-DAMPER ──
        target_head_hpr = Vec3(0, 0, 0)
        if self.is_holding_gun():
            torso_pos = self.root_np.getPos()
            aim_vec = (target_3d_point - (torso_pos + Vec3(0, 0, 0.38))).normalized()
            pitch_deg = math.degrees(math.asin(max(-0.95, min(0.95, aim_vec.z))))
            active_gun = self.get_active_gun()
            torso_blade_yaw = -16.0 if active_gun and active_gun.weapon_type != "pistol" else -8.0
            target_head_hpr = Vec3(-torso_blade_yaw, -pitch_deg * 0.5, 0)
        elif self.held_prop:
            lift_s = math.sin(self.lift_progress * math.pi * 0.5)
            # Ballistica head tilt (+0.5 rad = 28.6 deg): look past the held object
            head_look_up = 28.6 * lift_s
            target_head_hpr = Vec3(0, head_look_up + self.head_jolt_pitch, 0)
        elif self.footing:
            if speed > 0.3:
                gas  = self.run_gas
                torso_pitch = gas * (14.0 if is_sprinting else 9.0) + accel_pitch
                head_pitch  = -torso_pitch * 0.6 + self.head_jolt_pitch
                target_head_hpr = Vec3(0, head_pitch, 0)
            else:
                breath = math.sin(self.anim_time * 3.5)
                sway   = math.cos(self.anim_time * 1.8)
                target_head_hpr = Vec3(sway * 2.0, -breath * 2.0 + self.head_jolt_pitch, 0)
        else:
            target_head_hpr = Vec3(0, 15.0 + self.head_jolt_pitch, 0)

        smoothed_head_hpr = self.head_hpr_spring.update(target_head_hpr, dt)
        self.head_pivot.setHpr(smoothed_head_hpr)

        # ── CLOSED-FORM 3D TWO-BONE INVERSE KINEMATICS & ARM LAYERED ACTIONS ──
        if self.is_holding_gun():
            active_gun = self.get_active_gun()
            if active_gun and hasattr(active_gun, "grip_socket") and hasattr(active_gun, "guard_socket"):
                L1 = self.arm_l1
                L2 = self.arm_l2
                fwd = self.get_forward_vector()
                rgt = Vec3(fwd.y, -fwd.x, 0)
                up  = Vec3(0, 0, 1)

                # Right Arm: Firing hand locks to weapon grip socket
                r_sh_pos = self.arm_pivots["right"].getPos(self.render)
                r_target = active_gun.grip_socket.getPos(self.render)
                r_pole   = rgt * 0.85 - up * 0.50 - fwd * 0.15  # Outward flared combat elbow
                r_elbow  = solve_two_bone_ik_3d(r_sh_pos, r_target, L1, L2, r_pole)
                self._orient_downward_joint(self.arm_pivots["right"], r_elbow, r_sh_pos)
                self._orient_downward_joint(self.elbow_pivots["right"], r_target, r_elbow)

                # Left Arm: Support hand locks to handguard / forend socket
                l_sh_pos = self.arm_pivots["left"].getPos(self.render)
                l_target = active_gun.guard_socket.getPos(self.render)
                l_pole   = -rgt * 0.35 + fwd * 0.40 - up * 0.85  # Tucked tactical support elbow
                l_elbow  = solve_two_bone_ik_3d(l_sh_pos, l_target, L1, L2, l_pole)
                self._orient_downward_joint(self.arm_pivots["left"], l_elbow, l_sh_pos)
                self._orient_downward_joint(self.elbow_pivots["left"], l_target, l_elbow)

        elif self.held_prop:
            # ── BALLISTICA TWO-HANDED PROP CARRY IK (spaz_node.cc / prop_node.cc) ──
            p_crate = self.held_prop.get_pos()
            fwd = self.get_forward_vector()
            rgt = Vec3(fwd.y, -fwd.x, 0)
            up  = Vec3(0, 0, 1)

            # Ballistica grip targets: prop +/- right * 0.15 - up * 0.04 + fwd * 0.02
            crate_left  = p_crate - rgt * 0.15 - up * 0.04 + fwd * 0.02
            crate_right = p_crate + rgt * 0.15 - up * 0.04 + fwd * 0.02

            L1 = self.arm_l1
            L2 = self.arm_l2

            l_sh_pos = self.arm_pivots["left"].getPos(self.render)
            l_pole = -rgt * 0.85 - up * 0.35 + fwd * 0.20
            l_elbow = solve_two_bone_ik_3d(l_sh_pos, crate_left, L1, L2, l_pole)
            self._orient_downward_joint(self.arm_pivots["left"], l_elbow, l_sh_pos)
            self._orient_downward_joint(self.elbow_pivots["left"], crate_left, l_elbow)

            r_sh_pos = self.arm_pivots["right"].getPos(self.render)
            r_pole = rgt * 0.85 - up * 0.35 + fwd * 0.20
            r_elbow = solve_two_bone_ik_3d(r_sh_pos, crate_right, L1, L2, r_pole)
            self._orient_downward_joint(self.arm_pivots["right"], r_elbow, r_sh_pos)
            self._orient_downward_joint(self.elbow_pivots["right"], crate_right, r_elbow)

        elif self.punch_timer > 0.0:
            # ── 3-PHASE MARTIAL ARTS COMBO CROSS / JAB ──
            elapsed_ms = (PUNCH_DURATION - self.punch_timer) * 1000.0
            mirror = 1.0 if self.punch_right else -1.0

            if self.is_spin_punch:
                self.arm_pivots["left"].setHpr(0, -60.0, -45.0)
                self.elbow_pivots["left"].setHpr(0, -20.0, 0)
                self.arm_pivots["right"].setHpr(0, -60.0, 45.0)
                self.elbow_pivots["right"].setHpr(0, -20.0, 0)
            else:
                if elapsed_ms < 65.0:
                    prog = elapsed_ms / 65.0
                    punch_pitch = -25.0 * prog
                    punch_roll  = 12.0 * prog * mirror
                    elbow_bend  = -75.0 * prog
                    opp_pitch   = -35.0 * prog
                    opp_roll    = -15.0 * prog * mirror
                    opp_elbow   = -65.0 * prog
                elif elapsed_ms < 145.0:
                    prog = (elapsed_ms - 65.0) / 80.0
                    thrust = math.sin(prog * math.pi * 0.5)
                    punch_pitch = -25.0 - thrust * 62.0
                    punch_roll  = (12.0 + thrust * 18.0) * mirror
                    elbow_bend  = -75.0 * (1.0 - thrust)
                    opp_pitch   = -35.0
                    opp_roll    = -15.0 * mirror
                    opp_elbow   = -65.0
                else:
                    prog = (elapsed_ms - 145.0) / 135.0
                    ret_s = 1.0 - prog
                    punch_pitch = -87.0 * ret_s
                    punch_roll  = 30.0 * ret_s * mirror
                    elbow_bend  = -20.0 * (1.0 - ret_s)
                    opp_pitch   = -35.0 * ret_s
                    opp_roll    = -15.0 * ret_s * mirror
                    opp_elbow   = -65.0 * ret_s - 15.0 * (1.0 - ret_s)

                if self.punch_right:
                    self.arm_pivots["right"].setHpr(0, punch_pitch, punch_roll)
                    self.elbow_pivots["right"].setHpr(0, elbow_bend, 0)
                    self.arm_pivots["left"].setHpr(0, opp_pitch, opp_roll)
                    self.elbow_pivots["left"].setHpr(0, opp_elbow, 0)
                else:
                    self.arm_pivots["left"].setHpr(0, punch_pitch, punch_roll)
                    self.elbow_pivots["left"].setHpr(0, elbow_bend, 0)
                    self.arm_pivots["right"].setHpr(0, opp_pitch, opp_roll)
                    self.elbow_pivots["right"].setHpr(0, opp_elbow, 0)

        elif self.throw_timer > 0.0:
            # ── 2-PHASE PHYSICAL HEAVE THROW ──
            throw_prog = 1.0 - (self.throw_timer / 0.22)
            if throw_prog < 0.25:
                snap_pitch = -20.0 - (throw_prog / 0.25) * 15.0
                el_pitch = -70.0
            else:
                thrust = math.sin(((throw_prog - 0.25) / 0.75) * math.pi * 0.5)
                snap_pitch = -35.0 - thrust * 55.0
                el_pitch = -70.0 * (1.0 - thrust)

            self.arm_pivots["left"].setHpr(0, snap_pitch, -12.0)
            self.elbow_pivots["left"].setHpr(0, el_pitch, 0)
            self.arm_pivots["right"].setHpr(0, snap_pitch, 12.0)
            self.elbow_pivots["right"].setHpr(0, el_pitch, 0)

        elif self.footing:
            if speed > 0.3:
                # Ballistica Quadrature Elliptical Arm Swing (spaz_node.cc:2935-2972)
                (l_p, l_r, l_elb), (r_p, r_r, r_elb) = calculate_ballistica_arm_swing(self.roll_amt, self.run_gas)
                self.arm_pivots["left"].setHpr(0, l_p, l_r)
                self.elbow_pivots["left"].setHpr(0, l_elb, 0)
                self.arm_pivots["right"].setHpr(0, r_p, r_r)
                self.elbow_pivots["right"].setHpr(0, r_elb, 0)
            else:
                breath = math.sin(self.anim_time * 3.6)
                self.arm_pivots["left"].setHpr(0, breath * 2.5, -10.0)
                self.elbow_pivots["left"].setHpr(0, -14.0, 0)
                self.arm_pivots["right"].setHpr(0, -breath * 2.5, 10.0)
                self.elbow_pivots["right"].setHpr(0, -14.0, 0)
        else:
            # Ballistica Airborne Counter-Rotating Flail Kinematics (spaz_node.cc:2838-2859)
            l_arm, l_elb, r_arm, r_elb = calculate_ballistica_airborne_flail(self.anim_time)
            self.arm_pivots["left"].setHpr(l_arm[0], l_arm[1], l_arm[2])
            self.elbow_pivots["left"].setHpr(l_elb[0], l_elb[1], l_elb[2])
            self.arm_pivots["right"].setHpr(r_arm[0], r_arm[1], r_arm[2])
            self.elbow_pivots["right"].setHpr(r_elb[0], r_elb[1], r_elb[2])


    def get_torso_pos(self):
        return self.root_np.getPos()
