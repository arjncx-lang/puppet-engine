# character.py
# Complete PuppetCharacter Engine with Procedural IK, Weapon Cycling & Zero-Drift
import math, random
from panda3d.core import Vec3, TransformState, Point3
from panda3d.bullet import (BulletRigidBodyNode, BulletCapsuleShape, ZUp)
from physics_constants import *


def box_normalize_to_circle(lr, ud):
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

        # Balance & Footing
        self.balance       = MAX_BALANCE
        self.footing       = True

        # Exact 3-Phase Spring Punch state
        self.punch_timer     = 0.0
        self.punch_right     = False
        self.punch_cooldown  = 0.0
        self.is_spin_punch   = False

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

        # Back Holster Sockets (Spine Mounts)
        self.back_holster_sockets = {
            1: self.torso_pivot.attachNewNode("holster_slot1"),
            2: self.torso_pivot.attachNewNode("holster_slot2"),
            3: self.torso_pivot.attachNewNode("holster_slot3"),
        }
        self.back_holster_sockets[1].setPos(-0.16, -0.08, -0.05)
        self.back_holster_sockets[1].setHpr(0, 75, 10)

        self.back_holster_sockets[2].setPos(0.08, -0.16, 0.08)
        self.back_holster_sockets[2].setHpr(25, 40, -35)

        self.back_holster_sockets[3].setPos(-0.08, -0.16, 0.08)
        self.back_holster_sockets[3].setHpr(-25, 40, 35)

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

            # Arms & Gloves
            for side in ("left", "right"):
                pivot = self.arm_pivots[side]
                arm = L.loadModel("models/misc/sphere")
                arm.setScale(ARM_RADIUS, ARM_RADIUS, ARM_LENGTH * 0.5)
                arm.setColor(0.88, 0.56, 0.18, 1)
                arm.setPos(0, 0, -(ARM_LENGTH * 0.35))
                arm.reparentTo(pivot)

                glove = L.loadModel("models/misc/sphere")
                glove.setScale(HAND_RADIUS, HAND_RADIUS * 1.1, HAND_RADIUS)
                glove.setColor(0.92, 0.20, 0.18, 1)
                glove.setPos(0, 0.02, -(ARM_LENGTH * 0.75))
                glove.reparentTo(pivot)
                self.gloves.append(glove)

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
        self.gun_recoil_pitch = 16.0 if gun.weapon_type != "shotgun" else 24.0

        if self.sfx_callback:
            self.sfx_callback(gun.cfg["sound"])

        torso_pos = self.root_np.getPos()
        fwd = self.get_forward_vector()
        muzzle_pos = torso_pos + fwd * 0.50 + Vec3(0, 0, 0.38)

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
        self.punch_right = not self.punch_right

        if self.sfx_callback:
            self.sfx_callback("punch_whoosh")

        self.is_spin_punch = (abs(self.angular_vel_y) > 4.5 or abs(self.turn_diff) > 45.0)

        torso_pos = self.root_np.getPos()
        rad = math.radians(self.facing)
        fwd = Vec3(-math.sin(rad), math.cos(rad), 0)

        hit_reach = 1.35 if self.is_spin_punch else 0.95
        punch_point = torso_pos + fwd * (0.68 if not self.is_spin_punch else 0.0)
        impulse_mag = PUNCH_IMPULSE * (SPIN_PUNCH_MULT if self.is_spin_punch else 1.0)

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
        if self.gun_recoil_pitch > 0:
            self.gun_recoil_pitch = max(0.0, self.gun_recoil_pitch - dt * 90.0)

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

        cur_speed = cur_v.length()
        if cur_speed > MAX_LINEAR_VELOCITY:
            cur_v = cur_v * (MAX_LINEAR_VELOCITY / cur_speed)
            body.setLinearVelocity(cur_v)

        pos_z = self.root_np.getPos().z
        self.footing = (pos_z <= 0.65 and abs(cur_v.z) < 1.5)

        if self.footing:
            if self.balance < 100: self.balance += 20
            elif self.balance < 235: self.balance += 20
            elif self.balance < MAX_BALANCE: self.balance += 1
        else:
            if self.balance > 100: self.balance -= 20
            elif self.balance > 0: self.balance -= 5

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
        self.run_gas += (target_gas - self.run_gas) * min(1.0, dt * 12.0)

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

        if do_jump and self.jump_ready and self.footing:
            body.setLinearVelocity(Vec3(new_vx, new_vy, JUMP_VELOCITY))
            self.jump_ready = False
            if self.sfx_callback:
                self.sfx_callback("jump")
            if self.dust_callback:
                t_pos = self.root_np.getPos()
                self.dust_callback((t_pos.x, t_pos.y, 0.02))

        if not do_jump:
            self.jump_ready = True

        torso_pos = self.root_np.getPos()
        fwd = self.get_forward_vector()

        for g_type, gun in self.weapons_inventory.items():
            slot_idx = gun.cfg["slot"]
            if g_type == self.active_gun_slot:
                gun_pos = torso_pos + fwd * 0.42 + Vec3(0, 0, 0.38)
                gun.set_pos(gun_pos)
                gun.look_at(target_3d_point)
            else:
                holster_socket = self.back_holster_sockets[slot_idx]
                gun.set_pos(holster_socket.getPos(self.render))
                gun.set_hpr(holster_socket.getHpr(self.render))

        if self.held_prop:
            lift_s = math.sin(self.lift_progress * math.pi * 0.5)
            stride_bob  = math.sin(self.roll_amt * 2.0) * 0.02 * self.run_gas
            stride_sway = math.cos(self.roll_amt) * 0.03 * self.run_gas
            floor_z = 0.30
            overhead_z = 0.74 + stride_bob
            cur_z = floor_z * (1.0 - lift_s) + overhead_z * lift_s
            fwd_offset = (0.50 * (1.0 - lift_s)) + (0.02 * lift_s)
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
        if self.footing:
            if speed > 0.3:
                stride_mult = (1.5 if self.ice_mode else (3.2 if is_sprinting else 2.4))
                self.roll_amt += speed * dt * stride_mult
                if self.roll_amt > 2.0 * math.pi:
                    self.roll_amt -= 2.0 * math.pi
                    if is_sprinting and self.dust_callback and not self.ice_mode:
                        t_pos = self.root_np.getPos()
                        self.dust_callback((t_pos.x, t_pos.y, 0.02))

                roll = self.roll_amt
                gas  = self.run_gas
                l_leg_pitch = math.sin(roll) * (32.0 + gas * 18.0)
                r_leg_pitch = -math.sin(roll) * (32.0 + gas * 18.0)
                self.leg_pivots["left"].setHpr(0, l_leg_pitch, 0)
                self.leg_pivots["right"].setHpr(0, r_leg_pitch, 0)
            else:
                self.leg_pivots["left"].setHpr(0, 0, 0)
                self.leg_pivots["right"].setHpr(0, 0, 0)
        else:
            self.roll_amt -= dt * 10.0
            kick = math.sin(self.roll_amt) * 25.0
            self.leg_pivots["left"].setHpr(0, kick, -8.0)
            self.leg_pivots["right"].setHpr(0, -kick, 8.0)

        if self.is_holding_gun():
            torso_pos = self.root_np.getPos()
            aim_vec = (target_3d_point - (torso_pos + Vec3(0, 0, 0.38))).normalized()
            pitch_deg = math.degrees(math.asin(max(-0.95, min(0.95, aim_vec.z))))
            arm_p = -pitch_deg - self.gun_recoil_pitch - 75.0

            self.arm_pivots["right"].setHpr(0, arm_p, 12.0)
            self.arm_pivots["left"].setHpr(0, arm_p + 4.0, -16.0)

        elif self.punch_timer > 0.0:
            elapsed_ms = (PUNCH_DURATION - self.punch_timer) * 1000.0

            if self.is_spin_punch:
                self.arm_pivots["left"].setHpr(0, -60.0, -45.0)
                self.arm_pivots["right"].setHpr(0, -60.0, 45.0)
            else:
                if elapsed_ms < 80.0:
                    prog = elapsed_ms / 80.0
                    punch_pitch = -20.0 + prog * 45.0
                    punch_roll  = -prog * 25.0
                    opp_pitch   = -prog * 35.0
                elif elapsed_ms < 200.0:
                    prog = (elapsed_ms - 80.0) / 120.0
                    thrust_curve = math.sin(prog * math.pi * 0.5)
                    punch_pitch = 25.0 - thrust_curve * 115.0
                    punch_roll  = 20.0 * thrust_curve
                    opp_pitch   = -35.0 + (1.0 - thrust_curve) * 15.0
                else:
                    prog = (elapsed_ms - 200.0) / 100.0
                    punch_pitch = -90.0 + prog * 90.0
                    punch_roll  = 20.0 * (1.0 - prog)
                    opp_pitch   = -20.0 * (1.0 - prog)

                if self.punch_right:
                    self.arm_pivots["right"].setHpr(0, punch_pitch, punch_roll)
                    self.arm_pivots["left"].setHpr(0, opp_pitch, -12.0)
                else:
                    self.arm_pivots["left"].setHpr(0, punch_pitch, -punch_roll)
                    self.arm_pivots["right"].setHpr(0, opp_pitch, 12.0)

        elif self.held_prop:
            lift_s = math.sin(self.lift_progress * math.pi * 0.5)
            reach_pitch = -25.0 * (1.0 - lift_s) + (-95.0 * lift_s)
            clasp_roll  = 24.0 * lift_s
            self.arm_pivots["left"].setHpr(0, reach_pitch, -clasp_roll)
            self.arm_pivots["right"].setHpr(0, reach_pitch, clasp_roll)

        elif self.throw_timer > 0.0:
            throw_prog = 1.0 - (self.throw_timer / 0.22)
            if throw_prog < 0.25:
                snap_pitch = -95.0 - (throw_prog / 0.25) * 15.0
            else:
                snap_pitch = -110.0 + ((throw_prog - 0.25) / 0.75) * 160.0

            self.arm_pivots["left"].setHpr(0, snap_pitch, -12.0)
            self.arm_pivots["right"].setHpr(0, snap_pitch, 12.0)

        elif self.footing:
            if speed > 0.3:
                roll = self.roll_amt
                gas  = self.run_gas
                l_arm_pitch = -math.sin(roll) * (40.0 + gas * 25.0)
                r_arm_pitch =  math.sin(roll) * (40.0 + gas * 25.0)
                self.arm_pivots["left"].setHpr(0, l_arm_pitch, -12.0)
                self.arm_pivots["right"].setHpr(0, r_arm_pitch, 12.0)
            else:
                breath = math.sin(self.anim_time * 3.5)
                self.arm_pivots["left"].setHpr(0, breath * 3.0, -12.0)
                self.arm_pivots["right"].setHpr(0, -breath * 3.0, 12.0)
        else:
            wave1 = math.sin(self.anim_time * 14.0) * 25.0
            self.arm_pivots["left"].setHpr(0, -65.0 + wave1, -25.0)
            self.arm_pivots["right"].setHpr(0, -65.0 - wave1, 25.0)

        if self.is_holding_gun():
            torso_pos = self.root_np.getPos()
            aim_vec = (target_3d_point - (torso_pos + Vec3(0, 0, 0.38))).normalized()
            pitch_deg = math.degrees(math.asin(max(-0.95, min(0.95, aim_vec.z))))
            self.torso_pivot.setHpr(0, -pitch_deg * 0.35, 0)
        elif self.punch_timer > 0.0:
            if self.is_spin_punch:
                progress = 1.0 - (self.punch_timer / PUNCH_DURATION)
                spin_h = progress * 360.0
                self.torso_pivot.setHpr(spin_h, 8.0, 0)
            else:
                elapsed_ms = (PUNCH_DURATION - self.punch_timer) * 1000.0
                mirror = 1.0 if self.punch_right else -1.0
                if elapsed_ms < 80.0:
                    coil = (elapsed_ms / 80.0) * -15.0 * mirror
                elif elapsed_ms < 200.0:
                    prog = (elapsed_ms - 80.0) / 120.0
                    coil = (-15.0 + math.sin(prog * math.pi * 0.5) * 39.0) * mirror
                else:
                    prog = (elapsed_ms - 200.0) / 100.0
                    coil = (24.0 * (1.0 - prog)) * mirror
                self.torso_pivot.setHpr(coil, 6.0, 0)

        elif self.throw_timer > 0.0:
            throw_prog = 1.0 - (self.throw_timer / 0.22)
            snap_lean = math.sin(throw_prog * math.pi) * 18.0
            self.torso_pivot.setHpr(0, snap_lean, 0)
        elif self.held_prop:
            lift_s = math.sin(self.lift_progress * math.pi * 0.5)
            torso_lean = -6.0 * lift_s
            self.torso_pivot.setHpr(0, torso_lean, 0)
            self.torso_pivot.setPos(0, 0, 0)
        elif self.footing:
            if speed > 0.3:
                roll = self.roll_amt
                gas  = self.run_gas
                torso_bob   = abs(math.sin(roll)) * (0.05 if is_sprinting else 0.04) * gas
                torso_pitch = gas * (16.0 if is_sprinting else 10.0)
                torso_roll  = -self.turn_diff * (0.45 if self.ice_mode else 0.25) * gas
                self.torso_pivot.setPos(0, 0, torso_bob)
                self.torso_pivot.setHpr(0, torso_pitch, torso_roll)
            else:
                breath = math.sin(self.anim_time * 3.5)
                sway   = math.cos(self.anim_time * 1.8)
                self.torso_pivot.setPos(0, 0, breath * 0.008)
                self.torso_pivot.setHpr(sway * 1.2, breath * 1.5, 0)
        else:
            self.torso_pivot.setPos(0, 0, 0)
            self.torso_pivot.setHpr(0, -8.0, 0)

        if self.is_holding_gun():
            torso_pos = self.root_np.getPos()
            aim_vec = (target_3d_point - (torso_pos + Vec3(0, 0, 0.38))).normalized()
            pitch_deg = math.degrees(math.asin(max(-0.95, min(0.95, aim_vec.z))))
            self.head_pivot.setHpr(0, -pitch_deg * 0.5, 0)
        elif self.held_prop:
            lift_s = math.sin(self.lift_progress * math.pi * 0.5)
            head_look_up = 26.0 * lift_s
            self.head_pivot.setHpr(0, head_look_up + self.head_jolt_pitch, 0)
        elif self.footing:
            if speed > 0.3:
                gas  = self.run_gas
                torso_pitch = gas * (16.0 if is_sprinting else 10.0)
                head_pitch  = -torso_pitch * 0.6 + self.head_jolt_pitch
                self.head_pivot.setHpr(0, head_pitch, 0)
            else:
                breath = math.sin(self.anim_time * 3.5)
                sway   = math.cos(self.anim_time * 1.8)
                self.head_pivot.setHpr(sway * 2.0, -breath * 2.0 + self.head_jolt_pitch, 0)
        else:
            self.head_pivot.setHpr(0, 15.0 + self.head_jolt_pitch, 0)

    def get_torso_pos(self):
        return self.root_np.getPos()
