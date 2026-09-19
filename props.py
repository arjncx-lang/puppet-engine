# props.py
import random
from panda3d.core import Vec3, Point3, BitMask32
from panda3d.bullet import (BulletRigidBodyNode, BulletBoxShape, BulletCapsuleShape,
                            BulletSphereShape, BulletCylinderShape, ZUp)
from physics_constants import *
from physics_math import calculate_off_center_torque, apply_semi_implicit_drag


_PROTOTYPE_CACHE = {}

def get_cached_model(loader, model_path):
    if not loader:
        return None
    if model_path not in _PROTOTYPE_CACHE:
        _PROTOTYPE_CACHE[model_path] = loader.loadModel(model_path)
    return _PROTOTYPE_CACHE[model_path]


def make_sharp_box(loader, parent, size=(0.1, 0.1, 0.1), pos=(0, 0, 0), hpr=(0, 0, 0), color=(0.2, 0.2, 0.2, 1.0)):
    proto = get_cached_model(loader, "models/box")
    np = parent.attachNewNode("box_part")
    if proto:
        m = proto.copyTo(np)
        m.setPos(-0.5, -0.5, -0.5)
        m.setColor(*color)
    np.setScale(size[0], size[1], size[2])
    np.setPos(*pos)
    np.setHpr(*hpr)
    return np


class InteractiveCrate:
    def __init__(self, world, render, loader, pos, size=(0.28, 0.28, 0.28), mass=1.4):
        self.world = world
        self.render = render
        self.is_held = False
        self.is_gun = False
        self.has_exploded = False
        self.is_attached_to_world = True
        self.holder = None
        self.name = "Wooden Crate"
        self.release_grace_timer = 0.0
        self.last_pos = Vec3(*pos)

        shape = BulletBoxShape(Vec3(size[0], size[1], size[2]))
        node  = BulletRigidBodyNode("crate")
        node.setMass(mass)
        node.addShape(shape)
        node.setFriction(0.65)
        node.setRestitution(0.3)
        node.setLinearDamping(0.15)
        node.setAngularDamping(0.35)
        node.setIntoCollideMask(BitMask32.bit(COLLISION_GROUP_PROP))

        self.np = render.attachNewNode(node)
        self.np.setPos(*pos)
        world.attachRigidBody(node)
        self.node = node
        self.mesh = None
        self.base_color = (0.68, 0.46, 0.24, 1)
        self.flash_timer = 0.0

        if loader:
            m = make_sharp_box(loader, self.np, (size[0]*2.0, size[1]*2.0, size[2]*2.0), (0, 0, 0), color=self.base_color)
            make_sharp_box(loader, self.np, (size[0]*2.02, size[1]*0.18, size[2]*2.02), (0, 0, 0), color=(0.28, 0.24, 0.20, 1))
            make_sharp_box(loader, self.np, (size[0]*0.18, size[1]*2.02, size[2]*2.02), (0, 0, 0), color=(0.28, 0.24, 0.20, 1))
            self.mesh = m

    def get_pos(self):
        if hasattr(self, "np") and not self.np.isEmpty():
            p = self.np.getPos()
            self.last_pos = Vec3(p)
            return p
        return getattr(self, "last_pos", Vec3(0, 0, 0))

    def set_pos(self, pos):
        if hasattr(self, "np") and not self.np.isEmpty():
            self.np.setPos(pos)
            self.last_pos = Vec3(pos)

    def trigger_impact_flash(self):
        self.flash_timer = 0.065
        if self.mesh:
            self.mesh.setColor(1.0, 0.96, 0.70, 1.0)

    def set_held(self, held, holder=None):
        if self.is_held == held: return
        self.is_held = held
        self.holder = holder if held else None
        if held:
            if self.world and self.node and self.is_attached_to_world:
                self.world.removeRigidBody(self.node)
                self.is_attached_to_world = False
        else:
            if self.world and self.node and not self.is_attached_to_world:
                self.node.setIntoCollideMask(BitMask32.bit(COLLISION_GROUP_HELD))
                self.world.attachRigidBody(self.node)
                self.is_attached_to_world = True
            self.release_grace_timer = THROW_GRACE_TIME
            self.node.setKinematic(False)
            self.node.setActive(True)

    def set_linear_velocity(self, vel):
        if self.node:
            self.node.setActive(True)
            self.node.setLinearVelocity(vel)

    def apply_impulse(self, impulse_vec, hit_pos=None):
        if not self.is_held and not self.has_exploded and self.node:
            self.node.setActive(True)
            self.node.applyCentralImpulse(impulse_vec)
            self.trigger_impact_flash()
            if hit_pos is not None and hasattr(self, "np") and not self.np.isEmpty():
                body_pos = self.np.getPos()
                torque = calculate_off_center_torque(hit_pos, body_pos, impulse_vec, max_lever_arm=0.25)
                self.node.applyTorqueImpulse(torque)
            ang_v = self.node.getAngularVelocity()
            if ang_v.lengthSquared() > MAX_ANGULAR_VELOCITY * MAX_ANGULAR_VELOCITY:
                self.node.setAngularVelocity(ang_v.normalized() * MAX_ANGULAR_VELOCITY)

    def update_physics(self, dt):
        if self.has_exploded:
            return False

        if self.release_grace_timer > 0.0:
            self.release_grace_timer -= dt
            if self.release_grace_timer <= 0.0 and self.node:
                self.node.setIntoCollideMask(BitMask32.bit(COLLISION_GROUP_PROP))

        if self.flash_timer > 0.0:
            self.flash_timer -= dt
            if self.flash_timer <= 0.0 and self.mesh:
                self.mesh.setColor(*self.base_color)

        if not self.is_held and self.node and self.node.isActive():
            vel = self.node.getLinearVelocity()
            ang_vel = self.node.getAngularVelocity()
            new_v, new_w = apply_semi_implicit_drag(vel, ang_vel, dt, AERO_DRAG_COEFF, ROT_DRAG_COEFF, MAX_ANGULAR_VELOCITY)
            self.node.setLinearVelocity(new_v)
            self.node.setAngularVelocity(new_w)
        return True


class BowlingPin:
    def __init__(self, world, render, loader, pos):
        self.world = world
        self.render = render
        self.is_held = False
        self.is_gun = False
        self.has_exploded = False
        self.is_attached_to_world = True
        self.holder = None
        self.name = "Bowling Pin"
        self.release_grace_timer = 0.0
        self.last_pos = Vec3(*pos)

        shape = BulletCapsuleShape(0.10, 0.35, ZUp)
        node = BulletRigidBodyNode("pin")
        node.setMass(0.8)
        node.addShape(shape)
        node.setFriction(0.4)
        node.setRestitution(0.5)
        node.setIntoCollideMask(BitMask32.bit(COLLISION_GROUP_PROP))

        self.np = render.attachNewNode(node)
        self.np.setPos(*pos)
        world.attachRigidBody(node)
        self.node = node
        self.mesh = None
        self.base_color = (0.95, 0.95, 0.95, 1)
        self.flash_timer = 0.0

        if loader:
            proto = get_cached_model(loader, "models/misc/sphere")
            if proto:
                m = proto.copyTo(self.np)
                m.setScale(0.12, 0.12, 0.30)
                m.setColor(*self.base_color)
                self.mesh = m

    def get_pos(self):
        if hasattr(self, "np") and not self.np.isEmpty():
            p = self.np.getPos()
            self.last_pos = Vec3(p)
            return p
        return getattr(self, "last_pos", Vec3(0, 0, 0))

    def set_pos(self, pos):
        if hasattr(self, "np") and not self.np.isEmpty():
            self.np.setPos(pos)
            self.last_pos = Vec3(pos)

    def trigger_impact_flash(self):
        self.flash_timer = 0.065
        if self.mesh:
            self.mesh.setColor(1.0, 0.85, 0.40, 1.0)

    def set_held(self, held, holder=None):
        if self.is_held == held: return
        self.is_held = held
        self.holder = holder if held else None
        if held:
            if self.world and self.node and self.is_attached_to_world:
                self.world.removeRigidBody(self.node)
                self.is_attached_to_world = False
        else:
            if self.world and self.node and not self.is_attached_to_world:
                self.node.setIntoCollideMask(BitMask32.bit(COLLISION_GROUP_HELD))
                self.world.attachRigidBody(self.node)
                self.is_attached_to_world = True
            self.release_grace_timer = THROW_GRACE_TIME
            self.node.setKinematic(False)
            self.node.setActive(True)

    def set_linear_velocity(self, vel):
        if self.node:
            self.node.setActive(True)
            self.node.setLinearVelocity(vel)

    def apply_impulse(self, impulse_vec, hit_pos=None):
        if not self.is_held and not self.has_exploded and self.node:
            self.node.setActive(True)
            self.node.applyCentralImpulse(impulse_vec)
            self.trigger_impact_flash()
            if hit_pos is not None and hasattr(self, "np") and not self.np.isEmpty():
                body_pos = self.np.getPos()
                torque = calculate_off_center_torque(hit_pos, body_pos, impulse_vec, max_lever_arm=0.18)
                self.node.applyTorqueImpulse(torque)
            ang_v = self.node.getAngularVelocity()
            if ang_v.lengthSquared() > MAX_ANGULAR_VELOCITY * MAX_ANGULAR_VELOCITY:
                self.node.setAngularVelocity(ang_v.normalized() * MAX_ANGULAR_VELOCITY)

    def update_physics(self, dt):
        if self.has_exploded:
            return False

        if self.release_grace_timer > 0.0:
            self.release_grace_timer -= dt
            if self.release_grace_timer <= 0.0 and self.node:
                self.node.setIntoCollideMask(BitMask32.bit(COLLISION_GROUP_PROP))

        if self.flash_timer > 0.0:
            self.flash_timer -= dt
            if self.flash_timer <= 0.0 and self.mesh:
                self.mesh.setColor(*self.base_color)

        if not self.is_held and self.node and self.node.isActive():
            vel = self.node.getLinearVelocity()
            ang_vel = self.node.getAngularVelocity()
            new_v, new_w = apply_semi_implicit_drag(vel, ang_vel, dt, AERO_DRAG_COEFF, ROT_DRAG_COEFF, MAX_ANGULAR_VELOCITY)
            self.node.setLinearVelocity(new_v)
            self.node.setAngularVelocity(new_w)
        return True


WEAPON_CONFIGS = {
    "pistol": {
        "name": "Tactical Heavy Pistol",
        "slot": 1,
        "is_auto": False,
        "fire_rate": 0.20,
        "mag_size": 12,
        "reserve_ammo": 48,
        "ammo_pickup": 24,
        "pellets": 1,
        "spread": 0.0,
        "force": 40.0,
        "sound": "pistol_shot",
    },
    "rifle": {
        "name": "M4 Spec-Ops Assault Rifle",
        "slot": 2,
        "is_auto": True,
        "fire_rate": 0.09,
        "mag_size": 30,
        "reserve_ammo": 120,
        "ammo_pickup": 30,
        "pellets": 1,
        "spread": 0.012,
        "force": 34.0,
        "sound": "rifle_shot",
    },
    "shotgun": {
        "name": "SPAS Combat Shotgun",
        "slot": 3,
        "is_auto": False,
        "fire_rate": 0.60,
        "mag_size": 8,
        "reserve_ammo": 32,
        "ammo_pickup": 16,
        "pellets": 6,
        "spread": 0.060,
        "force": 54.0,
        "sound": "shotgun_shot",
    }
}


class GunWeapon:
    """
    Sharp, High-Definition Procedural 3D Weapon with Physics Collision Isolation.
    Guns on the floor collide with Environment and Props, but NEVER collide with the character.
    """
    def __init__(self, world, render, loader, pos, weapon_type="pistol"):
        self.world = world
        self.render = render
        self.loader = loader
        self.weapon_type = weapon_type
        self.cfg = WEAPON_CONFIGS[weapon_type]
        self.name = self.cfg["name"]
        self.is_gun = True
        self.is_held = False
        self.is_holstered = False
        self.has_exploded = False
        self.is_attached_to_world = True
        self.last_pos = Vec3(*pos)

        self.ammo_mag = self.cfg["mag_size"]
        self.ammo_reserve = self.cfg["reserve_ammo"]

        shape = BulletBoxShape(Vec3(0.20, 0.10, 0.12))
        node = BulletRigidBodyNode(f"gun_{weapon_type}")
        node.setMass(1.0)
        node.addShape(shape)
        node.setFriction(0.6)
        node.setRestitution(0.2)
        node.setIntoCollideMask(BitMask32.bit(COLLISION_GROUP_WEAPON))

        self.np = render.attachNewNode(node)
        self.np.setPos(*pos)
        world.attachRigidBody(node)
        self.node = node

        self.grip_socket  = self.np.attachNewNode("grip_socket")
        self.guard_socket = self.np.attachNewNode("guard_socket")

        if loader:
            self._build_sharp_weapon_model(weapon_type)

    def _build_sharp_weapon_model(self, w_type):
        L = self.loader
        R = self.np

        if w_type == "pistol":
            # Tactical Combat Pistol (Sharp Modular Geometry)
            make_sharp_box(L, R, (0.048, 0.21, 0.055), (0, 0.04, 0.04), color=(0.18, 0.19, 0.21, 1))
            make_sharp_box(L, R, (0.042, 0.19, 0.045), (0, 0.03, 0.0), color=(0.12, 0.13, 0.14, 1))
            make_sharp_box(L, R, (0.038, 0.07, 0.13), (0, -0.04, -0.07), hpr=(0, 16, 0), color=(0.10, 0.11, 0.12, 1))
            make_sharp_box(L, R, (0.012, 0.015, 0.018), (0, 0.135, 0.073), color=(0.95, 0.95, 0.95, 1))
            make_sharp_box(L, R, (0.032, 0.02, 0.018), (0, -0.055, 0.073), color=(0.20, 0.85, 0.30, 1))
            make_sharp_box(L, R, (0.026, 0.06, 0.026), (0, 0.16, 0.04), color=(0.10, 0.10, 0.11, 1))
            self.grip_socket.setPos(0, -0.04, -0.07)
            self.guard_socket.setPos(0, -0.04, -0.09)

        elif w_type == "rifle":
            # M4A1 Tactical Carbine (Sharp Modular Geometry)
            make_sharp_box(L, R, (0.055, 0.26, 0.08), (0, 0.0, 0.03), color=(0.18, 0.20, 0.22, 1))
            make_sharp_box(L, R, (0.045, 0.28, 0.06), (0, 0.22, 0.03), color=(0.14, 0.15, 0.17, 1))
            make_sharp_box(L, R, (0.025, 0.20, 0.025), (0, 0.44, 0.03), color=(0.10, 0.10, 0.12, 1))
            make_sharp_box(L, R, (0.035, 0.06, 0.035), (0, 0.55, 0.03), color=(0.25, 0.26, 0.28, 1))
            make_sharp_box(L, R, (0.035, 0.09, 0.18), (0, 0.06, -0.09), hpr=(0, -14, 0), color=(0.12, 0.13, 0.15, 1))
            make_sharp_box(L, R, (0.04, 0.06, 0.14), (0, -0.09, -0.07), hpr=(0, 20, 0), color=(0.10, 0.11, 0.12, 1))
            make_sharp_box(L, R, (0.045, 0.22, 0.11), (0, -0.22, 0.03), color=(0.16, 0.17, 0.19, 1))
            make_sharp_box(L, R, (0.04, 0.10, 0.05), (0, 0.02, 0.09), color=(0.12, 0.12, 0.14, 1))
            make_sharp_box(L, R, (0.025, 0.01, 0.025), (0, 0.02, 0.09), color=(0.2, 0.95, 0.3, 0.8))
            self.grip_socket.setPos(0, -0.09, -0.07)
            self.guard_socket.setPos(0, 0.035, 0.01)

        elif w_type == "shotgun":
            # SPAS-12 Tactical Combat Shotgun (Sharp Modular Geometry)
            make_sharp_box(L, R, (0.065, 0.30, 0.08), (0, 0.0, 0.03), color=(0.17, 0.18, 0.20, 1))
            make_sharp_box(L, R, (0.040, 0.48, 0.04), (0, 0.34, 0.04), color=(0.12, 0.12, 0.14, 1))
            make_sharp_box(L, R, (0.038, 0.44, 0.038), (0, 0.32, -0.005), color=(0.10, 0.10, 0.12, 1))
            make_sharp_box(L, R, (0.055, 0.18, 0.055), (0, 0.24, -0.005), color=(0.22, 0.23, 0.25, 1))
            make_sharp_box(L, R, (0.045, 0.08, 0.15), (0, -0.10, -0.07), hpr=(0, 22, 0), color=(0.11, 0.12, 0.13, 1))
            make_sharp_box(L, R, (0.040, 0.26, 0.05), (0, -0.22, 0.06), hpr=(0, -8, 0), color=(0.14, 0.15, 0.16, 1))
            make_sharp_box(L, R, (0.050, 0.08, 0.05), (0, 0.60, 0.04), color=(0.28, 0.29, 0.31, 1))
            self.grip_socket.setPos(0, -0.10, -0.07)
            self.guard_socket.setPos(0, 0.015, 0.0)

    def get_pos(self):
        if hasattr(self, "np") and not self.np.isEmpty():
            p = self.np.getPos()
            self.last_pos = Vec3(p)
            return p
        return getattr(self, "last_pos", Vec3(0, 0, 0))

    def set_pos(self, pos):
        if hasattr(self, "np") and not self.np.isEmpty():
            self.np.setPos(pos)
            self.last_pos = Vec3(pos)

    def set_hpr(self, hpr):
        if hasattr(self, "np") and not self.np.isEmpty():
            self.np.setHpr(hpr)

    def look_at(self, target_point):
        if hasattr(self, "np") and not self.np.isEmpty():
            self.np.lookAt(target_point)

    def set_held(self, held):
        if self.is_held == held: return
        self.is_held = held
        if held:
            if self.world and self.node and self.is_attached_to_world:
                self.world.removeRigidBody(self.node)
                self.is_attached_to_world = False
        else:
            if self.world and self.node and not self.is_attached_to_world:
                self.node.setIntoCollideMask(BitMask32.bit(COLLISION_GROUP_WEAPON))
                self.world.attachRigidBody(self.node)
                self.is_attached_to_world = True
            self.node.setKinematic(False)
            self.node.setActive(True)

    def reload(self):
        needed = self.cfg["mag_size"] - self.ammo_mag
        if needed <= 0 or self.ammo_reserve <= 0:
            return False
        transfer = min(needed, self.ammo_reserve)
        self.ammo_mag += transfer
        self.ammo_reserve -= transfer
        return True

    def add_ammo(self, count):
        self.ammo_reserve += count

    def set_linear_velocity(self, vel):
        if self.node:
            self.node.setActive(True)
            self.node.setLinearVelocity(vel)

    def apply_impulse(self, impulse_vec, hit_pos=None):
        if not self.is_held and not self.has_exploded and self.node:
            self.node.setActive(True)
            self.node.applyCentralImpulse(impulse_vec)
            if hit_pos is not None and hasattr(self, "np") and not self.np.isEmpty():
                body_pos = self.np.getPos()
                torque = calculate_off_center_torque(hit_pos, body_pos, impulse_vec, max_lever_arm=0.15)
                self.node.applyTorqueImpulse(torque)
            ang_v = self.node.getAngularVelocity()
            if ang_v.lengthSquared() > MAX_ANGULAR_VELOCITY * MAX_ANGULAR_VELOCITY:
                self.node.setAngularVelocity(ang_v.normalized() * MAX_ANGULAR_VELOCITY)

    def update_physics(self, dt):
        if self.has_exploded:
            return False
        if not self.is_held and self.node and self.node.isActive():
            vel = self.node.getLinearVelocity()
            ang_vel = self.node.getAngularVelocity()
            new_v, new_w = apply_semi_implicit_drag(vel, ang_vel, dt, AERO_DRAG_COEFF, ROT_DRAG_COEFF, MAX_ANGULAR_VELOCITY)
            self.node.setLinearVelocity(new_v)
            self.node.setAngularVelocity(new_w)
        return True


class SpentCasing:
    """
    Physical Ejected Shell Casing with Bullet Rigid Body Dynamics:
    Spawns from weapon ejection port, tumbles through the air,
    bounces off the ground/walls, and rolls to a rest.
    """
    def __init__(self, world, render, loader, muzzle_pos, fwd, rgt, up, is_shotgun=False):
        self.life = 5.0
        self.world = world
        shape = BulletBoxShape(Vec3(0.015, 0.035, 0.015))
        node = BulletRigidBodyNode("spent_casing")
        node.setMass(0.03)
        node.addShape(shape)
        node.setFriction(0.80)
        node.setRestitution(0.35)
        node.setLinearDamping(0.25)
        node.setAngularDamping(0.50)
        node.setIntoCollideMask(BitMask32.bit(COLLISION_GROUP_DEBRIS))

        self.np = render.attachNewNode(node)
        spawn_pos = muzzle_pos - fwd * 0.25 + rgt * 0.08 + up * 0.04
        self.np.setPos(spawn_pos)
        world.attachRigidBody(node)
        self.node = node
        self.is_attached_to_world = True
        self.is_settled = False
        self.rest_time = 0.0

        # Ejection impulse (arcs right, slightly backward and upward)
        eject_vel = rgt * random.uniform(3.0, 4.5) + up * random.uniform(1.8, 2.8) - fwd * random.uniform(0.4, 1.0)
        node.setLinearVelocity(eject_vel)
        node.setAngularVelocity(Vec3(random.uniform(-15, 15), random.uniform(-20, 20), random.uniform(-15, 15)))

        if loader:
            m = make_sharp_box(loader, self.np, (0.018, 0.045, 0.018), (0, 0, 0),
                               color=(0.85, 0.15, 0.12, 1) if is_shotgun else (0.92, 0.78, 0.28, 1))
            self.mesh = m

    def destroy(self):
        if self.world and self.node and getattr(self, "is_attached_to_world", False):
            self.world.removeRigidBody(self.node)
            self.is_attached_to_world = False
        if hasattr(self, "np") and not self.np.isEmpty():
            self.np.removeNode()

    def update(self, dt):
        self.life -= dt
        if self.life <= 0.0:
            self.destroy()
            return False

        # Physics auto-sleep when resting on floor: freeze as kinematic to eliminate solver CPU and jitter
        if not self.is_settled and self.node:
            lin_spd = self.node.getLinearVelocity().length()
            if lin_spd < 0.50:
                self.rest_time += dt
                if self.rest_time > 0.25:
                    self.is_settled = True
                    self.node.setKinematic(True)
                    self.node.setLinearVelocity(Vec3(0, 0, 0))
                    self.node.setAngularVelocity(Vec3(0, 0, 0))
            else:
                self.rest_time = 0.0

        return True


class ThrowableFragGrenade:
    """
    Tactical Procedural Frag Grenade (adapted from A3P Grenade & Ballistica bomb physics):
    - Physics-simulated tumbling sphere with high bounce restitution.
    - Safety lever & fuse mechanics: 2.2s fuse time.
    - Plays bounce sound upon ground/wall impacts.
    - Triggers catastrophic radial explosion with shockwave, debris impulse, screen trauma, and hitstop.
    """
    def __init__(self, world, render, loader, spawn_pos, throw_vel, sfx_callback=None, explode_callback=None):
        self.world = world
        self.render = render
        self.sfx_callback = sfx_callback
        self.explode_callback = explode_callback
        self.fuse_time = 2.2
        self.has_exploded = False
        self.last_bounce_time = 0.0
        self.is_held = False
        self.is_gun = False
        self.name = "Frag Grenade"
        self.last_pos = Vec3(spawn_pos)

        shape = BulletSphereShape(0.09)
        node = BulletRigidBodyNode("frag_grenade")
        node.setMass(0.45)
        node.addShape(shape)
        node.setFriction(0.55)
        node.setRestitution(0.62)
        node.setLinearDamping(0.12)
        node.setAngularDamping(0.25)
        node.setIntoCollideMask(BitMask32.bit(COLLISION_GROUP_GRENADE))

        self.np = render.attachNewNode(node)
        self.np.setPos(spawn_pos)
        world.attachRigidBody(node)
        self.node = node
        self.is_attached_to_world = True

        # Throw velocity & spin
        node.setLinearVelocity(throw_vel)
        node.setAngularVelocity(Vec3(random.uniform(-10, 10), random.uniform(15, 25), random.uniform(-10, 10)))

        # Procedural 3D fragmentation mesh
        if loader:
            proto_sph = get_cached_model(loader, "models/misc/sphere")
            if proto_sph:
                body = proto_sph.copyTo(self.np)
                body.setScale(0.09)
                body.setColor(0.18, 0.28, 0.16, 1.0)
            make_sharp_box(loader, self.np, (0.04, 0.04, 0.05), (0, 0, 0.08), color=(0.55, 0.52, 0.46, 1.0))
            make_sharp_box(loader, self.np, (0.018, 0.08, 0.015), (0.03, 0.02, 0.06), hpr=(0, 25, 0), color=(0.75, 0.75, 0.78, 1.0))
            if proto_sph:
                ring = proto_sph.copyTo(self.np)
                ring.setScale(0.022, 0.008, 0.022)
                ring.setPos(-0.035, 0, 0.085)
                ring.setColor(0.85, 0.85, 0.20, 1.0)

        self.prev_vel = Vec3(throw_vel)

    def get_pos(self):
        if hasattr(self, "np") and not self.np.isEmpty():
            p = self.np.getPos()
            self.last_pos = Vec3(p)
            return p
        return getattr(self, "last_pos", Vec3(0, 0, 0))

    def update(self, dt):
        if self.has_exploded:
            return False

        self.fuse_time -= dt

        if not hasattr(self, "np") or self.np.isEmpty():
            return False

        # Detect bounce impacts: abrupt velocity change or ground contact
        cur_vel = self.node.getLinearVelocity() if self.node else Vec3(0, 0, 0)
        accel = (cur_vel - self.prev_vel).length()
        pos = self.get_pos()
        if accel > 3.8 and (pos.z < 0.35 or accel > 6.5):
            if self.sfx_callback and (self.fuse_time < 2.12):
                self.sfx_callback("grenade-bounce")
        self.prev_vel = Vec3(cur_vel)

        if self.fuse_time <= 0.0 or pos.z < -10.0:
            self.detonate()
            return False
        return True

    def detonate(self):
        if self.has_exploded:
            return
        self.has_exploded = True
        pos = self.get_pos()
        self.last_pos = Vec3(pos)

        if self.world and self.node and getattr(self, "is_attached_to_world", False):
            self.world.removeRigidBody(self.node)
            self.is_attached_to_world = False
        if hasattr(self, "np") and not self.np.isEmpty():
            self.np.removeNode()

        if self.explode_callback:
            self.explode_callback(pos, radius=7.0, max_force=60.0, trauma=0.55, source_obj=self)


class ExplosiveBarrel:
    """
    Hazard Explosive Fuel Barrel (adapted from A3P explosive entities):
    - Interactive physics prop: can be shot, pushed, or carried and hurled.
    - Taking bullet damage or strong kinetic trauma causes it to ignite, hiss, and detonate!
    - Radial chain-reaction explosion that violently launches surrounding objects.
    - Crash-proof: safe node teardown, holder notification, and empty-nodepath guards.
    """
    def __init__(self, world, render, loader, pos, mass=12.0):
        self.world = world
        self.render = render
        self.loader = loader
        self.is_held = False
        self.is_gun = False
        self.is_barrel = True
        self.holder = None
        self.name = "Explosive Barrel"
        self.health = 25.0
        self.is_ignited = False
        self.fuse_timer = 0.35
        self.has_exploded = False
        self.explode_callback = None
        self.sfx_callback = None
        self.release_grace_timer = 0.0
        self.last_pos = Vec3(*pos)

        radius = 0.28
        height = 0.72
        shape = BulletCylinderShape(radius, height, ZUp)
        node = BulletRigidBodyNode("explosive_barrel")
        node.setMass(mass)
        node.addShape(shape)
        node.setFriction(0.60)
        node.setRestitution(0.25)
        node.setLinearDamping(0.15)
        node.setAngularDamping(0.30)
        node.setIntoCollideMask(BitMask32.bit(COLLISION_GROUP_PROP))

        self.np = render.attachNewNode(node)
        self.np.setPos(*pos)
        world.attachRigidBody(node)
        self.node = node
        self.is_attached_to_world = True

        self.meshes = []
        self.base_color = (0.85, 0.18, 0.12, 1.0)
        if loader:
            core = make_sharp_box(loader, self.np, (radius*1.8, radius*1.8, height), (0, 0, 0), color=self.base_color)
            core2 = make_sharp_box(loader, self.np, (radius*1.8, radius*1.8, height), (0, 0, 0), hpr=(45, 0, 0), color=self.base_color)
            band1 = make_sharp_box(loader, self.np, (radius*1.85, radius*1.85, height*0.22), (0, 0, 0), color=(0.96, 0.82, 0.14, 1.0))
            band2 = make_sharp_box(loader, self.np, (radius*1.85, radius*1.85, height*0.22), (0, 0, 0), hpr=(45, 0, 0), color=(0.96, 0.82, 0.14, 1.0))
            rim_top = make_sharp_box(loader, self.np, (radius*1.88, radius*1.88, 0.05), (0, 0, height*0.46), color=(0.18, 0.18, 0.18, 1.0))
            rim_bot = make_sharp_box(loader, self.np, (radius*1.88, radius*1.88, 0.05), (0, 0, -height*0.46), color=(0.18, 0.18, 0.18, 1.0))
            self.meshes.extend([core, core2, band1, band2, rim_top, rim_bot])

    def get_pos(self):
        if hasattr(self, "np") and not self.np.isEmpty():
            p = self.np.getPos()
            self.last_pos = Vec3(p)
            return p
        return getattr(self, "last_pos", Vec3(0, 0, 0))

    def set_pos(self, pos):
        if hasattr(self, "np") and not self.np.isEmpty():
            self.np.setPos(pos)
            self.last_pos = Vec3(pos)

    def set_callbacks(self, explode_callback, sfx_callback):
        self.explode_callback = explode_callback
        self.sfx_callback = sfx_callback

    def set_held(self, held, holder=None):
        if self.is_held == held: return
        self.is_held = held
        self.holder = holder if held else None
        if held:
            if self.world and self.node and self.is_attached_to_world:
                self.world.removeRigidBody(self.node)
                self.is_attached_to_world = False
        else:
            if self.world and self.node and not self.is_attached_to_world:
                self.node.setIntoCollideMask(BitMask32.bit(COLLISION_GROUP_HELD))
                self.world.attachRigidBody(self.node)
                self.is_attached_to_world = True
            self.release_grace_timer = THROW_GRACE_TIME
            self.node.setKinematic(False)
            self.node.setActive(True)

    def set_linear_velocity(self, vel):
        if self.node:
            self.node.setActive(True)
            self.node.setLinearVelocity(vel)

    def apply_impulse(self, impulse_vec, hit_pos=None):
        if not self.is_held and not self.has_exploded and self.node:
            self.node.setActive(True)
            self.node.applyCentralImpulse(impulse_vec)
            if hit_pos is not None and hasattr(self, "np") and not self.np.isEmpty():
                body_pos = self.np.getPos()
                torque = calculate_off_center_torque(hit_pos, body_pos, impulse_vec, max_lever_arm=0.25)
                self.node.applyTorqueImpulse(torque)
            ang_v = self.node.getAngularVelocity()
            if ang_v.lengthSquared() > MAX_ANGULAR_VELOCITY * MAX_ANGULAR_VELOCITY:
                self.node.setAngularVelocity(ang_v.normalized() * MAX_ANGULAR_VELOCITY)

    def take_damage(self, amount, hit_pos=None):
        if self.has_exploded:
            return
        self.health -= amount
        if self.health <= 0.0 and not self.is_ignited:
            self.is_ignited = True
            if self.sfx_callback:
                self.sfx_callback("punch_hit")

    def update_physics(self, dt):
        if self.has_exploded:
            return False

        if self.release_grace_timer > 0.0:
            self.release_grace_timer -= dt
            if self.release_grace_timer <= 0.0 and self.node:
                self.node.setIntoCollideMask(BitMask32.bit(COLLISION_GROUP_PROP))

        if self.is_ignited:
            self.fuse_timer -= dt
            flash_state = int(self.fuse_timer * 22) % 2 == 0
            color = (1.0, 0.95, 0.4, 1.0) if flash_state else (1.0, 0.2, 0.1, 1.0)
            for m in self.meshes:
                if not m.isEmpty():
                    m.setColor(*color)

            if self.fuse_timer <= 0.0:
                self.detonate()
                return False

        if not self.is_held and self.node and self.node.isActive():
            vel = self.node.getLinearVelocity()
            ang_vel = self.node.getAngularVelocity()
            new_v, new_w = apply_semi_implicit_drag(vel, ang_vel, dt, AERO_DRAG_COEFF, ROT_DRAG_COEFF, MAX_ANGULAR_VELOCITY)
            self.node.setLinearVelocity(new_v)
            self.node.setAngularVelocity(new_w)
        return True

    def detonate(self):
        if self.has_exploded:
            return
        self.has_exploded = True
        pos = self.get_pos()
        self.last_pos = Vec3(pos)

        # If currently held by the puppet, safely detach before removing nodes
        if self.holder and hasattr(self.holder, "on_held_prop_destroyed"):
            self.holder.on_held_prop_destroyed(self)
        self.holder = None

        if self.world and self.node and getattr(self, "is_attached_to_world", False):
            self.world.removeRigidBody(self.node)
            self.is_attached_to_world = False
        if hasattr(self, "np") and not self.np.isEmpty():
            self.np.removeNode()

        if self.explode_callback:
            self.explode_callback(pos, radius=8.5, max_force=75.0, trauma=0.75, source_obj=self)


