# props.py
# Sharp, Realistic 3D Procedural Weapons & Physics Props
from panda3d.core import Vec3, Point3
from panda3d.bullet import BulletRigidBodyNode, BulletBoxShape, BulletCapsuleShape, ZUp


class InteractiveCrate:
    def __init__(self, world, render, loader, pos, size=(0.28, 0.28, 0.28), mass=1.4):
        self.world = world
        self.render = render
        self.is_held = False
        self.is_gun = False
        self.name = "Wooden Crate"

        shape = BulletBoxShape(Vec3(size[0], size[1], size[2]))
        node  = BulletRigidBodyNode("crate")
        node.setMass(mass)
        node.addShape(shape)
        node.setFriction(0.65)
        node.setRestitution(0.3)
        node.setLinearDamping(0.15)
        node.setAngularDamping(0.35)

        self.np = render.attachNewNode(node)
        self.np.setPos(*pos)
        world.attachRigidBody(node)
        self.node = node

        if loader:
            m = loader.loadModel("models/misc/sphere")
            m.setScale(size[0] * 1.4, size[1] * 1.4, size[2] * 1.4)
            m.setColor(0.68, 0.46, 0.24, 1)
            m.reparentTo(self.np)

    def get_pos(self): return self.np.getPos()
    def set_pos(self, pos): self.np.setPos(pos)

    def set_held(self, held):
        if self.is_held == held: return
        self.is_held = held
        if held:
            if self.world:
                self.world.removeRigidBody(self.node)
        else:
            if self.world:
                self.world.attachRigidBody(self.node)
            self.node.setKinematic(False)
            self.node.setActive(True)

    def set_linear_velocity(self, vel):
        self.node.setActive(True)
        self.node.setLinearVelocity(vel)

    def apply_impulse(self, impulse_vec):
        if not self.is_held:
            self.node.setActive(True)
            self.node.applyCentralImpulse(impulse_vec)


class BowlingPin:
    def __init__(self, world, render, loader, pos):
        self.world = world
        self.render = render
        self.is_held = False
        self.is_gun = False
        self.name = "Bowling Pin"

        shape = BulletCapsuleShape(0.10, 0.35, ZUp)
        node = BulletRigidBodyNode("pin")
        node.setMass(0.8)
        node.addShape(shape)
        node.setFriction(0.4)
        node.setRestitution(0.5)

        self.np = render.attachNewNode(node)
        self.np.setPos(*pos)
        world.attachRigidBody(node)
        self.node = node

        if loader:
            m = loader.loadModel("models/misc/sphere")
            m.setScale(0.12, 0.12, 0.30)
            m.setColor(0.95, 0.95, 0.95, 1)
            m.reparentTo(self.np)

    def get_pos(self): return self.np.getPos()
    def set_pos(self, pos): self.np.setPos(pos)

    def set_held(self, held):
        if self.is_held == held: return
        self.is_held = held
        if held:
            if self.world:
                self.world.removeRigidBody(self.node)
        else:
            if self.world:
                self.world.attachRigidBody(self.node)
            self.node.setKinematic(False)
            self.node.setActive(True)

    def set_linear_velocity(self, vel):
        self.node.setActive(True)
        self.node.setLinearVelocity(vel)

    def apply_impulse(self, impulse_vec):
        if not self.is_held:
            self.node.setActive(True)
            self.node.applyCentralImpulse(impulse_vec)


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

        self.ammo_mag = self.cfg["mag_size"]
        self.ammo_reserve = self.cfg["reserve_ammo"]

        shape = BulletBoxShape(Vec3(0.20, 0.10, 0.12))
        node = BulletRigidBodyNode(f"gun_{weapon_type}")
        node.setMass(1.0)
        node.addShape(shape)
        node.setFriction(0.6)
        node.setRestitution(0.2)

        self.np = render.attachNewNode(node)
        self.np.setPos(*pos)
        world.attachRigidBody(node)
        self.node = node

        if loader:
            self._build_sharp_weapon_model(weapon_type)

    def _build_sharp_weapon_model(self, w_type):
        L = self.loader
        R = self.np

        if w_type == "pistol":
            slide = L.loadModel("models/misc/sphere")
            slide.setScale(0.065, 0.22, 0.065)
            slide.setColor(0.16, 0.18, 0.22, 1)
            slide.setPos(0, 0.04, 0.03)
            slide.reparentTo(R)

            barrel = L.loadModel("models/misc/sphere")
            barrel.setScale(0.045, 0.16, 0.045)
            barrel.setColor(0.10, 0.11, 0.13, 1)
            barrel.setPos(0, 0.15, 0.03)
            barrel.reparentTo(R)

            grip = L.loadModel("models/misc/sphere")
            grip.setScale(0.055, 0.07, 0.14)
            grip.setColor(0.08, 0.08, 0.10, 1)
            grip.setPos(0, -0.04, -0.06)
            grip.setHpr(0, 15, 0)
            grip.reparentTo(R)

            diode = L.loadModel("models/misc/sphere")
            diode.setScale(0.022, 0.06, 0.022)
            diode.setColor(0.20, 0.88, 0.98, 1)
            diode.setPos(0, 0.12, -0.01)
            diode.reparentTo(R)

            sight = L.loadModel("models/misc/sphere")
            sight.setScale(0.018, 0.02, 0.025)
            sight.setColor(0.95, 0.95, 0.95, 1)
            sight.setPos(0, 0.18, 0.065)
            sight.reparentTo(R)

        elif w_type == "rifle":
            rec = L.loadModel("models/misc/sphere")
            rec.setScale(0.07, 0.28, 0.08)
            rec.setColor(0.18, 0.20, 0.24, 1)
            rec.setPos(0, 0.0, 0.02)
            rec.reparentTo(R)

            barrel = L.loadModel("models/misc/sphere")
            barrel.setScale(0.04, 0.32, 0.04)
            barrel.setColor(0.10, 0.11, 0.13, 1)
            barrel.setPos(0, 0.24, 0.02)
            barrel.reparentTo(R)

            brake = L.loadModel("models/misc/sphere")
            brake.setScale(0.05, 0.08, 0.05)
            brake.setColor(0.25, 0.28, 0.32, 1)
            brake.setPos(0, 0.40, 0.02)
            brake.reparentTo(R)

            mag = L.loadModel("models/misc/sphere")
            mag.setScale(0.045, 0.09, 0.18)
            mag.setColor(0.12, 0.12, 0.14, 1)
            mag.setPos(0, 0.06, -0.10)
            mag.setHpr(0, -18, 0)
            mag.reparentTo(R)

            stock = L.loadModel("models/misc/sphere")
            stock.setScale(0.055, 0.18, 0.10)
            stock.setColor(0.14, 0.15, 0.18, 1)
            stock.setPos(0, -0.22, 0.0)
            stock.reparentTo(R)

            holo = L.loadModel("models/misc/sphere")
            holo.setScale(0.045, 0.09, 0.05)
            holo.setColor(0.98, 0.60, 0.12, 1)
            holo.setPos(0, 0.02, 0.085)
            holo.reparentTo(R)

        elif w_type == "shotgun":
            rec = L.loadModel("models/misc/sphere")
            rec.setScale(0.09, 0.26, 0.10)
            rec.setColor(0.12, 0.13, 0.15, 1)
            rec.setPos(0, -0.02, 0.02)
            rec.reparentTo(R)

            b1 = L.loadModel("models/misc/sphere")
            b1.setScale(0.06, 0.28, 0.06)
            b1.setColor(0.08, 0.09, 0.10, 1)
            b1.setPos(0, 0.22, 0.04)
            b1.reparentTo(R)

            b2 = L.loadModel("models/misc/sphere")
            b2.setScale(0.05, 0.26, 0.05)
            b2.setColor(0.14, 0.15, 0.18, 1)
            b2.setPos(0, 0.20, -0.01)
            b2.reparentTo(R)

            pump = L.loadModel("models/misc/sphere")
            pump.setScale(0.075, 0.14, 0.075)
            pump.setColor(0.22, 0.24, 0.28, 1)
            pump.setPos(0, 0.14, 0.01)
            pump.reparentTo(R)

            glow = L.loadModel("models/misc/sphere")
            glow.setScale(0.045, 0.07, 0.045)
            glow.setColor(0.95, 0.18, 0.15, 1)
            glow.setPos(0.04, 0.0, 0.03)
            glow.reparentTo(R)

    def get_pos(self): return self.np.getPos()
    def set_pos(self, pos): self.np.setPos(pos)
    def set_hpr(self, hpr): self.np.setHpr(hpr)
    def look_at(self, target_point): self.np.lookAt(target_point)

    def set_held(self, held):
        if self.is_held == held: return
        self.is_held = held
        if held:
            if self.world:
                self.world.removeRigidBody(self.node)
        else:
            if self.world:
                self.world.attachRigidBody(self.node)
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
        self.node.setActive(True)
        self.node.setLinearVelocity(vel)

    def apply_impulse(self, impulse_vec):
        if not self.is_held:
            self.node.setActive(True)
            self.node.applyCentralImpulse(impulse_vec)
