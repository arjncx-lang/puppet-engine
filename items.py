# items.py
# Physics interactive items: Bombs, Explosions, and Stackable Wooden Crates
import math
from panda3d.core import Vec3, Vec4, Point3, CardMaker
from panda3d.bullet import (BulletRigidBodyNode, BulletSphereShape,
                              BulletBoxShape)
from physics_constants import *


class PhysicsCrate:
    """Interactive wooden crate on the arena floor."""
    def __init__(self, world, render, loader, pos, size=(0.3, 0.3, 0.3), mass=1.2):
        self.world = world
        self.render = render

        shape = BulletBoxShape(Vec3(size[0], size[1], size[2]))
        node  = BulletRigidBodyNode("crate")
        node.setMass(mass)
        node.addShape(shape)
        node.setFriction(0.6)
        node.setLinearDamping(0.2)
        node.setAngularDamping(0.4)

        self.np = render.attachNewNode(node)
        self.np.setPos(*pos)
        world.attachRigidBody(node)
        self.node = node

        # Visual cube model
        if loader:
            m = loader.loadModel("models/misc/sphere")  # placeholder, scale into box
            m.setScale(size[0]*1.4, size[1]*1.4, size[2]*1.4)
            m.setColor(0.65, 0.45, 0.22, 1)  # wooden brown
            m.reparentTo(self.np)

    def get_pos(self):
        return self.np.getPos()

    def apply_impulse(self, impulse_vec):
        self.node.setActive(True)
        self.node.applyCentralImpulse(impulse_vec)


class Bomb:
    """
    BombSquad signature black round bomb with ticking fuse and radial blast.
    """
    def __init__(self, world, render, loader, spawn_pos, throw_vel):
        self.world      = world
        self.render     = render
        self.loader     = loader
        self.fuse_time  = BOMB_FUSE_TIME
        self.exploded   = False
        self.flash_time = 0.0

        shape = BulletSphereShape(BOMB_RADIUS)
        node  = BulletRigidBodyNode("bomb")
        node.setMass(1.5)
        node.addShape(shape)
        node.setFriction(0.7)
        node.setRestitution(0.4)  # Bouncy!
        node.setLinearDamping(0.1)

        self.np = render.attachNewNode(node)
        self.np.setPos(*spawn_pos)
        world.attachRigidBody(node)
        self.node = node

        # Apply initial throwing velocity
        node.setLinearVelocity(throw_vel)

        # Bomb visual (black sphere + red fuse top)
        if loader:
            self.vis = loader.loadModel("models/misc/sphere")
            self.vis.setScale(BOMB_RADIUS * 2.0)
            self.vis.setColor(0.12, 0.12, 0.14, 1)
            self.vis.reparentTo(self.np)

            # Glowing fuse cap
            self.fuse_vis = loader.loadModel("models/misc/sphere")
            self.fuse_vis.setScale(BOMB_RADIUS * 0.5)
            self.fuse_vis.setColor(1.0, 0.2, 0.1, 1)
            self.fuse_vis.setPos(0, 0, BOMB_RADIUS * 0.8)
            self.fuse_vis.reparentTo(self.np)
        else:
            self.vis = None
            self.fuse_vis = None

        # Explosion flash node
        self.flash_np = None

    def update(self, dt, all_crates, spaz):
        if self.exploded:
            if self.flash_np:
                self.flash_time -= dt
                if self.flash_time > 0:
                    scale = (1.0 - (self.flash_time / 0.25)) * BLAST_RADIUS * 1.5
                    self.flash_np.setScale(scale)
                    alpha = self.flash_time / 0.25
                    self.flash_np.setColor(1.0, 0.6, 0.1, alpha)
                else:
                    self.flash_np.removeNode()
                    self.flash_np = None
            return False  # Done

        self.fuse_time -= dt

        # Fuse blinking faster as it approaches 0
        if self.fuse_vis:
            blink_rate = 8.0 + (BOMB_FUSE_TIME - self.fuse_time) * 12.0
            blink = (math.sin(self.fuse_time * blink_rate) > 0.0)
            if blink:
                self.fuse_vis.setColor(1.0, 0.9, 0.2, 1)  # bright yellow spark
            else:
                self.fuse_vis.setColor(1.0, 0.1, 0.1, 1)  # red

        if self.fuse_time <= 0.0:
            self.detonate(all_crates, spaz)
            return True

        return True

    def detonate(self, all_crates, spaz):
        self.exploded = True
        b_pos = self.np.getPos()

        # Visual explosion fireball flash
        if self.loader:
            self.flash_np = self.render.attachNewNode(self.loader.loadModel("models/misc/sphere").node())
            self.flash_np.setPos(b_pos)
            self.flash_np.setScale(BLAST_RADIUS * 0.5)
            self.flash_np.setColor(1.0, 0.7, 0.1, 1.0)
            self.flash_time = 0.25

        # Blast impulse on crates
        for crate in all_crates:
            c_pos = crate.get_pos()
            diff = c_pos - b_pos
            dist = diff.length()
            if dist < BLAST_RADIUS and dist > 0.01:
                dir_vec = diff.normalized()
                power = BLAST_FORCE * (1.0 - (dist / BLAST_RADIUS))
                impulse = (dir_vec + Vec3(0, 0, 0.6)) * power
                crate.apply_impulse(impulse)

        # Blast impulse on Spaz character
        spaz_pos = spaz.get_torso_pos()
        diff = spaz_pos - b_pos
        dist = diff.length()
        if dist < BLAST_RADIUS and dist > 0.01:
            dir_vec = diff.normalized()
            power = BLAST_FORCE * (1.0 - (dist / BLAST_RADIUS))
            impulse = (dir_vec + Vec3(0, 0, 0.5)) * (power * 0.8)
            for b in spaz._bodies.values():
                b.setActive(True)
                b.applyCentralImpulse(impulse)

        # Remove physical bomb body from world
        self.world.removeRigidBody(self.node)
        if self.vis:
            self.vis.removeNode()
        if self.fuse_vis:
            self.fuse_vis.removeNode()
