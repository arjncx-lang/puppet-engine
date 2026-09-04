# main.py  -  PuppetEngine: Procedural Physics & Tactical TPS Action Sandbox
# Controls:
#   Mouse Move         : Free Look (360° Horizontal + Full Vertical Pitch)
#   Left Click (Hold)  : Continuous Full-Auto (Rifle) / Semi-Auto (Pistol/Shotgun)
#   Right Click / E    : Pick Up Weapon or Crate / Take Ammo
#   Mouse Scroll       : Cycle Weapons (Forward/Backward)
#   + / - Buttons      : Cycle Weapons (Next/Previous)
#   1, 2, 3            : Direct Weapon Select (1: Pistol, 2: Rifle, 3: Shotgun)
#   V Key              : Toggle 3 Camera Zoom Angles (Close, Normal, Wide)
#   R                  : Reload Active Weapon
#   WASD / Arrows      : Run & Tactical Strafe
#   SHIFT              : Sprint Boost
#   SPACE              : Jump
#   H                  : Toggle Ice Mode
#   TAB                : Toggle Mouse Cursor Lock
#   ESC                : Quit
import sys, math, random, pathlib
from direct.showbase.ShowBase import ShowBase
from panda3d.core import (Vec3, Vec4, Point3, Filename, DirectionalLight, AmbientLight,
                           PointLight, CardMaker, LineSegs, AntialiasAttrib,
                           loadPrcFileData, TextNode, KeyboardButton,
                           WindowProperties, MouseButton)
from panda3d.bullet import BulletWorld, BulletPlaneShape, BulletRigidBodyNode
from direct.gui.OnscreenText import OnscreenText

from physics_constants import *
from physics_math import (SpringDamper1D, SpringDamper3D, CameraTraumaSystem,
                          calculate_off_center_torque, HitStopManager,
                          calculate_ricochet_reflection, calculate_radial_explosion_impulse)
from character import PuppetCharacter, box_normalize_to_circle
from props import (InteractiveCrate, BowlingPin, GunWeapon, SpentCasing,
                   ThrowableFragGrenade, ExplosiveBarrel)

loadPrcFileData("", "window-title PuppetEngine - Pure Python TPS & Physics Sandbox")
loadPrcFileData("", "win-size 1280 720")
loadPrcFileData("", "sync-video 1")


class BulletTracer:
    def __init__(self, render, p_from, p_to):
        self.life = 0.07
        ls = LineSegs()
        ls.setColor(1.0, 0.75, 0.20, 1.0)
        ls.setThickness(3.0)
        ls.moveTo(p_from)
        ls.drawTo(p_to)
        self.np = render.attachNewNode(ls.create())

    def update(self, dt):
        self.life -= dt
        if self.life <= 0:
            self.np.removeNode()
            return False
        return True


class MuzzleFlash:
    def __init__(self, render, loader, pos):
        self.life = 0.04
        self.np = render.attachNewNode(loader.loadModel("models/misc/sphere").node())
        self.np.setPos(pos)
        self.np.setScale(0.12)
        self.np.setColor(1.0, 0.90, 0.30, 1.0)

    def update(self, dt):
        self.life -= dt
        if self.life <= 0:
            self.np.removeNode()
            return False
        return True


class DustPuff:
    def __init__(self, render, loader, pos):
        self.life = 0.35
        self.max_life = 0.35
        self.np = render.attachNewNode(loader.loadModel("models/misc/sphere").node())
        self.np.setPos(*pos)
        self.np.setScale(0.08)
        self.np.setColor(0.9, 0.9, 0.9, 0.6)

    def update(self, dt):
        self.life -= dt
        if self.life <= 0:
            self.np.removeNode()
            return False
        progress = 1.0 - (self.life / self.max_life)
        scale = 0.08 + progress * 0.22
        alpha = (self.life / self.max_life) * 0.6
        self.np.setScale(scale)
        self.np.setColor(0.9, 0.9, 0.9, alpha)
        return True


class KineticSpark:
    """
    Directional Kinetic Spark (adapted from A3P SparkParticleGroup):
    Arcs off bullet impacts and explosions along the reflection vector,
    drawn as a fast-moving glowing line segment under gravity.
    """
    def __init__(self, render, pos, vel, life=0.25):
        self.render = render
        self.pos = Point3(pos)
        self.vel = Vec3(vel)
        self.life = life
        self.max_life = life
        self.ls = LineSegs("spark")
        self.ls.setColor(1.0, 0.85, 0.35, 1.0)
        self.ls.setThickness(2.0)
        self.ls.moveTo(self.pos)
        self.ls.drawTo(self.pos - self.vel * 0.015)
        self.np = render.attachNewNode(self.ls.create())

    def update(self, dt):
        self.life -= dt
        if self.life <= 0:
            self.np.removeNode()
            return False
        # Gravity on sparks
        self.vel.z -= 28.0 * dt
        new_pos = self.pos + self.vel * dt
        # Bounce off floor if hitting ground
        if new_pos.z < 0.01:
            new_pos.z = 0.01
            self.vel.z = -self.vel.z * 0.35
            self.vel.x *= 0.65
            self.vel.y *= 0.65
        self.pos = new_pos

        self.np.removeNode()
        self.ls.reset()
        alpha = max(0.0, self.life / self.max_life)
        self.ls.setColor(1.0, 0.82 * alpha + 0.15, 0.20 * alpha, alpha)
        self.ls.setThickness(2.0)
        self.ls.moveTo(self.pos)
        self.ls.drawTo(self.pos - self.vel * 0.02)
        self.np = self.render.attachNewNode(self.ls.create())
        return True


class ExplosionFireball:
    """
    Expanding Kinetic Fireball (adapted from A3P ExplosionParticleGroup):
    Rapidly expands from blast center, shifting from white-hot core to fiery orange,
    then fading to dark smoke.
    """
    def __init__(self, render, loader, pos, radius=1.8):
        self.render = render
        self.life = 0.45
        self.max_life = 0.45
        self.max_radius = radius
        self.np = render.attachNewNode(loader.loadModel("models/misc/sphere").node())
        self.np.setPos(pos)
        self.np.setScale(0.2)
        self.np.setColor(1.0, 0.95, 0.8, 1.0)

    def update(self, dt):
        self.life -= dt
        if self.life <= 0:
            self.np.removeNode()
            return False
        progress = 1.0 - (self.life / self.max_life)
        # Expansion curve
        scale = 0.2 + (self.max_radius - 0.2) * (progress ** 0.6)
        self.np.setScale(scale)
        # Color transition: white-hot -> fiery orange -> dark smoke
        if progress < 0.3:
            r = 1.0; g = 0.95 - progress * 1.5; b = 0.6 - progress * 1.5; a = 0.95
        else:
            p2 = (progress - 0.3) / 0.7
            r = 0.9 * (1.0 - p2) + 0.15 * p2
            g = 0.4 * (1.0 - p2) + 0.15 * p2
            b = 0.1 * (1.0 - p2) + 0.15 * p2
            a = max(0.0, 0.95 * (1.0 - p2))
        self.np.setColor(r, g, b, a)
        return True


class DynamicFlashLight:
    """
    Zero-Hitch Dynamic Flash Point Light (adapted from A3P Light pooling):
    Reuses a persistent PointLight attached to render. Flash events trigger
    instant intensity leaps with exponential decay without causing Panda3D
    to recompile runtime GLSL shaders.
    """
    def __init__(self, render):
        self.plight = PointLight("dynamic_flash_light")
        self.plight.setColor(Vec4(0, 0, 0, 1))
        self.plight.setAttenuation(Vec3(0, 0, 0.05))
        self.np = render.attachNewNode(self.plight)
        render.setLight(self.np)
        self.intensity = 0.0
        self.base_color = Vec4(1.0, 0.75, 0.3, 1)

    def trigger(self, pos, color=Vec4(1.0, 0.8, 0.35, 1), intensity=1.0):
        self.np.setPos(pos)
        self.base_color = color
        self.intensity = intensity
        self.plight.setColor(self.base_color * self.intensity)

    def update(self, dt):
        if self.intensity > 0.001:
            self.intensity = max(0.0, self.intensity - dt * 9.5)
            self.plight.setColor(self.base_color * self.intensity)
        elif self.intensity != 0.0:
            self.intensity = 0.0
            self.plight.setColor(Vec4(0, 0, 0, 1))


class PuppetEngine(ShowBase):

    def __init__(self):
        super().__init__()
        self.disableMouse()
        self.setBackgroundColor(0.14, 0.17, 0.24, 1)
        self.render.setAntialias(AntialiasAttrib.MAuto)
        self.render.setShaderAuto()

        self.bullet = BulletWorld()
        self.bullet.setGravity(Vec3(0, 0, GRAVITY))

        self._make_ground()
        self._make_lights()

        self.sounds = {}
        sfx_dir = pathlib.Path(__file__).parent / "sfx"
        sound_names = (
            "punch_whoosh", "punch_hit", "jump", "throw", "stun",
            "pistol_shot", "rifle_shot", "shotgun_shot", "reload", "gun_pickup",
            "ricochet1", "ricochet2", "ricochet3", "grenade", "grenade-bounce",
            "large-explosion", "large-explosion2"
        )
        for sfx_name in sound_names:
            for ext in (".wav", ".ogg"):
                p = (sfx_dir / f"{sfx_name}{ext}").resolve()
                if p.exists():
                    fn = Filename.fromOsSpecific(str(p))
                    snd = self.loader.loadSfx(fn)
                    if snd:
                        self.sounds[sfx_name] = snd
                        break

        # ── Puppet Character ──
        self.puppet = PuppetCharacter(self.bullet, self.render, self.loader, (0, 0, 0))
        self.puppet.set_dust_callback(self.spawn_dust)
        self.puppet.set_sfx_callback(self.play_sfx)
        self.puppet.set_shoot_callback(self._execute_bullet_fire)
        self.puppet.set_casing_callback(self._eject_casing)
        self.puppet.set_grenade_callback(self._spawn_grenade)

        self.props = []
        self._spawn_props()

        self.dust_puffs = []
        self.tracers    = []
        self.flashes    = []
        self.casings    = []
        self.sparks     = []
        self.fireballs  = []
        self.grenades   = []

        cm = CardMaker("shadow")
        cm.setFrame(-0.35, 0.35, -0.35, 0.35)
        self.shadow_np = self.render.attachNewNode(cm.generate())
        self.shadow_np.setP(-90)
        self.shadow_np.setColor(0.08, 0.16, 0.08, 0.6)
        self.shadow_np.setPos(0, 0, 0.004)

        # 3-Stage Camera Zoom Presets (V Key)
        self.cam_zoom_presets = [4.2, 7.0, 11.5]
        self.cam_zoom_index   = 1
        self.cam_target_dist  = self.cam_zoom_presets[self.cam_zoom_index]
        self.cam_dist         = self.cam_target_dist

        self.cam_yaw          = 180.0
        self.cam_pitch        = 15.0
        self.shoulder_x       = 0.0
        self.mouse_locked     = True
        self.current_3d_target = Point3(0, 10, 1)

        # Exact 2nd-order camera spring-damper stabilizers & Non-linear Trauma Shake
        self.cam_pos_spring  = SpringDamper3D((0.0, 0.0, 0.0), omega=SPRING_OMEGA_CAM, zeta=SPRING_ZETA_CAM)
        self.cam_fov_spring  = SpringDamper1D(CAM_FOV_BASE, omega=12.0, zeta=1.0)
        self.trauma_system   = CameraTraumaSystem(decay_rate=1.8, max_yaw=2.2, max_pitch=2.8, max_roll=1.6)
        self.hit_stop        = HitStopManager()

        self.cam_pivot = self.render.attachNewNode("cam_pivot")
        self.cam_pitch_pivot = self.cam_pivot.attachNewNode("cam_pitch_pivot")
        self.camera.reparentTo(self.cam_pitch_pivot)
        self.camera.setPos(0, -self.cam_dist, 0.20)
        self.camera.lookAt(Point3(0, 0, 0.20))

        self._set_mouse_lock(True)
        self._bind_actions()

        self.crosshair = OnscreenText(
            text="+", pos=(0, 0.005), scale=0.065, fg=(1, 1, 0.2, 0.95),
            align=TextNode.ACenter, mayChange=True)
        self.crosshair.hide()

        self.prompt_text = OnscreenText(
            text="", pos=(0, -0.22), scale=0.046, fg=(1.0, 0.95, 0.3, 1.0),
            shadow=(0, 0, 0, 0.8), align=TextNode.ACenter, mayChange=True)

        self.hud_ammo = OnscreenText(
            text="[UNARMED] (PUNCH)", pos=(0.90, -0.86), scale=0.048, fg=(1, 1, 1, 0.95),
            shadow=(0, 0, 0, 0.8), align=TextNode.ARight, mayChange=True)

        self.hud_slots = OnscreenText(
            text="[1] Pistol  |  [2] Assault Rifle  |  [3] Scatter Shotgun",
            pos=(0, -0.88), scale=0.036, fg=(0.7, 0.85, 1.0, 0.90),
            align=TextNode.ACenter, mayChange=True)

        OnscreenText(
            text="Mouse: Look | LMB: Shoot/Punch | G: Frag Grenade | Wheel / +/-: Switch Gun | V: Zoom | R: Reload",
            pos=(0, -0.95), scale=0.034, fg=(0.9, 0.9, 0.9, 0.85),
            align=TextNode.ACenter, mayChange=False)

        self.taskMgr.add(self._update, "update")

    def _set_mouse_lock(self, lock):
        self.mouse_locked = lock
        props = WindowProperties()
        props.setCursorHidden(lock)
        if hasattr(self.win, "requestProperties"):
            self.win.requestProperties(props)

    def play_sfx(self, name):
        if name in self.sounds:
            snd = self.sounds[name]
            snd.setPlayRate(random.uniform(0.96, 1.04))
            snd.play()

    def spawn_dust(self, pos):
        puff = DustPuff(self.render, self.loader, pos)
        self.dust_puffs.append(puff)

    def _cycle_camera_zoom(self):
        self.cam_zoom_index = (self.cam_zoom_index + 1) % len(self.cam_zoom_presets)
        self.cam_target_dist = self.cam_zoom_presets[self.cam_zoom_index]

    def _eject_casing(self, muzzle_pos, fwd, is_shotgun):
        rgt = Vec3(fwd.y, -fwd.x, 0)
        up  = Vec3(0, 0, 1)
        casing = SpentCasing(self.bullet, self.render, self.loader, muzzle_pos, fwd, rgt, up, is_shotgun)
        self.casings.append(casing)

    def _execute_bullet_fire(self, muzzle_pos, target_3d_point, bullet_force):
        p_from = Point3(muzzle_pos.x, muzzle_pos.y, muzzle_pos.z)
        bullet_diff = target_3d_point - p_from
        if bullet_diff.lengthSquared() < 0.001:
            bullet_dir = Vec3(0, 1, 0)
        else:
            bullet_dir = bullet_diff.normalized()
        p_to = Point3(p_from + bullet_dir * 90.0)

        self.flashes.append(MuzzleFlash(self.render, self.loader, p_from))
        if hasattr(self, "flash_light"):
            self.flash_light.trigger(p_from, Vec4(1.0, 0.85, 0.35, 1), intensity=1.5)

        active_gun = self.puppet.get_active_gun()
        if active_gun:
            t_amt = 0.10 if active_gun.weapon_type == "pistol" else (0.14 if active_gun.weapon_type == "rifle" else 0.35)
            self.trauma_system.add_trauma(t_amt)

        result = self.bullet.rayTestClosest(p_from, p_to)
        if result.hasHit():
            hit_node = result.getNode()
            if hit_node == self.puppet.physics_body:
                # Ignore self collision from muzzle offset
                self.tracers.append(BulletTracer(self.render, p_from, p_to))
                return

            hit_pos = result.getHitPos()
            hit_norm = result.getHitNormal()
            self.tracers.append(BulletTracer(self.render, p_from, hit_pos))
            self.spawn_dust(hit_pos)

            # Directional kinetic spark burst along surface reflection (A3P inspired)
            num_sparks = random.randint(7, 14)
            for _ in range(num_sparks):
                refl_dir = calculate_ricochet_reflection(bullet_dir, hit_norm, spread=0.35)
                s_vel = refl_dir * random.uniform(8.0, 18.0) + Vec3(0, 0, random.uniform(1.0, 3.5))
                self.sparks.append(KineticSpark(self.render, hit_pos, s_vel, life=random.uniform(0.18, 0.32)))

            # Authentic bullet ricochet audio
            if random.random() < 0.40:
                ric_keys = [k for k in ("ricochet1", "ricochet2", "ricochet3") if k in self.sounds]
                if ric_keys:
                    self.play_sfx(random.choice(ric_keys))

            hit_prop_matched = False
            for prop in self.props:
                if getattr(prop, "node", None) == hit_node:
                    impulse = bullet_dir * bullet_force + Vec3(0, 0, 4.5)
                    prop.apply_impulse(impulse, hit_pos)
                    if hasattr(prop, "take_damage"):
                        prop.take_damage(bullet_force * 0.65, hit_pos)
                    self.play_sfx("punch_hit")
                    self.hit_stop.trigger(0.038)
                    hit_prop_matched = True
                    break

            if not hit_prop_matched and isinstance(hit_node, BulletRigidBodyNode) and hit_node.getMass() > 0:
                impulse = bullet_dir * bullet_force + Vec3(0, 0, 4.5)
                hit_node.setActive(True)
                hit_node.applyCentralImpulse(impulse)
                body_pos = hit_node.getTransform().getPos()
                torque = calculate_off_center_torque(hit_pos, body_pos, impulse, max_lever_arm=0.25)
                hit_node.applyTorqueImpulse(torque)
                ang_v = hit_node.getAngularVelocity()
                if ang_v.lengthSquared() > MAX_ANGULAR_VELOCITY * MAX_ANGULAR_VELOCITY:
                    hit_node.setAngularVelocity(ang_v.normalized() * MAX_ANGULAR_VELOCITY)
                self.play_sfx("punch_hit")
                self.hit_stop.trigger(0.038)
        else:
            self.tracers.append(BulletTracer(self.render, p_from, p_to))

    def trigger_explosion(self, blast_pos, radius=7.5, max_force=65.0, trauma=0.55, source_obj=None):
        """
        Radial shockwave detonation engine (adapted from A3P entityGroup.explode):
        - Applies inverse-distance impulse and upward kinetic lift to all nearby props.
        - Triggers camera screen trauma, micro-freeze hitstop, fireball VFX, and spark showers.
        """
        sfx_candidates = [k for k in ("large-explosion", "large-explosion2", "grenade") if k in self.sounds]
        if sfx_candidates:
            self.play_sfx(random.choice(sfx_candidates))
        else:
            self.play_sfx("punch_hit")

        self.trauma_system.add_trauma(trauma)
        self.hit_stop.trigger(0.065)

        if hasattr(self, "flash_light"):
            self.flash_light.trigger(blast_pos, Vec4(1.0, 0.70, 0.25, 1), intensity=3.5)

        self.fireballs.append(ExplosionFireball(self.render, self.loader, blast_pos, radius=radius * 0.35))
        for _ in range(8):
            offset = Vec3(random.uniform(-0.6, 0.6), random.uniform(-0.6, 0.6), random.uniform(0.1, 0.8))
            self.dust_puffs.append(DustPuff(self.render, self.loader, blast_pos + offset))

        for _ in range(24):
            rand_dir = Vec3(random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(0.2, 1.2)).normalized()
            s_vel = rand_dir * random.uniform(12.0, 26.0)
            self.sparks.append(KineticSpark(self.render, blast_pos, s_vel, life=random.uniform(0.3, 0.6)))

        # Radial impulse on props
        for prop in list(self.props):
            if prop == source_obj or getattr(prop, "is_held", False):
                continue
            prop_pos = prop.get_pos()
            impulse, ratio, is_in = calculate_radial_explosion_impulse(blast_pos, prop_pos, max_force, radius, upward_lift=7.0)
            if is_in:
                prop.apply_impulse(impulse, blast_pos)
                if hasattr(prop, "take_damage"):
                    prop.take_damage(40.0 * ratio, blast_pos)

        # Radial impulse and balance decay on puppet character
        puppet_pos = self.puppet.get_torso_pos()
        p_impulse, p_ratio, p_is_in = calculate_radial_explosion_impulse(blast_pos, puppet_pos, max_force * 0.9, radius, upward_lift=6.0)
        if p_is_in:
            self.puppet.take_blast_impact(p_impulse, p_ratio)

    def _spawn_grenade(self, spawn_pos, throw_vel):
        grenade = ThrowableFragGrenade(
            self.bullet, self.render, self.loader, spawn_pos, throw_vel,
            sfx_callback=self.play_sfx,
            explode_callback=self.trigger_explosion
        )
        self.grenades.append(grenade)

    def _do_throw_grenade(self):
        self.puppet.trigger_grenade_throw(self.current_3d_target)

    def _make_ground(self):
        shape = BulletPlaneShape(Vec3(0, 0, 1), 0)
        node  = BulletRigidBodyNode("ground")
        node.setFriction(0.5)
        node.addShape(shape)
        self.render.attachNewNode(node).setPos(0, 0, 0)
        self.bullet.attachRigidBody(node)
        self.ground_node = node

        cm = CardMaker("arena")
        cm.setFrame(-60, 60, -60, 60)
        self.gv = self.render.attachNewNode(cm.generate())
        self.gv.setP(-90)
        self.gv.setPos(0, 0, 0.001)
        self.gv.setColor(0.20, 0.46, 0.22, 1)

        ls = LineSegs()
        ls.setColor(0.16, 0.38, 0.18, 1)
        ls.setThickness(1)
        for i in range(-30, 31, 2):
            ls.moveTo(i, -60, 0.003); ls.drawTo(i, 60, 0.003)
            ls.moveTo(-60, i, 0.003); ls.drawTo(60, i, 0.003)
        self.render.attachNewNode(ls.create())

    def _make_lights(self):
        d = DirectionalLight("sun")
        d.setColor(Vec4(1.0, 0.96, 0.88, 1))
        dn = self.render.attachNewNode(d)
        dn.setHpr(45, -50, 0)
        self.render.setLight(dn)

        a = AmbientLight("fill")
        a.setColor(Vec4(0.38, 0.40, 0.48, 1))
        self.render.setLight(self.render.attachNewNode(a))

        self.flash_light = DynamicFlashLight(self.render)

    def _spawn_props(self):
        crate_pos = [
            (2.5,  2.0, 0.3), (2.5,  2.0, 0.9), (3.1,  2.0, 0.3),
            (-3.0, 2.5, 0.3), (-3.0, 3.1, 0.3), (-3.0, 2.5, 0.9),
            (0.0,  5.0, 0.3), (0.6,  5.0, 0.3), (0.3,  5.0, 0.9),
            (-2.5,-3.0, 0.3), ( 2.5,-3.0, 0.3), ( 0.0,-4.0, 0.3),
        ]
        for pos in crate_pos:
            self.props.append(InteractiveCrate(self.bullet, self.render, self.loader, pos))

        pin_pos = [
            (0.0, 2.5, 0.3),
            (-0.25, 2.9, 0.3), (0.25, 2.9, 0.3),
            (-0.50, 3.3, 0.3), (0.0, 3.3, 0.3), (0.50, 3.3, 0.3)
        ]
        for pos in pin_pos:
            self.props.append(BowlingPin(self.bullet, self.render, self.loader, pos))

        self.props.append(GunWeapon(self.bullet, self.render, self.loader, (-1.5, 0.8, 0.2), "pistol"))
        self.props.append(GunWeapon(self.bullet, self.render, self.loader, (0.0, 1.2, 0.2), "rifle"))
        self.props.append(GunWeapon(self.bullet, self.render, self.loader, (1.5, 0.8, 0.2), "shotgun"))

        self.props.append(GunWeapon(self.bullet, self.render, self.loader, (-2.5, 4.0, 0.2), "rifle"))
        self.props.append(GunWeapon(self.bullet, self.render, self.loader, ( 2.5, 4.0, 0.2), "shotgun"))

        # Hazardous Explosive Fuel Barrels (A3P inspired)
        barrel_positions = [
            ( 1.8,  3.6, 0.38),
            (-2.2,  3.8, 0.38),
            ( 4.2, -1.8, 0.38),
            (-3.8, -2.2, 0.38),
        ]
        for b_pos in barrel_positions:
            barrel = ExplosiveBarrel(self.bullet, self.render, self.loader, b_pos)
            barrel.set_callbacks(self.trigger_explosion, self.play_sfx)
            self.props.append(barrel)

    def _bind_actions(self):
        for pk in ("mouse1", "f", "F", "j", "J"): self.accept(pk, self._do_primary_click)
        for ek in ("mouse3", "e", "E"): self.accept(ek, self._do_pickup)
        for rk in ("r", "R"): self.accept(rk, self.puppet.trigger_reload)
        for gk in ("g", "G"): self.accept(gk, self._do_throw_grenade)

        # Mouse Scroll & +/- Weapon Switching
        self.accept("wheel_up",   lambda: self.puppet.cycle_weapon(1))
        self.accept("wheel_down", lambda: self.puppet.cycle_weapon(-1))
        self.accept("+",          lambda: self.puppet.cycle_weapon(1))
        self.accept("=",          lambda: self.puppet.cycle_weapon(1))
        self.accept("-",          lambda: self.puppet.cycle_weapon(-1))
        self.accept("_",          lambda: self.puppet.cycle_weapon(-1))

        # Direct Weapon Keys
        self.accept("1", lambda: self.puppet.switch_weapon_slot(1))
        self.accept("2", lambda: self.puppet.switch_weapon_slot(2))
        self.accept("3", lambda: self.puppet.switch_weapon_slot(3))

        # V Key: 3-Stage Camera Zoom Toggle
        for vk in ("v", "V"): self.accept(vk, self._cycle_camera_zoom)

        for kk in ("k", "K"): self.accept(kk, lambda: self.puppet.trigger_knockout(1.5))
        for hk in ("h", "H"): self.accept(hk, self._toggle_ice)

        self.accept("tab", lambda: self._set_mouse_lock(not self.mouse_locked))
        self.accept("escape", sys.exit)

    def _do_primary_click(self):
        if not self.puppet.is_holding_gun():
            t_pos = self.puppet.get_torso_pos()
            for p in self.props:
                if not getattr(p, "is_held", False) and (p.get_pos() - t_pos).length() < 1.35:
                    self.hit_stop.trigger(0.048)
                    self.trauma_system.add_trauma(0.24)
                    break
        self.puppet.trigger_primary_action(self.props, self.current_3d_target)

    def _do_pickup(self):
        self.puppet.trigger_pickup(self.props)

    def _toggle_ice(self):
        is_ice = self.puppet.toggle_ice_mode()
        self.ground_node.setFriction(0.04 if is_ice else 0.50)
        if is_ice:
            self.gv.setColor(0.35, 0.65, 0.85, 1)
        else:
            self.gv.setColor(0.20, 0.46, 0.22, 1)

    def _update(self, task):
        raw_dt = min(globalClock.getDt(), 0.05)
        dt = self.hit_stop.process_dt(raw_dt)

        # ── UNRESTRICTED FULL 360° MOUSE LOOK ──
        if self.mouse_locked and getattr(self, "mouseWatcherNode", None) and self.mouseWatcherNode.hasMouse():
            md = self.win.getPointer(0)
            cx = self.win.getXSize() // 2
            cy = self.win.getYSize() // 2
            dx = md.getX() - cx
            dy = md.getY() - cy

            if dx != 0 or dy != 0:
                self.cam_yaw   -= dx * CAM_SENSITIVITY
                self.cam_pitch  = max(-55.0, min(80.0, self.cam_pitch + dy * CAM_SENSITIVITY))
                self.win.movePointer(0, cx, cy)

        # ── SMOOTH CAMERA DISTANCE TRANSITION (V Key Preset Zoom) ──
        self.cam_dist += (self.cam_target_dist - self.cam_dist) * min(1.0, dt * 10.0)

        # ── DYNAMIC OVER-THE-SHOULDER CAMERA TRANSITION ──
        target_shoulder = 0.55 if self.puppet.is_holding_gun() else 0.0
        self.shoulder_x += (target_shoulder - self.shoulder_x) * min(1.0, dt * 10.0)

        # ── EXACT 2ND-ORDER CRITICALLY DAMPED CAMERA PIVOT TRACKING ──
        torso_pos = self.puppet.get_torso_pos()
        target_pos = torso_pos + Vec3(0, 0, 0.45)
        smoothed_pos = self.cam_pos_spring.update(target_pos, dt)
        self.cam_pivot.setPos(smoothed_pos)

        # Kinetic Trauma Screen Shake
        shake = self.trauma_system.update(dt)
        self.cam_pivot.setH(self.cam_yaw + shake.x)
        self.cam_pitch_pivot.setP(self.cam_pitch + shake.y)
        self.cam_pitch_pivot.setR(shake.z)

        # ── CAMERA OBSTACLE RAYCAST ANTI-CLIPPING ──
        ideal_cam_world = self.render.getRelativePoint(self.cam_pitch_pivot, Point3(self.shoulder_x, -self.cam_dist, 0.20))
        p_from = Point3(target_pos)
        p_to   = Point3(ideal_cam_world)
        cam_occ_ray = self.bullet.rayTestClosest(p_from, p_to)
        if cam_occ_ray.hasHit() and cam_occ_ray.getNode() != self.puppet.physics_body:
            occ_dist = (cam_occ_ray.getHitPos() - p_from).length()
            actual_dist = max(1.0, min(self.cam_dist, occ_dist - CAM_COLLISION_MARGIN))
        else:
            actual_dist = self.cam_dist

        self.camera.setPos(self.shoulder_x, -actual_dist, 0.20)

        # ── VELOCITY-COUPLED DYNAMIC FOV WARP ──
        char_v = self.puppet.physics_body.getLinearVelocity()
        cur_planar_speed = math.hypot(char_v.x, char_v.y)
        sprint_ratio = max(0.0, min(1.0, (cur_planar_speed - MOVE_SPEED) / max(0.1, SPRINT_SPEED - MOVE_SPEED)))
        target_fov = CAM_FOV_BASE + (CAM_FOV_SPRINT - CAM_FOV_BASE) * (sprint_ratio ** 1.8)
        cur_fov = self.cam_fov_spring.update(target_fov, dt)
        if getattr(self, "camLens", None):
            self.camLens.setFov(cur_fov)

        # ── TWO-RAY PINPOINT CROSSHAIR RAYCAST ──
        cam_world_pos  = self.camera.getPos(self.render)
        cam_world_quat = self.camera.getQuat(self.render)
        cam_fwd = cam_world_quat.getForward()

        p_from = Point3(cam_world_pos)
        p_to   = Point3(cam_world_pos + cam_fwd * 100.0)

        cam_ray = self.bullet.rayTestClosest(p_from, p_to)
        if cam_ray.hasHit() and cam_ray.getNode() != self.puppet.physics_body:
            self.current_3d_target = cam_ray.getHitPos()
        else:
            self.current_3d_target = p_to

        # ── DIRECT HARDWARE POLLING (Continuous Shooting & Movement) ──
        is_btn = self.mouseWatcherNode.isButtonDown if getattr(self, "mouseWatcherNode", None) else lambda k: False
        w_down = is_btn(KeyboardButton.asciiKey('w')) or is_btn(KeyboardButton.up())
        s_down = is_btn(KeyboardButton.asciiKey('s')) or is_btn(KeyboardButton.down())
        a_down = is_btn(KeyboardButton.asciiKey('a')) or is_btn(KeyboardButton.left())
        d_down = is_btn(KeyboardButton.asciiKey('d')) or is_btn(KeyboardButton.right())
        shift_down = (is_btn(KeyboardButton.shift()) or
                      is_btn(KeyboardButton.lshift()) or
                      is_btn(KeyboardButton.rshift()))
        space_down = is_btn(KeyboardButton.space())
        lmb_held   = is_btn(MouseButton.one()) or is_btn(KeyboardButton.asciiKey('f'))

        yr = math.radians(self.cam_yaw)
        fwd_x = -math.sin(yr); fwd_y =  math.cos(yr)
        rgt_x =  math.cos(yr); rgt_y =  math.sin(yr)

        raw_x = (1.0 if d_down else 0.0) - (1.0 if a_down else 0.0)
        raw_y = (1.0 if w_down else 0.0) - (1.0 if s_down else 0.0)

        norm_x, norm_y = box_normalize_to_circle(raw_x, raw_y)

        world_mx = rgt_x * norm_x + fwd_x * norm_y
        world_my = rgt_y * norm_x + fwd_y * norm_y

        self.puppet.apply_movement(world_mx, world_my, space_down, shift_down, lmb_held, dt, self.cam_yaw, self.current_3d_target)

        # ── CONTEXTUAL PROXIMITY PROMPTS ──
        puppet_pos = self.puppet.get_torso_pos()
        nearest_prop = None
        min_dist = 1.65

        for prop in self.props:
            if getattr(prop, "is_held", False):
                continue
            d = (prop.get_pos() - puppet_pos).length()
            if d < min_dist:
                min_dist = d
                nearest_prop = prop

        if nearest_prop:
            if getattr(nearest_prop, "is_gun", False):
                g_type = nearest_prop.weapon_type
                if g_type in self.puppet.weapons_inventory:
                    self.prompt_text.setText(f"[E / RMB] Take Ammo (+{nearest_prop.cfg['ammo_pickup']} Rounds)")
                else:
                    self.prompt_text.setText(f"[E / RMB] Pick Up {nearest_prop.name}")
            else:
                self.prompt_text.setText(f"[E / RMB] Pick Up {nearest_prop.name}")
        else:
            self.prompt_text.setText("")

        # ── WEAPON & AMMO HUD UPDATE ──
        grenade_tag = f"  |  FRAG: {self.puppet.grenades_count} [G]"
        if self.puppet.is_holding_gun():
            self.crosshair.show()
            gun = self.puppet.get_active_gun()
            if self.puppet.is_reloading:
                self.hud_ammo.setText(f"[{gun.name.upper()}]  |  [RELOADING...]{grenade_tag}")
            else:
                self.hud_ammo.setText(f"[{gun.name.upper()}]  |  AMMO: {gun.ammo_mag} / {gun.ammo_reserve}{grenade_tag}")
        elif self.puppet.held_prop:
            self.crosshair.hide()
            self.hud_ammo.setText(f"[HOLDING {self.puppet.held_prop.name.upper()}]{grenade_tag}")
        else:
            self.crosshair.hide()
            self.hud_ammo.setText(f"[UNARMED] (PUNCH / THROW){grenade_tag}")

        # Update VFX
        active_dust = [p for p in self.dust_puffs if p.update(dt)]
        self.dust_puffs = active_dust

        active_tracers = [t for t in self.tracers if t.update(dt)]
        self.tracers = active_tracers

        active_flashes = [f for f in self.flashes if f.update(dt)]
        self.flashes = active_flashes

        active_sparks = [s for s in self.sparks if s.update(dt)]
        self.sparks = active_sparks

        active_fireballs = [f for f in self.fireballs if f.update(dt)]
        self.fireballs = active_fireballs

        # Update Throwable Frag Grenades
        active_grenades = [g for g in self.grenades if g.update(dt)]
        self.grenades = active_grenades

        # Update Dynamic Flash Light
        if hasattr(self, "flash_light"):
            self.flash_light.update(dt)

        # Update Physical Spent Shell Casings
        self.casings = [c for c in self.casings if c.update(dt)]

        # Update Prop Aerodynamics & Clean Destroyed Props
        for prop in self.props:
            if hasattr(prop, "update_physics"):
                prop.update_physics(dt)
        self.props = [p for p in self.props if not getattr(p, "has_exploded", False)]

        # Step Bullet physics
        self.bullet.doPhysics(dt, 10, 1.0 / 180.0)

        # Update Drop Shadow
        t_pos = self.puppet.get_torso_pos()
        self.shadow_np.setPos(t_pos.x, t_pos.y, 0.004)
        h_factor = max(0.1, 1.0 - (t_pos.z * 0.25))
        self.shadow_np.setScale(h_factor)
        self.shadow_np.setColor(0.08, 0.16, 0.08, 0.6 * h_factor)

        return task.cont


if __name__ == "__main__":
    PuppetEngine().run()
