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
import sys, math, pathlib
from direct.showbase.ShowBase import ShowBase
from panda3d.core import (Vec3, Vec4, Point3, DirectionalLight, AmbientLight,
                           CardMaker, LineSegs, AntialiasAttrib,
                           loadPrcFileData, TextNode, KeyboardButton,
                           WindowProperties, MouseButton)
from panda3d.bullet import BulletWorld, BulletPlaneShape, BulletRigidBodyNode
from direct.gui.OnscreenText import OnscreenText

from physics_constants import *
from character import PuppetCharacter, box_normalize_to_circle
from props import InteractiveCrate, BowlingPin, GunWeapon

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


class PuppetEngine(ShowBase):

    def __init__(self):
        super().__init__()
        self.disableMouse()
        self.setBackgroundColor(0.14, 0.17, 0.24, 1)
        self.render.setAntialias(AntialiasAttrib.MAuto)

        self.bullet = BulletWorld()
        self.bullet.setGravity(Vec3(0, 0, GRAVITY))

        self._make_ground()
        self._make_lights()

        self.sounds = {}
        sfx_dir = pathlib.Path(__file__).parent / "sfx"
        for sfx_name in ("punch_whoosh", "punch_hit", "jump", "throw", "stun",
                         "pistol_shot", "rifle_shot", "shotgun_shot", "reload", "gun_pickup"):
            p = sfx_dir / f"{sfx_name}.wav"
            if p.exists():
                self.sounds[sfx_name] = self.loader.loadSfx(str(p))

        # ── Puppet Character ──
        self.puppet = PuppetCharacter(self.bullet, self.render, self.loader, (0, 0, 0))
        self.puppet.set_dust_callback(self.spawn_dust)
        self.puppet.set_sfx_callback(self.play_sfx)
        self.puppet.set_shoot_callback(self._execute_bullet_fire)

        self.props = []
        self._spawn_props()

        self.dust_puffs = []
        self.tracers    = []
        self.flashes    = []

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
            text="👊 UNARMED", pos=(0.90, -0.86), scale=0.048, fg=(1, 1, 1, 0.95),
            shadow=(0, 0, 0, 0.8), align=TextNode.ARight, mayChange=True)

        self.hud_slots = OnscreenText(
            text="[1] Pistol  |  [2] Assault Rifle  |  [3] Scatter Shotgun",
            pos=(0, -0.88), scale=0.036, fg=(0.7, 0.85, 1.0, 0.90),
            align=TextNode.ACenter, mayChange=True)

        OnscreenText(
            text="Mouse: Look | LMB: Shoot/Punch | Wheel / +/-: Switch Gun | V: Camera Zoom (3 Angles) | R: Reload",
            pos=(0, -0.95), scale=0.034, fg=(0.9, 0.9, 0.9, 0.85),
            align=TextNode.ACenter, mayChange=False)

        self.taskMgr.add(self._update, "update")

    def _set_mouse_lock(self, lock):
        self.mouse_locked = lock
        props = WindowProperties()
        props.setCursorHidden(lock)
        self.win.requestProperties(props)

    def play_sfx(self, name):
        if name in self.sounds:
            self.sounds[name].play()

    def spawn_dust(self, pos):
        puff = DustPuff(self.render, self.loader, pos)
        self.dust_puffs.append(puff)

    def _cycle_camera_zoom(self):
        self.cam_zoom_index = (self.cam_zoom_index + 1) % len(self.cam_zoom_presets)
        self.cam_target_dist = self.cam_zoom_presets[self.cam_zoom_index]

    def _execute_bullet_fire(self, muzzle_pos, target_3d_point, bullet_force):
        p_from = Point3(muzzle_pos.x, muzzle_pos.y, muzzle_pos.z)
        bullet_dir = (target_3d_point - p_from).normalized()
        p_to   = Point3(p_from + bullet_dir * 90.0)

        self.flashes.append(MuzzleFlash(self.render, self.loader, p_from))

        result = self.bullet.rayTestClosest(p_from, p_to)
        if result.hasHit():
            hit_pos  = result.getHitPos()
            hit_node = result.getNode()

            self.tracers.append(BulletTracer(self.render, p_from, hit_pos))
            self.spawn_dust(hit_pos)

            if isinstance(hit_node, BulletRigidBodyNode) and hit_node.getMass() > 0:
                impulse = bullet_dir * bullet_force + Vec3(0, 0, 4.5)
                hit_node.setActive(True)
                hit_node.applyCentralImpulse(impulse)
                self.play_sfx("punch_hit")
        else:
            self.tracers.append(BulletTracer(self.render, p_from, p_to))

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

    def _bind_actions(self):
        for pk in ("mouse1", "f", "F", "j", "J"): self.accept(pk, self._do_primary_click)
        for ek in ("mouse3", "e", "E"): self.accept(ek, self._do_pickup)
        for rk in ("r", "R"): self.accept(rk, self.puppet.trigger_reload)

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
        dt = min(globalClock.getDt(), 0.05)

        # ── UNRESTRICTED FULL 360° MOUSE LOOK ──
        if self.mouse_locked and self.mouseWatcherNode.hasMouse():
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

        torso_pos = self.puppet.get_torso_pos()
        target_pos = torso_pos + Vec3(0, 0, 0.45)
        self.cam_pivot.setPos(target_pos)
        self.cam_pivot.setH(self.cam_yaw)
        self.cam_pitch_pivot.setP(self.cam_pitch)
        self.camera.setPos(self.shoulder_x, -self.cam_dist, 0.20)

        # ── TWO-RAY PINPOINT CROSSHAIR RAYCAST ──
        cam_world_pos  = self.camera.getPos(self.render)
        cam_world_quat = self.camera.getQuat(self.render)
        cam_fwd = cam_world_quat.getForward()

        p_from = Point3(cam_world_pos)
        p_to   = Point3(cam_world_pos + cam_fwd * 100.0)

        cam_ray = self.bullet.rayTestClosest(p_from, p_to)
        if cam_ray.hasHit():
            self.current_3d_target = cam_ray.getHitPos()
        else:
            self.current_3d_target = p_to

        # ── DIRECT HARDWARE POLLING (Continuous Shooting & Movement) ──
        is_btn = self.mouseWatcherNode.isButtonDown
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
        if self.puppet.is_holding_gun():
            self.crosshair.show()
            gun = self.puppet.get_active_gun()
            if self.puppet.is_reloading:
                self.hud_ammo.setText(f"🔫 {gun.name.upper()} | ⏳ RELOADING...")
            else:
                self.hud_ammo.setText(f"🔫 {gun.name.upper()} | {gun.ammo_mag} / {gun.ammo_reserve}")
        elif self.puppet.held_prop:
            self.crosshair.hide()
            self.hud_ammo.setText(f"📦 HOLDING {self.puppet.held_prop.name.upper()}")
        else:
            self.crosshair.hide()
            self.hud_ammo.setText("👊 UNARMED (PUNCH)")

        # Update VFX
        active_dust = [p for p in self.dust_puffs if p.update(dt)]
        self.dust_puffs = active_dust

        active_tracers = [t for t in self.tracers if t.update(dt)]
        self.tracers = active_tracers

        active_flashes = [f for f in self.flashes if f.update(dt)]
        self.flashes = active_flashes

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
