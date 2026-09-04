# PuppetEngine - Procedural Physics & Tactical TPS Action Engine

> **The One & Only Procedural Spring-Damper Physics & Tactical Third-Person Shooter (TPS) Engine Built in Pure Python.**

[![Python 3.10+](https://img.shields.io/badge/Python-3.10%2B-3776AB?style=for-the-badge&logo=python&logoColor=white)](https://www.python.org/)
[![Panda3D](https://img.shields.io/badge/Render-Panda3D-E10098?style=for-the-badge&logo=panda3d&logoColor=white)](https://www.panda3d.org/)
[![Bullet Physics](https://img.shields.io/badge/Physics-Bullet_3D-FF6F00?style=for-the-badge)](https://pybullet.org/)
[![Zero Asset Bloat](https://img.shields.io/badge/Assets-100%25_Procedural-00C853?style=for-the-badge)](#)
[![Development Status](https://img.shields.io/badge/Status-Active%20In-Progress-FFA000?style=for-the-badge)](#)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg?style=for-the-badge)](LICENSE)

An ultra-lightweight (**< 100 KB**), clean-room **3D procedural character physics and tactical third-person shooter (TPS) engine** built in pure Python with zero external 3D asset downloads.

PuppetEngine fuses the **organic clay-puppet physics and procedural spring-damper animation of BombSquad (Ballistica)** with **AAA PUBG-style over-the-shoulder tactical shooting, visible back-holstering, two-ray pinpoint aiming, and kinetic ballistics**.

---

> [!IMPORTANT]
> **Development Status: Active In-Progress**
> PuppetEngine is currently under active development. Core character physics, layered multi-actions, two-ray parallax-free aiming, and procedural weapons are fully functional, with ongoing improvements to multiplayer networking, AI behaviors, and arena hazard systems.

---

## Key Highlights & Innovations

- **The One & Only in Python:** There is currently no other open-source engine in Python combining procedural spring-damper puppet ragdoll animation with a full PUBG-style TPS arsenal.
- **100% Procedural & Self-Contained:** Zero external 3D mesh or audio file downloads. Every weapon model (Tactical Pistol, M4 Carbine, SPAS Shotgun) is generated mathematically, and all sound effects are synthesized in real-time using raw waveform algorithms.
- **Reverse-Engineered Ballistica Mechanics:** Clean-room mathematical implementation of `BoxNormalizeToCircle`, dynamic footing/balance decay, 3-phase anticipation punch dynamics, overhead item lifting/clasping, and low-friction hockey skating.
- **Industry-Standard Two-Ray Aiming:** Eliminates parallax error by casting a camera sightline ray through the screen center crosshair and mapping physical bullet trajectories directly to the 3D impact point.
- **PUBG-Style Visible Back-Holster System:** Weapons stored in inventory but not active in hands are realistically slung across the spine sockets on the character's back.
- **Layered Simultaneous Action Channels:** Punch, sprint, jump, aim, and carry items simultaneously without action locking or animation cancellation.
- **A3P-Inspired Tactical Ballistics & Explosives:** Throwable fragmentation grenades (`G`), explosive hazard fuel barrels with chain reactions, kinetic bullet ricochets with surface reflection spark showers, and zero-hitch dynamic flash point lighting.

---

## Comparative Analysis: What Makes PuppetEngine Unique?

| Feature | Standard Game Engines (Unity/Unreal) | Traditional Panda3D Demos | **PuppetEngine** |
|:---|:---:|:---:|:---:|
| **Engine Footprint** | 15 GB to 35 GB | ~50 MB | **< 100 KB Pure Code** |
| **Boot & Load Time** | 10 to 30 seconds | 2 to 5 seconds | **< 0.5 seconds** |
| **Character Rigging** | Canned baked skeletal keyframes | Static rigid shapes | **Dynamic Spring-Damper Procedural IK** |
| **Punch Mechanics** | Generic hitboxes | Simple collision overlap | **3-Phase Anticipation & Coil Physics** |
| **Weapon System** | Downloaded 3D FBX assets | Rare / Basic cubes | **Procedural Multi-Part 3D Models + Holsters** |
| **Audio Engine** | Heavy pre-recorded MP3/WAVs | Bundled audio files | **Synthesized Pure Waveform Audio (`wave`)** |

---

## Mathematical & Physics Architecture

### 1. Vector Normalization (`BoxNormalizeToCircle`)
Eliminates diagonal speed exploits (e.g. $W+D$ moving $\sqrt{2} \times$ faster) while preserving analog stick fidelity:

$$\vec{v}_{\text{norm}} = \vec{v}_{\text{raw}} \cdot \frac{1}{\sqrt{\left(\frac{v_x}{\max(|v_x|, |v_y|)}\right)^2 + \left(\frac{v_y}{\max(|v_x|, |v_y|)}\right)^2}}$$

### 2. Dynamic Footing & Balance System
Character stability is measured from $0$ to $255$ balance points. In mid-air or during high-impact stumbles, balance decays, triggering flailing bicycle kicks until stable footing is restored on ground contacts.

### 3. 3-Phase Spring Punch & Ballistica Momentum Physics
1. **Anticipation Phase ($0\text{ ms} - 80\text{ ms}$):** Arm winds back behind shoulder ($\theta = +25^\circ$), torso coils backward ($-15^\circ$), non-punching fist anchors to chest.
2. **Maximum Velocity Forward Thrust ($80\text{ ms} - 200\text{ ms}$):** Fist rockets forward $0.70\text{ m}$ ($\theta = -90^\circ$), torso violently twists forward ($+24^\circ$).
3. **Damped Spring Recovery ($200\text{ ms} - 300\text{ ms}$):** Spring-damper forces smoothly return limbs and torso to neutral idle stance.
4. **360° Tornado Spin Punch:** Triggers when punching while rotating, spinning the character in a full $360^\circ$ horizontal spiral with double kinetic impulse ($2.4\times$).
5. **Ballistica Angular & Linear Momentum Accumulators:** Rotational angular velocity ($|\omega|$) and sprint velocity ($v$) feed 2-stage leaky integrators (`punch_mom_ang_m`, `punch_mom_lin_m`) that scale punch impulse up to $+150\%$, rewarding agile spinning strikes and sprint-punch combos.
6. **Spin-Coupled Dynamic Punch Hand Selection:** When spinning ($|\omega| > 0.35\text{ rad/s}$), punch hand automatically selects the leading centrifugal fist ($R$ on right spin, $L$ on left spin).

### 4. Ballistica Procedural Clay-Puppet Biomechanics (`spaz_node.cc`)
- **Quadrature Elliptical Running Arm Swing:** Running arm swings use an out-of-phase quadrature pump ($\sin(\text{roll} + \pi/2) \cdot 0.20$ vs $\cos(\text{roll}) \cdot 0.30$) blended with $run\_gas^2$, creating organic athletic circular arm pumping rather than flat planar swings.
- **Asymmetric Run Gas Smoothing:** Accelerates with higher responsiveness ($0.95$) than deceleration ($0.65$), with immediate airborne bleed.
- **Organic Breathing Oscillations:** Idle stance applies vertical oscillation ($z \pm \sin(3.6t) \cdot 0.012\text{ m}$) simulating ribcage breathing.
- **Dual-Grip Prop Placement & Look-Up Pitch:** Overhead prop/crate carry aligns hands snugly to crate sides ($[\pm 0.15, -0.04, +0.02]\text{ m}$) and tilts head upward $+28.6^\circ$ ($+0.5\text{ rad}$) so the puppet looks past the held object.
- **Airborne Counter-Rotating Flail Kinematics:** When footing is lost in mid-air, dual counter-rotating phase-shifted circular arm waves simulate frantic ragdoll recovery flailing.

### 5. Two-Ray Parallax-Free Aiming
1. **Ray 1 (Camera Aim Sightline):** Casts a ray from camera lens $\vec{C}$ through center crosshair $\hat{F}_{\text{cam}}$:
   $$\vec{P}_{\text{target}} = \text{RayCastClosest}(\vec{C}, \vec{C} + 100 \cdot \hat{F}_{\text{cam}})$$
2. **Ray 2 (Physical Bullet Trajectory):** Fires kinetic projectile from muzzle origin $\vec{M}$ to $\vec{P}_{\text{target}}$:
   $$\hat{D}_{\text{bullet}} = \frac{\vec{P}_{\text{target}} - \vec{M}}{\|\vec{P}_{\text{target}} - \vec{M}\|}$$

---

## Weapon Arsenal Breakdown

```
[1] Tactical Heavy Pistol      [2] M4 Spec-Ops Assault Rifle    [3] SPAS Combat Shotgun
  • Semi-Automatic               • Full-Auto (Continuous LMB)     • 6 Kinetic Scatter Pellets
  • 12 / 48 Ammo                 • 30 / 120 Ammo                  • 8 / 32 Ammo
  • Cyan Laser Diode             • Reflex Holo-Sight + Muzzle     • Twin Heavy Blast Tubes
```

---

## Controls & Gameplay Guide

| Control | Action | Details |
|---|---|---|
| **`Move Mouse`** | **Free 360° Look & Aim** | Unrestricted horizontal & vertical pitch ($-55^\circ$ to $+80^\circ$) |
| **`Left Click` (Hold)** | **Continuous Shoot / Punch** | Full-auto for Rifle, semi-auto for Pistol/Shotgun, punches when unarmed |
| **`Right Click` / `E`** | **Pick Up / Take Ammo / Throw** | Equips weapons, harvests reserve ammo on duplicates, lifts/throws props |
| **`G` Key** | **Throw Tactical Frag Grenade** | Bouncing physical grenade with 2.2s fuse, bounce SFX, and devastating radial shockwave |
| **`Mouse Scroll Wheel`** | **Cycle Weapons** | Scroll up/down to cycle forward/backward through inventory |
| **`+` / `-` Keys** | **Cycle Weapons** | Alternative keyboard weapon cycling |
| **`1`, `2`, `3`** | **Direct Weapon Slot** | Instantly select Pistol (`1`), Assault Rifle (`2`), or Shotgun (`3`) |
| **`V` Key** | **3-Stage Camera Zoom** | Cycles: `4.2m` (Tight Aim) $\rightarrow$ `7.0m` (Standard) $\rightarrow$ `11.5m` (Wide) |
| **`R` Key** | **Reload Magazine** | Refills magazine from reserve ammo with realistic cocking audio |
| **`SHIFT` + `WASD`** | **Sprint / Tactical Strafe** | Full sprint with athletic torso lean and lateral strafe alignment |
| **`SPACE`** | **Jump** | Mid-air bicycle kick with upward impulse |
| **`H` Key** | **Ice / Hockey Mode** | Low-friction skating and sliding physics |
| **`TAB`** | **Toggle Cursor Lock** | Free OS mouse cursor / lock mouselook |
| **`ESC`** | **Quit** | Exit sandbox |

---

## Installation & Running

### 1. Prerequisites
- **Python 3.10+**
- **Panda3D** with Bullet Physics bindings:
  ```bash
  pip install panda3d
  ```

### 2. Launch Sandbox
```bash
# Clone repository
git clone https://github.com/arjncx-lang/puppet-engine.git
cd puppet-engine

# Run game sandbox
python main.py
# Or double click run.bat on Windows
```

---

## Project Structure

```
puppet-engine/
├── character.py          # Complete PuppetCharacter engine, procedural IK & inventory
├── props.py              # Sharp 3D procedural weapon models, crates & bowling pins
├── physics_constants.py  # Tuned world physics, limits & safety clamping constants
├── main.py               # PuppetEngine ShowBase loop, TPS camera gimbal, crosshair & HUD
├── README.md             # Complete documentation, math & architecture breakdown
├── LICENSE               # MIT License
├── run.bat               # Windows quick launch batch script
└── sfx/                  # Procedural waveform audio files (gunshots, punches, reloads)
```

---

## License
This project is released under the **MIT License**. Free for educational, commercial, and personal use.
