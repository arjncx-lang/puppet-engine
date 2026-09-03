@echo off
title PuppetEngine
echo.
echo  ======================================================
echo   PuppetEngine - Procedural Physics ^& Tactical TPS Sandbox
echo  ======================================================
echo.
echo  Controls:
echo    Mouse Move    : Free 360 Camera Look ^& Pinpoint Aim
echo    Left Click    : Shoot (Full-Auto Rifle) / Punch
echo    Right Click/E : Pick Up Weapon / Take Ammo / Throw Crate
echo    Mouse Scroll  : Cycle Weapons (Forward/Backward)
echo    + / - Buttons : Cycle Weapons (Next/Previous)
echo    1, 2, 3       : Select Weapon (1: Pistol, 2: Rifle, 3: Shotgun)
echo    V Key         : 3-Stage Camera Zoom (Close, Normal, Wide)
echo    R             : Reload Active Weapon
echo    WASD / Arrows : Run ^& Tactical Strafe
echo    SHIFT         : Sprint Boost
echo    SPACE         : Jump
echo    H             : Toggle Ice Mode
echo    TAB           : Toggle Mouse Cursor Lock
echo    ESC           : Quit
echo.
python main.py
pause
