# -*- mode: python ; coding: utf-8 -*-


a = Analysis(
    ['C:/Tim/Varsity/3rd Year/PRJ381/Pro Package/groundstation/launcher.py'],
    pathex=[],
    binaries=[],
    datas=[('C:/Tim/Varsity/3rd Year/PRJ381/Pro Package/seayou-main/dist', 'webapp')],
    hiddenimports=['server', 'mission', 'auth', 'login_page'],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    noarchive=False,
    optimize=0,
)
pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.datas,
    [],
    name='SeaYouGroundStation',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    upx_exclude=[],
    runtime_tmpdir=None,
    console=True,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
)
