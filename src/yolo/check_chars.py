"""
check_chars.py - DH_Characters 경로 탐색 진단 스크립트
"""
import sys
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import omni.client
from isaacsim.storage.native import get_assets_root_path

assets_root = get_assets_root_path()
print(f"\n[Assets] root = {assets_root}")

# 시도할 경로 목록
candidates = [
    assets_root + "/Isaac/People/Characters",
    assets_root + "/Isaac/People/Characters/DH_Characters",
    assets_root + "/Isaac/People/Characters/DH_Characters_Extended",
    assets_root + "/Isaac/People",
]

for path in candidates:
    result, entries = omni.client.list(path)
    if result == omni.client.Result.OK:
        print(f"\n[OK] {path}")
        for e in list(entries)[:5]:
            print(f"     {e.relative_path}")
        if len(entries) > 5:
            print(f"     ... ({len(entries)}개 총)")
    else:
        print(f"[NO] {path}  ({result})")

simulation_app.close()
print("\n[완료]")
