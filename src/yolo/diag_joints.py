"""
diag_joints.py - DH_Characters 실제 조인트 이름 출력
"""
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import omni.client, omni.usd
from isaacsim.storage.native import get_assets_root_path
from isaacsim.core.utils.stage import add_reference_to_stage
from pxr import UsdSkel, Usd

assets_root = get_assets_root_path()
for base_label, base in [
    ("DH_Characters", assets_root + "/Isaac/People/DH_Characters"),
    ("Characters",    assets_root + "/Isaac/People/Characters"),
]:
    char_usd = None
    result, entries = omni.client.list(base)
    if result != omni.client.Result.OK:
        print(f"[{base_label}] 접근 불가")
        continue
    for entry in entries:
        sub = base + "/" + entry.relative_path
        r2, subs = omni.client.list(sub)
        if r2 != omni.client.Result.OK:
            continue
        for se in subs:
            if se.relative_path.endswith(".usd") and ".thumbnails" not in se.relative_path:
                char_usd = sub + "/" + se.relative_path
                break
        if char_usd:
            break

    if not char_usd:
        print(f"[{base_label}] USD 없음")
        continue

    print(f"\n[{base_label}] {char_usd}")
    add_reference_to_stage(char_usd, f"/World/Test_{base_label}")
    for _ in range(3):
        simulation_app.update()

    stage = omni.usd.get_context().get_stage()
    for prim in Usd.PrimRange(stage.GetPrimAtPath(f"/World/Test_{base_label}")):
        if prim.IsA(UsdSkel.Skeleton):
            skel = UsdSkel.Skeleton(prim)
            joints = skel.GetJointsAttr().Get()
            print(f"  Skeleton: {prim.GetPath()}")
            print(f"  조인트 수: {len(joints) if joints else 0}")
            if joints:
                for j in list(joints)[:50]:
                    print(f"    {j}")
            break
    else:
        print(f"  Skeleton 프림 없음")

simulation_app.close()
print("\n[완료]")
