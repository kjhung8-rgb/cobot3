"""머티리얼 prim 구조 확인용 진단 스크립트"""
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import omni.usd
import omni.kit.commands
from pxr import UsdShade, Sdf, Gf

stage = omni.usd.get_context().get_stage()

omni.kit.commands.execute(
    "CreateMdlMaterialPrim",
    mtl_url="OmniPBR.mdl",
    mtl_name="OmniPBR",
    mtl_path="/World/Looks/TestMat",
)

# 업데이트 한 번 돌려서 prim 생성 완료
simulation_app.update()

mat_prim = stage.GetPrimAtPath("/World/Looks/TestMat")

lines = []
lines.append(f"Material valid: {mat_prim.IsValid()}")
lines.append("Children:")
for child in mat_prim.GetChildren():
    lines.append(f"  {child.GetPath()}  type={child.GetTypeName()}")
    c2 = UsdShade.Shader(child)
    if c2:
        lines.append(f"    -> Shader id: {c2.GetIdAttr().Get()}")
        for inp in c2.GetInputs():
            lines.append(f"    input: {inp.GetFullName()} = {inp.Get()}")

with open("/tmp/mat_debug_out.txt", "w") as f:
    f.write("\n".join(lines))

simulation_app.close()
