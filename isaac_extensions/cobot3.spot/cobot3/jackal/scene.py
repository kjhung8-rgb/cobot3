"""Scene helper for spawning Clearpath Jackal as the secondary robot."""

from __future__ import annotations

import math

import carb
import numpy as np
import omni.usd

from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path
from pxr import Gf, UsdGeom, UsdLux

from .constants import (
    JACKAL_HEADLIGHT_COLOR_RGB,
    JACKAL_HEADLIGHT_CONE_ANGLE_DEG,
    JACKAL_HEADLIGHT_INTENSITY,
    JACKAL_HEADLIGHT_PRIM_PATH,
    JACKAL_HEADLIGHT_RADIUS,
    JACKAL_HEADLIGHT_ROTATION_XYZ_DEG,
    JACKAL_HEADLIGHT_SCALE,
    JACKAL_HEADLIGHT_TRANSLATION,
    JACKAL_PRIM_PATH,
    JACKAL_RESCUE_BOX_COLOR_RGB,
    JACKAL_RESCUE_BOX_PATH,
    JACKAL_RESCUE_BOX_ROTATION_XYZ_DEG,
    JACKAL_RESCUE_BOX_SCALE,
    JACKAL_RESCUE_BOX_TRANSLATION,
    JACKAL_SPAWN_POSITION,
    JACKAL_SPAWN_YAW_DEG,
    JACKAL_USD_ASSET_REL_PATH,
)


def _yaw_to_quat_wxyz(yaw_deg):
    half_yaw = math.radians(yaw_deg) * 0.5
    return np.array([math.cos(half_yaw), 0.0, 0.0, math.sin(half_yaw)], dtype=np.float64)


def add_jackal_to_world(sample):
    """Spawn Clearpath Jackal and set ``sample.jackal``.

    Idempotent: 튜닝 USD에 이미 /World/Jackal이 있으면 reference 단계 skip하고
    SingleArticulation으로 existing prim wrap만 수행.
    """
    stage = omni.usd.get_context().get_stage()

    if stage.GetPrimAtPath(JACKAL_PRIM_PATH).IsValid():
        # 튜닝 USD에서 이미 로드된 상태 — reference 추가 skip
        print(f"[cobot3.spot/jackal] {JACKAL_PRIM_PATH} 이미 존재, reference skip")
    else:
        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            carb.log_error("[cobot3.spot/jackal] spawn: Isaac assets root 못 찾음")
            return
        jackal_usd = f"{assets_root_path.rstrip('/')}/{JACKAL_USD_ASSET_REL_PATH}"
        add_reference_to_stage(usd_path=jackal_usd, prim_path=JACKAL_PRIM_PATH)
        print(f"[cobot3.spot/jackal] spawn: {jackal_usd}")

    sample.jackal = SingleArticulation(
        prim_path=JACKAL_PRIM_PATH,
        name="Jackal",
        position=np.array(JACKAL_SPAWN_POSITION, dtype=np.float64),
        orientation=_yaw_to_quat_wxyz(JACKAL_SPAWN_YAW_DEG),
    )

    _add_jackal_headlight()
    _add_jackal_rescue_box()


def _add_jackal_headlight():
    """Forward-facing spotlight parented under jackal chassis. Mirror of spot
    /World/Spot/body/headlight setup — SphereLight + ShapingAPI cone.

    NOTE: jackal_0/base_link USD often has a non-identity rotation. The default
    rotation `(0, 90, 0)` should aim cone toward +X of base_link's local frame
    but may need user tuning in Isaac UI to compensate. After tuning, copy
    Translate/Rotate values back into JACKAL_HEADLIGHT_* constants.
    """
    import omni.kit.commands

    stage = omni.usd.get_context().get_stage()
    created = False
    if not stage.GetPrimAtPath(JACKAL_HEADLIGHT_PRIM_PATH).IsValid():
        omni.kit.commands.execute(
            "CreatePrimWithDefaultXform",
            prim_type="SphereLight",
            prim_path=JACKAL_HEADLIGHT_PRIM_PATH,
        )
        created = True

    light_prim = stage.GetPrimAtPath(JACKAL_HEADLIGHT_PRIM_PATH)
    UsdGeom.Imageable(light_prim).MakeVisible()

    # 기존 op precision 변경하면 AddXformOp 에러. 있는 op는 Set, 없는 op만 Add.
    xform = UsdGeom.Xformable(light_prim)
    existing = {op.GetOpName(): op for op in xform.GetOrderedXformOps()}

    if "xformOp:translate" in existing:
        existing["xformOp:translate"].Set(Gf.Vec3d(*JACKAL_HEADLIGHT_TRANSLATION))
    else:
        xform.AddTranslateOp().Set(Gf.Vec3d(*JACKAL_HEADLIGHT_TRANSLATION))

    if "xformOp:rotateXYZ" in existing:
        existing["xformOp:rotateXYZ"].Set(Gf.Vec3f(*JACKAL_HEADLIGHT_ROTATION_XYZ_DEG))
    else:
        xform.AddRotateXYZOp().Set(Gf.Vec3f(*JACKAL_HEADLIGHT_ROTATION_XYZ_DEG))

    if "xformOp:scale" in existing:
        existing["xformOp:scale"].Set(Gf.Vec3f(*JACKAL_HEADLIGHT_SCALE))
    else:
        xform.AddScaleOp().Set(Gf.Vec3f(*JACKAL_HEADLIGHT_SCALE))

    sphere_light = UsdLux.SphereLight(light_prim)
    sphere_light.CreateIntensityAttr().Set(JACKAL_HEADLIGHT_INTENSITY)
    sphere_light.CreateRadiusAttr().Set(JACKAL_HEADLIGHT_RADIUS)
    sphere_light.CreateColorAttr().Set(Gf.Vec3f(*JACKAL_HEADLIGHT_COLOR_RGB))

    # ShapingAPI cone 의도적으로 제거 — jackal base_link의 비표준 rotation 때문에
    # cone 방향이 기대한 +X 전방이 아니라 엉뚱한 곳을 향함. 사용자가 Isaac UI에서
    # rotation을 base_link 좌표계 기준으로 튜닝하면 cone 다시 추가 가능.
    # 지금은 omni 발산 = jackal 주변 균등 조명.
    # 만약 이전 호출에서 ShapingAPI 적용된 상태였다면 cone 속성 제거.
    if light_prim.HasAPI(UsdLux.ShapingAPI):
        # cone angle을 매우 크게 = 사실상 omni
        shaping = UsdLux.ShapingAPI(light_prim)
        cone_attr = shaping.GetShapingConeAngleAttr()
        if cone_attr:
            cone_attr.Set(180.0)

    action = "추가" if created else "pose/속성 업데이트"
    print(f"[cobot3.spot/jackal] 헤드라이트 {action} 완료 ({JACKAL_HEADLIGHT_PRIM_PATH}) [omni mode]")


def _add_jackal_rescue_box():
    """Jackal 상단에 빨간 구호상자 (Cube prim, visual only).

    Idempotent — 기존 prim 있으면 transform/색상만 갱신.
    """
    import omni.kit.commands

    stage = omni.usd.get_context().get_stage()
    created = False
    if not stage.GetPrimAtPath(JACKAL_RESCUE_BOX_PATH).IsValid():
        omni.kit.commands.execute(
            "CreatePrim",
            prim_type="Cube",
            prim_path=JACKAL_RESCUE_BOX_PATH,
        )
        created = True

    box_prim = stage.GetPrimAtPath(JACKAL_RESCUE_BOX_PATH)
    UsdGeom.Imageable(box_prim).MakeVisible()

    xform = UsdGeom.Xformable(box_prim)
    existing = {op.GetOpName(): op for op in xform.GetOrderedXformOps()}

    if "xformOp:translate" in existing:
        existing["xformOp:translate"].Set(Gf.Vec3d(*JACKAL_RESCUE_BOX_TRANSLATION))
    else:
        xform.AddTranslateOp().Set(Gf.Vec3d(*JACKAL_RESCUE_BOX_TRANSLATION))

    if "xformOp:rotateXYZ" in existing:
        existing["xformOp:rotateXYZ"].Set(Gf.Vec3f(*JACKAL_RESCUE_BOX_ROTATION_XYZ_DEG))
    else:
        xform.AddRotateXYZOp().Set(Gf.Vec3f(*JACKAL_RESCUE_BOX_ROTATION_XYZ_DEG))

    if "xformOp:scale" in existing:
        existing["xformOp:scale"].Set(Gf.Vec3f(*JACKAL_RESCUE_BOX_SCALE))
    else:
        xform.AddScaleOp().Set(Gf.Vec3f(*JACKAL_RESCUE_BOX_SCALE))

    cube = UsdGeom.Cube(box_prim)
    cube.CreateDisplayColorAttr([Gf.Vec3f(*JACKAL_RESCUE_BOX_COLOR_RGB)])

    action = "추가" if created else "pose/색상 업데이트"
    print(f"[cobot3.spot/jackal] 구호상자 {action} 완료 ({JACKAL_RESCUE_BOX_PATH})")
