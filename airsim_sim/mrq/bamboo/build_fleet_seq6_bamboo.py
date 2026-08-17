# Runs HEADLESS in UE 5.6 (KungfuRender project):
#   UnrealEditor-Cmd KungfuRender.uproject -ExecutePythonScript=build_fleet_seq6.py -nullrhi
#
# Port of hercules build_fleet_seq.py (5.2) to 5.6, merged with the kungfu
# project's PROVEN MRQ crash recipe (ue5/importer/mrq_render.py):
#   * WARM=3 margin: camera-cut + every transform section starts at -WARM so the
#     TSR warm-up tick at frame -1 always has an active camera cut (their 2nd
#     root cause: no cut during warm-up -> UpdateViewTarget null SIGSEGV).
#   * aa.use_camera_cut_for_warm_up=True, render_warm_up_frames=False.
#   * CineCamera Tick crash workaround: disable actor tick (live + serialized
#     start_with_tick_enabled), clear lookat_tracking (enable=False, actor=None)
#     and focus tracking actor handle, focus DISABLE. (Their S5 crash: the
#     camera's Tick resolving a stale packed object handle -> MakeObjectRef OOB.)
#   * LINEAR key interpolation on dense per-frame keys (AUTO cubic tangents
#     ripple between frames under TSR temporal sub-sampling).
#   * GameOverride: GameModeBase + soft_game_mode_override, texture FULLY_LOAD,
#     flush_streaming_managers, cinematic_quality_settings, use_lod_zero,
#     disable HLODs.
#   * SSR cvar re-enable (project ini disables SSR globally; the pack's PPV
#     requests it).
# DELTA vs their recipe: the camera here is a SPAWNABLE (their possessable-cam
# fix requires saving the LEVEL, which is off-limits in this precious project).
# Their spawnable crash was in the in-editor PIE executor; this pipeline renders
# via the -game manifest bootstrap (hercules stage-B recipe, no PIE duplicate).
# All other defenses are kept.
#
# NOTHING outside /Game/FleetMRQ is ever saved. The map is loaded, never saved.
#
# Env: KEYS_JSON (required), OUT_DIR, TEST=0/1, TEST_START, TEST_END,
#      DRONE_SCALE (default 1.0), FBX_PATH, TEX_DIR, MRQ_MAP_PATH
import json
import os
import unreal

W = "/home/lucas/UE5/hercules-sim-big/mrq_work/bamboo"
_PROG = open(W + "/stageA_clean_progress.txt", "a")


def P(*a):
    s = " ".join(str(x) for x in a)
    print(s)
    _PROG.write(s + "\n")
    _PROG.flush()


KEYS_JSON = os.environ["KEYS_JSON"]
OUT_DIR = os.environ.get("OUT_DIR", W + "/out/frames")
TEST = os.environ.get("TEST", "0") == "1"
TEST_START = int(os.environ.get("TEST_START", "600"))
TEST_END = int(os.environ.get("TEST_END", "615"))
DRONE_SCALE = float(os.environ.get("DRONE_SCALE", "1.0"))
FBX_PATH = os.environ.get("FBX_PATH", W + "/quad_export/QuadCopter.fbx")
TEX_DIR = os.environ.get("TEX_DIR", W + "/quad_export")
MAP_PATH = os.environ.get("MRQ_MAP_PATH", "/Game/SwarmVideo/BambooGrove")
SEQ_DIR = "/Game/FleetMRQ"
SEQ_NAME = "SEQ_FleetTemple"
MESH_ASSET = SEQ_DIR + "/QuadCopter"     # the FBX's mesh name (5.2 export)
WARM = 3

K = json.load(open(KEYS_JSON))
FPS, N = K["fps"], K["nframes"]
P(f"[fleet6] keys: fps={FPS} nframes={N} run={K['info']['run']} anchor={K['info'].get('temple_transform')}")

les = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
eas = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
ues = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)
at = unreal.AssetToolsHelpers.get_asset_tools()

# ---------- delete stale sequence + paint assets FIRST (the old sequence
# references the old MICs/materials; deleting in this order avoids
# reference-blocked deletes and guarantees fresh HLSL on every bake) ----------
for stale in ([f"{SEQ_DIR}/SEQ_FleetTemple"] +
              [f"{SEQ_DIR}/MIC_Ribbon_{v}" for v in ("ghost", "delta", "buckshee", "thunderstrike")] +
              [f"{SEQ_DIR}/M_CoveragePaint", f"{SEQ_DIR}/M_Ribbon", f"{SEQ_DIR}/MPC_Coverage"]):
    if unreal.EditorAssetLibrary.does_asset_exist(stale):
        try:
            _o = unreal.load_asset(stale)
            unreal.EditorAssetLibrary.delete_loaded_asset(_o)
        except Exception as _de:
            P(f"[fleet6] stale delete {stale} failed:", _de)

# ---------- import the quad FBX + textures into /Game/FleetMRQ (BEFORE map load;
# kungfu note: Map_Load must not share heavy work, so front-load the imports) ----------
mesh = None
if unreal.EditorAssetLibrary.does_asset_exist(MESH_ASSET):
    m = unreal.EditorAssetLibrary.load_asset(MESH_ASSET)
    if isinstance(m, unreal.StaticMesh):
        mesh = m
        P("[fleet6] quad mesh already imported:", MESH_ASSET)
if mesh is None:
    tasks = []
    t = unreal.AssetImportTask()
    t.filename = FBX_PATH
    t.destination_path = SEQ_DIR
    t.automated = True
    t.save = True
    t.replace_existing = True
    ui = unreal.FbxImportUI()
    ui.import_mesh = True
    ui.import_as_skeletal = False
    ui.import_animations = False
    ui.import_materials = True
    ui.import_textures = True
    ui.static_mesh_import_data.combine_meshes = True
    ui.static_mesh_import_data.remove_degenerates = True
    t.options = ui
    tasks.append(t)
    for tex in os.listdir(TEX_DIR):
        if tex.lower().endswith(".tga"):
            tt = unreal.AssetImportTask()
            tt.filename = os.path.join(TEX_DIR, tex)
            tt.destination_path = SEQ_DIR
            tt.automated = True
            tt.save = True
            tt.replace_existing = True
            tasks.append(tt)
    at.import_asset_tasks(tasks)
    for a in unreal.EditorAssetLibrary.list_assets(SEQ_DIR):
        P("[fleet6] imported:", a)
    for cand in unreal.EditorAssetLibrary.list_assets(SEQ_DIR):
        aa_ = unreal.EditorAssetLibrary.load_asset(cand.split(".")[0])
        if isinstance(aa_, unreal.StaticMesh):
            mesh = aa_
            break
if mesh is None:
    raise RuntimeError("no StaticMesh under " + SEQ_DIR + " after import")
bb = mesh.get_bounding_box()
P(f"[fleet6] mesh {mesh.get_path_name()} extent="
  f"({bb.max.x-bb.min.x:.0f},{bb.max.y-bb.min.y:.0f},{bb.max.z-bb.min.z:.0f}) cm")

# if any slot has no real material (gray WorldGridMaterial), build a simple one
# from the body texture so the drones read dark against the temple.
try:
    mats = mesh.get_editor_property("static_materials")
    body_tex = None
    for cand in unreal.EditorAssetLibrary.list_assets(SEQ_DIR):
        if "Body" in cand:
            a2 = unreal.EditorAssetLibrary.load_asset(cand.split(".")[0])
            if isinstance(a2, unreal.Texture2D):
                body_tex = a2
                break
    need_fix = []
    for i, sm in enumerate(mats):
        mi = sm.get_editor_property("material_interface")
        nm = mi.get_name() if mi else "None"
        P(f"[fleet6] slot {i} -> {nm}")
        if mi is None or "WorldGrid" in nm:
            need_fix.append(i)
    if need_fix:
        mat = None
        if unreal.EditorAssetLibrary.does_asset_exist(SEQ_DIR + "/M_QuadBody"):
            mat = unreal.EditorAssetLibrary.load_asset(SEQ_DIR + "/M_QuadBody")
        else:
            mat = at.create_asset("M_QuadBody", SEQ_DIR, unreal.Material, unreal.MaterialFactoryNew())
            mel = unreal.MaterialEditingLibrary
            if body_tex is not None:
                ts = mel.create_material_expression(mat, unreal.MaterialExpressionTextureSample, -400, 0)
                ts.texture = body_tex
                mel.connect_material_property(ts, "RGB", unreal.MaterialProperty.MP_BASE_COLOR)
            else:
                c = mel.create_material_expression(mat, unreal.MaterialExpressionConstant3Vector, -400, 0)
                c.constant = unreal.LinearColor(0.04, 0.04, 0.05, 1.0)
                mel.connect_material_property(c, "", unreal.MaterialProperty.MP_BASE_COLOR)
            r = mel.create_material_expression(mat, unreal.MaterialExpressionConstant, -400, 220)
            r.r = 0.55
            mel.connect_material_property(r, "", unreal.MaterialProperty.MP_ROUGHNESS)
            mel.recompile_material(mat)
        for i in need_fix:
            sm = mats[i]
            sm.set_editor_property("material_interface", mat)
            mats[i] = sm
        mesh.set_editor_property("static_materials", mats)
        P("[fleet6] patched", len(need_fix), "gray slots with M_QuadBody")
except Exception as e:
    P("[fleet6] material patch skipped:", e)

unreal.EditorAssetLibrary.save_directory(SEQ_DIR, only_if_is_dirty=True)

# ============ COVERAGE PAINT + RIBBONS: REMOVED (user feedback: clean
# cinematic, no embedded graphics). See build_fleet_seq6.py for the
# original sections. Stale-asset deletion above still scrubs old paint
# assets out of /Game/FleetMRQ. ============
#
# v6 (EXECUTION-PLAN-V6 §2 SHOT 5): ribbons are BACK, env-gated so the clean
# default is untouched.  RIBBONS=1 imports temple/ribbon_<v>.obj (baked by
# bake_ribbons_v6.py, u = chronological flight fraction), builds an unlit
# additive reveal material driven by MPC CoverageTime, and (below, after the
# camera-cut track) adds 4 ribbon spawnables + CoverageTime keys that hold 0
# until RIBBON_REVEAL="f0:f1", sweep 0->1 across [f0,f1], then hold 1.
RIBBONS = os.environ.get("RIBBONS", "0") == "1"
rib_meshes, rib_mics, mpc = {}, {}, None
if RIBBONS:
    DRONE_ORDER = ["ghost", "delta", "buckshee", "thunderstrike"]
    # §3 legend colors: 3f8cff / 3fff72 / ff4cf2 / ff941e
    DRONE_COLORS = {"ghost": (0.247, 0.549, 1.00),
                    "delta": (0.247, 1.00, 0.447),
                    "buckshee": (1.00, 0.298, 0.949),
                    "thunderstrike": (1.00, 0.580, 0.118)}
    mel = unreal.MaterialEditingLibrary
    tasks = []
    for v in DRONE_ORDER:
        to = unreal.AssetImportTask()
        to.filename = W + f"/ribbon_{v}.obj"
        to.destination_path = SEQ_DIR
        to.automated = True
        to.save = False
        to.replace_existing = True
        tasks.append(to)
    at.import_asset_tasks(tasks)
    sms_sub = None
    try:
        sms_sub = unreal.get_editor_subsystem(unreal.StaticMeshEditorSubsystem)
    except Exception:
        pass
    for v in DRONE_ORDER:
        rm = unreal.EditorAssetLibrary.load_asset(f"{SEQ_DIR}/ribbon_{v}")
        assert isinstance(rm, unreal.StaticMesh), f"ribbon_{v} import failed"
        try:  # Interchange auto-enables Nanite; unlit translucent needs it OFF
            ns = rm.get_editor_property("nanite_settings")
            if ns.get_editor_property("enabled"):
                ns.set_editor_property("enabled", False)
                if sms_sub is not None:
                    sms_sub.set_nanite_settings(rm, ns, True)
                else:
                    rm.set_editor_property("nanite_settings", ns)
        except Exception as _ne:
            P(f"[fleet6] ribbon_{v} nanite disable failed:", _ne)
        rib_meshes[v] = rm
        bb2 = rm.get_bounding_box()
        P(f"[fleet6] ribbon_{v} bounds X[{bb2.min.x:.0f},{bb2.max.x:.0f}] "
          f"Y[{bb2.min.y:.0f},{bb2.max.y:.0f}] Z[{bb2.min.z:.0f},{bb2.max.z:.0f}]")
    mpc = at.create_asset("MPC_Coverage", SEQ_DIR, unreal.MaterialParameterCollection,
                          unreal.MaterialParameterCollectionFactoryNew())
    _sp = unreal.CollectionScalarParameter()
    _sp.set_editor_property("parameter_name", "CoverageTime")
    _sp.set_editor_property("default_value", 0.0)
    mpc.set_editor_property("scalar_parameters", [_sp])
    RIBBON_HLSL = """
float u = UV.x;
if (u > CT) return float4(0,0,0,0);
float dt = CT - u;
float head = saturate(1.0 - dt*40.0);
float3 e = COL.rgb * (0.85 + 10.0*head*head);
return float4(e, 1.0);
"""

    def custom_input(nm_):
        ci = unreal.CustomInput()
        ci.set_editor_property("input_name", nm_)
        return ci

    m_rib = at.create_asset("M_Ribbon", SEQ_DIR, unreal.Material,
                            unreal.MaterialFactoryNew())
    m_rib.set_editor_property("blend_mode", unreal.BlendMode.BLEND_ADDITIVE)
    m_rib.set_editor_property("shading_model", unreal.MaterialShadingModel.MSM_UNLIT)
    m_rib.set_editor_property("two_sided", True)
    cu2 = mel.create_material_expression(m_rib, unreal.MaterialExpressionCustom, -700, 0)
    cu2.set_editor_property("code", RIBBON_HLSL)
    cu2.set_editor_property("output_type", unreal.CustomMaterialOutputType.CMOT_FLOAT4)
    cu2.set_editor_property("inputs", [custom_input("UV"), custom_input("CT"),
                                       custom_input("COL")])
    uvx = mel.create_material_expression(m_rib, unreal.MaterialExpressionTextureCoordinate, -1000, -100)
    cp2 = mel.create_material_expression(m_rib, unreal.MaterialExpressionCollectionParameter, -1000, 0)
    cp2.set_editor_property("collection", mpc)
    cp2.set_editor_property("parameter_name", "CoverageTime")
    vcol = mel.create_material_expression(m_rib, unreal.MaterialExpressionVectorParameter, -1000, 100)
    vcol.set_editor_property("parameter_name", "DroneColor")
    vcol.set_editor_property("default_value", unreal.LinearColor(1, 1, 1, 1))
    mel.connect_material_expressions(uvx, "", cu2, "UV")
    mel.connect_material_expressions(cp2, "", cu2, "CT")
    mel.connect_material_expressions(vcol, "", cu2, "COL")
    mk2 = mel.create_material_expression(m_rib, unreal.MaterialExpressionComponentMask, -400, 0)
    mk2.set_editor_property("r", True)
    mk2.set_editor_property("g", True)
    mk2.set_editor_property("b", True)
    mk2.set_editor_property("a", False)
    mel.connect_material_expressions(cu2, "", mk2, "")
    mel.connect_material_property(mk2, "", unreal.MaterialProperty.MP_EMISSIVE_COLOR)
    mel.recompile_material(m_rib)
    for v in DRONE_ORDER:
        mic = at.create_asset(f"MIC_Ribbon_{v}", SEQ_DIR, unreal.MaterialInstanceConstant,
                              unreal.MaterialInstanceConstantFactoryNew())
        mel.set_material_instance_parent(mic, m_rib)
        r, g, b = DRONE_COLORS[v]
        # RIBBON_GAIN: the reveal material is UNLIT ADDITIVE, so its brightness
        # competes with whatever is already in the frame.  The temple was a dusk
        # ruin; a sunlit bamboo grove is ~an order of magnitude brighter, and at
        # gain 1 the trails vanish into the litter.  Scale the emissive colour.
        _g = float(os.environ.get("RIBBON_GAIN", "1.0"))
        mel.set_material_instance_vector_parameter_value(
            mic, "DroneColor", unreal.LinearColor(r * _g, g * _g, b * _g, 1))
        rib_mics[v] = mic
    P("[fleet6] v6 ribbons: 4 OBJs + M_Ribbon + MICs + MPC built")
    unreal.EditorAssetLibrary.save_directory(SEQ_DIR, only_if_is_dirty=True)

# ---------- load the map (never saved) ----------
ok = les.load_level(MAP_PATH)
P(f"[fleet6] load_level({MAP_PATH}) -> {ok}")
world = ues.get_editor_world()

# ---------- sanity: ground below frame0 + path-clearance sweep ----------
g0 = K["drones"]["ghost"][0]
try:
    hit = unreal.SystemLibrary.line_trace_single(
        world, unreal.Vector(g0[0], g0[1], g0[2]),
        unreal.Vector(g0[0], g0[1], g0[2] - 1500.0),
        unreal.TraceTypeQuery.TRACE_TYPE_QUERY1, True, [],
        unreal.DrawDebugTrace.NONE, True)
    if hit:
        ip = hit.to_tuple()[4]
        P(f"[fleet6] ghost frame0 z={g0[2]:.0f}, ground below at z={ip.z:.0f} (clearance {g0[2]-ip.z:.0f} cm)")
    else:
        P("[fleet6] note: no ground hit below ghost frame0 (over water?)")
except Exception as e:
    P(f"[fleet6] trace sanity skipped: {e}")


def sweep(name, rows, zoff=0.0):
    blocked = 0
    first = None
    for k in range(0, len(rows) - 10, 10):
        a, b = rows[k], rows[k + 10]
        hit = unreal.SystemLibrary.line_trace_single(
            world, unreal.Vector(a[0], a[1], a[2] + zoff),
            unreal.Vector(b[0], b[1], b[2] + zoff),
            unreal.TraceTypeQuery.TRACE_TYPE_QUERY1, True, [],
            unreal.DrawDebugTrace.NONE, True)
        if hit:
            blocked += 1
            if first is None:
                ip = hit.to_tuple()[4]
                first = (k, round(ip.x), round(ip.y), round(ip.z))
    P(f"[fleet6] sweep {name}: {blocked} blocked segments" + (f" first@frame{first}" if first else ""))
    return blocked


tot = 0
for name, rows in K["drones"].items():
    tot += sweep(name, rows)
tot += sweep("camera", K["camera"])
P(f"[fleet6] total blocked segments: {tot} (0 expected)")

# ---------- fresh sequence asset (kungfu delete/reuse fallback) ----------
seq_path = f"{SEQ_DIR}/{SEQ_NAME}"
seq = None
if unreal.EditorAssetLibrary.does_asset_exist(seq_path):
    try:
        _old = unreal.load_asset(seq_path)
        unreal.EditorAssetLibrary.delete_loaded_asset(_old)
    except Exception as _de:
        P("[fleet6] delete_loaded_asset failed:", _de)
if not unreal.EditorAssetLibrary.does_asset_exist(seq_path):
    seq = at.create_asset(SEQ_NAME, SEQ_DIR, unreal.LevelSequence, unreal.LevelSequenceFactoryNew())
if seq is None:
    seq = unreal.load_asset(seq_path)
    X = unreal.MovieSceneSequenceExtensions
    for _b in list(X.get_bindings(seq)):
        unreal.MovieSceneBindingExtensions.remove(_b)
    for _t in list(X.get_tracks(seq)):
        X.remove_track(seq, _t)
    P("[fleet6] REUSED+CLEARED existing sequence")
assert seq is not None
seq.set_display_rate(unreal.FrameRate(FPS, 1))
seq.set_playback_start(0)
seq.set_playback_end(N)
P(f"[fleet6] created {seq_path}")

# 5.2 name was SequenceTimeUnit; 5.3+ renamed it MovieSceneTimeUnit
_TU = getattr(unreal, "MovieSceneTimeUnit", None) or getattr(unreal, "SequenceTimeUnit")
DR = _TU.DISPLAY_RATE
LIN = unreal.MovieSceneKeyInterpolation.LINEAR


def key_channel(ch, vals):
    add = ch.add_key
    add(unreal.FrameNumber(-WARM), vals[0], 0.0, DR, LIN)     # warm-up seed
    for f, v in enumerate(vals):
        add(unreal.FrameNumber(f), v, 0.0, DR, LIN)


def add_transform_keys(binding, loc_xyz, rot_pyr, scale=1.0):
    tr = binding.add_track(unreal.MovieScene3DTransformTrack)
    sec = tr.add_section()
    sec.set_range(-WARM, N)
    ch = sec.get_all_channels()  # LocX,LocY,LocZ,Roll,Pitch,Yaw,SclX,SclY,SclZ
    for i in range(3):
        key_channel(ch[i], loc_xyz[i])
    for i, vals in enumerate(rot_pyr):
        if vals is not None:
            key_channel(ch[3 + i], vals)
    for i in (6, 7, 8):
        ch[i].set_default(scale)
    return sec


# 5.6 DELTA: eas.spawn_actor_from_class SIGFPEs under -nullrhi (the 5.6 editor
# routes spawns through FLevelEditorViewportClient::TryPlacingActorFromObject ->
# FViewport::GetHitProxy -> FSceneViewport::EnqueueBeginRenderFrame, which
# divides by the zero-size headless viewport; confirmed in crash pid-2324509).
# So: NEVER spawn a template actor. Build spawnables from CLASS and configure
# the binding's OBJECT TEMPLATE directly (also strictly safer for the precious
# map: no actor ever enters the level, even transiently).
def make_spawnable_from_class(cls, name):
    try:
        sp = seq.add_spawnable_from_class(cls)
    except AttributeError:
        sp = unreal.MovieSceneSequenceExtensions.add_spawnable_from_class(seq, cls)
    sp.set_name(name)
    tmpl = None
    try:
        tmpl = unreal.MovieSceneBindingExtensions.get_object_template(sp)
    except Exception as e:
        P("[fleet6] get_object_template unavailable:", e)
    return sp, tmpl


# ---------- hide the kungfu project's staged G1 robots (test frame 0607 showed
# ~10 white humanoids in shot). Possessable binding + visibility=False track in
# THIS sequence hides them at render time WITHOUT touching the level. ----------
hidden = []


def _is_robot(a):
    try:
        lbl = a.get_actor_label()
    except Exception:
        return False, ""
    if lbl.startswith("G1") or lbl.startswith("Class_"):
        return True, lbl
    # RuralCabins showcase clutter: glowing orange gate markers floating in the
    # scene (11x GateMarker actors) — hide at render time, never touch the level
    if lbl.startswith("GateMarker") or "GateMarker" in a.get_class().get_name():
        return True, lbl
    # SwarmVideo/BambooGrove set dressing that is NOT ours: the 10 parked swarm
    # drones (Swarm0..9_Body/_PropA..D) and the whole Beat-3 explainer VFX rig
    # (B3_Rev*/B3_Plan*/B3_Vox*/B3_Sharo*/B3_Cone/B3_Pulse/...).  Hidden by a
    # visibility track in OUR sequence — the level is never touched.
    if lbl.startswith("Swarm") or lbl.startswith("B3_"):
        return True, lbl
    # Bamboo_Forest_Nanite_Map greenscreen rig (GS_SUB backdrop, GS_KEY spot,
    # GS_*Cam) + the human scale reference, if that map is ever used instead.
    if lbl.startswith("GS_") or lbl == "SM_Male_Reference":
        return True, lbl
    try:  # any poseable/skeletal-mesh carrier is kungfu set dressing in this map
        if list(a.get_components_by_class(unreal.PoseableMeshComponent)):
            return True, lbl
        if list(a.get_components_by_class(unreal.SkeletalMeshComponent)):
            return True, lbl
    except Exception:
        pass
    return False, lbl


for a in eas.get_all_level_actors():
    ok_, lbl = _is_robot(a)
    if ok_:
        try:
            b = seq.add_possessable(a)
            vt = b.add_track(unreal.MovieSceneVisibilityTrack)
            vs = vt.add_section()
            vs.set_range(-WARM, N)
            vch = vs.get_all_channels()[0]
            try:
                vch.add_key(unreal.FrameNumber(-WARM), False, 0.0, DR)
            except Exception:
                vch.add_key(unreal.FrameNumber(-WARM), False)
            hidden.append(lbl)
        except Exception as _he:
            P(f"[fleet6] hide {lbl} failed:", _he)
P(f"[fleet6] hid {len(hidden)} G1 actors: {hidden}")

# ---------- drone spawnables ----------
for name, rows in K["drones"].items():
    sp, tmpl = make_spawnable_from_class(unreal.StaticMeshActor, f"Fleet_{name}")
    assert tmpl is not None, "FATAL: no object template on spawnable " + name
    smc = tmpl.get_editor_property("static_mesh_component")
    smc.set_editor_property("mobility", unreal.ComponentMobility.MOVABLE)
    smc.set_static_mesh(mesh)
    smc.set_collision_enabled(unreal.CollisionEnabled.NO_COLLISION)
    xs = [r[0] for r in rows]
    ys = [r[1] for r in rows]
    zs = [r[2] for r in rows]
    yaws = [r[3] for r in rows]
    add_transform_keys(sp, (xs, ys, zs), (None, None, yaws), scale=DRONE_SCALE)
    P(f"[fleet6] baked {name}: {N} frames")

# ---------- camera spawnable (kungfu Tick-crash defenses applied) ----------
cam_rows = K["camera"]
cam_sp, tmp = make_spawnable_from_class(unreal.CineCameraActor, "FleetCam")
assert tmp is not None, "FATAL: no object template on camera spawnable"
# 5.6 DELTA: on the spawnable OBJECT TEMPLATE the native getter
# get_cine_camera_component() returns None (cached pointer not fixed up on
# templates -- the same null-component disease behind kungfu's PIE crashes).
# Read the CameraComponent UPROPERTY directly instead (that worked for the
# StaticMeshActor templates' static_mesh_component too).
cc = None
for tag, getter in (("native_getter", lambda a: a.get_cine_camera_component()),
                    ("camera_component_prop", lambda a: a.get_editor_property("camera_component")),
                    ("components_by_class", lambda a: (list(a.get_components_by_class(unreal.CineCameraComponent)) + [None])[0])):
    try:
        cc = getter(tmp)
    except Exception as _ge:
        P(f"[fleet6] cam component via {tag} failed:", _ge)
    if cc is not None:
        P(f"[fleet6] cam component via {tag}: {cc.get_class().get_name()}")
        break
assert cc is not None, "FATAL: no CineCameraComponent reachable on camera template"
fb = cc.get_editor_property("filmback")
fb.sensor_width = 23.76
fb.sensor_height = 13.365
cc.set_editor_property("filmback", fb)
cc.set_editor_property("current_focal_length", 18.0)

# ---- exposure ----------------------------------------------------------
# BambooGrove's unbound PostProcessVolume runs MANUAL exposure at bias +5.0,
# which is tuned for the editor viewport; through MRQ's -game bootstrap the
# same setting renders the grove blown out to near-white.  The camera's own
# post-process is applied AFTER every volume, so overriding the bias here
# fixes exposure without touching the level.  CAM_EV_BIAS is the absolute
# auto-exposure bias to use; leave it unset to inherit the volume.
_ev = os.environ.get("CAM_EV_BIAS", "")
if _ev != "":
    try:
        pp = cc.get_editor_property("post_process_settings")
        pp.set_editor_property("auto_exposure_method",
                               unreal.AutoExposureMethod.AEM_MANUAL)
        pp.set_editor_property("override_auto_exposure_method", True)
        pp.set_editor_property("auto_exposure_bias", float(_ev))
        pp.set_editor_property("override_auto_exposure_bias", True)
        pp.set_editor_property("auto_exposure_min_brightness", 1.0)
        pp.set_editor_property("auto_exposure_max_brightness", 1.0)
        pp.set_editor_property("override_auto_exposure_min_brightness", True)
        pp.set_editor_property("override_auto_exposure_max_brightness", True)
        for _k, _v in [("bloom_intensity", float(os.environ.get("CAM_BLOOM", "0.35"))),
                       ("vignette_intensity", float(os.environ.get("CAM_VIGNETTE", "0.35")))]:
            pp.set_editor_property(_k, _v)
            pp.set_editor_property("override_" + _k, True)
        cc.set_editor_property("post_process_settings", pp)
        cc.set_editor_property("post_process_blend_weight", 1.0)
        P(f"[fleet6] CAM_EV_BIAS applied: {_ev}")
    except Exception as _ee:
        P("[fleet6] CAM_EV_BIAS failed:", _ee)
fs = cc.get_editor_property("focus_settings")
fs.focus_method = unreal.CameraFocusMethod.DISABLE
try:  # null the focus tracking handle (kungfu: stale handle -> MakeObjectRef SIGSEGV)
    _tfs = fs.get_editor_property("tracking_focus_settings")
    _tfs.set_editor_property("actor_to_track", None)
    fs.set_editor_property("tracking_focus_settings", _tfs)
except Exception as _fe:
    P("[fleet6] focus actor_to_track clear failed:", _fe)
cc.set_editor_property("focus_settings", fs)
try:
    tmp.set_actor_tick_enabled(False)
except Exception as _te:
    P("[fleet6] template set_actor_tick_enabled skipped:", _te)
try:  # serialized -> the render-world copy won't tick either
    pat = tmp.get_editor_property("primary_actor_tick")
    pat.set_editor_property("start_with_tick_enabled", False)
    tmp.set_editor_property("primary_actor_tick", pat)
    P("[fleet6] CAM_TICK_DISABLED (start_with_tick_enabled=False)")
except Exception as _e:
    P("[fleet6] cam tick serialized set failed:", _e)
try:  # clear lookat tracking (the S5 Tick crash handle)
    _lts = tmp.get_editor_property("lookat_tracking_settings")
    _lts.set_editor_property("enable_look_at_tracking", False)
    _lts.set_editor_property("actor_to_track", None)
    tmp.set_editor_property("lookat_tracking_settings", _lts)
    P("[fleet6] LOOKAT_TRACKING_CLEARED")
except Exception as _le:
    P("[fleet6] lookat tracking clear failed:", _le)
add_transform_keys(
    cam_sp,
    ([r[0] for r in cam_rows], [r[1] for r in cam_rows], [r[2] for r in cam_rows]),
    (None, [r[3] for r in cam_rows], [r[4] for r in cam_rows]))

cut = seq.add_track(unreal.MovieSceneCameraCutTrack)
cs = cut.add_section()
cs.set_range(-WARM, N)
try:
    bid = seq.get_binding_id(cam_sp)
except Exception:
    try:
        bid = cam_sp.get_binding_id()
    except Exception:
        bid = unreal.MovieSceneSequenceExtensions.get_binding_id(seq, cam_sp)
cs.set_camera_binding_id(bid)
try:
    cs.set_completion_mode(unreal.MovieSceneCompletionMode.KEEP_STATE)
    P("[fleet6] CUT_KEEP_STATE ok")
except Exception as _e:
    P("[fleet6] CUT_KEEP_STATE failed", _e)
P("[fleet6] camera + cut track done")


# ---------- coverage-paint spawnables: REMOVED (clean cinematic) ----------

# ---------- MPC clock track: REMOVED (clean cinematic) ----------

# ---------- v6 ribbon spawnables + CoverageTime reveal track ----------
if RIBBONS:
    def static_transform(binding, loc, rot_pyr=(0.0, 0.0, 0.0)):
        tr2 = binding.add_track(unreal.MovieScene3DTransformTrack)
        s2 = tr2.add_section()
        s2.set_range(-WARM, N)
        ch2 = s2.get_all_channels()
        for i, v2 in enumerate(loc):
            ch2[i].set_default(float(v2))
        ch2[3].set_default(float(rot_pyr[2]))
        ch2[4].set_default(float(rot_pyr[0]))
        ch2[5].set_default(float(rot_pyr[1]))
        for i in (6, 7, 8):
            ch2[i].set_default(1.0)

    for v in ["ghost", "delta", "buckshee", "thunderstrike"]:
        rsp, rtmpl = make_spawnable_from_class(unreal.StaticMeshActor, f"Ribbon_{v}")
        assert rtmpl is not None
        rsmc = rtmpl.get_editor_property("static_mesh_component")
        rsmc.set_editor_property("mobility", unreal.ComponentMobility.MOVABLE)
        rsmc.set_static_mesh(rib_meshes[v])
        rsmc.set_collision_enabled(unreal.CollisionEnabled.NO_COLLISION)
        try:
            rsmc.set_material(0, rib_mics[v])
        except Exception as _me:
            P(f"[fleet6] ribbon {v} material override failed:", _me)
        static_transform(rsp, (0.0, 0.0, 0.0))
    # CoverageTime: 0 until f0, sweep to 1 across [f0,f1], hold 1
    f0, f1 = (int(x) for x in os.environ.get("RIBBON_REVEAL", "0:60").split(":"))
    mpct = seq.add_track(unreal.MovieSceneMaterialParameterCollectionTrack)
    mpct.set_editor_property("mpc", mpc)
    msec = mpct.add_section()
    msec.set_range(-WARM, N)
    # -0.01 pre-reveal so even the u=0 first path point stays hidden
    msec.add_scalar_parameter_key("CoverageTime", unreal.FrameNumber(0), -0.01)
    mch = msec.get_all_channels()[0]
    for f2, val in ((f0, -0.01), (f1, 1.0), (N - 1, 1.0)):
        if f2 != 0:
            mch.add_key(unreal.FrameNumber(f2), float(val), 0.0, DR, LIN)
    P(f"[fleet6] 4 ribbon spawnables + reveal keys [{f0},{f1}]")

unreal.EditorAssetLibrary.save_directory(SEQ_DIR, only_if_is_dirty=False)
P("[fleet6] saved /Game/FleetMRQ (nothing else)")

# ---------- MRQ queue -> manifest (hercules stage-B bootstrap + kungfu settings) ----------
qsub = unreal.get_editor_subsystem(unreal.MoviePipelineQueueSubsystem)
q = qsub.get_queue()
for j in list(q.get_jobs()):
    q.delete_job(j)
# TEST_RANGES="a:b,c:d,..." -> one queue job per range (single -game process,
# single map load, N tiny renders) for cheap multi-timestamp shot verification.
TEST_RANGES = os.environ.get("TEST_RANGES", "")
if TEST_RANGES:
    ranges = [tuple(int(x) for x in r.split(":")) for r in TEST_RANGES.split(",")]
elif TEST:
    ranges = [(TEST_START, TEST_END)]
else:
    ranges = [None]

for ri, rng in enumerate(ranges):
    job = q.allocate_new_job(unreal.MoviePipelineExecutorJob)
    job.job_name = f"FleetTemple_{ri}"
    job.map = unreal.SoftObjectPath(f"{MAP_PATH}.{MAP_PATH.split('/')[-1]}")
    job.sequence = unreal.SoftObjectPath(f"{seq_path}.{SEQ_NAME}")
    cfg = job.get_configuration()

    outset = cfg.find_or_add_setting_by_class(unreal.MoviePipelineOutputSetting)
    outset.output_directory = unreal.DirectoryPath(OUT_DIR)
    outset.file_name_format = "frame_{frame_number}"
    outset.output_resolution = unreal.IntPoint(1920, 1080)
    outset.override_existing_output = True
    outset.zero_pad_frame_numbers = 4
    outset.use_custom_frame_rate = True
    outset.output_frame_rate = unreal.FrameRate(FPS, 1)
    if rng is not None:
        outset.use_custom_playback_range = True
        outset.custom_start_frame = rng[0]
        outset.custom_end_frame = rng[1]
        P(f"[fleet6] job{ri} range [{rng[0]},{rng[1]})")

    cfg.find_or_add_setting_by_class(unreal.MoviePipelineImageSequenceOutput_PNG)
    cfg.find_or_add_setting_by_class(unreal.MoviePipelineDeferredPassBase)
    aa = cfg.find_or_add_setting_by_class(unreal.MoviePipelineAntiAliasingSetting)
    aa.spatial_sample_count = 1
    aa.temporal_sample_count = 4
    aa.override_anti_aliasing = True
    aa.anti_aliasing_method = unreal.AntiAliasingMethod.AAM_TSR
    aa.render_warm_up_frames = False
    aa.render_warm_up_count = 16
    try:
        aa.use_camera_cut_for_warm_up = True
    except Exception as _e:
        P("[fleet6] USE_CAMERACUT_WARMUP failed", _e)

    go = cfg.find_or_add_setting_by_class(unreal.MoviePipelineGameOverrideSetting)
    try:
        go.set_editor_property("game_mode_override", unreal.GameModeBase)
    except Exception as _e:
        P("[fleet6] GM_OVERRIDE class failed", _e)
    try:
        go.set_editor_property("soft_game_mode_override", unreal.GameModeBase)
    except Exception as _e:
        P("[fleet6] GM_OVERRIDE soft failed", _e)
    for prop, val in [("texture_streaming", unreal.MoviePipelineTextureStreamingMethod.FULLY_LOAD),
                      ("flush_streaming_managers", True),
                      ("cinematic_quality_settings", True),
                      ("disable_hlo_ds", True),
                      ("use_lod_zero", True)]:
        try:
            go.set_editor_property(prop, val)
        except Exception as _e:
            P(f"[fleet6] GO {prop} failed:", _e)

    cv = cfg.find_or_add_setting_by_class(unreal.MoviePipelineConsoleVariableSetting)
    CVARS = [("r.MotionBlurQuality", 0.0), ("r.Tonemapper.Sharpen", 0.3),
             ("r.ScreenSpaceReflections", 1.0), ("r.SSR.Quality", 4.0)]
    try:
        for k2, v2 in CVARS:
            cv.add_or_update_console_variable(k2, v2)
    except Exception:
        _cvars = cv.get_editor_property("console_variables")
        for k2, v2 in CVARS:
            _cvars[k2] = v2
        cv.set_editor_property("console_variables", _cvars)
P(f"[fleet6] queued {len(ranges)} job(s)")

res = unreal.MoviePipelineEditorLibrary.save_queue_to_manifest_file(q)
P(f"[fleet6] manifest saved: {res}")
P("[fleet6] BUILD_OK")
