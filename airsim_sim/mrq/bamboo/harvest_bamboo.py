# Headless UE 5.6 SwarmReplay: harvest the Bamboo_Forest_Nanite_Map into
#   (a) a ground-height grid over the region of interest (line traces),
#   (b) compact per-instance obstacle records (XY + radius + z-top) for every
#       foliage instance in the ROI,
#   (c) a full list of non-foliage actors with world AABBs.
# READ-ONLY: load_level only, nothing is ever saved.
#
# env: HARVEST_OUT, HARVEST_MAP, ROI="x0,x1,y0,y1" cm, TRACE_STEP cm
import json, os, unreal

W = "/home/lucas/UE5/hercules-sim-big/mrq_work/bamboo"
OUT = os.environ.get("HARVEST_OUT", W + "/bamboo_harvest.json")
MAP = os.environ.get("HARVEST_MAP", "/Game/Bamboo_Forest/Maps/Bamboo_Forest_Nanite_Map")
ROI = [float(v) for v in os.environ.get("ROI", "186000,218000,186000,218000").split(",")]
TRACE_STEP = float(os.environ.get("TRACE_STEP", "400"))
X0, X1, Y0, Y1 = ROI
PROG = open(W + "/scout_progress.txt", "a")


def P(*a):
    s = " ".join(str(x) for x in a)
    print(s); PROG.write(s + "\n"); PROG.flush()


les = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
ues = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)
eas = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
les.load_level(MAP)
world = ues.get_editor_world()
P("HARVEST loaded", MAP, "ROI", ROI)

# ---------- (c) non-foliage actors ----------
actors = []
for a in eas.get_all_level_actors():
    cn = a.get_class().get_name()
    try:
        lbl = a.get_actor_label()
    except Exception:
        lbl = "?"
    rec = {"label": lbl, "cls": cn}
    try:
        o, e = a.get_actor_bounds(False)
        rec["o"] = [round(o.x), round(o.y), round(o.z)]
        rec["e"] = [round(e.x), round(e.y), round(e.z)]
    except Exception:
        pass
    try:
        loc = a.get_actor_location()
        rec["loc"] = [round(loc.x), round(loc.y), round(loc.z)]
    except Exception:
        pass
    actors.append(rec)
P("HARVEST actors", len(actors))

# ---------- (b) foliage instances in the ROI ----------
inst = []
meshinfo = {}
n_seen = 0
for a in eas.get_all_level_actors():
    # NOTE: FoliageInstancedStaticMeshComponent is missed by both the base-class
    # filter and isinstance() in this build (swarm lush_bamboo_report gotcha) —
    # match on the component class-name substring instead.
    try:
        comps = [c for c in a.get_components_by_class(unreal.ActorComponent)
                 if "InstancedStaticMesh" in c.get_class().get_name()]
    except Exception:
        comps = []
    for c in comps:
        try:
            sm = c.get_editor_property("static_mesh")
        except Exception:
            sm = getattr(c, "static_mesh", None)
        if not sm:
            continue
        name = sm.get_name()
        lb = sm.get_bounding_box()
        if name not in meshinfo:
            meshinfo[name] = {"min": [round(lb.min.x, 1), round(lb.min.y, 1), round(lb.min.z, 1)],
                              "max": [round(lb.max.x, 1), round(lb.max.y, 1), round(lb.max.z, 1)],
                              "count_roi": 0, "count_total": 0}
        rx = max(abs(lb.min.x), abs(lb.max.x))
        ry = max(abs(lb.min.y), abs(lb.max.y))
        rlocal = max(rx, ry)
        ztop_local = lb.max.z
        n = c.get_instance_count()
        meshinfo[name]["count_total"] += n
        n_seen += n
        for i in range(n):
            xf = c.get_instance_transform(i, True)
            if not isinstance(xf, unreal.Transform):
                xf = xf[1]
            t = xf.translation
            if not (X0 <= t.x <= X1 and Y0 <= t.y <= Y1):
                continue
            s = xf.scale3d
            sxy = max(abs(s.x), abs(s.y))
            inst.append([round(t.x, 1), round(t.y, 1), round(t.z, 1),
                         round(rlocal * sxy, 1), round(ztop_local * abs(s.z), 1), name])
            meshinfo[name]["count_roi"] += 1
P("HARVEST instances total", n_seen, "in ROI", len(inst))

# non-instanced StaticMeshActor props (BambooGrove ground tiles, swarm drone
# meshes, individual stalks) — treat as obstacles unless they read as ground.
smactors = []
for a in eas.get_all_level_actors():
    if a.get_class().get_name() != "StaticMeshActor":
        continue
    try:
        lbl = a.get_actor_label()
        o, e = a.get_actor_bounds(False)
    except Exception:
        continue
    smactors.append({"label": lbl,
                     "wmin": [round(o.x - e.x, 1), round(o.y - e.y, 1), round(o.z - e.z, 1)],
                     "wmax": [round(o.x + e.x, 1), round(o.y + e.y, 1), round(o.z + e.z, 1)]})
P("HARVEST staticmesh actors", len(smactors))
for k, v in sorted(meshinfo.items(), key=lambda kv: -kv[1]["count_total"]):
    P("   mesh", k, "total", v["count_total"], "roi", v["count_roi"],
      "bb", v["min"], v["max"])

# ---------- (a) ground grid ----------
def trace(x, y):
    hit = unreal.SystemLibrary.line_trace_single(
        world, unreal.Vector(x, y, 60000.0), unreal.Vector(x, y, -20000.0),
        unreal.TraceTypeQuery.TRACE_TYPE_QUERY1, True, [],
        unreal.DrawDebugTrace.NONE, True)
    if hit:
        t = hit.to_tuple()
        act = t[9]
        return [round(t[4].z, 1), (act.get_actor_label() if act else "?")]
    return None


grid = {}
x = X0
while x <= X1:
    y = Y0
    while y <= Y1:
        grid["%d,%d" % (int(x), int(y))] = trace(x, y)
        y += TRACE_STEP
    x += TRACE_STEP
P("HARVEST traced", len(grid))

json.dump({"map": MAP, "roi": ROI, "trace_step": TRACE_STEP,
           "grid": grid, "inst": inst, "meshinfo": meshinfo, "actors": actors, "smactors": smactors},
          open(OUT, "w"))
P("HARVEST_OK", OUT, os.path.getsize(OUT))
