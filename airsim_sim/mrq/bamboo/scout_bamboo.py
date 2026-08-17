# Headless in UE 5.6 RuralCabins project: load Rural_Cabins READ-ONLY, census
# actors, wide grid line-trace for ground height -> SCOUT_OUT json.
import collections, json, os, unreal

MAP = os.environ.get("SCOUT_MAP", "/Game/Modular_Rural_Cabin/Maps/Rural_Cabins")
OUT = os.environ["SCOUT_OUT"]
GRID = os.environ.get("SCOUT_GRID", "-20000,20001,-20000,20001,1000")
x0, x1, y0, y1, step = (int(v) for v in GRID.split(","))
PROG = open("/home/lucas/UE5/hercules-sim-big/mrq_work/bamboo/scout_progress.txt", "a")

def P(*a):
    s = " ".join(str(x) for x in a)
    print(s); PROG.write(s + "\n"); PROG.flush()

les = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
ues = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)
eas = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
ok = les.load_level(MAP)
world = ues.get_editor_world()
P("LOADED", MAP, ok, world.get_name() if world else None)

counts = collections.Counter()
boxes = []
for a in eas.get_all_level_actors():
    cn = a.get_class().get_name()
    counts[cn] += 1
    try:
        o, e = a.get_actor_bounds(False)
        if e.x > 50 and cn not in ("Landscape", "LandscapeStreamingProxy"):
            boxes.append([a.get_actor_label(), cn, round(o.x), round(o.y), round(o.z),
                          round(e.x), round(e.y), round(e.z)])
    except Exception:
        pass
P("ACTORS", json.dumps(dict(counts.most_common(40))))

def trace(x, y, z0=100000.0, z1=-100000.0):
    hit = unreal.SystemLibrary.line_trace_single(
        world, unreal.Vector(x, y, z0), unreal.Vector(x, y, z1),
        unreal.TraceTypeQuery.TRACE_TYPE_QUERY1, True, [],
        unreal.DrawDebugTrace.NONE, True)
    if hit:
        t = hit.to_tuple()
        act = t[9]
        return [round(t[4].z, 1), act.get_actor_label() if act else "?"]
    return None

grid = {}
n = 0
for x in range(x0, x1, step):
    for y in range(y0, y1, step):
        grid["%d,%d" % (x, y)] = trace(float(x), float(y))
        n += 1
P("TRACED", n)
json.dump({"map": MAP, "grid": grid, "step": step, "boxes": boxes}, open(OUT, "w"))
P("SCOUT_OK", OUT)
