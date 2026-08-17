# Headless UE 5.6 SwarmReplay: census the candidate Bamboo_Forest maps
# READ-ONLY (load_level only, never save) -> census_bamboo.json
import collections, json, os, unreal

W = "/home/lucas/UE5/hercules-sim-big/mrq_work/bamboo"
OUT = os.environ.get("CENSUS_OUT", W + "/census_bamboo.json")
MAPS = os.environ.get("CENSUS_MAPS", ",".join([
    "/Game/Bamboo_Forest/Maps/Bamboo_Forest_Nanite_Map",
    "/Game/Bamboo_Forest/Maps/Bamboo_Forest_LOD_Map",
    "/Game/Bamboo_Forest/Maps/Bamboo_Forest_Overview",
])).split(",")
PROG = open(W + "/scout_progress.txt", "a")


def P(*a):
    s = " ".join(str(x) for x in a)
    print(s); PROG.write(s + "\n"); PROG.flush()


les = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
ues = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)
eas = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)

res = {}
for MAP in MAPS:
    try:
        ok = les.load_level(MAP)
    except Exception as e:
        P("LOAD_FAIL", MAP, e); res[MAP] = {"error": str(e)}; continue
    world = ues.get_editor_world()
    P("LOADED", MAP, ok, world.get_name() if world else None)

    counts = collections.Counter()
    ext = [1e9, 1e9, 1e9, -1e9, -1e9, -1e9]
    foliage_inst = 0
    lights = []
    big = []
    for a in eas.get_all_level_actors():
        cn = a.get_class().get_name()
        counts[cn] += 1
        try:
            o, e = a.get_actor_bounds(False)
            if e.x < 1e7 and cn != "SkyAtmosphere" and "Sky" not in cn:
                ext[0] = min(ext[0], o.x - e.x); ext[1] = min(ext[1], o.y - e.y); ext[2] = min(ext[2], o.z - e.z)
                ext[3] = max(ext[3], o.x + e.x); ext[4] = max(ext[4], o.y + e.y); ext[5] = max(ext[5], o.z + e.z)
            if e.x > 3000 or e.y > 3000:
                big.append([a.get_actor_label(), cn, round(o.x), round(o.y), round(o.z),
                            round(e.x), round(e.y), round(e.z)])
        except Exception:
            pass
        try:
            for c in a.get_components_by_class(unreal.InstancedStaticMeshComponent):
                foliage_inst += c.get_instance_count()
        except Exception:
            pass
        if "Light" in cn:
            try:
                lights.append([a.get_actor_label(), cn])
            except Exception:
                pass
    res[MAP] = {"actors": dict(counts.most_common(50)), "extent": [round(v) for v in ext],
                "ism_instances": foliage_inst, "lights": lights, "big": big[:40],
                "n_actors": sum(counts.values())}
    P("  actors", sum(counts.values()), "ism_inst", foliage_inst,
      "extent", [round(v) for v in ext])
    P("  top", json.dumps(dict(counts.most_common(12))))

json.dump(res, open(OUT, "w"), indent=1)
P("CENSUS_OK", OUT)
