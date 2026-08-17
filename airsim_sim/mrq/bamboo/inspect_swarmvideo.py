# Headless UE 5.6 SwarmReplay: read the SWARM VIDEO's art direction so the
# fleet cinematic looks like its sibling.  READ-ONLY (load_level, load_asset).
#   * per-map actor census with lighting / fog / PPV / camera settings
#   * CineCameraActor focal length, aperture, filmback, focus settings
#   * LevelSequence camera-cut bindings + camera component values
import json, os, unreal

W = "/home/lucas/UE5/hercules-sim-big/mrq_work/bamboo"
OUT = os.environ.get("INSPECT_OUT", W + "/swarmvideo_look.json")
MAPS = os.environ.get("INSPECT_MAPS",
                      "/Game/SwarmVideo/BambooGrove,/Game/SwarmVideo/FormationGrove").split(",")
SEQS = os.environ.get("INSPECT_SEQS",
                      "/Game/SwarmVideo/SEQ_FullFlight,/Game/SwarmVideo/SEQ_LushFly,"
                      "/Game/SwarmVideo/SEQ_Beat3,/Game/SEQ_BAMBOO_CINE").split(",")
PROG = open(W + "/scout_progress.txt", "a")


def P(*a):
    s = " ".join(str(x) for x in a)
    print(s); PROG.write(s + "\n"); PROG.flush()


def props(obj, names):
    d = {}
    for n in names:
        try:
            v = obj.get_editor_property(n)
        except Exception:
            continue
        try:
            if isinstance(v, (int, float, bool, str)):
                d[n] = v
            elif isinstance(v, unreal.Vector):
                d[n] = [round(v.x, 2), round(v.y, 2), round(v.z, 2)]
            elif isinstance(v, unreal.Rotator):
                d[n] = [round(v.pitch, 2), round(v.roll, 2), round(v.yaw, 2)]
            elif isinstance(v, unreal.LinearColor):
                d[n] = [round(v.r, 3), round(v.g, 3), round(v.b, 3), round(v.a, 3)]
            else:
                d[n] = str(v)[:400]
        except Exception:
            pass
    return d


les = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
ues = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)
eas = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)

res = {"maps": {}, "seqs": {}}
for MAP in MAPS:
    if not unreal.EditorAssetLibrary.does_asset_exist(MAP):
        P("MISSING MAP", MAP); continue
    les.load_level(MAP)
    P("LOADED", MAP)
    m = {"actors": [], "counts": {}, "ism": 0, "extent": None}
    ext = [1e9, 1e9, 1e9, -1e9, -1e9, -1e9]
    for a in eas.get_all_level_actors():
        cn = a.get_class().get_name()
        m["counts"][cn] = m["counts"].get(cn, 0) + 1
        try:
            lbl = a.get_actor_label()
        except Exception:
            lbl = "?"
        rec = {"label": lbl, "cls": cn}
        try:
            loc = a.get_actor_location(); rot = a.get_actor_rotation()
            rec["loc"] = [round(loc.x), round(loc.y), round(loc.z)]
            rec["rot"] = [round(rot.pitch, 1), round(rot.roll, 1), round(rot.yaw, 1)]
            o, e = a.get_actor_bounds(False)
            if e.x < 1e7 and "Sky" not in cn and "Cloud" not in cn:
                ext[0] = min(ext[0], o.x - e.x); ext[1] = min(ext[1], o.y - e.y); ext[2] = min(ext[2], o.z - e.z)
                ext[3] = max(ext[3], o.x + e.x); ext[4] = max(ext[4], o.y + e.y); ext[5] = max(ext[5], o.z + e.z)
            rec["e"] = [round(e.x), round(e.y), round(e.z)]
        except Exception:
            pass
        try:
            for c in a.get_components_by_class(unreal.InstancedStaticMeshComponent):
                m["ism"] += c.get_instance_count()
        except Exception:
            pass
        # -------- look-bearing actors --------
        if "Light" in cn:
            for c in a.get_components_by_class(unreal.LightComponent):
                rec["light"] = props(c, ["intensity", "light_color", "temperature",
                                         "use_temperature", "cast_shadows",
                                         "volumetric_scattering_intensity",
                                         "source_angle", "indirect_lighting_intensity"])
        if cn == "PostProcessVolume":
            try:
                s = a.get_editor_property("settings")
                rec["ppv"] = props(s, [
                    "auto_exposure_method", "auto_exposure_bias", "auto_exposure_min_brightness",
                    "auto_exposure_max_brightness", "bloom_intensity", "bloom_threshold",
                    "vignette_intensity", "film_slope", "film_toe", "film_shoulder",
                    "color_saturation", "color_contrast", "color_gamma", "color_gain",
                    "ambient_occlusion_intensity", "ambient_occlusion_radius",
                    "lumen_scene_lighting_quality", "lumen_final_gather_quality",
                    "ray_tracing_ao", "motion_blur_amount", "motion_blur_max",
                    "depth_of_field_focal_distance", "depth_of_field_fstop",
                    "color_grading_lut", "color_grading_intensity",
                    "dynamic_global_illumination_method", "reflection_method"])
                rec["ppv_unbound"] = a.get_editor_property("unbound")
                rec["ppv_priority"] = a.get_editor_property("priority")
                rec["ppv_blend_radius"] = a.get_editor_property("blend_radius")
            except Exception as e:
                rec["ppv_err"] = str(e)
        if cn == "ExponentialHeightFog":
            for c in a.get_components_by_class(unreal.ExponentialHeightFogComponent):
                rec["fog"] = props(c, ["fog_density", "fog_height_falloff", "fog_inscattering_color",
                                       "fog_max_opacity", "start_distance",
                                       "volumetric_fog", "volumetric_fog_scattering_distribution",
                                       "volumetric_fog_albedo", "volumetric_fog_extinction_scale",
                                       "volumetric_fog_distance", "directional_inscattering_exponent",
                                       "directional_inscattering_color",
                                       "second_fog_data"])
        if "CineCamera" in cn or "Camera" in cn:
            for c in a.get_components_by_class(unreal.CineCameraComponent):
                rec["cam"] = props(c, ["current_focal_length", "current_aperture",
                                       "field_of_view", "post_process_blend_weight"])
                try:
                    fb = c.get_editor_property("filmback")
                    rec["cam"]["filmback"] = [round(fb.sensor_width, 3), round(fb.sensor_height, 3)]
                except Exception:
                    pass
                try:
                    fs = c.get_editor_property("focus_settings")
                    rec["cam"]["focus"] = props(fs, ["focus_method", "manual_focus_distance",
                                                     "smooth_focus_changes"])
                except Exception:
                    pass
                try:
                    rec["cam"]["pp"] = props(c.get_editor_property("post_process_settings"),
                                             ["bloom_intensity", "auto_exposure_bias",
                                              "motion_blur_amount", "vignette_intensity"])
                except Exception:
                    pass
        m["actors"].append(rec)
    m["extent"] = [round(v) for v in ext]
    res["maps"][MAP] = m
    P("  actors", len(m["actors"]), "ism", m["ism"], "extent", m["extent"])
    P("  counts", json.dumps(dict(sorted(m["counts"].items(), key=lambda kv: -kv[1])[:16])))

# -------- level sequences --------
for S in SEQS:
    if not unreal.EditorAssetLibrary.does_asset_exist(S):
        P("MISSING SEQ", S); continue
    try:
        seq = unreal.load_asset(S)
        info = {"display_rate": str(seq.get_display_rate()),
                "playback_start": seq.get_playback_start(),
                "playback_end": seq.get_playback_end(), "bindings": []}
        for b in seq.get_bindings():
            bi = {"name": b.get_display_name(), "tracks": []}
            for t in b.get_tracks():
                bi["tracks"].append(t.get_class().get_name())
            try:
                tmpl = unreal.MovieSceneBindingExtensions.get_object_template(b)
                if tmpl:
                    for c in tmpl.get_components_by_class(unreal.CineCameraComponent):
                        bi["cam"] = props(c, ["current_focal_length", "current_aperture"])
                        try:
                            fb = c.get_editor_property("filmback")
                            bi["cam"]["filmback"] = [round(fb.sensor_width, 3), round(fb.sensor_height, 3)]
                        except Exception:
                            pass
            except Exception:
                pass
            info["bindings"].append(bi)
        res["seqs"][S] = info
        P("SEQ", S, "bindings", len(info["bindings"]), info["display_rate"],
          info["playback_start"], info["playback_end"])
        for bi in info["bindings"][:20]:
            P("   ", bi["name"], bi.get("cam", ""), bi["tracks"][:4])
    except Exception as e:
        P("SEQ FAIL", S, e)

json.dump(res, open(OUT, "w"), indent=1)
P("INSPECT_OK", OUT)
