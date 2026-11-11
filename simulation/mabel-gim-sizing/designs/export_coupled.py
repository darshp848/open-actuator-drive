"""Export every set of results/per_joint/coupled.json (picks per joint + torso_resized) for the MABEL rig.
Run on the Inspiron: python3 -B export_coupled.py   -> ./designs_coupled/<set>.json + sets.json, and a CAD-coverage report.
Exits with status 2 (and writes nothing usable) if the CAD index misses an entry, so a rerun is safe to repeat."""
import sys, json, subprocess
sys.dont_write_bytecode = True
from pathlib import Path
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import export_lib as X
OUT = HERE / "designs_coupled"
OUT.mkdir(exist_ok=True)
C = json.loads((X.RES / "coupled.json").read_text())
sets_done, missing = [], []
GROUP = {"J2": "J1", "J4": "J3", "J6": "J5", "J7": "J5"}
for name, s in C["sets"].items():
    out = {"set": name, "magnet": s["family"], "case": "coupled." + s["case"], "provenance": {
        "repo": X.REPO, "commit": X.commit, "cad_index": "robot_rotor/cad/per_joint/index.json",
        "files": ["robot_rotor/results/per_joint/coupled.json"], "export_script": "export_coupled.py",
        "converged": s["converged"], "passes": len(s["passes"]) if isinstance(s["passes"], list) else s["passes"],
        "arm_mass_kg": s["passes"][-1]["arm_mass_kg"] if isinstance(s["passes"], list) and s["passes"] else None,
        "torso": "torso_resized (sized for this set's arm masses)", "group_mapping": "J2/J4/J6/J7 reuse the J1/J3/J5 CAD entry when the pick is identical"}, "joints": {}}
    for j in range(1, 8):
        x = s["picks"]["J%d" % j]
        if x is None:
            missing.append((name, "J%d" % j, "NO PICK (infeasible)")); continue
        key = "J%d" % j
        for_cad = ("coupled." + name)
        grp = key
        hit = X.CAD_BY.get((key, x["family"], for_cad))
        if hit is None and key in GROUP:
            # build_pj_cad.py builds J1/J3/J5/Torso only for non-main sets; reuse the group's CAD when the pick is identical
            g = GROUP[key]
            xg = s["picks"][g]
            if xg is not None and all(xg.get(f) == x.get(f) for f in ("motor", "L", "gear", "ratio", "board_D")):
                grp = g
                hit = X.CAD_BY.get((g, x["family"], for_cad))
        if hit is None:
            missing.append((name, key, for_cad)); continue
        e = X.export(x, key, grp, x["family"], for_cad, req=s["req"])
        e["source"] = "coupled.json sets.%s.picks.%s" % (name, key)
        out["joints"]["Arm%d" % j] = e
    t = s["torso_resized"]
    if t is None:
        missing.append((name, "Torso", "NO torso_resized (infeasible)")); t = None
    cs = "coupled." + name + ".torso_resized"
    if t is None:
        pass
    elif X.CAD_BY.get(("Torso", t["family"], cs)) is None:
        missing.append((name, "Torso", cs))
    else:
        e = X.export(t, "Torso", "Torso", t["family"], cs, req=s["req"])
        e["source"] = "coupled.json sets.%s.torso_resized" % name
        out["joints"]["Torso"] = e
    if not any(m[0] == name for m in missing):
        (OUT / (name + ".json")).write_text(json.dumps(out, indent=1))
        sets_done.append(name)
(OUT / "sets.json").write_text(json.dumps(sets_done))
print("exported", len(sets_done), "of", len(C["sets"]), "sets")
if missing:
    print("MISSING CAD entries:", len(missing))
    for m in missing[:20]: print("  ", m)
    sys.exit(2)
