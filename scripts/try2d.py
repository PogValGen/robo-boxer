"""Scratch: try keyframe sequences in path2d. Each step: (dt, dict(A=shoulder,E=elbow,H=hip,K=knee,N=ankle)) -> held values."""
import sys, json
import path2d, gaits
MAP = {"A": "arm{}_shoulder", "E": "arm{}_elbow", "H": "leg{}_hippitch", "K": "leg{}_knee", "N": "leg{}_ankle"}
def build(steps):
    cur = {}; t = 0.5; seq = [(t, {})]
    for dt, d in steps:
        for k, v in d.items():
            for s in "AB":
                cur[MAP[k].format(s)] = float(v)
        t += dt; seq.append((t, dict(cur)))
    return gaits.keyseq(seq), t + 0.5
if __name__ == "__main__":
    steps = json.loads(sys.argv[2]); th0 = {"chest": 90.0, "back": -90.0}[sys.argv[1]]
    cmds, dur = build([(s[0], s[1]) for s in steps])
    out = path2d.run(cmds, dur, th0, 0.05, verbose=False)
    last = None
    for t, th, e in out:
        key = (round(th / 10), e["pa"][:8], e["pb"][:8])
        if key != last:
            print(f"{t:5.2f} th {th:+7.1f} margin {min(e['s'], e['L'] - e['s']):5.1f}/{e['L']:5.1f} h {e['hc']:5.1f} {e['pa']}..{e['pb']}")
        last = key
    t, th, e = out[-1]; print(f"END   th {th:+7.1f} margin {min(e['s'], e['L'] - e['s']):5.1f}/{e['L']:5.1f} {e['pa']}..{e['pb']}")
