#!/usr/bin/env python3
"""
Live 2D map of the sweep mission. Standalone -- no ROS/PX4 imports at all.
Polls the JSON snapshot px4_qr_sweep_node.py writes (~5 Hz) and redraws:
arena bounds, discovered red zone, found/rejected QR tags, the CURRENT
planned route (redrawn from scratch every replan), the flown trail, and
the drone's live position.

Run alongside the mission node, e.g. in a second terminal:
    python3 qr_mission_viewer.py
    python3 qr_mission_viewer.py --file ~/qr_mission_live.json --interval 200
"""
import argparse
import json
import os

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Polygon as MplPolygon, Rectangle

STATUS_STYLE = {
    #                color        marker  legend label    short on-map tag
    "read":          ("tab:green", "*", "read (no target set)", "READ"),
    "target_found":  ("gold",      "*", "TARGET MATCH",         "MATCH!"),
    "read_nonmatch": ("tab:orange","o", "read, not a match",    "no match"),
    "approach_timeout": ("tab:red","x", "approach timed out",   "timed out"),
    "unreachable":   ("tab:red",   "x", "unreachable (boxed in by zone)", "unreachable"),
    "lost":          ("tab:red",   "x", "lost mid-read",        "lost"),
    "unreadable":    ("tab:red",   "x", "could not decode",     "unreadable"),
}


def load(path):
    try:
        with open(path) as f:
            return json.load(f)
    except Exception:                                    # noqa: BLE001
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--file', default=os.path.expanduser('~/qr_mission_live.json'))
    ap.add_argument('--interval', type=int, default=200, help='redraw period, ms')
    args = ap.parse_args()

    fig, ax = plt.subplots(figsize=(12, 7))
    fig.subplots_adjust(left=0.07, right=0.72, top=0.88, bottom=0.08)  # reserve room for the side panel
    seen_labels = set()

    def draw(_frame):
        snap = load(args.file)
        ax.clear()
        seen_labels.clear()
        if snap is None:
            ax.text(0.5, 0.5, f"waiting for {args.file} ...", ha='center', va='center', transform=ax.transAxes)
            ax.set_xticks([]); ax.set_yticks([])
            return

        bounds = snap.get("bounds") or []
        if bounds:
            bx = [p[1] for p in bounds] + [bounds[0][1]]
            by = [p[0] for p in bounds] + [bounds[0][0]]
            ax.plot(bx, by, 'k-', lw=1.5, label='arena bounds')
            pad = 2.0
            ax.set_xlim(min(p[1] for p in bounds) - pad, max(p[1] for p in bounds) + pad)
            ax.set_ylim(min(p[0] for p in bounds) - pad, max(p[0] for p in bounds) + pad)

        # Sweep grid: same cells the coverage planner uses, shaded by status.
        cells = snap.get("cells") or []
        ch, cw = snap.get("cell_size", [0, 0]) or [0, 0]
        if cells and ch and cw:
            visited = set(snap.get("visited_cells") or [])
            blocked = set(snap.get("blocked_cells") or [])
            for idx, (n, e) in enumerate(cells):
                if idx in blocked:
                    fc, ec, a = 'red', 'darkred', 0.18
                elif idx in visited:
                    fc, ec, a = 'tab:green', 'darkgreen', 0.14
                else:
                    fc, ec, a = 'none', '0.8', 1.0
                ax.add_patch(Rectangle((e - cw / 2, n - ch / 2), cw, ch,
                                       facecolor=fc, edgecolor=ec, alpha=a, lw=0.6))
            ax.add_patch(Rectangle((0, 0), 0, 0, facecolor='tab:green', edgecolor='darkgreen',
                                   alpha=0.3, label='cell: covered'))
            ax.add_patch(Rectangle((0, 0), 0, 0, facecolor='red', edgecolor='darkred',
                                   alpha=0.3, label='cell: blocked'))
            ax.add_patch(Rectangle((0, 0), 0, 0, facecolor='none', edgecolor='0.8',
                                   label='cell: pending'))

        red = snap.get("red_zone") or []
        if len(red) >= 3:
            poly = MplPolygon([(p[1], p[0]) for p in red], closed=True,
                              facecolor='red', alpha=0.35, edgecolor='darkred', lw=1.5,
                              label=f"red zone (~{snap.get('red_zone_area', 0):.0f} m²)")
            ax.add_patch(poly)

        trail = snap.get("trail") or []
        if len(trail) > 1:
            ax.plot([p[1] for p in trail], [p[0] for p in trail], '-', color='0.6', lw=0.8, label='flown path')

        route = snap.get("route") or []
        if route:
            run = [route[0]]
            for prev, cur in zip(route, route[1:]):
                if cur[0] != prev[0]:
                    _draw_route_run(ax, run)
                    run = [cur]
                else:
                    run.append(cur)
            _draw_route_run(ax, run)
            ax.plot([], [], color='tab:blue', lw=1.8, label='planned lane')
            ax.plot([], [], color='tab:blue', lw=1.2, ls='--', label='planned transit')

        for tag in snap.get("tags") or []:
            color, marker, label, tag_word = STATUS_STYLE.get(
                tag["status"], ("gray", "o", tag["status"], tag["status"]))
            lg = label if label not in seen_labels else None
            seen_labels.add(label)
            ax.plot(tag["e"], tag["n"], marker=marker, color=color, ms=13, mew=2,
                   linestyle='None', label=lg)
            payload = tag.get("payload") or "?"
            ax.annotate(f"#{tag['id']} {payload}\n{tag_word}", (tag["e"], tag["n"]),
                       textcoords='offset points', xytext=(8, 6), fontsize=7.5,
                       color=color, fontweight='bold',
                       bbox=dict(boxstyle='round,pad=0.2', fc='white', ec=color, alpha=0.85))

        pos = snap.get("pos")
        if pos:
            ax.plot(pos[1], pos[0], marker='^', color='blue', ms=14, mec='k', mew=1, label='drone')

        target = snap.get("target_payload")
        target_line = f"TARGET TO FIND: '{target}'" if target else "TARGET: none configured (recording every tag)"
        status_line = f"state={snap.get('state', '?')}  h={snap.get('height', 0):.1f} m  t={snap.get('t', 0):.0f}s"
        ax.set_title(f"{target_line}\n{status_line}", fontsize=10,
                    color=('darkgoldenrod' if target else 'black'), fontweight='bold')
        ax.set_xlabel('east [m]'); ax.set_ylabel('north [m]')
        ax.set_aspect('equal')
        # Legend moves out of the map entirely (it used to sit on top of the
        # bottom-left corner) -- placed in the reserved right margin, below
        # the status panel.
        ax.legend(loc='upper left', bbox_to_anchor=(1.02, 0.5), fontsize=7, framealpha=0.9)

        # Status readout: what's been found so far and whether the target has
        # turned up, independent of colour perception / marker crowding.
        tags = snap.get("tags") or []
        found = next((t for t in tags if t["status"] == "target_found"), None)
        lines = []
        if target:
            lines.append(f"TARGET: '{target}'  ->  "
                         + (f"FOUND (#{found['id']})" if found else "searching..."))
        else:
            lines.append("No target configured -- recording every tag read.")
        for t in tags:
            _, _, _, w = STATUS_STYLE.get(t["status"], ("gray", "o", t["status"], t["status"]))
            lines.append(f"  #{t['id']}: {t.get('payload') or '(undecoded)'}  [{w}]")
        ax.text(1.02, 1.0, "\n".join(lines), transform=ax.transAxes, fontsize=8,
               va='top', ha='left', family='monospace',
               bbox=dict(boxstyle='round', fc='ivory', ec='0.5'))

    def _draw_route_run(ax, run):
        style = dict(color='tab:blue', lw=1.8) if run[0][0] >= 0 else dict(color='tab:blue', lw=1.2, ls='--')
        ax.plot([p[2] for p in run], [p[1] for p in run], **style)

    anim = FuncAnimation(fig, draw, interval=args.interval, cache_frame_data=False)
    plt.show()


if __name__ == '__main__':
    main()
