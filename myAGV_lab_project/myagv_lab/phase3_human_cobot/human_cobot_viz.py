"""
myagv_lab/phase3_human_cobot/human_cobot_viz.py
================================================
Two-panel live visualiser for Phase 3 (human-in-the-loop delivery).

Identical to Phase 4's MissionVisualizer, except:
  • HumanCobot is used instead of SimCobot
  • When a load/unload step is waiting for the human helper, the right
    panel shows a highlighted "★ WAITING FOR HUMAN" banner and the
    matching plan step is styled with a yellow badge.

Usage (standalone demo)
-----------------------
  python3 -m myagv_lab.phase3_human_cobot.human_cobot_viz
  python3 -m myagv_lab.phase3_human_cobot.human_cobot_viz --scenario deliver_AB

Programmatic
------------
  from myagv_lab.phase3_human_cobot.human_cobot_viz import HumanCobotVisualizer
  viz = HumanCobotVisualizer()
  ok  = viz.run("Deliver package_A and return home.",
                use_llm=False, fallback_scenario="deliver_A")
  viz.show()

Threading model
---------------
The mission pipeline runs in a daemon thread.  The matplotlib event loop
runs on the main thread via plt.pause().  Thread-safe communication uses
queue.Queue of (event_type, data) tuples.  Human prompts print to the
terminal; the background thread blocks on input() while the main thread
keeps animating the plot.
"""

from __future__ import annotations

import math
import os
import queue
import sys
import threading
import time
from pathlib import Path
from typing import Optional

import matplotlib
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from myagv_lab.sim_layer import (
    SimMap, SimRobot, Pose2D,
    get_robot, get_map,
)
from myagv_lab.phase2_nav.nav_node import NavigationManager, WAYPOINTS
from myagv_lab.phase4_delivery.pddl_planner.llm_translator import (
    natural_language_to_pddl, fallback_pddl,
)
from myagv_lab.phase4_delivery.pddl_planner.pddl_solver import (
    solve_pddl, PlanStep,
)
from myagv_lab.phase4_delivery.pddl_planner.primitive_executor import (
    PrimitiveExecutor, StepResult,
)
from myagv_lab.phase3_human_cobot.human_cobot import HumanCobot

# ── Colour palette ────────────────────────────────────────────────────────────
_C = {
    "wall":       "#2d2d2d",
    "free":       "#f5f5f0",
    "trail":      "#4fc3f7",
    "robot":      "#ef5350",
    "goal":       "#66bb6a",
    "start":      "#ffa726",
    "wp":         "#7e57c2",
    "wp_text":    "#4a148c",
    "panel_bg":   "#1e1e2e",
    "text":       "#cdd6f4",
    "muted":      "#585b70",
    "pending":    "#6c7086",
    "running":    "#cba6f7",
    "done":       "#a6e3a1",
    "failed":     "#f38ba8",
    "waiting":    "#f9e2af",   # yellow — human action
    "stage_ok":   "#a6e3a1",
    "stage_run":  "#fab387",
    "stage_wait": "#6c7086",
}

_STEP_STYLE = {
    "pending":  ("○", _C["pending"]),
    "running":  ("►", _C["running"]),
    "waiting":  ("★", _C["waiting"]),   # human action pending
    "done":     ("✓", _C["done"]),
    "failed":   ("✗", _C["failed"]),
}

_STAGE_STYLE = {
    "pending": ("[ ]", _C["stage_wait"]),
    "running": ("[►]", _C["stage_run"]),
    "done":    ("[✓]", _C["stage_ok"]),
    "failed":  ("[✗]", _C["failed"]),
}


# ═══════════════════════════════════════════════════════════════════════════════
#  HUMAN COBOT VISUALIZER
# ═══════════════════════════════════════════════════════════════════════════════

class HumanCobotVisualizer:
    """
    Two-panel live visualiser for Phase 3 human-in-the-loop missions.

    Parameters
    ----------
    sim_map   : SimMap — uses global sim map if None
    waypoints : dict   — uses WAYPOINTS from nav_node if None
    """

    ARROW_LEN  = 0.25
    TRAIL_MAX  = 3000
    TIMELINE_N = 10

    def __init__(
        self,
        sim_map:   Optional[SimMap]            = None,
        waypoints: Optional[dict[str, Pose2D]] = None,
    ):
        self._map       = sim_map   or get_map()
        self._waypoints = waypoints or WAYPOINTS

        # ── Shared state ───────────────────────────────────────────────────────
        self._events:   queue.Queue         = queue.Queue()
        self._trail_x:  list[float]         = []
        self._trail_y:  list[float]         = []
        self._goal:     Optional[Pose2D]    = None
        self._start:    Optional[Pose2D]    = None
        self._robot:    Optional[SimRobot]  = None

        self._stages = [
            {"label": "Stage 1 — NL → PDDL",   "status": "pending"},
            {"label": "Stage 2 — PDDL → Plan",  "status": "pending"},
            {"label": "Stage 3 — Execution",     "status": "pending"},
        ]
        self._plan:           list[PlanStep]          = []
        self._step_state:     dict[int, str]          = {}
        self._current_status: str                     = "Initialising …"
        self._waiting_msg:    Optional[str]           = None
        self._timeline:       list[tuple[float, str]] = []
        self._t0:             float                   = time.monotonic()
        self._result:         Optional[bool]          = None

        # ── Figure setup ───────────────────────────────────────────────────────
        plt.ion()
        self._fig = plt.figure(figsize=(16, 7), facecolor=_C["panel_bg"])
        self._fig.canvas.manager.set_window_title(
            "Phase 3 — Human-in-the-loop Mission"
        )
        gs = GridSpec(1, 2, figure=self._fig,
                      width_ratios=[1.6, 1], wspace=0.04)
        self._ax_map  = self._fig.add_subplot(gs[0])
        self._ax_prog = self._fig.add_subplot(gs[1])

        self._build_map_layer()
        self._setup_progress_panel()

        # Dynamic map handles
        self._trail_line    = None
        self._robot_arrow   = None
        self._robot_circle  = None
        self._goal_marker   = None
        self._start_marker  = None

    # ══════════════════════════════════════════════════════════════════════════
    #  MAP PANEL
    # ══════════════════════════════════════════════════════════════════════════

    def _build_map_layer(self) -> None:
        ax  = self._ax_map
        res = self._map.resolution

        ax.set_facecolor(_C["free"])
        ax.set_title("Robot Navigation Map", color=_C["text"],
                     fontsize=11, pad=8)
        ax.set_xlabel("x  (m)", color=_C["muted"])
        ax.set_ylabel("y  (m)", color=_C["muted"])
        ax.tick_params(colors=_C["muted"])
        for spine in ax.spines.values():
            spine.set_edgecolor(_C["muted"])
        ax.set_aspect("equal")

        for row_idx, row in enumerate(self._map.grid):
            for col_idx, cell in enumerate(row):
                if cell == 1:
                    ax.add_patch(plt.Rectangle(
                        (col_idx * res, row_idx * res), res, res,
                        color=_C["wall"], zorder=1,
                    ))

        w = self._map.width  * res
        h = self._map.height * res
        ax.set_xlim(-0.1, w + 0.1)
        ax.set_ylim(-0.1, h + 0.1)

        for name, wp in self._waypoints.items():
            ax.plot(wp.x, wp.y, "o", color=_C["wp"],
                    markersize=8, zorder=3)
            ax.annotate(name, xy=(wp.x, wp.y),
                        xytext=(4, 6), textcoords="offset points",
                        fontsize=7.5, color=_C["wp_text"], zorder=4)

        ax.grid(True, linestyle=":", linewidth=0.4, color="#cccccc", zorder=0)

        handles = [
            mpatches.Patch(color=_C["wall"],  label="Wall"),
            mpatches.Patch(color=_C["free"],  label="Free"),
            plt.Line2D([0],[0], color=_C["trail"], lw=2,  label="Trail"),
            mpatches.Patch(color=_C["robot"], label="Robot"),
            mpatches.Patch(color=_C["goal"],  label="Goal"),
            plt.Line2D([0],[0], marker="o", color="w",
                       markerfacecolor=_C["wp"], markersize=8, label="Waypoint"),
        ]
        ax.legend(handles=handles, loc="upper right",
                  fontsize=7, framealpha=0.85)

    def _redraw_map(self) -> None:
        if self._robot is None:
            return
        ax   = self._ax_map
        pose = self._robot.position

        self._trail_x.append(pose.x)
        self._trail_y.append(pose.y)
        if len(self._trail_x) > self.TRAIL_MAX:
            self._trail_x = self._trail_x[-self.TRAIL_MAX:]
            self._trail_y = self._trail_y[-self.TRAIL_MAX:]

        if self._trail_line is not None:
            self._trail_line.remove()
        self._trail_line, = ax.plot(
            self._trail_x, self._trail_y,
            color=_C["trail"], linewidth=1.5, zorder=2, alpha=0.7,
        )

        if self._start_marker is not None:
            self._start_marker.remove()
            self._start_marker = None
        if self._start is not None:
            self._start_marker, = ax.plot(
                self._start.x, self._start.y, "*",
                color=_C["start"], markersize=14, zorder=4,
            )

        if self._goal_marker is not None:
            self._goal_marker.remove()
            self._goal_marker = None
        if self._goal is not None:
            self._goal_marker, = ax.plot(
                self._goal.x, self._goal.y, "D",
                color=_C["goal"], markersize=10, zorder=4,
            )

        if self._robot_circle is not None:
            self._robot_circle.remove()
        self._robot_circle = plt.Circle(
            (pose.x, pose.y), 0.10, color=_C["robot"], zorder=5,
        )
        ax.add_patch(self._robot_circle)

        if self._robot_arrow is not None:
            self._robot_arrow.remove()
        dx = self.ARROW_LEN * math.cos(pose.yaw)
        dy = self.ARROW_LEN * math.sin(pose.yaw)
        self._robot_arrow = ax.annotate(
            "", xy=(pose.x + dx, pose.y + dy),
            xytext=(pose.x, pose.y),
            arrowprops=dict(arrowstyle="-|>", color=_C["robot"],
                            lw=2.5, mutation_scale=18),
            zorder=6,
        )

    # ══════════════════════════════════════════════════════════════════════════
    #  PROGRESS PANEL
    # ══════════════════════════════════════════════════════════════════════════

    def _setup_progress_panel(self) -> None:
        ax = self._ax_prog
        ax.set_facecolor(_C["panel_bg"])
        ax.set_xlim(0, 1)
        ax.set_ylim(0, 1)
        ax.axis("off")
        self._prog_text = ax.text(
            0.05, 0.97, "",
            transform=ax.transAxes,
            fontsize=8.5,
            verticalalignment="top",
            fontfamily="monospace",
            color=_C["text"],
        )

    def _build_progress_text(self) -> str:
        lines: list[str] = []

        lines.append("══ MISSION PROGRESS ══════════════════")
        lines.append("")

        # Pipeline stages
        for s in self._stages:
            icon, _ = _STAGE_STYLE[s["status"]]
            lines.append(f"  {icon}  {s['label']}")
        lines.append("")

        # Plan steps
        if self._plan:
            lines.append("── Plan Steps ────────────────────────")
            for step in self._plan:
                state      = self._step_state.get(step.index, "pending")
                icon, _    = _STEP_STYLE[state]
                label      = f"{step.index}. [{step.name}]  {'  '.join(step.args)}"
                if len(label) > 38:
                    label = label[:36] + "…"
                lines.append(f"  {icon}  {label}")
            lines.append("")

        # Human-waiting banner (shown when a load/unload is waiting)
        if self._waiting_msg:
            lines.append("── ★ HUMAN ACTION REQUIRED ───────────")
            msg = self._waiting_msg
            while len(msg) > 34:
                lines.append(f"  {msg[:34]}")
                msg = msg[34:]
            lines.append(f"  {msg}")
            lines.append("  → Press Enter in the terminal …")
            lines.append("")

        # Current status
        lines.append("── Status ────────────────────────────")
        status = self._current_status
        while len(status) > 36:
            lines.append(f"  {status[:36]}")
            status = status[36:]
        lines.append(f"  {status}")

        if self._robot is not None:
            cargo = self._robot.carrying or "—"
            lines.append(f"  Cargo  :  {cargo}")
        lines.append("")

        # Timeline
        lines.append("── Timeline ──────────────────────────")
        for t, ev in self._timeline[-self.TIMELINE_N:]:
            entry = f"t={t:5.1f}s  {ev}"
            if len(entry) > 38:
                entry = entry[:36] + "…"
            lines.append(f"  {entry}")

        if self._result is not None:
            lines.append("")
            lines.append("  ✓  MISSION COMPLETE" if self._result
                         else "  ✗  MISSION FAILED")

        return "\n".join(lines)

    def _redraw_progress(self) -> None:
        self._prog_text.set_text(self._build_progress_text())

    # ══════════════════════════════════════════════════════════════════════════
    #  EVENT QUEUE
    # ══════════════════════════════════════════════════════════════════════════

    def _post(self, event_type: str, data=None) -> None:
        self._events.put((event_type, data))

    def _drain_events(self) -> None:
        while not self._events.empty():
            try:
                etype, data = self._events.get_nowait()
            except queue.Empty:
                break

            if etype == "status":
                self._current_status = data
                t = time.monotonic() - self._t0
                self._timeline.append((t, data))

                if data.startswith("NAVIGATING:"):
                    loc = data.split(":", 1)[1]
                    self._goal  = self._waypoints.get(loc)
                    self._start = (self._robot.position.copy()
                                   if self._robot else None)
                elif data in ("MISSION_COMPLETE", "PLAN_ABORTED", "CHARGED"):
                    self._goal = None
                elif data == "WAITING_HUMAN":
                    pass  # waiting_msg set via separate "human_prompt" event
                elif data == "HUMAN_CONFIRMED":
                    self._waiting_msg = None

            elif etype == "stage":
                idx, status, _ = data
                self._stages[idx]["status"] = status

            elif etype == "plan":
                self._plan = data
                self._step_state = {s.index: "pending" for s in data}

            elif etype == "step_start":
                step: PlanStep = data
                self._step_state[step.index] = "running"

            elif etype == "step_waiting":
                step: PlanStep = data
                self._step_state[step.index] = "waiting"

            elif etype == "step_done":
                step, result = data
                self._step_state[step.index] = (
                    "done" if result.success else "failed"
                )

            elif etype == "human_prompt":
                self._waiting_msg = data

            elif etype == "result":
                self._result = data

    # ══════════════════════════════════════════════════════════════════════════
    #  PIPELINE WORKER  (background thread)
    # ══════════════════════════════════════════════════════════════════════════

    def _pipeline_worker(
        self,
        task_description: str,
        use_llm:           bool,
        fallback_scenario: str,
        use_astar:         bool,
        result_box:        list,
    ) -> None:
        try:
            # ── Stage 1: NL → PDDL ───────────────────────────────────────────
            self._post("stage", (0, "running", "NL → PDDL"))
            self._post("status", "LLM_TRANSLATING" if use_llm
                       else f"FALLBACK:{fallback_scenario}")
            try:
                if use_llm:
                    domain_pddl, problem_pddl = natural_language_to_pddl(
                        task_description
                    )
                else:
                    domain_pddl, problem_pddl = fallback_pddl(fallback_scenario)
            except Exception as e:
                self._post("stage", (0, "failed", ""))
                self._post("status", f"LLM_FAILED: {e}")
                self._post("result", False)
                result_box.append(False)
                return
            self._post("stage", (0, "done",    ""))
            self._post("status", "PDDL_GENERATED")

            # ── Stage 2: PDDL → Plan ─────────────────────────────────────────
            self._post("stage", (1, "running", ""))
            self._post("status", "PLANNING")
            try:
                plan = solve_pddl(domain_pddl, problem_pddl,
                                  use_astar=use_astar)
            except Exception as e:
                self._post("stage", (1, "failed", ""))
                self._post("status", f"PLANNING_FAILED: {e}")
                self._post("result", False)
                result_box.append(False)
                return
            self._post("stage", (1, "done", ""))
            self._post("plan",  plan)
            self._post("status", f"PLAN_READY ({len(plan)} steps)")

            # ── Stage 3: Execution ────────────────────────────────────────────
            self._post("stage", (2, "running", ""))
            self._post("status", "EXECUTING_PLAN")

            def on_prompt(msg: str) -> None:
                self._post("human_prompt", msg)
                input()  # blocks background thread; main thread keeps animating

            cobot = HumanCobot(
                on_status=lambda s: self._post("status", s),
                on_prompt=on_prompt,
            )
            executor = PrimitiveExecutor(
                robot=self._robot,
                cobot=cobot,
                on_status=lambda s: self._post("status", s),
                on_step=lambda step: None,
            )
            nav = NavigationManager(
                robot=self._robot,
                on_status=lambda s: self._post("status", s),
            )
            executor._nav = nav

            all_ok = True
            for step in plan:
                self._post("step_start", step)
                self._post("status",     f"EXEC: {step.name} {' '.join(step.args)}")

                # Mark load/deliver steps as "waiting" just before they run
                if step.name in ("load-package", "deliver-package"):
                    self._post("step_waiting", step)

                try:
                    result = executor._dispatch(step)
                except Exception as e:
                    result = StepResult(step, False, str(e))

                self._post("step_done", (step, result))
                if not result.success:
                    self._post("status", f"FAILED: {result.message}")
                    all_ok = False
                    break

            if all_ok:
                self._post("stage",  (2, "done",   ""))
                self._post("status", "MISSION_COMPLETE")
            else:
                self._post("stage",  (2, "failed", ""))
                self._post("status", "PLAN_ABORTED")

            self._post("result", all_ok)
            result_box.append(all_ok)

        except Exception as e:
            self._post("status", f"UNEXPECTED ERROR: {e}")
            self._post("result", False)
            result_box.append(False)

    # ══════════════════════════════════════════════════════════════════════════
    #  PUBLIC API
    # ══════════════════════════════════════════════════════════════════════════

    def run(
        self,
        task_description:  str,
        use_llm:           bool = False,
        fallback_scenario: str  = "deliver_A",
        use_astar:         bool = False,
    ) -> bool:
        """
        Run the full pipeline with live visualisation.
        Returns True if the mission succeeded.
        """
        self._t0   = time.monotonic()
        self._robot = get_robot(self._waypoints["home"])

        result_box: list[bool] = []
        worker = threading.Thread(
            target=self._pipeline_worker,
            args=(task_description, use_llm, fallback_scenario,
                  use_astar, result_box),
            daemon=True,
        )
        worker.start()

        while worker.is_alive():
            self._drain_events()
            self._redraw_map()
            self._redraw_progress()
            self._fig.canvas.draw_idle()
            plt.pause(0.08)

        self._drain_events()
        self._redraw_map()
        self._redraw_progress()
        self._fig.canvas.draw_idle()
        plt.pause(0.2)

        return result_box[0] if result_box else False

    def save(self, path: str) -> None:
        self._fig.savefig(path, dpi=150, bbox_inches="tight",
                          facecolor=_C["panel_bg"])
        print(f"[HumanCobotViz] Saved → {path}")

    def show(self) -> None:
        """Block until the window is closed."""
        plt.ioff()
        plt.show()


# ═══════════════════════════════════════════════════════════════════════════════
#  STANDALONE DEMO
# ═══════════════════════════════════════════════════════════════════════════════

def main() -> None:
    os.environ.setdefault("MYAGV_USE_SIM", "1")
    import argparse

    parser = argparse.ArgumentParser(
        description="Phase 3 human-in-the-loop mission visualiser demo"
    )
    parser.add_argument(
        "--scenario", default="deliver_A",
        choices=["deliver_A", "deliver_AB", "recharge_then_deliver"],
    )
    args = parser.parse_args()

    print("╔══════════════════════════════════════════════╗")
    print("║  Phase 3 — Human-in-the-loop Mission Viz     ║")
    print(f"║  Scenario : {args.scenario:<33}║")
    print("╚══════════════════════════════════════════════╝\n")

    viz = HumanCobotVisualizer()
    ok  = viz.run(
        task_description=f"[demo: {args.scenario}]",
        use_llm=False,
        fallback_scenario=args.scenario,
    )
    print(f"\nMission {'succeeded' if ok else 'FAILED'}.")
    print("Close the window to exit.")
    viz.show()


if __name__ == "__main__":
    main()
