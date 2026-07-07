"""
myagv_lab/phase3_human_cobot/human_cobot.py
============================================
Drop-in replacement for SimCobot (Phase 4) where a human helper
performs the physical loading / unloading instead of a robot arm.

The interface is identical to SimCobot so it can be passed directly
to PrimitiveExecutor as the ``cobot`` parameter.

Phase 3 vs Phase 4 difference
------------------------------
  Phase 3  →  HumanCobot.load()   pauses, waits for human, returns True
  Phase 4  →  SimCobot.load()     sleeps LOAD_DURATION seconds, returns True
  Phase 4  →  CobotInterfaceNode  drives a real arm via ROS2 topics

The on_prompt hook replaces input() in automated tests:
  cobot = HumanCobot(on_prompt=lambda msg: None)   # instant confirm
"""

from __future__ import annotations

import logging
from typing import Callable, Optional

log = logging.getLogger("human_cobot")

_RESET  = "\033[0m"
_YELLOW = "\033[93m"
_GREEN  = "\033[92m"
_CYAN   = "\033[96m"
_BOLD   = "\033[1m"


class HumanCobot:
    """
    Simulates the cobot arm by asking a human helper to load / unload
    packages manually.  Matches the SimCobot.load() / unload() interface.

    Parameters
    ----------
    on_status : callback(str) — fired at each state change, same
                protocol as SimCobot (LOADING, LOAD_COMPLETE, etc.)
    on_prompt : optional callable(str) that replaces input() for
                non-interactive use (testing, GUI).  Must block until
                the action is complete; pass ``lambda msg: None`` to
                auto-confirm instantly.
    """

    def __init__(
        self,
        on_status: Optional[Callable[[str], None]] = None,
        on_prompt:  Optional[Callable[[str], None]] = None,
    ):
        self._on_status = on_status or (lambda s: None)
        self._on_prompt = on_prompt
        self._state     = "IDLE"

    # ── Internal helpers ──────────────────────────────────────────────────────

    def _publish(self, status: str) -> None:
        self._state = status
        log.info(f"[HumanCobot] status → {status}")
        self._on_status(status)

    def _human_prompt(self, message: str) -> None:
        """
        Print a prompt and wait for the human helper to press Enter
        (or call on_prompt if set).
        """
        self._publish("WAITING_HUMAN")
        print()
        print(f"{_YELLOW}{_BOLD}★  HUMAN ACTION REQUIRED{_RESET}")
        print(f"   {message}")
        print(f"   {_CYAN}Press Enter when done…{_RESET} ", end="", flush=True)
        if self._on_prompt is not None:
            self._on_prompt(message)
        else:
            input()
        print(f"{_GREEN}   ✓  Confirmed.{_RESET}")
        self._publish("HUMAN_CONFIRMED")

    # ── Public interface (mirrors SimCobot) ───────────────────────────────────

    def load(self, package: str, agv=None) -> bool:
        """
        Ask the human helper to place ``package`` on the AGV.

        Parameters
        ----------
        package : str — package name (e.g. "package_A")
        agv     : SimRobot | None — if provided, pick_up() is called after
                  confirmation (sim accounting); pass None in real mode.

        Returns True unconditionally (human confirmed the action).
        """
        self._publish("LOADING")
        self._human_prompt(
            f"Please place  {_CYAN}{_BOLD}{package}{_RESET}"
            f"  onto the AGV platform at the loading area."
        )
        if agv is not None:
            agv.pick_up(package)
        self._publish("LOAD_COMPLETE")
        return True

    def unload(self, package: str, agv=None) -> bool:
        """
        Ask the human helper to remove ``package`` from the AGV.

        Parameters
        ----------
        package : str — package name
        agv     : SimRobot | None — if provided, put_down() is called after
                  confirmation.

        Returns True unconditionally.
        """
        self._publish("UNLOADING")
        self._human_prompt(
            f"Please remove  {_CYAN}{_BOLD}{package}{_RESET}"
            f"  from the AGV platform at the delivery area."
        )
        if agv is not None:
            agv.put_down()
        self._publish("UNLOAD_COMPLETE")
        return True

    @property
    def state(self) -> str:
        return self._state
