# langfuse_client/client.py
# ─────────────────────────────────────────────────────────────────────────────
# Langfuse-instrumented LLM client for the myAGV Summer School.
#
# Every API call made through this client is automatically traced in the
# shared Langfuse project, tagged with the student's last name as user_id.
#
# Required environment variables (set in ~/.bashrc or .env):
#   LANGFUSE_PUBLIC_KEY  — shared public key  (instructor provides)
#   LANGFUSE_SECRET_KEY  — shared secret key  (instructor provides)
#   LANGFUSE_HOST or LANGFUSE_BASE_URL — Langfuse server URL (instructor provides)
#   DEEPSEEK_API_KEY     — DeepSeek API key    (instructor provides)
#   STUDENT_ID           — your last name in lowercase  (each student sets this)
#
# Usage:
#   export STUDENT_ID="smith"
#   from langfuse_client.client import get_llm_client
#   client, user_id = get_llm_client()
#   response = client.chat.completions.create(
#       model="deepseek-chat",
#       messages=[...],
#       user=user_id,       # ← links the trace to the student in Langfuse
#   )
# ─────────────────────────────────────────────────────────────────────────────

from __future__ import annotations

import os
import sys
from pathlib import Path

# Make sure the project root is importable regardless of where the script runs
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from langfuse_client.students import validate_student_id

DEEPSEEK_BASE_URL = "https://api.deepseek.com"
DEFAULT_MODEL     = "deepseek-chat"


def get_llm_client(student_id: str = None) -> tuple:
    """
    Return a (client, user_id) pair ready for API calls.

    Parameters
    ----------
    student_id : str, optional
        Student's last name.  Falls back to the STUDENT_ID environment
        variable if not provided.

    Returns
    -------
    client   : Langfuse-instrumented OpenAI-compatible client
    user_id  : str — the validated student last name

    Raises
    ------
    EnvironmentError
        If any required environment variable is missing.
    ValueError
        If student_id is not in the registered roster.
    """
    # ── Resolve student ID ────────────────────────────────────────────────────
    if student_id is None:
        student_id = os.environ.get("STUDENT_ID", "").strip().lower()
    if not student_id:
        raise EnvironmentError(
            "STUDENT_ID is not set.\n"
            "  export STUDENT_ID='yourlastname'   (lowercase)\n"
            "  Use your registered last name."
        )

    validate_student_id(student_id)

    # ── Check required keys ───────────────────────────────────────────────────
    # Langfuse SDK accepts LANGFUSE_HOST or LANGFUSE_BASE_URL interchangeably;
    # normalise to LANGFUSE_HOST so the rest of the code has one name to use.
    if not os.environ.get("LANGFUSE_HOST") and os.environ.get("LANGFUSE_BASE_URL"):
        os.environ["LANGFUSE_HOST"] = os.environ["LANGFUSE_BASE_URL"]

    missing = [
        v for v in ("LANGFUSE_PUBLIC_KEY", "LANGFUSE_SECRET_KEY",
                    "LANGFUSE_HOST", "DEEPSEEK_API_KEY")
        if not os.environ.get(v)
    ]
    if missing:
        raise EnvironmentError(
            f"Missing environment variables: {missing}\n"
            "  Ask your instructor for the shared keys and add them to ~/.bashrc."
        )

    # ── Build Langfuse-instrumented client ────────────────────────────────────
    from langfuse.openai import OpenAI

    client = OpenAI(
        api_key=os.environ["DEEPSEEK_API_KEY"],
        base_url=DEEPSEEK_BASE_URL,
    )

    return client, student_id


def chat(messages: list[dict],
         student_id: str = None,
         model: str = DEFAULT_MODEL,
         max_tokens: int = 1024,
         **kwargs) -> str:
    """
    Convenience wrapper: send messages and return the reply text.

    Parameters
    ----------
    messages   : list of {"role": ..., "content": ...} dicts
    student_id : student last name (or read from STUDENT_ID env var)
    model      : LLM model name
    max_tokens : maximum tokens in the reply

    Returns
    -------
    str — the assistant's reply text
    """
    client, user_id = get_llm_client(student_id)
    response = client.chat.completions.create(
        model=model,
        messages=messages,
        max_tokens=max_tokens,
        user=user_id,
        **kwargs,
    )
    return response.choices[0].message.content.strip()
