# langfuse_client/students.py
# ─────────────────────────────────────────────────────────────────────────────
# myAGV Summer School 2026 — student roster
#
# Each student sets:  export STUDENT_ID="<their user_id below>"
#
# Naming rules applied:
#   • Lowercase last name as user_id
#   • Compound / hyphenated surnames joined with underscore
#   • Duplicate last names disambiguated with first initial prefix
#     (a_williams / f_williams / dare_williams)
# ─────────────────────────────────────────────────────────────────────────────

# Full name → user_id mapping (for instructor reference)
STUDENTS: dict[str, str] = {
    "Tengku Faris Zuhri":           "zuhri",
    "Yuvraj Bedi":                  "bedi",
    "Eli Gilinsky":                 "gilinsky",
    "Thomas Knowler":               "knowler",
    "Engjull Doci":                 "doci",
    "Osarenkhoe Izekor":            "izekor",
    "Francisco Andres Macaya Matas":"macaya_matas",
    "Jinze Ge":                     "ge",
    "Lucy Hope":                    "hope",
    "Ruiling Guan":                 "guan",
    "Erblin Xhabafti":              "xhabafti",
    "Amber Williams":               "a_williams",
    "Klaudia Niedzialkowska":       "niedzialkowska",
    "Daniel Korth":                 "korth",
    "Aminata Gaye":                 "gaye",
    "Iyad Elgibali":                "elgibali",
    "WenZhuo Sun":                  "sun",
    "Bors Farago":                  "farago",
    "Parvez Bohoran":               "bohoran",
    "Divit Kothari":                "kothari",
    "Raima Singh":                  "singh",
    "Dhriti Shah":                  "shah",
    "Hoi Lam Wong":                 "wong",
    "Michal Wojteczko":             "wojteczko",
    "James Patel Powell":           "patel_powell",
    "Frank Williams":               "f_williams",
    "Neil Dare-Williams":           "dare_williams",
    "Karsten Siu":                  "siu",
    "Jeevan Hatti":                 "hatti",
    "Arjun Manoj":                  "manoj",
    "Feridun Emre Kizan":           "kizan",
    "Saee Kul":                     "kul",
    "Mikhail Matiukhov":            "matiukhov",
    "Amna Ikram":                   "ikram",
    "Pian Yu":                        "yu",
    "Daqi Huang":                       "huang",
}

# Flat list of valid user_ids (used for validation)
STUDENT_LAST_NAMES: list[str] = list(STUDENTS.values())


def validate_student_id(student_id: str) -> None:
    """
    Raise ValueError if student_id is not in the roster.
    Call this before making any API request so unregistered users
    are blocked immediately.
    """
    if student_id.lower() not in STUDENT_LAST_NAMES:
        raise ValueError(
            f"Unknown student ID: {student_id!r}\n"
            f"  Set your STUDENT_ID to your registered user_id.\n"
            f"  Ask your instructor if you are unsure which ID to use.\n"
            f"  Registered IDs: {STUDENT_LAST_NAMES}"
        )


def lookup_full_name(student_id: str) -> str:
    """Return the full name for a given user_id, or the id itself if not found."""
    reverse = {v: k for k, v in STUDENTS.items()}
    return reverse.get(student_id.lower(), student_id)
