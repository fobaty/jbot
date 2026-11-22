"""Host-side paths for bench recordings.

Captures must not accumulate in the repository root: tools write logs into
`logs/` and telemetry CSVs into `csv/`, both at the repository root. An
explicitly absolute path is kept as-is so an operator can redirect a capture.
"""

import os

TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(TOOLS_DIR)
LOG_DIR = os.path.join(REPO_ROOT, "logs")
CSV_DIR = os.path.join(REPO_ROOT, "csv")


def resolve(directory, path):
    """Return a writable path under `directory`, creating the directory."""
    if not os.path.isabs(path):
        path = os.path.join(directory, path)
    parent = os.path.dirname(path)
    if parent:
        os.makedirs(parent, exist_ok=True)
    return path


def log_path(name):
    """Resolve an event/log filename into `logs/`."""
    return resolve(LOG_DIR, name)


def csv_path(name):
    """Resolve a CSV filename into `csv/`."""
    return resolve(CSV_DIR, name)
