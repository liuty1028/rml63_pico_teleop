import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rml63_teleop.cli import dry_run_main

if __name__ == "__main__":
    dry_run_main()
