import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), "src"))
from transcribe.stt import run_stt, NEW_TEXT_EVENT


if __name__ == "__main__":
    run_stt()