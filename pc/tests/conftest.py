import os
import sys
from pathlib import Path

# GUI テストはヘッドレスで実行する
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
# pip install -e せずに pytest しても import できるようにする
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
