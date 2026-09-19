"""Construct upstream Qt windows offscreen, with no radio/serial connection."""
import os
import faulthandler
from pathlib import Path
import sys

os.environ["QT_QPA_PLATFORM"] = "offscreen"
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "apps/python/app"))

from PyQt5 import QtWidgets
from flightpath import PathEditorWindow, interpolate
from plot import PlotApp
import qasync
import serial_asyncio


def main():
    app = QtWidgets.QApplication([])
    windows = [PathEditorWindow(), PlotApp()]
    app.processEvents()
    assert interpolate([(0., 0., 0., 0.), (2., 2., 4., 6.)], 1.) == (1., 2., 3.)
    for window in windows:
        window.close()
    app.processEvents()
    print("PASS: Qt windows + waypoint interpolation; no serial port opened.")


if __name__ == "__main__":
    # A graphics/plugin problem must not leave this diagnostic running forever.
    faulthandler.dump_traceback_later(30, exit=True)
    main()
    faulthandler.cancel_dump_traceback_later()
