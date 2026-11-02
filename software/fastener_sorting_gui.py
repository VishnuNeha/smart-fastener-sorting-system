"""
Smart Fastener Sorting System - FINAL single-file desktop GUI
=============================================================

STM32F401 + HC-05 Bluetooth + Decision-Tree Machine Learning

Run:
    pip install pyserial pandas matplotlib scikit-learn joblib Pillow
    pip install opencv-python          # optional, only for the camera panel
    python fastener_sorting_gui.py

Works directly with the CURRENT firmware (main.c), which sends text such as:

    OBJECT #12
    Metal: DETECTED
    Height: 3 cm            (or "Height: INVALID")
    RESULT: ACCEPTED
    Reason: METAL + HEIGHT OK
    Servo returned to REST.
    --------------------------
    ===== LIVE STATUS =====
    Objects: 12
    Metal status: DETECTED
    Object height: 0 cm
    Colour: R=120 G=65 B=40
    =======================

It also accepts the compact machine format, if you add it to the firmware later:

    DATA,Object_ID,Metal,Height,Red,Green,Blue,Label      e.g. DATA,12,1,3.5,120,65,40,1

Threading: serial reader / connect / training threads only push events into a
queue. The Tk main thread drains that queue every 50 ms and is the ONLY code
that touches widgets, so the GUI never freezes.
"""
from __future__ import annotations

import csv
import math
import os
import queue
import re
import shutil
import sys
import threading
import traceback
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime
from typing import Optional

import tkinter as tk
from tkinter import filedialog, messagebox, ttk

try:
    import serial
    import serial.tools.list_ports
except ImportError:                                    # reported nicely in main()
    serial = None

try:
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    from matplotlib.figure import Figure
except ImportError:
    Figure = None

try:
    import cv2
    CV2_OK = True
except Exception:
    cv2 = None
    CV2_OK = False

try:
    from PIL import Image, ImageTk
    PIL_OK = True
except Exception:
    PIL_OK = False


# =============================================================================
#  SETTINGS - adjust here
# =============================================================================
APP_DIR = os.path.dirname(os.path.abspath(sys.argv[0] if sys.argv[0] else __file__))
DEFAULT_CSV = "fastener_dataset.csv"
MODEL_FILE = os.path.join(APP_DIR, "fastener_model.joblib")
DEFAULT_BAUD = "9600"            # USART2 BRR = 0x0683 @ 16 MHz -> 9600 baud
BAUD_RATES = ["1200", "2400", "4800", "9600", "19200", "38400", "57600", "115200"]

# The firmware reads the TCS3200 in its live loop, not inside the object block.
#   "before" -> use the last colour reading received BEFORE "OBJECT #n"
#   "after"  -> wait for the first colour reading received AFTER the result
# Pick whichever matches where the colour sensor sits on your conveyor.
OBJECT_RGB_SOURCE = "before"

MAX_HEIGHT_CM = 400.0
MAX_COLOR_VALUE = 65535
HISTORY_LIMIT = 500
LOG_LIMIT = 800

# Colours / fonts
BG = "#eef2f7"
HEADER = "#0b3a6e"
PANEL = "#ffffff"
BORDER = "#cbd5e1"
CARD = "#f4f8fc"
TEXT = "#0f2740"
GRAY = "#6b7280"
GREEN = "#1b9e3e"
GREEN_BG = "#d9f5df"
RED = "#d93a3a"
RED_BG = "#fde2e2"
ORANGE = "#d97706"
BLUE = "#1565c0"

F_TITLE = ("Segoe UI", 18, "bold")
F_PANEL = ("Segoe UI", 11, "bold")
F_BODY = ("Segoe UI", 9)
F_SMALL = ("Segoe UI", 8)


# =============================================================================
#  DATA MODEL
# =============================================================================
@dataclass
class SensorRecord:
    object_id: int
    metal: int                       # 1 = metal, 0 = non-metal
    height: Optional[float]          # cm, None = ultrasonic timeout ("INVALID")
    red: Optional[int] = None
    green: Optional[int] = None
    blue: Optional[int] = None
    label: Optional[int] = None      # STM32 decision: 1 accepted, 0 rejected
    reason: str = ""
    timestamp: str = ""

    def key(self) -> tuple:
        h = None if self.height is None else round(self.height, 3)
        return (self.object_id, self.metal, h, self.red, self.green, self.blue, self.label)


class ParseError(ValueError):
    pass


def validate_record(rec: SensorRecord) -> None:
    if rec.object_id < 0:
        raise ParseError(f"Object_ID must be >= 0 (got {rec.object_id})")
    if rec.metal not in (0, 1):
        raise ParseError(f"Metal must be 0 or 1 (got {rec.metal})")
    if rec.height is not None and not 0.0 <= rec.height <= MAX_HEIGHT_CM:
        raise ParseError(f"Height out of range 0-{MAX_HEIGHT_CM:g} cm (got {rec.height})")
    for name, v in (("Red", rec.red), ("Green", rec.green), ("Blue", rec.blue)):
        if v is not None and not 0 <= v <= MAX_COLOR_VALUE:
            raise ParseError(f"{name} out of range 0-{MAX_COLOR_VALUE} (got {v})")
    if rec.label not in (None, 0, 1):
        raise ParseError(f"Label must be 0 or 1 (got {rec.label})")


def _num(text: str, name: str, integer: bool = False):
    try:
        value = float(text)
    except ValueError:
        raise ParseError(f"{name} is not a number: {text!r}") from None
    if not math.isfinite(value):
        raise ParseError(f"{name} is not finite: {text!r}")
    if integer:
        if not value.is_integer():
            raise ParseError(f"{name} must be a whole number: {text!r}")
        return int(value)
    return value


def parse_data_line(text: str) -> SensorRecord:
    """DATA,Object_ID,Metal,Height,Red,Green,Blue[,Label]"""
    f = [x.strip() for x in text.split(",")]
    if len(f) not in (7, 8):
        raise ParseError(f"expected 7 or 8 fields after DATA, got {len(f) - 1}")
    label = _num(f[7], "Label", True) if len(f) == 8 and f[7] != "" else None
    height = None if f[3].upper() in ("", "INVALID", "NAN") else _num(f[3], "Height")
    rec = SensorRecord(
        object_id=_num(f[1], "Object_ID", True), metal=_num(f[2], "Metal", True),
        height=height, red=_num(f[4], "Red", True), green=_num(f[5], "Green", True),
        blue=_num(f[6], "Blue", True), label=label)
    validate_record(rec)
    return rec


# =============================================================================
#  FIRMWARE TEXT PARSER  (the only place that knows the wire format)
# =============================================================================
RE_OBJECT = re.compile(r"^OBJECT\s*#\s*(\d+)", re.I)
RE_METAL = re.compile(r"^Metal:\s*(.+)$", re.I)
RE_HEIGHT = re.compile(r"^Height:\s*(.+)$", re.I)
RE_RESULT = re.compile(r"^RESULT:\s*(ACCEPTED|REJECTED)", re.I)
RE_REASON = re.compile(r"^Reason:\s*(.+)$", re.I)
RE_COLOUR = re.compile(r"^Colou?r:\s*R\s*=\s*(\d+)\s*G\s*=\s*(\d+)\s*B\s*=\s*(\d+)", re.I)
RE_LIVE_METAL = re.compile(r"^Metal status:\s*(.+)$", re.I)
RE_LIVE_HEIGHT = re.compile(r"^Object height:\s*(.+)$", re.I)
RE_LIVE_COUNT = re.compile(r"^Objects:\s*(\d+)", re.I)
RE_METAL_COUNT = re.compile(r"^Metal count:\s*(\d+)", re.I)
RE_DASHES = re.compile(r"^-{5,}$")
RE_NUMBER = re.compile(r"(\d+(?:\.\d+)?)")


def _metal_flag(text: str) -> int:
    t = text.upper()
    return 0 if ("NOT" in t or "NON" in t) else (1 if "DETECT" in t else 0)


def _height_value(text: str) -> Optional[float]:
    if "INVALID" in text.upper():
        return None
    m = RE_NUMBER.search(text)
    return float(m.group(1)) if m else None


class FirmwareParser:
    """
    Feed it one line at a time. Returns a list of events for the GUI:
      ("log", text, tag)          text for the Bluetooth log
      ("ready",)                  firmware banner received
      ("object_start", id)        IR at PA0 detected a new object
      ("checking", "metal"|"height")
      ("metal", 0|1)              object metal result
      ("height", value|None)      object height (None = INVALID)
      ("waiting_sorter",)         firmware waits for PA4 IR
      ("result", 0|1)             STM32 decision
      ("servo", text)
      ("live_rgb", r, g, b) / ("live_metal", 0|1) / ("live_height", v|None)
      ("live_count", n) / ("metal_count", n)
      ("record", SensorRecord)    one complete object -> table, graphs, CSV
      ("invalid", message)
    """

    def __init__(self, rgb_source: str = OBJECT_RGB_SOURCE):
        self.rgb_source = rgb_source
        self.pending: Optional[SensorRecord] = None
        self.waiting_rgb: Optional[SensorRecord] = None
        self.live_rgb: tuple = (None, None, None)

    def reset(self) -> None:
        self.pending = None
        self.waiting_rgb = None

    # -------------------------------------------------------------- helpers
    def _finish(self, rec: SensorRecord, out: list) -> None:
        try:
            validate_record(rec)
        except ParseError as exc:
            out.append(("invalid", f"Object #{rec.object_id}: {exc}"))
            return
        out.append(("record", rec))

    def _flush_pending(self, out: list) -> None:
        """Close the current object block (called at the dashed line)."""
        rec = self.pending
        self.pending = None
        if rec is None:
            return
        if rec.label is None:
            out.append(("invalid", f"Object #{rec.object_id}: block ended without RESULT"))
            return
        if self.rgb_source == "after":
            self.waiting_rgb = rec
        else:
            self._finish(rec, out)

    def _flush_waiting(self, out: list) -> None:
        if self.waiting_rgb is not None:
            rec, self.waiting_rgb = self.waiting_rgb, None
            self._finish(rec, out)

    # ----------------------------------------------------------------- feed
    def feed(self, raw: str) -> list:
        out: list = []
        text = raw.strip().strip("\x00").strip()
        if not text:
            return out

        # ---- compact machine format
        if text.split(",", 1)[0].strip().upper() == "DATA":
            try:
                rec = parse_data_line(text)
            except ParseError as exc:
                out.append(("log", text, "warn"))
                out.append(("invalid", f"{text!r} -> {exc}"))
                return out
            out.append(("log", text, "data"))
            out.append(("record", rec))
            return out

        tag = ""
        m = None

        if "SORTING SYSTEM READY" in text.upper():
            self.reset()
            out.append(("ready",))
            tag = "sys"

        elif (m := RE_OBJECT.match(text)):
            self._flush_waiting(out)
            if self.pending is not None and self.pending.label is not None:
                self._flush_pending(out)
                self._flush_waiting(out)
            elif self.pending is not None:
                out.append(("invalid", f"Object #{self.pending.object_id}: no RESULT received"))
            r, g, b = self.live_rgb
            self.pending = SensorRecord(object_id=int(m.group(1)), metal=0, height=None,
                                        red=r, green=g, blue=b)
            out.append(("object_start", int(m.group(1))))
            tag = "data"

        elif text.lower().startswith("checking metal"):
            out.append(("checking", "metal"))
        elif text.lower().startswith("checking height"):
            out.append(("checking", "height"))

        elif (m := RE_METAL.match(text)):
            flag = _metal_flag(m.group(1))
            if self.pending is not None:
                self.pending.metal = flag
            out.append(("metal", flag))
            tag = "data"

        elif (m := RE_HEIGHT.match(text)):
            h = _height_value(m.group(1))
            if self.pending is not None:
                self.pending.height = h
            out.append(("height", h))
            tag = "data" if h is not None else "warn"

        elif text.lower().startswith("checks complete"):
            out.append(("waiting_sorter",))

        elif (m := RE_RESULT.match(text)):
            label = 1 if m.group(1).upper() == "ACCEPTED" else 0
            if self.pending is not None:
                self.pending.label = label
            out.append(("result", label))
            tag = "data"

        elif (m := RE_REASON.match(text)):
            if self.pending is not None:
                self.pending.reason = m.group(1).strip()

        elif "servo" in text.lower():
            out.append(("servo", text.rstrip(".")))

        elif RE_DASHES.match(text):
            if self.pending is not None and self.pending.label is not None:
                self._flush_pending(out)

        elif (m := RE_COLOUR.match(text)):
            r, g, b = (int(m.group(i)) for i in (1, 2, 3))
            self.live_rgb = (r, g, b)
            out.append(("live_rgb", r, g, b))
            if self.waiting_rgb is not None:
                self.waiting_rgb.red, self.waiting_rgb.green, self.waiting_rgb.blue = r, g, b
                self._flush_waiting(out)

        elif (m := RE_LIVE_METAL.match(text)):
            out.append(("live_metal", _metal_flag(m.group(1))))
        elif (m := RE_LIVE_HEIGHT.match(text)):
            out.append(("live_height", _height_value(m.group(1))))
        elif (m := RE_LIVE_COUNT.match(text)):
            out.append(("live_count", int(m.group(1))))
        elif (m := RE_METAL_COUNT.match(text)):
            out.append(("metal_count", int(m.group(1))))

        out.insert(0, ("log", text, tag))
        return out


# =============================================================================
#  BLUETOOTH (HC-05 virtual COM port)
# =============================================================================
def list_ports() -> list:
    if serial is None:
        return []
    try:
        ports = serial.tools.list_ports.comports()
    except Exception:
        return []
    return sorted(((p.device, p.description or "") for p in ports), key=lambda x: x[0])


class SerialReader(threading.Thread):
    MAX_BUFFER = 4096

    def __init__(self, ser, out_queue: queue.Queue):
        super().__init__(daemon=True, name="SerialReader")
        self._ser = ser
        self._q = out_queue
        self._stop_evt = threading.Event()
        self._buf = bytearray()
        self.bytes_received = 0

    def stop(self) -> None:
        self._stop_evt.set()

    def run(self) -> None:
        while not self._stop_evt.is_set():
            try:
                chunk = self._ser.read(max(1, self._ser.in_waiting))
            except (serial.SerialException, OSError, TypeError, ValueError) as exc:
                if not self._stop_evt.is_set():
                    self._q.put(("disconnected", str(exc) or "device error"))
                return
            if chunk:
                self.bytes_received += len(chunk)
                self._feed(chunk)

    def _feed(self, chunk: bytes) -> None:
        self._buf.extend(chunk.replace(b"\r", b"\n"))
        while True:
            idx = self._buf.find(b"\n")
            if idx < 0:
                break
            line = bytes(self._buf[:idx])
            del self._buf[:idx + 1]
            if line:
                self._q.put(("line", line.decode("ascii", errors="replace")))
        if len(self._buf) > self.MAX_BUFFER:
            self._buf.clear()
            self._q.put(("overflow", "Receive buffer discarded (no line ending received)"))


class BluetoothLink:
    def __init__(self, out_queue: queue.Queue):
        self._q = out_queue
        self._ser = None
        self._reader: Optional[SerialReader] = None
        self._connecting = False

    @property
    def connected(self) -> bool:
        return bool(self._ser is not None and self._ser.is_open
                    and self._reader is not None and self._reader.is_alive())

    def connect_async(self, port: str, baud: int) -> None:
        if self._connecting or self.connected:
            return
        self._connecting = True
        threading.Thread(target=self._worker, args=(port, baud), daemon=True).start()

    def _worker(self, port: str, baud: int) -> None:
        try:
            ser = serial.Serial(port=port, baudrate=baud, timeout=0.1)
        except (serial.SerialException, OSError, ValueError) as exc:
            self._connecting = False
            self._q.put(("connect_failed", self._explain(port, exc)))
            return
        try:
            ser.reset_input_buffer()
        except Exception:
            pass
        self._ser = ser
        self._reader = SerialReader(ser, self._q)
        self._reader.start()
        self._connecting = False
        self._q.put(("connected", port, baud))

    @staticmethod
    def _explain(port: str, exc: Exception) -> str:
        text = str(exc)
        low = text.lower()
        if "permission" in low or "access is denied" in low or "(13" in low:
            return (f"Cannot open {port}: access denied.\nThe port is probably already open "
                    f"in another program (PuTTY, Keil, another copy of this GUI).")
        if "could not open" in low or "filenotfound" in low or "(2" in low:
            return (f"Cannot open {port}: port not available.\nCheck that the HC-05 is powered "
                    f"and paired, then press Refresh.\n\n{text}")
        return f"Cannot open {port}: {text}"

    @property
    def bytes_received(self) -> int:
        return self._reader.bytes_received if self._reader is not None else 0

    def disconnect(self) -> None:
        reader, ser = self._reader, self._ser
        self._reader = None
        self._ser = None
        if reader is not None:
            reader.stop()
            reader.join(timeout=1.0)
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass


# =============================================================================
#  CSV DATASET
# =============================================================================
COLUMNS = ["Timestamp", "Object_ID", "Metal", "Height", "Red", "Green", "Blue", "Label"]


class DatasetError(Exception):
    pass


class DatasetManager:
    """Creates / appends the CSV. Every row is written and closed immediately."""

    def __init__(self, path: str):
        self.path = os.path.abspath(path)
        self.total_rows = 0
        self.session_rows = 0
        try:
            os.makedirs(os.path.dirname(self.path), exist_ok=True)
            if os.path.isfile(self.path) and os.path.getsize(self.path) > 0:
                self.total_rows = self._validate_existing(self.path)
                with open(self.path, "rb+") as f:
                    f.seek(-1, os.SEEK_END)
                    if f.read(1) not in (b"\n", b"\r"):
                        f.write(b"\r\n")
            else:
                with open(self.path, "w", newline="", encoding="utf-8") as f:
                    csv.writer(f).writerow(COLUMNS)
        except PermissionError:
            raise DatasetError(f"Permission denied: {self.path}\nClose the file if it is "
                               f"open in Excel, or choose another location.") from None
        except OSError as exc:
            raise DatasetError(f"Cannot use dataset file {self.path}: {exc}") from None

    @staticmethod
    def _validate_existing(path: str) -> int:
        try:
            with open(path, newline="", encoding="utf-8-sig") as f:
                reader = csv.reader(f)
                header = next(reader, None)
                if header is None or [h.strip() for h in header] != COLUMNS:
                    raise DatasetError(f"{os.path.basename(path)} is not a fastener dataset.\n"
                                       f"Expected header: {','.join(COLUMNS)}")
                return sum(1 for row in reader if row)
        except UnicodeDecodeError:
            raise DatasetError(f"{os.path.basename(path)} is not a valid CSV file.") from None
        except csv.Error as exc:
            raise DatasetError(f"Invalid CSV file: {exc}") from None

    def append(self, rec: SensorRecord) -> None:
        def blank(v):
            return "" if v is None else v
        row = [rec.timestamp, rec.object_id, rec.metal,
               "" if rec.height is None else f"{rec.height:.2f}",
               blank(rec.red), blank(rec.green), blank(rec.blue), blank(rec.label)]
        try:
            with open(self.path, "a", newline="", encoding="utf-8") as f:
                csv.writer(f).writerow(row)
        except PermissionError:
            raise DatasetError(f"Permission denied writing {self.path}\n"
                               f"Is the file open in Excel?") from None
        except OSError as exc:
            raise DatasetError(f"Cannot write {self.path}: {exc}") from None
        self.total_rows += 1
        self.session_rows += 1

    def export_copy(self, destination: str) -> None:
        try:
            shutil.copyfile(self.path, destination)
        except OSError as exc:
            raise DatasetError(f"Export failed: {exc}") from None

    @staticmethod
    def read_dataframe(path: str):
        try:
            import pandas as pd
        except ImportError:
            raise DatasetError("pandas is not installed. Run: pip install pandas") from None
        if not os.path.isfile(path):
            raise DatasetError(f"Dataset file not found:\n{path}")
        try:
            df = pd.read_csv(path, encoding="utf-8-sig")
        except Exception as exc:
            raise DatasetError(f"Cannot read dataset: {exc}") from None
        df.columns = [str(c).strip() for c in df.columns]
        missing = [c for c in COLUMNS if c not in df.columns]
        if missing:
            raise DatasetError(f"The dataset is missing columns: {', '.join(missing)}")
        return df


# =============================================================================
#  MACHINE LEARNING (Decision Tree) + C EXPORT
# =============================================================================
FEATURES = ["Metal", "Height", "Red", "Green", "Blue"]
TARGET = "Label"
C_ARGS = ["metal", "height_cm", "red", "green", "blue"]
MIN_RECORDS = 10
RELIABLE_RECORDS = 50
TEST_FRACTION = 0.25
MAX_DEPTH = 5


class MLError(Exception):
    pass


@dataclass
class TrainingResult:
    model: object
    accuracy: float
    train_accuracy: float
    confusion: list
    report: str
    importances: dict
    n_total: int
    n_train: int
    n_test: int
    class_counts: dict
    stratified: bool
    trained_at: str
    dataset_path: str
    warnings: list = field(default_factory=list)

    @property
    def reliable(self) -> bool:
        return self.n_total >= RELIABLE_RECORDS and self.n_test >= 10


def train_from_dataframe(df, dataset_path: str = "") -> TrainingResult:
    try:
        import pandas as pd
        from sklearn.metrics import accuracy_score, classification_report, confusion_matrix
        from sklearn.model_selection import train_test_split
        from sklearn.tree import DecisionTreeClassifier
    except ImportError as exc:
        raise MLError(f"Missing package ({exc.name}). Run: pip install scikit-learn pandas") from None

    data = df[FEATURES + [TARGET]].apply(pd.to_numeric, errors="coerce")
    total = len(data)
    data = data.dropna()
    data = data[data[TARGET].isin([0, 1]) & data["Metal"].isin([0, 1])]
    warnings = []
    if total - len(data):
        warnings.append(f"{total - len(data)} row(s) with missing/invalid values were ignored "
                        f"(e.g. INVALID height, no colour reading, no label).")
    n = len(data)
    if n < MIN_RECORDS:
        raise MLError(f"Only {n} valid labelled record(s). At least {MIN_RECORDS} are required; "
                      f"100+ balanced records are recommended.")
    counts = data[TARGET].astype(int).value_counts().to_dict()
    counts = {0: int(counts.get(0, 0)), 1: int(counts.get(1, 0))}
    if min(counts.values()) == 0:
        raise MLError(f"The dataset contains only one class (Rejected={counts[0]}, "
                      f"Accepted={counts[1]}). Collect both accepted and rejected objects.")

    X = data[FEATURES].astype(float)
    y = data[TARGET].astype(int)
    stratified = True
    try:
        X_tr, X_te, y_tr, y_te = train_test_split(X, y, test_size=TEST_FRACTION,
                                                  random_state=42, stratify=y)
    except ValueError:
        stratified = False
        X_tr, X_te, y_tr, y_te = train_test_split(X, y, test_size=TEST_FRACTION, random_state=42)
        warnings.append("Too few samples in one class for a stratified split; random split used.")

    model = DecisionTreeClassifier(max_depth=MAX_DEPTH, min_samples_leaf=2 if n >= 40 else 1,
                                   random_state=42)
    model.fit(X_tr, y_tr)
    y_pred = model.predict(X_te)
    acc = float(accuracy_score(y_te, y_pred))
    tr_acc = float(model.score(X_tr, y_tr))
    cm = confusion_matrix(y_te, y_pred, labels=[0, 1]).tolist()
    report = classification_report(y_te, y_pred, labels=[0, 1],
                                   target_names=["Rejected (0)", "Accepted (1)"], zero_division=0)
    importances = {f: float(v) for f, v in zip(FEATURES, model.feature_importances_)}

    if n < RELIABLE_RECORDS:
        warnings.append(f"Small dataset ({n} records): the accuracy value is NOT reliable.")
    if len(y_te) < 10:
        warnings.append(f"The test set has only {len(y_te)} samples; accuracy is very coarse.")
    if min(counts.values()) / n < 0.2:
        warnings.append("The classes are strongly imbalanced; collect more of the minority class.")
    if tr_acc - acc > 0.15:
        warnings.append("Training accuracy is much higher than test accuracy (over-fitting).")

    return TrainingResult(model, acc, tr_acc, cm, report, importances, n, len(y_tr), len(y_te),
                          counts, stratified, datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                          dataset_path, warnings)


def save_result(result: TrainingResult, path: str) -> None:
    try:
        import joblib
        joblib.dump(result, path)
    except Exception:
        pass


def load_result(path: str):
    if not os.path.isfile(path):
        return None
    try:
        import joblib
        obj = joblib.load(path)
        return obj if hasattr(obj, "model") and hasattr(obj, "accuracy") else None
    except Exception:
        return None


def predict_record(result: TrainingResult, rec: SensorRecord) -> Optional[int]:
    """Run the trained tree on one record in the GUI (None if a feature is missing)."""
    values = [rec.metal, rec.height, rec.red, rec.green, rec.blue]
    if result is None or any(v is None for v in values):
        return None
    try:
        import pandas as pd
        X = pd.DataFrame([values], columns=FEATURES, dtype=float)
        return int(result.model.predict(X)[0])
    except Exception:
        return None


def details_text(r: TrainingResult) -> str:
    from sklearn.tree import export_text
    tn, fp = r.confusion[0]
    fn, tp = r.confusion[1]
    lines = [
        "DECISION TREE MODEL DETAILS", "=" * 60,
        f"Trained at     : {r.trained_at}",
        f"Dataset        : {r.dataset_path}",
        f"Valid records  : {r.n_total}  (Rejected={r.class_counts[0]}, Accepted={r.class_counts[1]})",
        f"Train / test   : {r.n_train} / {r.n_test}  "
        f"({'stratified' if r.stratified else 'random'} split)",
        f"Tree depth     : {r.model.get_depth()}   Leaves: {r.model.get_n_leaves()}", "",
        f"Test accuracy  : {r.accuracy * 100:.1f} %"
        f"{'' if r.reliable else '   (NOT reliable - dataset too small)'}",
        f"Train accuracy : {r.train_accuracy * 100:.1f} %", "",
        "CONFUSION MATRIX (rows = actual, columns = predicted)",
        f"{'':18}{'Pred Rejected':>15}{'Pred Accepted':>15}",
        f"{'Actual Rejected':18}{tn:>15}{fp:>15}",
        f"{'Actual Accepted':18}{fn:>15}{tp:>15}", "",
        "CLASSIFICATION REPORT", r.report, "FEATURE IMPORTANCE",
    ]
    for name, value in sorted(r.importances.items(), key=lambda kv: -kv[1]):
        lines.append(f"  {name:<8} {value * 100:5.1f} %  {'#' * int(round(value * 30))}")
    if r.warnings:
        lines += ["", "WARNINGS"] + [f"  - {w}" for w in r.warnings]
    lines += ["", "TREE RULES", export_text(r.model, feature_names=FEATURES)]
    return "\n".join(lines)


def _emit_node(tree, node, classes, depth, out):
    pad = "    " * depth
    if tree.children_left[node] == tree.children_right[node]:
        values = list(tree.value[node][0])
        cls = int(classes[max(range(len(values)), key=values.__getitem__)])
        out.append(f"{pad}return {cls}u;")
        return
    arg = C_ARGS[int(tree.feature[node])]
    out.append(f"{pad}if ({arg} <= {float(tree.threshold[node]):.6f}f) {{")
    _emit_node(tree, int(tree.children_left[node]), classes, depth + 1, out)
    out.append(f"{pad}}} else {{")
    _emit_node(tree, int(tree.children_right[node]), classes, depth + 1, out)
    out.append(f"{pad}}}")


def export_c_files(r: TrainingResult, directory: str) -> list:
    model = r.model
    classes = [int(c) for c in model.classes_]
    if sorted(classes) != [0, 1]:
        raise MLError("Model classes must be exactly 0 (Rejected) and 1 (Accepted).")
    info = "\n".join([
        f" * Generated by Smart Fastener Sorting System GUI on {r.trained_at}",
        f" * DecisionTreeClassifier (depth {model.get_depth()}, {model.get_n_leaves()} leaves)",
        f" * Dataset: {r.n_total} records (Rejected={r.class_counts[0]}, "
        f"Accepted={r.class_counts[1]})",
        f" * Test accuracy on laptop: {r.accuracy * 100:.1f} %"
        f"{'' if r.reliable else ' (NOT reliable - small dataset)'}",
        " * Features (in this order): Metal, Height, Red, Green, Blue",
        " * DO NOT EDIT BY HAND - re-train and re-export instead.",
    ])
    header = f"""/*
{info}
 */
#ifndef FASTENER_MODEL_H
#define FASTENER_MODEL_H

#include <stdint.h>

#define FASTENER_REJECTED 0u
#define FASTENER_ACCEPTED 1u

/* metal: 1.0f/0.0f, height_cm: cm, red/green/blue: TCS3200 counts (same scale as CSV) */
uint8_t fastener_predict(float metal, float height_cm,
                         float red, float green, float blue);

#endif /* FASTENER_MODEL_H */
"""
    body: list = []
    _emit_node(model.tree_, 0, classes, 1, body)
    source = f"""/*
{info}
 */
#include "fastener_model.h"

uint8_t fastener_predict(float metal, float height_cm,
                         float red, float green, float blue)
{{
    (void)metal; (void)height_cm; (void)red; (void)green; (void)blue;
{chr(10).join(body)}
}}
"""
    os.makedirs(directory, exist_ok=True)
    paths = []
    for name, text in (("fastener_model.h", header), ("fastener_model.c", source)):
        p = os.path.join(directory, name)
        with open(p, "w", encoding="utf-8", newline="\n") as f:
            f.write(text)
        paths.append(p)
    return paths


# =============================================================================
#  GRAPHS
# =============================================================================
class LiveGraphs:
    def __init__(self, parent, max_points: int = 50):
        self.ids = deque(maxlen=max_points)
        self.heights = deque(maxlen=max_points)
        self.reds = deque(maxlen=max_points)
        self.greens = deque(maxlen=max_points)
        self.blues = deque(maxlen=max_points)
        self._dirty = False

        self.fig = Figure(figsize=(6, 2.4), dpi=100, facecolor="white", constrained_layout=True)
        self.ax_h = self.fig.add_subplot(1, 2, 1)
        self.ax_c = self.fig.add_subplot(1, 2, 2)
        self.ax_h.set_title("Height (cm)", fontsize=10, fontweight="bold")
        self.ax_c.set_title("RGB Values", fontsize=10, fontweight="bold")
        for ax, yl in ((self.ax_h, "Height (cm)"), (self.ax_c, "Sensor Value")):
            ax.set_xlabel("Object Number", fontsize=8)
            ax.set_ylabel(yl, fontsize=8)
            ax.tick_params(labelsize=8)
            ax.grid(True, alpha=0.3)
        (self.line_h,) = self.ax_h.plot([], [], "o-", color=BLUE, markersize=3, linewidth=1.5)
        (self.line_r,) = self.ax_c.plot([], [], "o-", color="red", markersize=3, lw=1.5, label="Red")
        (self.line_g,) = self.ax_c.plot([], [], "o-", color="green", markersize=3, lw=1.5, label="Green")
        (self.line_b,) = self.ax_c.plot([], [], "o-", color="blue", markersize=3, lw=1.5, label="Blue")
        self.ax_c.legend(fontsize=7, loc="upper right")
        self.canvas = FigureCanvasTkAgg(self.fig, master=parent)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

    @staticmethod
    def _nan(v):
        return float("nan") if v is None else v

    def add(self, rec: SensorRecord) -> None:
        self.ids.append(rec.object_id)
        self.heights.append(self._nan(rec.height))
        self.reds.append(self._nan(rec.red))
        self.greens.append(self._nan(rec.green))
        self.blues.append(self._nan(rec.blue))
        self._dirty = True

    def clear(self) -> None:
        for d in (self.ids, self.heights, self.reds, self.greens, self.blues):
            d.clear()
        self._dirty = True

    def refresh(self) -> None:
        if not self._dirty:
            return
        self._dirty = False
        x = list(self.ids)
        self.line_h.set_data(x, list(self.heights))
        self.line_r.set_data(x, list(self.reds))
        self.line_g.set_data(x, list(self.greens))
        self.line_b.set_data(x, list(self.blues))
        for ax in (self.ax_h, self.ax_c):
            ax.relim()
            ax.autoscale_view()
            ax.set_ylim(bottom=0)
        self.canvas.draw_idle()


class PieChart:
    def __init__(self, parent):
        self.fig = Figure(figsize=(2.6, 2.2), dpi=100, facecolor="white", constrained_layout=True)
        self.ax = self.fig.add_subplot(1, 1, 1)
        self.canvas = FigureCanvasTkAgg(self.fig, master=parent)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        self.update(0, 0)

    def update(self, accepted: int, rejected: int) -> None:
        ax = self.ax
        ax.clear()
        if accepted + rejected == 0:
            ax.text(0.5, 0.5, "No data yet", ha="center", va="center",
                    transform=ax.transAxes, color=GRAY, fontsize=10)
            ax.set_axis_off()
        else:
            wedges, _t, _a = ax.pie(
                [accepted, rejected], colors=[GREEN, RED], startangle=90,
                autopct=lambda p: f"{p:.0f}%" if p > 0 else "",
                textprops={"color": "white", "fontweight": "bold", "fontsize": 9})
            ax.legend(wedges, ["Accepted", "Rejected"], loc="lower center",
                      bbox_to_anchor=(0.5, -0.18), ncol=2, frameon=False, fontsize=8)
            ax.axis("equal")
        self.canvas.draw_idle()


# =============================================================================
#  OPTIONAL CAMERA PANEL
# =============================================================================
PLACEHOLDER = "Camera not connected"


class CameraView(tk.Frame):
    def __init__(self, parent, bg: str = "white"):
        super().__init__(parent, bg=bg)
        self._cap = None
        self._photo = None
        self._job = None
        self.holder = tk.Frame(self, bg="#dfe6ef", width=300, height=180)
        self.holder.pack(fill="both", expand=True)
        self.holder.pack_propagate(False)
        self.view = tk.Label(self.holder, text=PLACEHOLDER, bg="#dfe6ef", fg="#5b6b7d",
                             font=("Segoe UI", 11), justify="center")
        self.view.pack(fill="both", expand=True)
        bar = tk.Frame(self, bg=bg)
        bar.pack(fill="x", pady=(6, 0))
        tk.Label(bar, text="Camera index:", bg=bg, font=F_BODY).pack(side="left")
        self.index_var = tk.StringVar(value="0")
        ttk.Spinbox(bar, from_=0, to=9, width=3, textvariable=self.index_var,
                    state="readonly").pack(side="left", padx=4)
        self.btn = ttk.Button(bar, text="Start Camera", command=self.toggle)
        self.btn.pack(side="right")
        if not (CV2_OK and PIL_OK):
            self.btn.state(["disabled"])
            self.view.config(text=PLACEHOLDER + "\n(optional: pip install opencv-python)")

    def toggle(self) -> None:
        self.stop() if self._cap is not None else self.start()

    def start(self) -> None:
        if not (CV2_OK and PIL_OK):
            return
        try:
            index = int(self.index_var.get())
            cap = cv2.VideoCapture(index, cv2.CAP_DSHOW) if os.name == "nt" else cv2.VideoCapture(index)
        except Exception:
            self.view.config(image="", text=PLACEHOLDER + "\n(could not open camera)")
            return
        if cap is None or not cap.isOpened():
            if cap is not None:
                cap.release()
            self.view.config(image="", text=PLACEHOLDER + f"\n(no camera at index {index})")
            return
        self._cap = cap
        self.btn.config(text="Stop Camera")
        self._tick()

    def stop(self, message: str = PLACEHOLDER) -> None:
        if self._job is not None:
            try:
                self.after_cancel(self._job)
            except Exception:
                pass
            self._job = None
        if self._cap is not None:
            try:
                self._cap.release()
            except Exception:
                pass
            self._cap = None
        self._photo = None
        self.view.config(image="", text=message)
        self.btn.config(text="Start Camera")

    def _tick(self) -> None:
        if self._cap is None:
            return
        try:
            ok, frame = self._cap.read()
            if not ok:
                self.stop(PLACEHOLDER + "\n(camera stopped sending frames)")
                return
            img = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            img.thumbnail((max(self.holder.winfo_width(), 60), max(self.holder.winfo_height(), 60)))
            self._photo = ImageTk.PhotoImage(img)
            self.view.config(image=self._photo, text="")
        except Exception:
            self.stop(PLACEHOLDER + "\n(camera error)")
            return
        self._job = self.after(40, self._tick)


# =============================================================================
#  MAIN WINDOW
# =============================================================================
class ScrollFrame(tk.Frame):
    """Vertical scroll container for the left column (scrollbar only when needed)."""

    def __init__(self, parent, width: int, bg: str):
        super().__init__(parent, bg=bg)
        self.rowconfigure(0, weight=1)
        self.canvas = tk.Canvas(self, width=width, bg=bg, highlightthickness=0)
        self.vbar = ttk.Scrollbar(self, orient="vertical", command=self.canvas.yview)
        self.canvas.configure(yscrollcommand=self.vbar.set)
        self.canvas.grid(row=0, column=0, sticky="ns")
        self.inner = tk.Frame(self.canvas, bg=bg)
        self.canvas.create_window((0, 0), window=self.inner, anchor="nw", width=width)
        self.inner.bind("<Configure>", self._on_configure)
        self.canvas.bind("<Configure>", self._on_configure)
        self.canvas.bind("<Enter>", lambda e: self.canvas.bind_all("<MouseWheel>", self._wheel))
        self.canvas.bind("<Leave>", lambda e: self.canvas.unbind_all("<MouseWheel>"))

    def _on_configure(self, _e=None):
        self.canvas.configure(scrollregion=self.canvas.bbox("all"))
        if self.inner.winfo_reqheight() > self.canvas.winfo_height():
            self.vbar.grid(row=0, column=1, sticky="ns")
        else:
            self.vbar.grid_remove()

    def _wheel(self, event):
        if self.inner.winfo_reqheight() > self.canvas.winfo_height():
            self.canvas.yview_scroll(int(-event.delta / 120), "units")


class FastenerApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        root.title("Smart Fastener Sorting System")
        sw, sh = root.winfo_screenwidth(), root.winfo_screenheight()
        w, h = min(1360, sw - 40), min(840, sh - 80)
        root.geometry(f"{w}x{h}+{max((sw - w) // 2, 0)}+10")
        root.minsize(1000, 600)
        root.configure(bg=BG)

        self.events: queue.Queue = queue.Queue()
        self.link = BluetoothLink(self.events)
        self.parser = FirmwareParser()
        self.dataset: Optional[DatasetManager] = None
        self.collecting = False
        self.training = False
        self.model_result: Optional[TrainingResult] = None
        self.port_map: dict = {}
        self.recent_keys: deque = deque(maxlen=50)
        self.stats = {"total": 0, "accepted": 0, "rejected": 0, "agree": 0, "compared": 0}
        self.invalid_count = 0
        self.duplicate_count = 0
        self._ir_timer = None
        self.inspecting = False          # True between OBJECT #n and its result
        self.connected_at = None
        self.last_line_at = None
        self.lines_received = 0
        self.garbled_lines = 0
        self.no_data_warned = False
        self.garbled_warned = False

        self._setup_style()
        self._build_ui()
        self.refresh_ports()

        cached = load_result(MODEL_FILE)
        if cached is not None:
            self.model_result = cached
            self._show_model_summary(cached, loaded=True)

        root.protocol("WM_DELETE_WINDOW", self.on_close)
        root.after(50, self._poll)
        root.after(250, self._refresh_graphs)
        root.after(1000, self._watchdog)

    # ------------------------------------------------------------- styling
    def _setup_style(self) -> None:
        st = ttk.Style()
        try:
            st.theme_use("clam")
        except tk.TclError:
            pass
        st.configure("TButton", font=F_BODY, padding=(8, 4))
        for name, color, hover in (("Green", GREEN, "#168033"), ("Red", RED, "#b82e2e"),
                                   ("Blue", BLUE, "#0f4c92")):
            st.configure(f"{name}.TButton", background=color, foreground="white",
                         font=("Segoe UI", 9, "bold"), padding=(8, 6), borderwidth=0)
            st.map(f"{name}.TButton", background=[("disabled", "#b6bfcb"), ("active", hover)],
                   foreground=[("disabled", "#f3f4f6")])
        st.map("TCombobox", fieldbackground=[("readonly", "white")])
        st.configure("Treeview", rowheight=22, font=F_BODY, background="white",
                     fieldbackground="white")
        st.configure("Treeview.Heading", font=("Segoe UI", 9, "bold"), background="#e3ebf5",
                     foreground=TEXT)

    def _panel(self, parent, title: str):
        outer = tk.Frame(parent, bg=PANEL, highlightbackground=BORDER, highlightthickness=1)
        tk.Label(outer, text=title, font=F_PANEL, fg=HEADER, bg=PANEL,
                 anchor="w").pack(fill="x", padx=10, pady=(8, 2))
        body = tk.Frame(outer, bg=PANEL)
        body.pack(fill="both", expand=True, padx=10, pady=(0, 8))
        return outer, body

    def _card(self, parent, title, value="--", bg=CARD, fg_title=GRAY, fg_value=TEXT, size=16):
        frame = tk.Frame(parent, bg=bg, highlightbackground=BORDER, highlightthickness=1)
        tk.Label(frame, text=title, font=("Segoe UI", 9, "bold"), fg=fg_title, bg=bg).pack(pady=(6, 0))
        lbl = tk.Label(frame, text=value, font=("Segoe UI", size, "bold"), fg=fg_value, bg=bg)
        lbl.pack(pady=(0, 8), padx=4, expand=True)
        return frame, lbl

    # ---------------------------------------------------------------- build
    def _build_ui(self) -> None:
        root = self.root
        header = tk.Frame(root, bg=HEADER)
        header.pack(fill="x")
        tk.Label(header, text="\u2699  Smart Fastener Sorting System", font=F_TITLE, fg="white",
                 bg=HEADER).pack(side="left", padx=14, pady=8)
        tk.Label(header, text="STM32 + Bluetooth + Machine Learning", font=("Segoe UI", 10),
                 fg="#cfe0f5", bg=HEADER).pack(side="right", padx=14)

        self.status_var = tk.StringVar(value="Ready. Bluetooth is not connected.")
        self.status_bar = tk.Label(root, textvariable=self.status_var, anchor="w", font=F_BODY,
                                   bg="#dfe7f1", fg=TEXT, padx=10, pady=3)
        self.status_bar.pack(side="bottom", fill="x")

        main = tk.Frame(root, bg=BG)
        main.pack(fill="both", expand=True, padx=8, pady=8)
        main.columnconfigure(1, weight=1)
        main.rowconfigure(0, weight=1)

        scroll = ScrollFrame(main, width=290, bg=BG)
        scroll.grid(row=0, column=0, sticky="ns", padx=(0, 8))
        self._build_bluetooth_panel(scroll.inner)
        self._build_data_panel(scroll.inner)
        self._build_ml_panel(scroll.inner)

        right = tk.Frame(main, bg=BG)
        right.grid(row=0, column=1, sticky="nsew")
        right.columnconfigure(0, weight=3)
        right.columnconfigure(1, weight=1, minsize=300)
        right.rowconfigure(1, weight=3)
        right.rowconfigure(2, weight=2)
        self._build_right(right)

    def _build_bluetooth_panel(self, parent) -> None:
        outer, body = self._panel(parent, "Bluetooth Connection")
        outer.pack(fill="x", pady=(0, 8))
        body.columnconfigure(1, weight=1)
        tk.Label(body, text="COM Port:", font=F_BODY, bg=PANEL).grid(row=0, column=0, sticky="w", pady=3)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(body, textvariable=self.port_var, state="readonly", width=24)
        self.port_combo.grid(row=0, column=1, sticky="ew", padx=(6, 0), pady=3)
        ttk.Button(body, text="Refresh COM ports", command=self.refresh_ports).grid(
            row=1, column=0, columnspan=2, sticky="ew", pady=3)
        tk.Label(body, text="Baud Rate:", font=F_BODY, bg=PANEL).grid(row=2, column=0, sticky="w", pady=3)
        self.baud_var = tk.StringVar(value=DEFAULT_BAUD)
        self.baud_combo = ttk.Combobox(body, textvariable=self.baud_var, values=BAUD_RATES,
                                       state="readonly", width=10)
        self.baud_combo.grid(row=2, column=1, sticky="w", padx=(6, 0), pady=3)

        st = tk.Frame(body, bg=PANEL)
        st.grid(row=3, column=0, columnspan=2, pady=6)
        self.led = tk.Canvas(st, width=18, height=18, bg=PANEL, highlightthickness=0)
        self.led.pack(side="left")
        self.led_dot = self.led.create_oval(2, 2, 16, 16, fill=RED, outline="")
        self.conn_label = tk.Label(st, text="Disconnected", font=("Segoe UI", 11, "bold"),
                                   fg=RED, bg=PANEL)
        self.conn_label.pack(side="left", padx=6)

        btns = tk.Frame(body, bg=PANEL)
        btns.grid(row=4, column=0, columnspan=2, sticky="ew")
        btns.columnconfigure((0, 1), weight=1)
        self.btn_connect = ttk.Button(btns, text="Connect", style="Green.TButton", command=self.connect)
        self.btn_connect.grid(row=0, column=0, sticky="ew", padx=(0, 3))
        self.btn_disconnect = ttk.Button(btns, text="Disconnect", style="Red.TButton",
                                         command=self.disconnect, state="disabled")
        self.btn_disconnect.grid(row=0, column=1, sticky="ew", padx=(3, 0))
        self.rx_label = tk.Label(body, text="Received: 0 bytes, 0 lines", font=F_SMALL,
                                 fg=GRAY, bg=PANEL, anchor="w")
        self.rx_label.grid(row=5, column=0, columnspan=2, sticky="w", pady=(4, 0))

    def _build_data_panel(self, parent) -> None:
        outer, body = self._panel(parent, "Data Collection")
        outer.pack(fill="x", pady=(0, 8))
        body.columnconfigure(0, weight=1)
        tk.Label(body, text="Dataset File:", font=F_BODY, bg=PANEL).grid(row=0, column=0, columnspan=2, sticky="w")
        self.dataset_var = tk.StringVar(value=DEFAULT_CSV)
        self.dataset_entry = ttk.Entry(body, textvariable=self.dataset_var)
        self.dataset_entry.grid(row=1, column=0, sticky="ew", pady=3)
        self.btn_browse = ttk.Button(body, text="Browse...", command=self.browse_dataset, width=9)
        self.btn_browse.grid(row=1, column=1, padx=(4, 0), pady=3)
        self.btn_start = ttk.Button(body, text="\u25B6  Start Collection", style="Green.TButton",
                                    command=self.start_collection)
        self.btn_start.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(4, 2))
        self.btn_stop = ttk.Button(body, text="\u25A0  Stop Collection", style="Red.TButton",
                                   command=self.stop_collection, state="disabled")
        self.btn_stop.grid(row=3, column=0, columnspan=2, sticky="ew", pady=2)
        self.col_status = tk.Label(body, text="Status: Idle (not collecting)", font=F_BODY,
                                   fg=GRAY, bg=PANEL, anchor="w")
        self.col_status.grid(row=4, column=0, columnspan=2, sticky="w", pady=(4, 0))
        self.col_count = tk.Label(body, text="Collected this session: 0", font=F_BODY, bg=PANEL, anchor="w")
        self.col_count.grid(row=5, column=0, columnspan=2, sticky="w")
        self.col_file_count = tk.Label(body, text="Records in file: -", font=F_BODY, bg=PANEL, anchor="w")
        self.col_file_count.grid(row=6, column=0, columnspan=2, sticky="w")
        self.col_skipped = tk.Label(body, text="Invalid: 0   Duplicates: 0", font=F_BODY,
                                    fg=GRAY, bg=PANEL, anchor="w")
        self.col_skipped.grid(row=7, column=0, columnspan=2, sticky="w")
        ttk.Button(body, text="Export CSV...", command=self.export_csv).grid(
            row=8, column=0, columnspan=2, sticky="ew", pady=(6, 0))

    def _build_ml_panel(self, parent) -> None:
        outer, body = self._panel(parent, "ML Model")
        outer.pack(fill="x")
        body.columnconfigure(0, weight=1)
        self.btn_train = ttk.Button(body, text="Train Model", style="Blue.TButton", command=self.train_model)
        self.btn_train.grid(row=0, column=0, sticky="ew", pady=2)
        ttk.Button(body, text="View Model Details", command=self.view_model_details).grid(
            row=1, column=0, sticky="ew", pady=2)
        ttk.Button(body, text="Export to C Code", command=self.export_c).grid(
            row=2, column=0, sticky="ew", pady=2)
        tk.Label(body, text="Model: Decision Tree", font=F_BODY, bg=PANEL, anchor="w").grid(
            row=3, column=0, sticky="w", pady=(6, 0))
        self.ml_status = tk.Label(body, text="Status: Not trained", font=F_BODY, bg=PANEL, fg=GRAY,
                                  anchor="w", justify="left", wraplength=250)
        self.ml_status.grid(row=4, column=0, sticky="w")
        self.ml_acc = tk.Label(body, text="Accuracy: --", font=("Segoe UI", 11, "bold"), bg=PANEL,
                               fg=TEXT, anchor="w", justify="left", wraplength=250)
        self.ml_acc.grid(row=5, column=0, sticky="w")
        self.ml_agree = tk.Label(body, text="", font=F_SMALL, bg=PANEL, fg=GRAY, anchor="w",
                                 justify="left", wraplength=250)
        self.ml_agree.grid(row=6, column=0, sticky="w")

    def _build_right(self, right) -> None:
        sensor_outer, sb = self._panel(right, "Live Sensor Readings")
        sensor_outer.grid(row=0, column=0, sticky="nsew", padx=(0, 8), pady=(0, 8))
        for c in range(4):
            sb.columnconfigure(c, weight=1, uniform="card")

        f, self.id_val = self._card(sb, "Object Number", fg_value=BLUE)
        f.grid(row=0, column=0, sticky="nsew", padx=3, pady=3)
        f, self.ir_val = self._card(sb, "IR Sensor", "Waiting", size=12, fg_value=GRAY)
        f.grid(row=0, column=1, sticky="nsew", padx=3, pady=3)
        f, self.metal_val = self._card(sb, "Metal Sensor", size=12)
        f.grid(row=0, column=2, sticky="nsew", padx=3, pady=3)
        f, self.height_val = self._card(sb, "Object Height", fg_value=ORANGE)
        f.grid(row=0, column=3, sticky="nsew", padx=3, pady=3)

        rgb = tk.Frame(sb, bg=PANEL)
        rgb.grid(row=1, column=0, columnspan=2, sticky="nsew")
        rgb.columnconfigure((0, 1, 2), weight=1, uniform="rgb")
        self.rgb_vals = []
        for i, (name, color) in enumerate((("R", "#e53935"), ("G", "#2e9e43"), ("B", "#1e6fd9"))):
            f, v = self._card(rgb, name, "--", bg=color, fg_title="white", fg_value="white", size=18)
            f.grid(row=0, column=i, sticky="nsew", padx=3, pady=3)
            self.rgb_vals.append(v)

        self.pred_card = tk.Frame(sb, bg=CARD, highlightbackground=BORDER, highlightthickness=1)
        self.pred_card.grid(row=1, column=2, columnspan=2, sticky="nsew", padx=3, pady=3)
        self.pred_title = tk.Label(self.pred_card, text="ML Prediction / Sorting Result",
                                   font=("Segoe UI", 9, "bold"), fg=GRAY, bg=CARD)
        self.pred_title.pack(pady=(6, 0))
        self.pred_val = tk.Label(self.pred_card, text="Waiting", font=("Segoe UI", 16, "bold"),
                                 fg=GRAY, bg=CARD)
        self.pred_val.pack()
        self.pred_class = tk.Label(self.pred_card, text="Class: --", font=F_BODY, bg=CARD, fg=TEXT)
        self.pred_class.pack()
        self.pred_status = tk.Label(self.pred_card, text="Status: waiting for data", font=F_SMALL,
                                    bg=CARD, fg=GRAY)
        self.pred_status.pack(pady=(0, 6))

        cam_outer, cb = self._panel(right, "Live Camera / Conveyor View")
        cam_outer.grid(row=0, column=1, sticky="nsew", pady=(0, 8))
        self.camera = CameraView(cb, bg=PANEL)
        self.camera.pack(fill="both", expand=True)

        graph_outer, gb = self._panel(right, "Real-time Graphs")
        graph_outer.grid(row=1, column=0, sticky="nsew", padx=(0, 8), pady=(0, 8))
        self.graphs = LiveGraphs(gb, max_points=50)

        stat_outer, tb = self._panel(right, "Sorting Statistics")
        stat_outer.grid(row=1, column=1, rowspan=2, sticky="nsew")
        self._build_stats(tb)
        self._build_history(right)

    def _build_stats(self, body) -> None:
        body.columnconfigure((0, 1), weight=1, uniform="st")
        f, self.st_acc = self._card(body, "Accepted", "0", bg=GREEN_BG, fg_value=GREEN, size=18)
        f.grid(row=0, column=0, sticky="nsew", padx=2, pady=2)
        f, self.st_rej = self._card(body, "Rejected", "0", bg=RED_BG, fg_value=RED, size=18)
        f.grid(row=0, column=1, sticky="nsew", padx=2, pady=2)
        f, self.st_total = self._card(body, "Total Objects", "0", bg="#e5f0fb", fg_value=HEADER, size=18)
        f.grid(row=1, column=0, columnspan=2, sticky="nsew", padx=2, pady=2)
        self.st_pct = tk.Label(body, text="Accepted: 0.0 %    Rejected: 0.0 %", font=F_BODY,
                               bg=PANEL, fg=TEXT)
        self.st_pct.grid(row=2, column=0, columnspan=2, pady=(4, 0))
        holder = tk.Frame(body, bg=PANEL)
        holder.grid(row=3, column=0, columnspan=2, sticky="nsew")
        body.rowconfigure(3, weight=1)
        self.pie = PieChart(holder)

    def _build_history(self, right) -> None:
        frame = tk.Frame(right, bg=PANEL, highlightbackground=BORDER, highlightthickness=1)
        frame.grid(row=2, column=0, sticky="nsew", padx=(0, 8))
        nb = ttk.Notebook(frame)
        nb.pack(fill="both", expand=True, padx=6, pady=6)

        hist = tk.Frame(nb, bg=PANEL)
        nb.add(hist, text="Object History / Dataset")
        top = tk.Frame(hist, bg=PANEL)
        top.pack(fill="x", pady=(2, 4))
        self.total_label = tk.Label(top, text="Total objects: 0   |   In table: 0",
                                    font=("Segoe UI", 9, "bold"), bg=PANEL, fg=TEXT)
        self.total_label.pack(side="left")
        ttk.Button(top, text="Clear Table", command=self.clear_table).pack(side="right")

        tf = tk.Frame(hist, bg=PANEL)
        tf.pack(fill="both", expand=True)
        cols = ("id", "time", "metal", "height", "red", "green", "blue", "pred", "result", "reason")
        heads = ("#", "Timestamp", "Metal", "Height (cm)", "Red", "Green", "Blue",
                 "ML Prediction", "Result", "Reason")
        widths = (45, 140, 50, 80, 50, 50, 50, 90, 80, 170)
        self.tree = ttk.Treeview(tf, columns=cols, show="headings", height=5)
        for c, h, w in zip(cols, heads, widths):
            self.tree.heading(c, text=h)
            self.tree.column(c, width=w, minwidth=40, anchor="center", stretch=True)
        self.tree.tag_configure("accepted", foreground="#12802f")
        self.tree.tag_configure("rejected", foreground="#c62828")
        self.tree.tag_configure("unknown", foreground=GRAY)
        sb = ttk.Scrollbar(tf, orient="vertical", command=self.tree.yview)
        self.tree.configure(yscrollcommand=sb.set)
        self.tree.pack(side="left", fill="both", expand=True)
        sb.pack(side="right", fill="y")

        logf = tk.Frame(nb, bg=PANEL)
        nb.add(logf, text="Bluetooth Log")
        ltop = tk.Frame(logf, bg=PANEL)
        ltop.pack(fill="x", pady=(2, 4))
        tk.Label(ltop, text="Raw incoming Bluetooth messages", font=F_BODY, bg=PANEL,
                 fg=GRAY).pack(side="left")
        ttk.Button(ltop, text="Clear Log", command=self.clear_log).pack(side="right")
        lf = tk.Frame(logf, bg=PANEL)
        lf.pack(fill="both", expand=True)
        self.log = tk.Text(lf, height=6, wrap="none", font=("Consolas", 9), state="disabled",
                           bg="#fbfcfe", relief="flat")
        self.log.tag_configure("data", foreground="#12802f")
        self.log.tag_configure("warn", foreground=ORANGE)
        self.log.tag_configure("sys", foreground=BLUE)
        lsb = ttk.Scrollbar(lf, orient="vertical", command=self.log.yview)
        self.log.configure(yscrollcommand=lsb.set)
        self.log.pack(side="left", fill="both", expand=True)
        lsb.pack(side="right", fill="y")

    # ------------------------------------------------------------ utilities
    def set_status(self, text: str, color: str = TEXT) -> None:
        self.status_var.set(text)
        self.status_bar.config(fg=color)

    def _log(self, text: str, tag: str = "") -> None:
        self.log.config(state="normal")
        self.log.insert("end", f"[{datetime.now():%H:%M:%S}] {text}\n", tag)
        lines = int(self.log.index("end-1c").split(".")[0])
        if lines > LOG_LIMIT:
            self.log.delete("1.0", f"{lines - LOG_LIMIT + 1}.0")
        self.log.see("end")
        self.log.config(state="disabled")

    def clear_log(self) -> None:
        self.log.config(state="normal")
        self.log.delete("1.0", "end")
        self.log.config(state="disabled")

    def _resolve_dataset_path(self) -> str:
        name = self.dataset_var.get().strip() or DEFAULT_CSV
        if not os.path.isabs(name):
            name = os.path.join(APP_DIR, name)
        return os.path.abspath(name)

    def _paint_pred(self, text, fg, bg, cls, status) -> None:
        for w in (self.pred_card,):
            w.config(bg=bg)
        self.pred_title.config(bg=bg)
        self.pred_val.config(text=text, fg=fg, bg=bg)
        self.pred_class.config(text=cls, bg=bg)
        self.pred_status.config(text=status, bg=bg)

    def _set_ir(self, text: str, color: str, idle_after_ms: int = 0) -> None:
        self.ir_val.config(text=text, fg=color)
        if self._ir_timer is not None:
            self.root.after_cancel(self._ir_timer)
            self._ir_timer = None
        if idle_after_ms:
            self._ir_timer = self.root.after(idle_after_ms, self._ir_idle)

    def _ir_idle(self) -> None:
        self._ir_timer = None
        self.ir_val.config(text="Waiting", fg=GRAY)

    def _show_metal(self, flag: Optional[int]) -> None:
        if flag == 1:
            self.metal_val.config(text="DETECTED", fg=GREEN)
        elif flag == 0:
            self.metal_val.config(text="NOT DETECTED", fg=ORANGE)

    def _show_height(self, h: Optional[float]) -> None:
        if h is None:
            self.height_val.config(text="INVALID", fg=RED)
        else:
            self.height_val.config(text=f"{h:g} cm", fg=ORANGE)

    def _show_rgb(self, r, g, b) -> None:
        for lbl, v in zip(self.rgb_vals, (r, g, b)):
            lbl.config(text="--" if v is None else str(v))

    # -------------------------------------------------------- bluetooth UI
    def refresh_ports(self) -> None:
        ports = list_ports()
        self.port_map = {f"{d} - {desc}" if desc else d: d for d, desc in ports}
        values = list(self.port_map)
        self.port_combo["values"] = values
        if self.port_var.get() not in self.port_map:
            self.port_var.set(values[0] if len(values) == 1 else "")
        if values:
            self.set_status(f"Found {len(values)} COM port(s). Select the HC-05 'Outgoing' port, "
                            f"then press Connect.")
        else:
            self.set_status("No COM ports found. Pair the HC-05 in Windows Bluetooth settings, "
                            "then press Refresh.", ORANGE)

    def connect(self) -> None:
        sel = self.port_var.get()
        if not sel:
            messagebox.showwarning("Select a port", "Select the HC-05 COM port first.\n"
                                   "Press 'Refresh COM ports' if it is not listed.")
            return
        port = self.port_map.get(sel, sel.split(" - ")[0])
        baud = int(self.baud_var.get())
        self.btn_connect.config(state="disabled")
        self.port_combo.config(state="disabled")
        self.baud_combo.config(state="disabled")
        self.conn_label.config(text="Connecting...", fg=ORANGE)
        self.led.itemconfig(self.led_dot, fill=ORANGE)
        self.set_status(f"Connecting to {port} at {baud} baud (Bluetooth can take a few seconds)...")
        self.link.connect_async(port, baud)

    def disconnect(self) -> None:
        self.link.disconnect()
        self.parser.reset()
        self._set_connected(False)
        self.set_status("Disconnected. Data collection state is unchanged.")
        self._log("Disconnected", "sys")

    def _set_connected(self, connected: bool) -> None:
        if connected:
            self.led.itemconfig(self.led_dot, fill=GREEN)
            self.conn_label.config(text="Connected", fg=GREEN)
            self.btn_connect.config(state="disabled")
            self.btn_disconnect.config(state="normal")
        else:
            self.led.itemconfig(self.led_dot, fill=RED)
            self.conn_label.config(text="Disconnected", fg=RED)
            self.btn_connect.config(state="normal")
            self.btn_disconnect.config(state="disabled")
            self.port_combo.config(state="readonly")
            self.baud_combo.config(state="readonly")

    # --------------------------------------------------------------- events
    def _poll(self) -> None:
        try:
            for _ in range(300):
                try:
                    event = self.events.get_nowait()
                except queue.Empty:
                    break
                try:
                    self._dispatch(event)
                except Exception:
                    self._log("Internal error: " + traceback.format_exc().strip().splitlines()[-1], "warn")
        finally:
            self.root.after(50, self._poll)

    def _dispatch(self, event: tuple) -> None:
        kind = event[0]
        if kind == "line":
            now = datetime.now()
            self.last_line_at = now
            self.lines_received += 1
            line = event[1]
            bad = sum(1 for ch in line if ch == "\ufffd" or (ord(ch) < 32 and ch != "\t"))
            if line and bad / len(line) > 0.2:
                self.garbled_lines += 1
                self._log("GARBLED: " + repr(line), "warn")
                if self.garbled_lines >= 3 and not self.garbled_warned:
                    self.garbled_warned = True
                    self._log("Data is arriving but is unreadable -> baud rate / clock mismatch. "
                              "Check the STM32 USART2 BRR and the HC-05 UART baud (AT+UART?).", "warn")
                    self.set_status("Receiving garbled data: baud rate mismatch (see Bluetooth Log).", RED)
                return
            for ev in self.parser.feed(line):
                self._on_parser_event(ev)
        elif kind == "connected":
            _, port, baud = event
            self.connected_at = datetime.now()
            self.last_line_at = None
            self.lines_received = 0
            self.garbled_lines = 0
            self.no_data_warned = False
            self.garbled_warned = False
            self.parser.reset()
            self._set_connected(True)
            self.set_status(f"Connected to {port} at {baud} baud. Waiting for data from STM32...", GREEN)
            self._log(f"Connected to {port} @ {baud}", "sys")
        elif kind == "connect_failed":
            self._set_connected(False)
            self.set_status("Connection failed.", RED)
            self._log("Connection failed: " + event[1].splitlines()[0], "warn")
            messagebox.showerror("Bluetooth connection failed", event[1])
        elif kind == "disconnected":
            self.link.disconnect()
            self.parser.reset()
            self._set_connected(False)
            self.set_status("Bluetooth connection lost: " + event[1], RED)
            self._log("Connection lost: " + event[1], "warn")
        elif kind == "overflow":
            self._log(event[1], "warn")
        elif kind == "train_done":
            self._on_train_done(event[1])
        elif kind == "train_failed":
            self._on_train_failed(event[1])

    def _on_parser_event(self, ev: tuple) -> None:
        kind = ev[0]
        if kind == "log":
            self._log(ev[1], ev[2])
        elif kind == "ready":
            self._paint_pred("System ready", BLUE, CARD, "Class: --", "Status: STM32 ready")
            self.set_status("STM32 sorting system ready.", GREEN)
        elif kind == "object_start":
            self.inspecting = True
            self.id_val.config(text=str(ev[1]))
            self._set_ir("Object detected", GREEN)
            self.metal_val.config(text="--", fg=TEXT)
            self.height_val.config(text="--", fg=ORANGE)
            self._paint_pred("Checking...", ORANGE, CARD, "Class: --",
                             f"Status: inspecting object #{ev[1]}")
        elif kind == "checking":
            if ev[1] == "metal":
                self.metal_val.config(text="Checking...", fg=ORANGE)
            else:
                self.height_val.config(text="Measuring...", fg=ORANGE)
        elif kind == "metal":
            self._show_metal(ev[1])
        elif kind == "height":
            self._show_height(ev[1])
        elif kind == "waiting_sorter":
            self._set_ir("Waiting at sorter (PA4)", ORANGE)
            self.pred_status.config(text="Status: waiting for object at sorting IR")
        elif kind == "result":
            self._set_ir("Sorted", BLUE, idle_after_ms=3000)
            if ev[1] == 1:
                self._paint_pred("\u2714 Accepted", GREEN, GREEN_BG, "Class: Valid Fastener",
                                 "Status: STM32 decision received")
            else:
                self._paint_pred("\u2716 Rejected", RED, RED_BG, "Class: Invalid object",
                                 "Status: STM32 decision received")
        elif kind == "servo":
            self.pred_status.config(text=f"Servo: {ev[1]}")
        elif kind == "live_rgb":
            self._show_rgb(ev[1], ev[2], ev[3])
        elif kind == "live_count":
            if ev[1] > 0:
                self.id_val.config(text=str(ev[1]))
        elif kind == "live_metal":
            if not self.inspecting:
                self._show_metal(ev[1])
        elif kind == "live_height":
            if not self.inspecting:
                self._show_height(ev[1])
        elif kind == "metal_count":
            pass
        elif kind == "invalid":
            self.inspecting = False
            self.invalid_count += 1
            self._update_skipped()
            self._log("RECORD IGNORED: " + ev[1], "warn")
            self.set_status("Warning: incomplete/invalid record ignored (see Bluetooth Log).", ORANGE)
        elif kind == "record":
            self.inspecting = False
            self._handle_record(ev[1])

    # -------------------------------------------------------------- records
    def _handle_record(self, rec: SensorRecord) -> None:
        key = rec.key()
        if key in self.recent_keys:
            self.duplicate_count += 1
            self._update_skipped()
            return
        self.recent_keys.append(key)
        rec.timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        # Cards
        self.id_val.config(text=str(rec.object_id))
        self._show_metal(rec.metal)
        self._show_height(rec.height)
        self._show_rgb(rec.red, rec.green, rec.blue)

        # ML prediction from the trained model (runs in the GUI)
        ml_pred = predict_record(self.model_result, rec) if self.model_result else None
        if ml_pred is not None and rec.label is not None:
            self.stats["compared"] += 1
            if ml_pred == rec.label:
                self.stats["agree"] += 1
            c = self.stats["compared"]
            self.ml_agree.config(text=f"Live agreement with STM32: {self.stats['agree']}/{c} "
                                      f"({100.0 * self.stats['agree'] / c:.0f} %)")
        if rec.label is not None:
            ml_txt = ("ML: " + ("Accepted" if ml_pred == 1 else "Rejected")) if ml_pred is not None \
                else "ML: no trained model"
            reason = f"  |  {rec.reason}" if rec.reason else ""
            if rec.label == 1:
                self._paint_pred("\u2714 Accepted", GREEN, GREEN_BG, "Class: Valid Fastener",
                                 ml_txt + reason)
            else:
                self._paint_pred("\u2716 Rejected", RED, RED_BG, "Class: Invalid object",
                                 ml_txt + reason)
        elif ml_pred is not None:
            self._paint_pred("Accepted" if ml_pred else "Rejected", GREEN if ml_pred else RED,
                             GREEN_BG if ml_pred else RED_BG, "Class: from ML model",
                             "Status: no STM32 decision in this record")

        self._update_stats(rec)
        self.graphs.add(rec)
        self._add_history_row(rec, ml_pred)
        if self.collecting:
            self._save_record(rec)

    def _update_stats(self, rec: SensorRecord) -> None:
        s = self.stats
        s["total"] += 1
        if rec.label == 1:
            s["accepted"] += 1
        elif rec.label == 0:
            s["rejected"] += 1
        done = s["accepted"] + s["rejected"]
        a = 100.0 * s["accepted"] / done if done else 0.0
        r = 100.0 * s["rejected"] / done if done else 0.0
        self.st_acc.config(text=str(s["accepted"]))
        self.st_rej.config(text=str(s["rejected"]))
        self.st_total.config(text=str(s["total"]))
        self.st_pct.config(text=f"Accepted: {a:.1f} %    Rejected: {r:.1f} %")
        self.pie.update(s["accepted"], s["rejected"])

    def _add_history_row(self, rec: SensorRecord, ml_pred: Optional[int]) -> None:
        if rec.label == 1:
            tag, result = "accepted", "Accepted"
        elif rec.label == 0:
            tag, result = "rejected", "Rejected"
        else:
            tag, result = "unknown", "N/A"

        def show(v):
            return "--" if v is None else v

        self.tree.insert("", 0, tags=(tag,), values=(
            rec.object_id, rec.timestamp, rec.metal,
            "INVALID" if rec.height is None else f"{rec.height:.2f}",
            show(rec.red), show(rec.green), show(rec.blue),
            "--" if ml_pred is None else ml_pred, result, rec.reason or "--"))
        children = self.tree.get_children()
        if len(children) > HISTORY_LIMIT:
            self.tree.delete(children[-1])
        self._update_total_label()

    def _update_total_label(self) -> None:
        self.total_label.config(
            text=f"Total objects: {self.stats['total']}   |   In table: {len(self.tree.get_children())}")

    def clear_table(self) -> None:
        self.tree.delete(*self.tree.get_children())
        self._update_total_label()
        self.set_status("Table cleared (the CSV dataset was not modified).")

    def _update_skipped(self) -> None:
        self.col_skipped.config(text=f"Invalid: {self.invalid_count}   Duplicates: {self.duplicate_count}")

    def _watchdog(self) -> None:
        """Once per second: show Rx counters and warn if a 'connected' port is silent."""
        try:
            if self.link.connected:
                self.rx_label.config(text=f"Received: {self.link.bytes_received} bytes, "
                                          f"{self.lines_received} lines")
                ref = self.last_line_at or self.connected_at
                silent = (datetime.now() - ref).total_seconds() if ref else 0
                if silent > 6 and not self.no_data_warned:
                    self.no_data_warned = True
                    nbytes = self.link.bytes_received
                    if nbytes == 0:
                        msg = ("Port is open but NO bytes received in 6 s. Usually: wrong COM port "
                               "(use the HC-05 'Outgoing' port), HC-05 not linked (LED still blinking "
                               "fast), STM32 PA2 not wired to HC-05 RXD, or the STM32 is not running.")
                    else:
                        msg = (f"{nbytes} bytes received but no complete line in 6 s. "
                               f"Check the baud rate and that the firmware sends \\r\\n.")
                    self._log(msg, "warn")
                    self.set_status(msg, RED)
                elif silent <= 6:
                    self.no_data_warned = False
        finally:
            self.root.after(1000, self._watchdog)

    def _refresh_graphs(self) -> None:
        try:
            self.graphs.refresh()
        finally:
            self.root.after(250, self._refresh_graphs)

    # ----------------------------------------------------------- collection
    def browse_dataset(self) -> None:
        path = filedialog.asksaveasfilename(
            title="Choose dataset CSV (existing files are appended to)", initialdir=APP_DIR,
            initialfile=os.path.basename(self._resolve_dataset_path()), defaultextension=".csv",
            confirmoverwrite=False, filetypes=[("CSV files", "*.csv"), ("All files", "*.*")])
        if path:
            self.dataset_var.set(path)

    def start_collection(self) -> None:
        path = self._resolve_dataset_path()
        try:
            self.dataset = DatasetManager(path)
        except DatasetError as exc:
            messagebox.showerror("Dataset error", str(exc))
            return
        self.collecting = True
        self.btn_start.config(state="disabled")
        self.btn_stop.config(state="normal")
        self.dataset_entry.config(state="disabled")
        self.btn_browse.config(state="disabled")
        self.col_status.config(text="Status: Collecting data...", fg=GREEN)
        self._update_collection_counts()
        msg = f"Collecting to {path}"
        if not self.link.connected:
            msg += "  (Bluetooth not connected yet - records are saved once data arrives)"
        self.set_status(msg, GREEN)

    def stop_collection(self) -> None:
        self.collecting = False
        self.btn_start.config(state="normal")
        self.btn_stop.config(state="disabled")
        self.dataset_entry.config(state="normal")
        self.btn_browse.config(state="normal")
        self.col_status.config(text="Status: Stopped", fg=GRAY)
        self.set_status("Collection stopped. Bluetooth connection is unchanged.")

    def _save_record(self, rec: SensorRecord) -> None:
        try:
            self.dataset.append(rec)
        except DatasetError as exc:
            self.stop_collection()
            self.root.after(10, lambda: messagebox.showerror("Cannot save record - collection stopped",
                                                             str(exc)))
            return
        self._update_collection_counts()

    def _update_collection_counts(self) -> None:
        if self.dataset is not None:
            self.col_count.config(text=f"Collected this session: {self.dataset.session_rows}")
            self.col_file_count.config(text=f"Records in file: {self.dataset.total_rows}")

    def export_csv(self) -> None:
        src = self.dataset.path if self.dataset is not None else self._resolve_dataset_path()
        if not os.path.isfile(src):
            messagebox.showwarning("Export CSV", f"No dataset file exists yet:\n{src}")
            return
        dest = filedialog.asksaveasfilename(title="Export dataset copy", defaultextension=".csv",
                                            initialfile="fastener_dataset_export.csv",
                                            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")])
        if not dest:
            return
        if os.path.abspath(dest) == os.path.abspath(src):
            messagebox.showinfo("Export CSV", "That is the dataset file itself; nothing to copy.")
            return
        try:
            (self.dataset or DatasetManager(src)).export_copy(dest)
        except DatasetError as exc:
            messagebox.showerror("Export failed", str(exc))
            return
        messagebox.showinfo("Export CSV", f"Dataset copied to:\n{dest}")

    # ------------------------------------------------------------------- ML
    def train_model(self) -> None:
        if self.training:
            return
        path = self.dataset.path if self.dataset is not None else self._resolve_dataset_path()
        if not os.path.isfile(path):
            messagebox.showwarning("Train Model", f"Dataset not found:\n{path}\n\nCollect data first.")
            return
        self.training = True
        self.btn_train.config(state="disabled")
        self.ml_status.config(text="Status: Training...", fg=ORANGE)
        threading.Thread(target=self._train_worker, args=(path,), daemon=True).start()

    def _train_worker(self, path: str) -> None:
        try:
            df = DatasetManager.read_dataframe(path)
            self.events.put(("train_done", train_from_dataframe(df, path)))
        except (DatasetError, MLError) as exc:
            self.events.put(("train_failed", str(exc)))
        except Exception as exc:
            self.events.put(("train_failed", f"Unexpected error: {exc}"))

    def _on_train_done(self, result: TrainingResult) -> None:
        self.training = False
        self.btn_train.config(state="normal")
        self.model_result = result
        self.stats["agree"] = self.stats["compared"] = 0
        self.ml_agree.config(text="")
        save_result(result, MODEL_FILE)
        self._show_model_summary(result)
        self.set_status("Model trained.", GREEN)
        self.view_model_details()

    def _on_train_failed(self, message: str) -> None:
        self.training = False
        self.btn_train.config(state="normal")
        self.ml_status.config(text="Status: Training failed", fg=RED)
        self.set_status("Model training failed.", RED)
        messagebox.showerror("Train Model", message)

    def _show_model_summary(self, r: TrainingResult, loaded: bool = False) -> None:
        self.ml_status.config(text=f"Status: {'Loaded saved model' if loaded else 'Trained'} "
                                   f"({r.n_total} records, {r.trained_at})", fg=GREEN)
        if r.reliable:
            self.ml_acc.config(text=f"Accuracy: {r.accuracy * 100:.1f} %", fg=GREEN)
        else:
            self.ml_acc.config(text=f"Accuracy: {r.accuracy * 100:.1f} %\n(NOT reliable - small dataset)",
                               fg=ORANGE)

    def view_model_details(self) -> None:
        if self.model_result is None:
            messagebox.showinfo("Model details", "No model yet. Collect data and press 'Train Model'.")
            return
        try:
            text = details_text(self.model_result)
        except Exception as exc:
            messagebox.showerror("Model details", f"Cannot build model details: {exc}")
            return
        win = tk.Toplevel(self.root)
        win.title("Model Details - Decision Tree")
        win.geometry("760x600")
        frame = tk.Frame(win)
        frame.pack(fill="both", expand=True)
        box = tk.Text(frame, wrap="none", font=("Consolas", 10))
        ys = ttk.Scrollbar(frame, orient="vertical", command=box.yview)
        xs = ttk.Scrollbar(frame, orient="horizontal", command=box.xview)
        box.configure(yscrollcommand=ys.set, xscrollcommand=xs.set)
        ys.pack(side="right", fill="y")
        xs.pack(side="bottom", fill="x")
        box.pack(side="left", fill="both", expand=True)
        box.insert("1.0", text)
        box.config(state="disabled")

    def export_c(self) -> None:
        if self.model_result is None:
            messagebox.showinfo("Export to C", "Train a model first.")
            return
        directory = filedialog.askdirectory(title="Choose folder for fastener_model.h / .c",
                                            initialdir=APP_DIR)
        if not directory:
            return
        try:
            paths = export_c_files(self.model_result, directory)
        except (MLError, OSError) as exc:
            messagebox.showerror("Export to C", str(exc))
            return
        note = "" if self.model_result.reliable else (
            "\n\nWARNING: trained on a small dataset. Collect more data before using it on the sorter.")
        messagebox.showinfo("Export to C", "Created:\n" + "\n".join(paths) + note)

    # ----------------------------------------------------------------- close
    def on_close(self) -> None:
        self.collecting = False
        try:
            self.link.disconnect()
        except Exception:
            pass
        try:
            self.camera.stop()
        except Exception:
            pass
        self.root.destroy()


# =============================================================================
#  ENTRY POINT
# =============================================================================
def main() -> int:
    missing = []
    if serial is None:
        missing.append("pyserial")
    if Figure is None:
        missing.append("matplotlib")
    if missing:
        root = tk.Tk()
        root.withdraw()
        messagebox.showerror("Missing package",
                             "Required package(s) missing: " + ", ".join(missing) +
                             "\n\nInstall with:\n    pip install pyserial pandas matplotlib "
                             "scikit-learn joblib Pillow")
        return 1
    root = tk.Tk()
    FastenerApp(root)
    root.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
