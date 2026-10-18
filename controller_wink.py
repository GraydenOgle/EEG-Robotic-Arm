"""
controller_wink.py — WINK version: tilt joystick + clench grab + WINK wrist.

Control scheme:
    - Nod forward / back  (pitch) -> REACH   (tilt & hold to move, level = stop)
    - Tilt ear-to-shoulder (roll)  -> LATERAL (tilt & hold to move, level = stop)
    - Jaw clench                   -> GRAB    (toggle gripper, works moving or still)
    - LEFT wink                    -> WRIST one way
    - RIGHT wink                   -> WRIST the other way
    - Normal (both-eye) blinks     -> IGNORED

This is the alternate control scheme: the wrist is rotated by winking left or
right instead of by a sharp head flick. Both forehead sensors (AF7, AF8) need
good skin contact for winks to be told apart. Winks are read only while the head
is still (motion smears the forehead signal).

Tilt is velocity mode ("joystick"): head angle sets a movement RATE, and the
arm holds position when your head is level. Wrist direction is set by how long
you hold your eyes squeezed shut.

Reads: EEG (clench + eye-hold) + Accelerometer (tilt + motion gate).
EEG gestures pause while the head is moving; settle, then act.

Sends targets to the Arduino over serial (see SERIAL_PORT).

Prereqs:  pip install pylsl pyserial | Petal open, LSL Streaming ON, Muse on head.
Run:      python controller.py   (Mac: python3)

Knobs:
    PITCH_INVERT / ROLL_INVERT  — flip a tilt axis.
    TILT_SPEED / TILT_DEADZONE  — joystick move speed / how far to tilt before moving.
    CLENCH_MULT                 — clench sensitivity (lower = more sensitive).
    BLINK_MULT                  — eye-hold sensitivity.
    WINK_RATIO                  — how lopsided a wink must be (AF7 vs AF8) to count.
    WRIST_CW_STEP               — degrees per wrist step (negate to swap dirs).

Ctrl+C to stop.
"""

import time
from collections import deque
from pylsl import resolve_byprop, StreamInlet

try:
    import serial   # pip install pyserial
except ImportError:
    serial = None

# ---- EEG channel indices (Petal order: TP9, AF7, AF8, TP10) ----
EAR_CHANNELS = [0, 3]   # TP9, TP10 — clench
AF7 = 1                 # left forehead  — blink
AF8 = 2                 # right forehead — blink

# ---- Clench tunables ----
EAR_WINDOW = 25
CLENCH_MULT = 4.0
MIN_CLENCH_THRESHOLD = 50  # never let the bar fall below this (idle noise spikes to ~27)
CLENCH_COOLDOWN = 1.0
CLENCH_SUSTAIN = 0.20    # clench must stay above the bar this long (rejects brief motion blips)
DEBUG_CLENCH = True      # print live clench energy vs threshold each heartbeat (set False to quiet)

# ---- Wink tunables (wrist control) ----
# A LEFT wink shows strongest on the left-forehead sensor (AF7), a RIGHT wink on
# the right-forehead sensor (AF8). A full both-eyes blink hits them about equally
# and is ignored. Both forehead sensors must have good skin contact.
BLINK_ALPHA = 0.002          # forehead baseline drift rate
BLINK_MULT = 5.0             # wink threshold = forehead noise * this
BLINK_RELEASE_FRACTION = 0.4
WINK_RATIO = 1.5             # one side must beat the other by this to count as a wink
WINK_MAX_EVENT = 0.7         # safety cap on a wink's length (s)
WINK_COOLDOWN = 0.5          # min seconds between winks
WRIST_STEP = 15
WRIST_CW_STEP = +WRIST_STEP  # negate to swap which wink turns which way

# ---- Tilt tunables (JOYSTICK / velocity mode) ----
# Tilt your head and HOLD to keep the arm moving that way; return to level to
# stop and hold. The deadzone stops a roughly-level head from drifting.
TILT_SMOOTHING = 0.25
PITCH_INVERT = False
ROLL_INVERT = False
TILT_DEADZONE = 0.20         # head must tilt past this fraction before the arm moves
TILT_SPEED = 70              # degrees/sec of arm motion at full head tilt
ACCEL_DT = 1.0 / 52.0        # accelerometer period (52 Hz), for integrating motion

# ---- Wrist flick (sharp side-tilt) ----
# A SHARP snap of the side-tilt flicks the wrist; a slow tilt still drives the
# base. Speed is what separates them.
FLICK_RATE = 3.0             # tilt speed (norm/sec) above which it's a flick, not a base move
FLICK_COOLDOWN = 0.5         # min seconds between flicks; base motion is paused this long after one
FLICK_SWAP = False           # set True if a left flick turns the wrist the wrong way

# ---- Motion gate ----
MOTION_GATE_FRACTION = 0.60
MOTION_ALPHA = 0.30
MOTION_RELEASE = 0.25

# ---- Arm behavior ----
GRIP_OPEN = 40
GRIP_CLOSED = 110

# ---- Serial link to the Arduino ----
SERIAL_PORT = "COM7"     # your Arduino's port (Windows COM#, or /dev/... on Mac)
SERIAL_BAUD = 115200
SEND_INTERVAL = 0.03     # send targets ~33x/sec (plenty smooth, won't flood)

# ---- Calibration durations ----
EEG_CAL_SECONDS = 5
LEVEL_CAL_SECONDS = 3
NOD_CAL_SECONDS = 5
ROLL_CAL_SECONDS = 5


def mean(values):
    return sum(values) / len(values) if values else 0.0


def median(values):
    if not values:
        return 0.0
    s = sorted(values)
    return s[len(s) // 2]


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def ear_energy_min(window):
    n = len(window) - 1
    if n <= 0:
        return 0.0
    energies = []
    for ch in EAR_CHANNELS:
        s = sum(abs(window[k][ch] - window[k - 1][ch]) for k in range(1, len(window)))
        energies.append(s / n)
    return min(energies)


def ear_energy_mean(window):
    """Average EMG energy of the two ear sensors -- the clench signal. Easier to
    trigger than the strict 'both ears' minimum, and blinks don't reach the ears,
    so it stays clench-specific."""
    n = len(window) - 1
    if n <= 0:
        return 0.0
    total = 0.0
    for ch in EAR_CHANNELS:
        total += sum(abs(window[k][ch] - window[k - 1][ch]) for k in range(1, len(window)))
    return total / (n * len(EAR_CHANNELS))


def face_energy(window):
    """Strongest high-frequency (EMG) energy across ALL four EEG sensors.
    Any big facial signal -- clench, grimace, hard squeeze -- spikes this on at
    least one channel. Blinks are low-frequency, so they stay low and don't
    trip it. Used to toggle the gripper on 'any major face signal'."""
    n = len(window) - 1
    if n <= 0:
        return 0.0
    best = 0.0
    for ch in range(4):
        s = sum(abs(window[k][ch] - window[k - 1][ch]) for k in range(1, len(window)))
        e = s / n
        if e > best:
            best = e
    return best


def motion_levels(rows):
    return [sum(abs(rows[k][i] - rows[k - 1][i]) for i in range(3))
            for k in range(1, len(rows))]


def axis_ranges(rows):
    mins = [min(r[i] for r in rows) for i in range(3)]
    maxs = [max(r[i] for r in rows) for i in range(3)]
    return [maxs[i] - mins[i] for i in range(3)]


def collect(inlet, seconds):
    rows = []
    t_end = time.time() + seconds
    while time.time() < t_end:
        s, _ = inlet.pull_sample(timeout=0.2)
        if s is not None:
            rows.append(s)
    return rows


class ClenchDetector:
    def __init__(self, threshold):
        self.threshold = threshold
        self.last_fire = 0.0
        self.above_since = 0.0     # when energy first rose above threshold

    def feed(self, ear_energy, now):
        if ear_energy > self.threshold:
            if self.above_since == 0.0:
                self.above_since = now
            # must stay above the bar for CLENCH_SUSTAIN (rejects brief noise spikes)
            elif (now - self.above_since) >= CLENCH_SUSTAIN and (now - self.last_fire) > CLENCH_COOLDOWN:
                self.last_fire = now
                self.above_since = 0.0
                return True
        else:
            self.above_since = 0.0
        return False


class WinkDetector:
    """Tells a LEFT wink (strongest on AF7) from a RIGHT wink (strongest on AF8).
    A full both-eyes blink hits both about equally and is ignored. Returns
    'LEFT' or 'RIGHT' on a clear one-sided wink, else None."""

    def __init__(self, threshold, base7, base8):
        self.threshold = threshold
        self.base7, self.base8 = base7, base8
        self.in_event = False
        self.peak7 = self.peak8 = 0.0
        self.event_start = 0.0
        self.last_fire = 0.0

    def feed(self, sample, now):
        if not self.in_event:
            self.base7 += BLINK_ALPHA * (sample[AF7] - self.base7)
            self.base8 += BLINK_ALPHA * (sample[AF8] - self.base8)
        dev7 = abs(sample[AF7] - self.base7)
        dev8 = abs(sample[AF8] - self.base8)

        if not self.in_event:
            if (dev7 > self.threshold or dev8 > self.threshold) and (now - self.last_fire) > WINK_COOLDOWN:
                self.in_event = True
                self.event_start = now
                self.peak7, self.peak8 = dev7, dev8
            return None

        self.peak7 = max(self.peak7, dev7)
        self.peak8 = max(self.peak8, dev8)
        ended = (dev7 < self.threshold * BLINK_RELEASE_FRACTION and
                 dev8 < self.threshold * BLINK_RELEASE_FRACTION)
        if ended or (now - self.event_start) > WINK_MAX_EVENT:
            self.in_event = False
            self.last_fire = now
            if self.peak7 > self.peak8 * WINK_RATIO:
                return "LEFT"
            if self.peak8 > self.peak7 * WINK_RATIO:
                return "RIGHT"
            return "BLINK"     # both eyes ~equal -> ignored
        return None

    def reset(self):
        self.in_event = False


class AxisTracker:
    """JOYSTICK / velocity mode: head tilt sets a movement RATE, not a position.
    Past the deadzone, the arm angle integrates in the tilt direction; a level
    head (inside the deadzone) holds the current angle.

    If flick=True (used on the side-tilt axis), a SHARP snap is detected as a
    wrist flick instead of a base move, and base motion is paused briefly so the
    flick doesn't drag the base along."""

    def __init__(self, axis, baseline, half_range, invert, flick=False):
        self.axis = axis
        self.baseline = baseline
        self.half = max(half_range, 1e-6)
        self.invert = invert
        self.smoothed = baseline
        self.position = 90.0        # start centered
        self.flick_enabled = flick
        self.prev_norm = 0.0
        self.flick_until = 0.0      # base motion suppressed until this time
        self.pending_flick = None   # "LEFT" / "RIGHT" waiting to be consumed

    def feed(self, sample, now):
        self.smoothed += TILT_SMOOTHING * (sample[self.axis] - self.smoothed)
        norm = (self.smoothed - self.baseline) / self.half
        if self.invert:
            norm = -norm
        norm = clamp(norm, -1.0, 1.0)

        # --- flick detection (side-tilt axis only) ---
        if self.flick_enabled:
            slope = (norm - self.prev_norm) / ACCEL_DT     # how fast the tilt is changing
            if abs(slope) > FLICK_RATE and now >= self.flick_until:
                self.pending_flick = "RIGHT" if slope > 0 else "LEFT"
                self.flick_until = now + FLICK_COOLDOWN     # also pauses base motion
        self.prev_norm = norm

        # --- base velocity (skip while a flick is in progress) ---
        if now < self.flick_until:
            return                                          # sharp movement -> don't move base

        if abs(norm) < TILT_DEADZONE:
            rate = 0.0
        else:
            sign = 1.0 if norm > 0 else -1.0
            rate = sign * (abs(norm) - TILT_DEADZONE) / (1.0 - TILT_DEADZONE)
        self.position += rate * TILT_SPEED * ACCEL_DT
        self.position = clamp(self.position, 0.0, 180.0)

    def take_flick(self):
        f = self.pending_flick
        self.pending_flick = None
        return f

    def angle(self):
        return int(round(self.position))


def get_inlet(stream_type):
    streams = resolve_byprop("type", stream_type, timeout=10)
    if not streams:
        return None
    print(f"  connected: {streams[0].name()!r} ({stream_type})")
    return StreamInlet(streams[0])


def main():
    print("Connecting to streams...")
    eeg = get_inlet("EEG")
    accel = get_inlet("Accelerometer")
    if eeg is None or accel is None:
        print("Missing a stream. Is Petal open with LSL Streaming ON?")
        return

    print("\nChecking data flow...")
    if not collect(eeg, 2) or not collect(accel, 2):
        print("A stream is silent. Toggle LSL Streaming off/on in Petal and retry.")
        return
    print("Both streams live.\n")

    # --- CAL 1: EEG relax -> clench + blink thresholds ---
    print(f"CALIBRATION 1/4 — relax your face, do NOT clench, for {EEG_CAL_SECONDS}s...")
    eeg_rows = collect(eeg, EEG_CAL_SECONDS)

    face_vals, buf = [], deque(maxlen=EAR_WINDOW)
    for s in eeg_rows:
        buf.append(s)
        if len(buf) == EAR_WINDOW:
            face_vals.append(ear_energy_mean(list(buf)))
    face_vals.sort()
    # Bar sits above the WORST relaxed noise, not the median: the signal is
    # spiky, so use the 95th percentile of relaxed energy as the reference.
    p95 = face_vals[min(len(face_vals) - 1, int(len(face_vals) * 0.95))] if face_vals else 1.0
    clench_threshold = max(p95 * CLENCH_MULT, MIN_CLENCH_THRESHOLD)

    b7, b8 = eeg_rows[0][AF7], eeg_rows[0][AF8]
    wink_devs = []
    for s in eeg_rows:
        b7 += BLINK_ALPHA * (s[AF7] - b7)
        b8 += BLINK_ALPHA * (s[AF8] - b8)
        wink_devs.append(max(abs(s[AF7] - b7), abs(s[AF8] - b8)))
    wink_threshold = median(wink_devs) * BLINK_MULT
    print(f"  clench bar {clench_threshold:.0f} | wink bar {wink_threshold:.0f}")

    # --- CAL 2: level hold ---
    print(f"CALIBRATION 2/4 — hold your head LEVEL and still for {LEVEL_CAL_SECONDS}s...")
    level_rows = collect(accel, LEVEL_CAL_SECONDS)
    baseline = [mean([r[i] for r in level_rows]) for i in range(3)]
    still_motion = mean(motion_levels(level_rows))

    # --- CAL 3: nod -> pitch axis ---
    print(f"CALIBRATION 3/4 — NOD forward and back a few times for {NOD_CAL_SECONDS}s...")
    nod_rows = collect(accel, NOD_CAL_SECONDS)
    nod_ranges = axis_ranges(nod_rows)
    pitch_axis = nod_ranges.index(max(nod_ranges))
    pitch_half = nod_ranges[pitch_axis] / 2.0
    nod_motion = mean(motion_levels(nod_rows))

    # --- CAL 4: side tilt -> roll axis ---
    print(f"CALIBRATION 4/4 — TILT your head ear-to-shoulder a few times for {ROLL_CAL_SECONDS}s...")
    roll_rows = collect(accel, ROLL_CAL_SECONDS)
    roll_ranges = axis_ranges(roll_rows)
    order = sorted(range(3), key=lambda i: roll_ranges[i], reverse=True)
    roll_axis = order[0] if order[0] != pitch_axis else order[1]
    roll_half = roll_ranges[roll_axis] / 2.0

    motion_gate = still_motion + MOTION_GATE_FRACTION * (nod_motion - still_motion)
    print(f"  pitch axis = {pitch_axis} | roll axis = {roll_axis} | motion gate = {motion_gate:.3f}")

    clench = ClenchDetector(clench_threshold)
    wink = WinkDetector(wink_threshold, b7, b8)
    pitch = AxisTracker(pitch_axis, baseline[pitch_axis], pitch_half, PITCH_INVERT)
    roll = AxisTracker(roll_axis, baseline[roll_axis], roll_half, ROLL_INVERT)  # no flick: wrist is winks

    grabbing = False
    lateral = 90
    reach = 90
    wrist = 90

    def show(tag, moving):
        g = "CLOSED" if grabbing else "OPEN"
        grip = GRIP_CLOSED if grabbing else GRIP_OPEN
        state = "moving" if moving else "still "
        print(f"[{state}] GRAB {g:6s} | LAT {lateral:3d} | REACH {reach:3d} | WRIST {wrist:3d}   {tag}")

    print("\nReady. Nod=reach, head-tilt=left/right, clench=grab,")
    print("long blink=wrist CW, double blink=wrist CCW. (head still for EEG gestures)")
    print("(Ctrl+C to stop)\n")
    show("(start)", False)

    # --- open the serial link to the Arduino ---
    ser = None
    if serial is None:
        print("pyserial not installed (pip install pyserial). Running print-only.")
    else:
        try:
            ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0)
            time.sleep(2.0)   # let the Arduino reset after the port opens
            print(f"Serial open on {SERIAL_PORT} -> arm connected.")
        except Exception as e:
            print(f"Couldn't open {SERIAL_PORT} ({e}). Running print-only.")
            print("Is the Arduino plugged in, the port right, and Serial Monitor CLOSED?")
            ser = None

    ear_buf = deque(maxlen=EAR_WINDOW)
    peak_clench_e = 0.0
    prev_accel = None
    motion_ema = still_motion
    last_motion_time = 0.0
    last_heartbeat = time.time()
    last_send = time.time()

    while True:
        now = time.time()
        changed = False

        accel_chunk, _ = accel.pull_chunk(timeout=0.0, max_samples=64)
        for s in accel_chunk:
            if prev_accel is not None:
                inst = sum(abs(s[i] - prev_accel[i]) for i in range(3))
                motion_ema += MOTION_ALPHA * (inst - motion_ema)
            prev_accel = s
            pitch.feed(s, now)
            roll.feed(s, now)

        moving = motion_ema > motion_gate
        if moving:
            last_motion_time = now
        eeg_suppressed = moving or (now - last_motion_time) < MOTION_RELEASE

        eeg_chunk, _ = eeg.pull_chunk(timeout=0.0, max_samples=256)
        for s in eeg_chunk:
            ear_buf.append(s)
            if len(ear_buf) == EAR_WINDOW:
                clench_e = ear_energy_mean(list(ear_buf))
                peak_clench_e = max(peak_clench_e, clench_e)   # for the debug readout
                # clench toggles the grip whether the head is moving or still
                if clench.feed(clench_e, now):
                    grabbing = not grabbing
                    changed = True
                    show("<- CLENCH (toggle grab)", moving)

            # winks rotate the wrist (only when the head is still, to avoid
            # motion smearing the forehead sensors)
            if not eeg_suppressed:
                w = wink.feed(s, now)
                if w == "RIGHT":
                    wrist = clamp(wrist + WRIST_CW_STEP, 0, 180)
                    changed = True
                    show("<- RIGHT wink (wrist)", moving)
                elif w == "LEFT":
                    wrist = clamp(wrist - WRIST_CW_STEP, 0, 180)
                    changed = True
                    show("<- LEFT wink (wrist)", moving)
                # "BLINK" (both eyes) -> ignored

        new_reach = pitch.angle()
        new_lateral = roll.angle()
        if abs(new_reach - reach) >= 2:
            reach = new_reach
            changed = True
        if abs(new_lateral - lateral) >= 2:
            lateral = new_lateral
            changed = True

        if changed or (now - last_heartbeat) > 0.8:
            if not changed:
                show("", moving)
            if DEBUG_CLENCH:
                pct = (peak_clench_e / clench_threshold) if clench_threshold else 0.0
                gate = "PAUSED (head moving)" if eeg_suppressed else "listening"
                print(f"    clench peak {peak_clench_e:6.0f} / thresh {clench_threshold:6.0f}"
                      f"  ({pct:4.0%})  [{gate}]")
            peak_clench_e = 0.0
            last_heartbeat = now

        # --- send targets to the arm: lateral,reach,wrist,grab ---
        if ser is not None and (now - last_send) > SEND_INTERVAL:
            last_send = now
            grab = 1 if grabbing else 0
            try:
                ser.write(f"{lateral},{reach},{wrist},{grab}\n".encode())
            except Exception:
                pass   # a dropped write is fine; next one goes through

        time.sleep(0.005)


if __name__ == "__main__":
    main()
