"""
muse_read.py — Stage 3: get the Muse stream into Python.

Prereqs:
    1. Muse is connected in Petal Metrics and streaming over LSL (you've done this).
    2. Install the one dependency:  pip install pylsl
       (If you use a virtual environment, activate it first.)

Run:
    python muse_read.py

You should see a line printing several times a second with the four EEG
channel values. Move your jaw / blink hard and watch the numbers jump —
that jump is the signal we'll turn into a control command in the next stage.

Press Ctrl+C to stop.
"""

from pylsl import resolve_byprop, StreamInlet

# We look for the stream by TYPE ("EEG") instead of by exact name, so it
# connects no matter what "Stream Name Prefix" is set in Petal.

# The four EEG sensors on the Muse 2, in the order Petal sends them.
# (There may be a 5th "aux" channel; we ignore it.)
CHANNELS = ["TP9", "AF7", "AF8", "TP10"]


def main():
    print("Looking for the Petal EEG stream...")
    # Find the EEG stream by type. Timeout after 10s if it isn't found.
    streams = resolve_byprop("type", "EEG", timeout=10)

    if not streams:
        print("No EEG stream found.")
        print("  - Is Petal Metrics open with LSL Streaming toggled ON (green)?")
        print("  - Is the STREAM button started (showing 'Stop')?")
        return

    inlet = StreamInlet(streams[0])
    print(f"Connected to stream: {streams[0].name()!r}")
    print("Streaming live EEG (Ctrl+C to stop):\n")

    # Header
    print("  ".join(f"{name:>8}" for name in CHANNELS))

    try:
        while True:
            # Grab one sample: a list of channel values + a timestamp.
            sample, timestamp = inlet.pull_sample()
            # sample[:4] = the four EEG channels, in microvolts.
            row = "  ".join(f"{value:8.1f}" for value in sample[:4])
            print(row)
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
