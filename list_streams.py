"""
list_streams.py — Diagnostic: show every LSL stream on this computer.

Run this while Petal Metrics is open and streaming:
    Windows:  python list_streams.py
    Mac:      python3 list_streams.py

It lists the name, type, and channel count of every LSL stream it can find.
If Petal is streaming over LSL, you should see 'PetalStream_eeg' in the list.
If the list is empty, Petal isn't broadcasting over LSL yet (check the
protocol setting and that the stream is actually started).
"""

from pylsl import resolve_streams

print("Scanning for LSL streams (5 seconds)...\n")
streams = resolve_streams(wait_time=5.0)

if not streams:
    print("No LSL streams found at all.")
    print("That means Petal isn't broadcasting over LSL right now:")
    print("  - Make sure the protocol is set to LSL (not OSC).")
    print("  - Make sure you've actually STARTED the stream in Petal.")
else:
    print(f"Found {len(streams)} stream(s):\n")
    for info in streams:
        print(f"  name = {info.name()!r}")
        print(f"    type          = {info.type()!r}")
        print(f"    channel_count = {info.channel_count()}")
        print(f"    sample_rate   = {info.nominal_srate()}")
        print()
    print("If you see 'PetalStream_eeg' above, the stream is working —")
    print("re-run muse_read.py and it should connect.")
