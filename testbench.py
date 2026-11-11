"""
Testbench for pantilt.py. Moves the real head, so keep clear of it and be ready to
press Ctrl+C (the head is stopped on exit).

Tests, in order:
  1. Axis test      - right, left, up, down, 1 s each
  2. Rotation test  - full pan sweep (right then left), then full tilt sweep
                      (down, up, back to centre), each axis separately
  3. Zig-zag test   - diagonal sweeps across the pan/tilt plane, pan left->right
                      then right->left with tilt alternating each leg

Usage:
    python testbench.py                     # real head on COM6, address 57, 9600
    python testbench.py --port COM3 --address 1 --baud 4800
    python testbench.py --dry-run           # no hardware, instant, prints frames
    python testbench.py --skip rotation     # skip a test (axis, rotation, zigzag)
"""
import argparse
import sys

from pantilt import PanTilt, MAX_SPEED

SPEED = MAX_SPEED   # 0x3F, the highest standard Pelco-D speed. The head may ignore it.


def log(pt, msg):
    print(msg)


def test_axes(pt):
    print("\n== 1. Axis test ==")
    for name, pan, tilt in (("right", 1, 0), ("left", -1, 0), ("up", 0, 1), ("down", 0, -1)):
        pt.move_for(pan, tilt, 1.0, SPEED)
        log(pt, f"moved {name} 1 s")
        pt.pause(0.5)


def test_rotation(pt):
    c = pt.config
    print("\n== 2. Rotation test ==")
    print(f"Pan:  full sweep is ~{c.pan_sweep_s:.0f} s each way")
    pt.drive_to_limit("pan", +1, SPEED)
    log(pt, "pan -> right limit")
    pt.pause(1.0)
    pt.drive_to_limit("pan", -1, SPEED)
    log(pt, "pan -> left limit")
    pt.pause(1.0)

    print(f"Tilt: full sweep is ~{c.tilt_sweep_s:.0f} s each way")
    pt.drive_to_limit("tilt", -1, SPEED)
    log(pt, "tilt -> bottom limit")
    pt.pause(1.0)
    pt.drive_to_limit("tilt", +1, SPEED)
    log(pt, "tilt -> top limit")
    pt.pause(1.0)
    pt.move_for(0, -1, c.tilt_sweep_s / 2, SPEED)   # timed move back to roughly the middle
    log(pt, "tilt -> roughly centre (timed)")
    pt.pause(1.0)


def test_zigzag(pt):
    print("\n== 3. Zig-zag test ==")
    print("Pass 1: pan left->right, tilt up/down alternating")
    pt.zigzag(pan_dir=+1, legs=4, first_tilt_dir=+1, speed=SPEED)
    log(pt, "pass 1 done")
    pt.pause(1.0)
    print("Pass 2: pan right->left, tilt down/up alternating")
    pt.zigzag(pan_dir=-1, legs=4, first_tilt_dir=-1, speed=SPEED)
    log(pt, "pass 2 done")


TESTS = {"axis": test_axes, "rotation": test_rotation, "zigzag": test_zigzag}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--port", default="COM6")
    ap.add_argument("--address", type=int, default=57, help="head address (DIP 1-6)")
    ap.add_argument("--baud", type=int, default=9600, help="1200/2400/4800/9600 (DIP 7-8)")
    ap.add_argument("--dry-run", action="store_true", help="no serial port, no waiting")
    ap.add_argument("--skip", action="append", default=[], choices=TESTS, help="skip a test")
    ap.add_argument("--verbose", action="store_true", help="print every frame sent")
    ap.add_argument("--yes", action="store_true", help="don't wait for Enter before starting")
    args = ap.parse_args()

    if args.dry_run:
        pt = PanTilt.dry_run(address=args.address, verbose=args.verbose)
    else:
        pt = PanTilt(args.port, args.address, args.baud, verbose=args.verbose)

    if not args.dry_run and not args.yes:
        input(f"About to move the head on {args.port} (address {args.address}, {args.baud} baud).\n"
              "Clear the area. Press Enter to start, Ctrl+C to abort... ")

    dry_port = pt._ser  # kept so the dry-run frame count survives close()
    try:
        with pt:
            for name, fn in TESTS.items():
                if name not in args.skip:
                    fn(pt)
        print("\nAll tests finished.")
    except KeyboardInterrupt:
        print("\nAborted - head stopped.")
        return 1
    if args.dry_run:
        print(f"(dry run: {len(dry_port.writes)} frames would have been sent)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
