"""Point an owlet-mounted webcam at an object and keep it centred.

    pip install ./tools/python opencv-python ultralytics
    python tools/python/examples/track_object.py --target person --show

Pipeline per frame: grab (timestamped) → YOLO detect → pick the target →
PixelController (pixel offset → pan/tilt rates, with latency compensation from
the pose history) → Tracker.set_rates (VEL, fire-and-forget). If this script
stalls or dies, owlet's watchdog stops the head by itself.

Measure your camera's field of view for accurate tracking; the defaults suit
a typical 720p webcam.
"""

from __future__ import annotations

import argparse
import sys
import time

from owlet import Owlet, PixelController, Tracker


def pick_target(result, target: str):
    """Centre (x, y) of the largest box of class `target`, or None."""
    best, best_area = None, 0.0
    names = result.names
    for (x1, y1, x2, y2), cls in zip(result.boxes.xyxy.tolist(), result.boxes.cls.tolist()):
        if names[int(cls)] != target:
            continue
        area = (x2 - x1) * (y2 - y1)
        if area > best_area:
            best, best_area = ((x1 + x2) / 2, (y1 + y2) / 2, (x1, y1, x2, y2)), area
    return best


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("-p", "--port", help="owlet serial port (default: auto-detect)")
    ap.add_argument("--camera", type=int, default=0, help="OpenCV camera index")
    ap.add_argument("--model", default="yolov8n.pt", help="ultralytics model")
    ap.add_argument("--target", default="person", help="class name to follow")
    ap.add_argument("--fov-h", type=float, default=70.0, help="horizontal FOV, degrees")
    ap.add_argument("--fov-v", type=float, default=43.0, help="vertical FOV, degrees")
    ap.add_argument("--gain", type=float, default=3.0, help="rate per degree of error (1/s)")
    ap.add_argument("--max-rate", type=float, default=120.0, help="deg/s limit")
    ap.add_argument("--show", action="store_true", help="display the annotated video")
    args = ap.parse_args(argv)

    try:
        import cv2
        from ultralytics import YOLO
    except ImportError as e:
        print(f"this example needs opencv-python and ultralytics: {e}", file=sys.stderr)
        return 2

    cap = cv2.VideoCapture(args.camera)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # stale buffered frames would defeat latency compensation
    if not cap.isOpened():
        print(f"cannot open camera {args.camera}", file=sys.stderr)
        return 1
    model = YOLO(args.model)
    ctl = PixelController(args.fov_h, args.fov_v, gain=args.gain, max_rate=args.max_rate)

    with Owlet(args.port) as head:
        head.enable()
        try:
            run(cap, model, ctl, head, args)
        finally:
            head.disable()  # whatever happened, leave the head limp

    cap.release()
    if args.show:
        cv2.destroyAllWindows()
    return 0


def run(cap, model, ctl, head, args) -> None:
    import cv2

    with Tracker(head) as tr:
        try:
            while True:
                ok, frame = cap.read()
                t_capture = time.monotonic()
                if not ok:
                    break
                h, w = frame.shape[:2]
                found = pick_target(model(frame, verbose=False)[0], args.target)

                if found is None:
                    tr.set_rates(0, 0)  # hold; keep feeding the watchdog
                else:
                    cx, cy, box = found
                    pan, tilt = ctl.update(cx, cy, w, h,
                                           pose_at_capture=tr.pose_at(t_capture),
                                           pose_now=tr.pose_at(time.monotonic()))
                    tr.set_rates(pan, tilt)  # raises if the tracker has failed
                    if args.show:
                        x1, y1, x2, y2 = map(int, box)
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

                if args.show:
                    cv2.imshow("owlet", frame)
                    if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                        break
                if tr.last_error is not None:
                    print(f"owlet: {tr.last_error}", file=sys.stderr)
                    tr.last_error = None
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    sys.exit(main())
