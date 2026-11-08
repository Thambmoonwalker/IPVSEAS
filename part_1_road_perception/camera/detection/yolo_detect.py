"""IPVSEAS Part 1: YOLO detection of road users on a recorded run or a video.

Run-folder mode (writes <run>/camera_dets.csv for the C++ replay tool):
    python yolo_detect.py --run ../../../datasets/runs/lead_brake_01

Video mode (preview + optional CSV, useful with dashcam footage, e.g. Indian roads):
    python yolo_detect.py --video ../../../datasets/sample_videos/road_video.mp4 --show

Only classes that matter for collision prediction are kept and mapped to IPVSEAS names:
    person -> pedestrian, bicycle, car, motorcycle, bus, truck,
    cat/dog/horse/sheep/cow/elephant/bear/zebra/giraffe -> animal
"""

import argparse
import csv
import sys
import time
from pathlib import Path

import cv2

try:
    from ultralytics import YOLO
except ImportError:
    sys.exit("pip install ultralytics")

# COCO class name -> IPVSEAS class
CLASS_MAP = {
    "person": "pedestrian", "bicycle": "bicycle", "car": "car", "motorcycle": "motorcycle",
    "bus": "bus", "truck": "truck",
    "cat": "animal", "dog": "animal", "horse": "animal", "sheep": "animal", "cow": "animal",
    "elephant": "animal", "bear": "animal", "zebra": "animal", "giraffe": "animal",
}
COLORS = {"pedestrian": (0, 0, 255), "animal": (0, 165, 255), "bicycle": (255, 0, 255)}


def detect(model, frame_bgr, conf, imgsz):
    """Return [(x1, y1, x2, y2, cls, conf)] for the IPVSEAS classes."""
    res = model.predict(frame_bgr, conf=conf, imgsz=imgsz, verbose=False)[0]
    names = res.names
    out = []
    for box, c, p in zip(res.boxes.xyxy.tolist(), res.boxes.cls.tolist(), res.boxes.conf.tolist()):
        cls = CLASS_MAP.get(names[int(c)])
        if cls:
            out.append((*box, cls, float(p)))
    return out


def draw(frame, dets):
    for x1, y1, x2, y2, cls, p in dets:
        col = COLORS.get(cls, (0, 255, 0))
        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), col, 2)
        cv2.putText(frame, f"{cls} {p:.2f}", (int(x1), int(y1) - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)
    return frame


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--run", type=Path, help="run folder with camera/<frame>.jpg|png")
    src.add_argument("--video", type=Path)
    ap.add_argument("--model", default="yolo26s.pt", help="any Ultralytics detection model")
    ap.add_argument("--conf", type=float, default=0.30)
    ap.add_argument("--imgsz", type=int, default=960, help="larger helps small, distant objects")
    ap.add_argument("--show", action="store_true")
    ap.add_argument("--csv", type=Path, help="CSV output path (video mode)")
    args = ap.parse_args()

    model = YOLO(args.model)
    rows, t_infer, n = [], 0.0, 0

    if args.run:
        images = sorted(list((args.run / "camera").glob("*.png")) + list((args.run / "camera").glob("*.jpg")))
        if not images:
            sys.exit(f"no images in {args.run / 'camera'}")
        frames = ((int(p.stem), cv2.imread(str(p))) for p in images)
        out_csv = args.run / "camera_dets.csv"
    else:
        cap = cv2.VideoCapture(str(args.video))
        if not cap.isOpened():
            sys.exit(f"cannot open {args.video}")

        def gen():
            i = 0
            while True:
                ok, f = cap.read()
                if not ok:
                    return
                yield i, f
                i += 1
        frames = gen()
        out_csv = args.csv

    for idx, img in frames:
        t0 = time.perf_counter()
        dets = detect(model, img, args.conf, args.imgsz)
        t_infer += time.perf_counter() - t0
        n += 1
        rows += [(idx, *d) for d in dets]
        if args.show:
            cv2.imshow("IPVSEAS - YOLO", draw(img, dets))
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    cv2.destroyAllWindows()

    if out_csv:
        with open(out_csv, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["frame", "x1", "y1", "x2", "y2", "cls", "conf"])
            for r in rows:
                w.writerow([r[0], *(f"{v:.1f}" for v in r[1:5]), r[5], f"{r[6]:.3f}"])
        print(f"wrote {len(rows)} detections to {out_csv}")
    if n:
        print(f"{n} frames, mean inference {1000 * t_infer / n:.1f} ms/frame")


if __name__ == "__main__":
    main()
