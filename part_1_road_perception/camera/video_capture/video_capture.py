"""IPVSEAS Part 1: play a road video frame by frame.

Run from anywhere:
    python part_1_road_perception/camera/video_capture/video_capture.py
    python video_capture.py --video path/to/other_video.mp4
"""

import argparse
import sys
from pathlib import Path

import cv2

# Repository root = three levels above this file:
# video_capture/ -> camera/ -> part_1_road_perception/ -> IPVSEAS/
REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_VIDEO = REPO_ROOT / "datasets" / "sample_videos" / "road_video.mp4"


def main() -> int:
    parser = argparse.ArgumentParser(description="IPVSEAS road video viewer")
    parser.add_argument("--video", type=Path, default=DEFAULT_VIDEO,
                        help=f"video file (default: {DEFAULT_VIDEO})")
    args = parser.parse_args()

    video_path = args.video.expanduser().resolve()
    if not video_path.is_file():
        print(f"Error: video not found: {video_path}")
        return 1

    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        print(f"Error: could not open video: {video_path}")
        return 1

    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    delay_ms = max(1, int(1000 / fps))
    print(f"IPVSEAS: loaded {video_path.name} ({fps:.1f} FPS). Press Q to quit.")

    try:
        while True:
            success, frame = cap.read()
            if not success:
                break
            cv2.imshow("IPVSEAS - Road Perception", frame)
            if cv2.waitKey(delay_ms) & 0xFF == ord("q"):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()

    print("Video processing completed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
