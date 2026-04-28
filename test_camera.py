import cv2
import time
import os

PREFERRED_DEVICES = [
    "/dev/video0",
    "/dev/video1",
    "/dev/video2",
]

WIDTH = 640
HEIGHT = 480
FPS = 30


def try_open_camera(source):
    cap = cv2.VideoCapture(source, cv2.CAP_V4L2)

    if not cap.isOpened():
        cap.release()
        return None

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, FPS)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    time.sleep(1.0)

    for _ in range(5):
        cap.read()

    ok, frame = cap.read()
    if not ok or frame is None:
        cap.release()
        return None

    return cap


def open_best_camera():
    for dev in PREFERRED_DEVICES:
        if os.path.exists(dev):
            print(f"Trying {dev}")
            cap = try_open_camera(dev)
            if cap is not None:
                print(f"Opened camera successfully using {dev}")
                return cap, dev

    for idx in range(6):
        print(f"Trying index {idx}")
        cap = try_open_camera(idx)
        if cap is not None:
            print(f"Opened camera successfully using index {idx}")
            return cap, idx

    return None, None


def main():
    cap, camera_source = open_best_camera()

    if cap is None:
        raise RuntimeError(
            "Could not open any camera. Check v4l2-ctl --list-devices"
        )

    print(f"Using camera source: {camera_source}")
    print("Press q to quit.")

    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            print("Failed to read frame")
            time.sleep(0.1)
            continue

        h, w = frame.shape[:2]
        cv2.putText(
            frame,
            f"Camera source: {camera_source}",
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
        )
        cv2.putText(
            frame,
            f"Resolution: {w}x{h}",
            (20, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
        )

        cv2.imshow("USB Camera Test", frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
