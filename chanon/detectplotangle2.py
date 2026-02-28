import cv2
import math
import numpy as np
from ultralytics import YOLO

# ----------------------------
# Load model
# ----------------------------
model = YOLO("chanon/rack_segm.pt")
cap = cv2.VideoCapture(2)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

deadzone_angle = 3.0  # องศาที่ยอมรับได้


while cap.isOpened():
    success, frame = cap.read()
    if not success:
        break

    # หมุนภาพ (แก้ภาพตะแคง)
    frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)

    h, w, _ = frame.shape
    screen_cx = w // 2

    results = model.track(frame, persist=True, conf=0.5)

    command = "SEARCHING"
    angle_deg = 0.0
    annotated_frame = frame.copy()

    # -----------------------------------
    # ใช้ Polygon จาก segmentation mask
    # -----------------------------------
    if results[0].masks is not None and len(results[0].masks.xy) > 0:

        annotated_frame = results[0].plot()

        best_area = 0
        best_cx = None
        best_cy = None

        for polygon in results[0].masks.xy:

            # polygon = Nx2 (float)
            pts = np.array(polygon, dtype=np.float32)

            # คำนวณพื้นที่
            area = cv2.contourArea(pts.astype(np.int32))

            if area > best_area:
                M = cv2.moments(pts)

                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])

                    best_area = area
                    best_cx = cx
                    best_cy = cy

        if best_cx is not None:

            obj_cx = best_cx
            obj_cy = best_cy

            # ----------------------------
            # คำนวณมุมเบี่ยงเบน
            # ----------------------------
            dist_x = obj_cx - screen_cx
            dist_y = h - obj_cy

            if dist_y == 0:
                dist_y = 1

            angle_rad = math.atan2(dist_x, dist_y)
            angle_deg = math.degrees(angle_rad)

            if angle_deg > deadzone_angle:
                command = "TURN RIGHT"
            elif angle_deg < -deadzone_angle:
                command = "TURN LEFT"
            else:
                command = "FORWARD (STAY CENTER)"

            # ----------------------------
            # Visual
            # ----------------------------
            cv2.circle(annotated_frame, (obj_cx, obj_cy), 6, (255, 0, 0), -1)

            cv2.line(
                annotated_frame,
                (screen_cx, h),
                (obj_cx, obj_cy),
                (0, 0, 255),
                2
            )

            cv2.putText(
                annotated_frame,
                f"{angle_deg:.1f} deg",
                (obj_cx, obj_cy - 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2
            )

    # เส้นกึ่งกลางจอ
    cv2.line(
        annotated_frame,
        (screen_cx, 0),
        (screen_cx, h),
        (0, 255, 255),
        2
    )

    # แสดงสถานะ
    cv2.rectangle(annotated_frame, (0, 0), (300, 40), (0, 0, 0), -1)
    cv2.putText(
        annotated_frame,
        f"ACTION: {command}",
        (10, 25),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (0, 255, 0),
        2
    )

    cv2.imshow("Cabbot - Segmentation Polygon Centroid", annotated_frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()