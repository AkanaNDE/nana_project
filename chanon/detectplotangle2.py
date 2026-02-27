import cv2
import math
from ultralytics import YOLO

# --- ส่วนที่ 1: ตั้งค่าเริ่มต้น ---
# เปลี่ยน path เป็นไฟล์ของคุณ
model = YOLO("best4.pt") 
cap = cv2.VideoCapture(2)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)




# --- ตั้งค่าการควบคุม (Control Settings) ---
deadzone_angle = 3.0  # ค่าความเพี้ยนที่ยอมรับได้ (องศา)

while cap.isOpened():
    success, frame = cap.read()
    if not success:
        break

    # --- ส่วนที่แก้ไข: หมุนภาพ 90 องศา ---
    # หากภาพตะแคงขวา ให้ใช้ cv2.ROTATE_90_CLOCKWISE
    # หากภาพตะแคงซ้าย ให้ใช้ cv2.ROTATE_90_COUNTERCLOCKWISE
    frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)

    # ดึงขนาดภาพใหม่หลังจากหมุนแล้ว
    h, w, _ = frame.shape
    screen_cx = w // 2  # จุดกึ่งกลางจอ (อ้างอิงแนวตั้งใหม่)

    # ส่งภาพที่หมุนแล้วให้ YOLO ประมวลผล
    # เพิ่ม conf=0.5 เพื่อกรองวัตถุที่ไม่มั่นใจออก
    results = model.track(frame, persist=True, conf=0.5)
    
    command = "SEARCHING"
    angle_deg = 0.0

    # ตรวจสอบการ Detection
    if results[0].boxes is not None and len(results[0].boxes) > 0:
        # ใช้ .plot() วาด Box ลงบนภาพที่หมุนแล้ว
        annotated_frame = results[0].plot()
        
        # ดึงข้อมูล Box ตัวแรก
        box = results[0].boxes[0]
        x1, y1, x2, y2 = box.xyxy[0].tolist()
        obj_cx = int((x1 + x2) / 2)
        obj_cy = int((y1 + y2) / 2)

        # --- ส่วนที่ 2: คำนวณมุมเบี่ยงเบน ---
        # คำนวณระยะห่างจากกึ่งกลางจอ (แกน X) และระยะลึก (แกน Y)
        dist_x = obj_cx - screen_cx
        dist_y = h - obj_cy  # ระยะจากขอบล่างจอขึ้นไปหาวัตถุ
        
        # ป้องกันการหารด้วยศูนย์
        if dist_y == 0: dist_y = 1
        
        angle_rad = math.atan2(dist_x, dist_y)
        angle_deg = math.degrees(angle_rad)

        # Logic การสั่งการ
        if angle_deg > deadzone_angle:
            command = "TURN RIGHT"
        elif angle_deg < -deadzone_angle:
            command = "TURN LEFT"
        else:
            command = "FORWARD (STAY CENTER)"

        # --- ส่วนที่ 3: วาด Visual ---
        # วาดเส้นจากกึ่งกลางล่างจอไปยังวัตถุ
        cv2.line(annotated_frame, (screen_cx, h), (obj_cx, obj_cy), (0, 0, 255), 2)
        # แสดงค่ามุม
        cv2.putText(annotated_frame, f"{angle_deg:.1f} deg", (obj_cx, obj_cy - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    else:
        # หากไม่เจอวัตถุ ให้ใช้ภาพเปล่าที่หมุนแล้ว
        annotated_frame = frame.copy()

    # วาดเส้นกึ่งกลางอ้างอิง (สีเหลือง)
    cv2.line(annotated_frame, (screen_cx, 0), (screen_cx, h), (0, 255, 255), 2)

    # แสดงแถบสถานะ ACTION
    cv2.rectangle(annotated_frame, (0, 0), (280, 40), (0, 0, 0), -1)
    cv2.putText(annotated_frame, f"ACTION: {command}", (10, 25), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    # แสดงผลหน้าจอ
    cv2.imshow("Cabbot - 90 Deg Corrected", annotated_frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()