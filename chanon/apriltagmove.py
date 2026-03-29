import cv2
from pupil_apriltags import Detector
import numpy as np

# --- ส่วนที่ 1: ตั้งค่าเริ่มต้น ---
def decode_tag_id(tag_id):
    if not (10000 <= tag_id <= 99999): return None
    return {
        "plating_dist": tag_id // 1000,
        "spacing_gap": {1: 5, 2: 10, 3: 15, 4: 20, 5: 25}.get((tag_id // 100) % 10, 0),
        "cabbage_int": tag_id % 100
    }

# --- ตัวแปรสำหรับ Calibrate (ปรับค่าให้ตรงกับกล้องจริง) ---
# ค่านี้หาได้จากการนำ AprilTag มาวางที่ระยะ Known Distance แล้วคำนวณกลับ
FOCAL_LENGTH = 400  

# ขนาดจริงของ AprilTag (cm) - วัดจากขอบสีดำด้านนอกสุด
KNOWN_TAG_SIZE_CM = 7.0 

at_detector = Detector(families='tagStandard52h13')
55
cap = cv2.VideoCapture(0)
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

# --- ตั้งค่า OFFSET เพื่อวิ่งด้านซ้ายของ Tag ---
target_offset = 200 
target_x = (frame_width // 2) + target_offset 
deadzone = 40 

while cap.isOpened():
    success, frame = cap.read()
    if not success: break

    # --- ส่วนของ AprilTag (ทำงานบน CPU) ---
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    tag_results = at_detector.detect(gray)
    
    command = "SEARCHING" 
    robot_pos_status = "UNKNOWN"
    distance_cm = None

    if tag_results:
        r = tag_results[0]
        tag_center_x = int(r.center[0])
        
        # --- คำนวณระยะห่าง (Distance) เหมือนโค้ด YOLO ---
        # 1. หาความกว้างของ Tag ในภาพเป็นพิกเซล (หาค่าเฉลี่ยจากมุม 4 มุม)
        corners = r.corners
        pixel_width = np.linalg.norm(corners[0] - corners[1]) 
        
        if pixel_width > 0:
            # สูตร: Distance = (Known Tag Size * Focal Length) / Pixel Width
            distance_cm = (KNOWN_TAG_SIZE_CM * FOCAL_LENGTH) / pixel_width
        
        # คำนวณ Error เทียบกับจุด Target
        error_x = tag_center_x - target_x

        # สรุปสถานะตำแหน่งรถ (Robot Position)
        if error_x > deadzone:
            command = "MOVE RIGHT (Adjusting)"
            robot_pos_status = "TOO FAR LEFT"
        elif error_x < -deadzone:
            command = "MOVE LEFT (Adjusting)"
            robot_pos_status = "TOO CLOSE TO WALL"
        else:
            command = "FORWARD (Path Clear)"
            robot_pos_status = "ALIGNED (Left of Tag)"

        # วาดตำแหน่ง Tag
        cv2.circle(frame, (tag_center_x, int(r.center[1])), 6, (255, 0, 0), -1) 
        cv2.putText(frame, f"ID: {r.tag_id}", (tag_center_x - 15, int(r.center[1]) - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)

    # --- ส่วนการแสดงผลสถานะบนหน้าจอ (UI) ---
    cv2.rectangle(frame, (0, 0), (200, 110), (0, 0, 0), -1) 
    
    cv2.putText(frame, f"CMD: {command}", (10, 25), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
    cv2.putText(frame, f"POS: {robot_pos_status}", (10, 50), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    # แสดงระยะห่าง (ถ้าคำนวณได้)
    if distance_cm is not None:
        cv2.putText(frame, f"Dist: {distance_cm:.1f} cm", (10, 75), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
    else:
        cv2.putText(frame, f"Dist: N/A", (10, 75), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                    
    cv2.putText(frame, f"OFFSET: {target_offset} px", (10, 100), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)

    # --- วาดเส้น Guide Lines ---
    cv2.line(frame, (frame_width // 2, 0), (frame_width // 2, frame_height), (200, 200, 200), 1)
    cv2.line(frame, (target_x, 0), (target_x, frame_height), (0, 255, 0), 2)
    cv2.line(frame, (target_x - deadzone, 0), (target_x - deadzone, frame_height), (0, 0, 255), 1)
    cv2.line(frame, (target_x + deadzone, 0), (target_x + deadzone, frame_height), (0, 0, 255), 1)

    cv2.imshow("Wall Avoidance & Distance System", frame)
    if cv2.waitKey(1) & 0xFF == ord("q"): break

cap.release()
cv2.destroyAllWindows()