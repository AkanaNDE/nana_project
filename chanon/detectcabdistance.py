import cv2
from ultralytics import YOLO

# Load the YOLO11 model
model = YOLO("bestcab2.pt")

# Open the video file / Webcam
cap = cv2.VideoCapture(0)

# --- ตั้งค่า Resolution เป็น 720p ---
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

# --- ตัวแปรสำหรับคำนวณระยะทาง ---
KNOWN_WIDTH_CM = 20.0  # สมมติว่ากะหล่ำปลีมีความกว้างจริง 20 ซม.
# Focal Length (หาได้จากการ calibrate กล้อง)
# สมมติค่าเบื้องต้น: (pixel_width * distance) / known_width
FOCAL_LENGTH = 800  

# ตัวแปรสำหรับเช็คขนาดความพร้อม
min_ready_width = 15.0 

while cap.isOpened():
    success, frame = cap.read()

    if success:
        # Run YOLO11 tracking
        results = model.track(frame, persist=True, device="0", verbose=False)
        annotated_frame = results[0].plot()

        if results[0].boxes is not None:
            for box in results[0].boxes:
                # ดึงค่าพิกเซล x, y, width, height (ใช้ CPU เพื่อคำนวณ)
                x, y, w, h = box.xywh[0].cpu() 
                
                # --- คำนวณขนาดและระยะทาง ---
                # 1. คำนวณความกว้างจริงในภาพ (cm)
                pixel_width = w.item()
                actual_width_cm = pixel_width / (FOCAL_LENGTH / 100) # นี่เป็นสูตรปรับปรุงง่ายๆ
                
                # 2. คำนวณระยะห่าง (Distance)
                # สูตร: Distance = (Known Width * Focal Length) / Pixel Width
                distance_cm = (KNOWN_WIDTH_CM * FOCAL_LENGTH) / pixel_width
                
                cx, cy = int(x), int(y)

                # วาดจุดกึ่งกลาง
                cv2.circle(annotated_frame, (cx, cy), 5, (0, 0, 255), -1)

                # --- เงื่อนไขการเช็คขนาด ---
                if actual_width_cm >= min_ready_width:
                    status_text = f"READY: {actual_width_cm:.1f} cm"
                    color = (0, 255, 0) # สีเขียว
                else:
                    status_text = f"NOT READY: {actual_width_cm:.1f} cm"
                    color = (0, 0, 255) # สีแดง

                # แสดงสถานะและระยะทางบนภาพ
                cv2.putText(
                    annotated_frame, 
                    status_text, 
                    (int(x - w/2), int(y - h/2) - 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 
                    0.6, color, 2
                )
                
                # แสดงระยะห่าง
                cv2.putText(
                    annotated_frame, 
                    f"Dist: {distance_cm:.1f} cm", 
                    (int(x - w/2), int(y - h/2) - 10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 
                    0.5, (255, 255, 0), 2 # สีฟ้า
                )

        # Display the annotated frame
        cv2.imshow("YOLO11 Distance Measurement", annotated_frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    else:
        break

cap.release()
cv2.destroyAllWindows()