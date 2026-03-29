import cv2

# --- ตั้งค่าตัวแปร (แทนที่การใช้ Parameters ใน ROS 2) ---
camera_index = 3      # ลำดับของกล้อง
width = 640            # ความกว้าง
height = 480           # ความสูง
jpeg_quality = 40      # คุณภาพไฟล์ (สำหรับตอน encode)
preview = True         # เปิดโชว์หน้าต่างภาพหรือไม่

def main():
    # 1. เริ่มต้นเปิดกล้อง
    cap = cv2.VideoCapture(camera_index)

    # 2. ตั้งค่าความละเอียด
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    if not cap.isOpened():
        print(f"ไม่สามารถเปิดกล้องลำดับที่ {camera_index} ได้")
        return

    print("กด 'q' เพื่อออกจากโปรแกรม")

    try:
        while True:
            # อ่านภาพจากกล้อง
            ret, frame = cap.read()

            if not ret:
                print("ไม่สามารถรับภาพจากกล้องได้")
                break

            # --- ส่วนการจำลอง jpeg_quality (ถ้าต้องการลดคุณภาพก่อนส่ง/บันทึก) ---
            # encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality]
            # result, encimg = cv2.imencode('.jpg', frame, encode_param)
            # frame = cv2.imdecode(encimg, 1) # นำกลับมาแสดงผลที่คุณภาพลดลงแล้ว

            # 3. แสดงผลภาพ
            if preview:
                cv2.imshow('Camera Preview', frame)

            # 4. รอการกดปุ่ม 'q' เพื่อออก
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        # ปล่อยกล้องและปิดหน้าต่าง
        cap.release()
        cv2.destroyAllWindows()
        print("ปิดโปรแกรมเรียบร้อย")

if __name__ == "__main__":
    main()