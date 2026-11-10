from flask import Flask, render_template, Response, request, jsonify
import cv2
import numpy as np
import time
import threading
from LOBOROBOT2 import LOBOROBOT, FORWARD, BACKWARD

try:
    from picamera2 import Picamera2
    HAS_PICAMERA2 = True
except ImportError:
    HAS_PICAMERA2 = False

app = Flask(__name__)

# ==================== 初始化真實車體與雲台角度 ====================
robot = LOBOROBOT()

pan_angle = 90
tilt_angle = 0

# 程式啟動時讓雲台歸位
robot.set_servo_angle(10, pan_angle, 0)
robot.set_servo_angle(9, tilt_angle, 0)

# ==================== 樹莓派硬體相機管理器 ====================
class PicamManager:
    def __init__(self):
        self.picam2 = None
        self.lock = threading.Lock()
        self.init_camera()

    def init_camera(self):
        if HAS_PICAMERA2:
            try:
                print("正在初始化 Picamera2 硬體相機...")
                self.picam2 = Picamera2()
                config = self.picam2.create_video_configuration(main={"size": (640, 480), "format": "RGB888"})
                self.picam2.configure(config)
                self.picam2.start()
                print("-> 【成功】Picamera2 硬體相機啟動成功！")
                return
            except Exception as e:
                print(f"Picamera2 啟動失敗: {e}")
        
        print("退回使用傳統 OpenCV 讀取...")
        for idx in [0, 1, 2, 19]:
            cap = cv2.VideoCapture(idx)
            if cap.isOpened():
                ret, frame = cap.read()
                if ret and frame is not None:
                    print(f"-> OpenCV 成功開啟通道: {idx}")
                    self.fallback_cap = cap
                    return
                cap.release()
        self.fallback_cap = None

    def get_frame(self):
        with self.lock:
            if HAS_PICAMERA2 and self.picam2 is not None:
                try:
                    frame = self.picam2.capture_array()
                    if frame is not None:
                        return frame
                except Exception as e:
                    print("Picamera2 擷取錯誤:", e)
            
            if hasattr(self, 'fallback_cap') and self.fallback_cap is not None:
                ret, frame = self.fallback_cap.read()
                if ret:
                    return frame
                    
            return None

cam_manager = PicamManager()


# ==================== OpenCV 車道中心線與 CTE 檢測處理 ====================
def process_lane_detection(frame):
    try:
        # 畫面方向校正
        frame = cv2.flip(frame, 0)
        frame = cv2.flip(frame, 1)

        height, width, _ = frame.shape
        original_frame = frame.copy()

        # 1. 轉換到 HSV 色彩空間並進行白色遮罩
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_white = np.array([0, 0, 160])
        upper_white = np.array([180, 50, 255])
        mask = cv2.inRange(hsv, lower_white, upper_white)

        # 2. 定義梯形 ROI 區域 (紅色外框)
        roi_mask = np.zeros_like(mask)
        polygon = np.array([[
            (int(width * 0.1), height),          
            (int(width * 0.32), int(height * 0.65)), 
            (int(width * 0.68), int(height * 0.65)), 
            (int(width * 0.9), height)           
        ]], dtype=np.int32)
        cv2.fillPoly(roi_mask, polygon, 255)
        mask = cv2.bitwise_and(mask, roi_mask)

        # 3. 形態學處理（去除雜訊）
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=1)
        mask = cv2.erode(mask, kernel, iterations=1)

        # 4. 繪製紅色梯形 ROI 區域
        cv2.polylines(frame, [polygon], isClosed=True, color=(0, 0, 255), thickness=2)

        # 5. 特定高度掃描左右檢測點
        scan_y = int(height * 0.8)
        row_pixels = mask[scan_y, :]
        white_indices = np.where(row_pixels > 0)[0]
        
        left_x, right_x = None, None
        if len(white_indices) > 0:
            left_indices = white_indices[white_indices < width / 2]
            right_indices = white_indices[white_indices >= width / 2]
            
            if len(left_indices) > 0:
                left_x = int(np.mean(left_indices))
                cv2.rectangle(frame, (left_x - 10, scan_y - 30), (left_x + 10, height), (0, 255, 0), -1)
                cv2.circle(frame, (left_x, scan_y), 6, (0, 200, 0), -1)
                
            if len(right_indices) > 0:
                right_x = int(np.mean(right_indices))
                cv2.rectangle(frame, (right_x - 10, scan_y - 30), (right_x + 10, height), (0, 255, 0), -1)
                cv2.circle(frame, (right_x, scan_y), 6, (0, 200, 0), -1)
                
        # 6. 計算中心線與偏移量 (CTE)
        cte = 0
        if left_x is not None and right_x is not None:
            center_x = int((left_x + right_x) / 2)
            frame_center = int(width / 2)
            cte = center_x - frame_center

            # 繪製藍色車道中心線
            cv2.line(frame, (frame_center, int(height * 0.65)), (frame_center, height), (255, 0, 0), 2, cv2.LINE_AA)
            cv2.circle(frame, (center_x, scan_y), 8, (255, 0, 0), -1)
            
            arrow_color = (255, 255, 0) if abs(cte) > 30 else (255, 0, 0)
            cv2.arrowedLine(frame, (frame_center, scan_y), (center_x, scan_y), arrow_color, 3, tipLength=0.3)
        elif left_x is not None:
            cte = left_x - int(width / 2)
        elif right_x is not None:
            cte = right_x - int(width / 2)

        # 7. 【強制在左上角疊加 CTE 像素數值】
        cv2.putText(frame, f"CTE: {cte} px", (30, 45), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 255), 3, cv2.LINE_AA)

        # 8. 融合原圖與標記畫面
        final_frame = cv2.addWeighted(original_frame, 0.4, frame, 0.6, 0)
        
    except Exception as e:
        print("Lane detection error:", e)
        final_frame = frame
        
    return final_frame


# ==================== 串流產生器 ====================
def generate_frames():
    while True:
        frame = cam_manager.get_frame()
        if frame is None:
            dummy_frame = np.zeros((480, 640, 3), dtype=np.uint8)
            dummy_frame[:] = (20, 20, 40)
            cv2.putText(dummy_frame, "Camera Stream Loading...", (70, 240), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2, cv2.LINE_AA)
            ret, buffer = cv2.imencode('.jpg', dummy_frame)
            if ret:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
            time.sleep(0.5)
            continue
            
        processed_frame = process_lane_detection(frame)

        ret, buffer = cv2.imencode('.jpg', processed_frame)
        if not ret:
            continue
            
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')


# ==================== Flask 路由 ====================

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/control', methods=['POST'])
def control():
    global pan_angle, tilt_angle
    try:
        data = request.json
        action = data.get('action')
        speed = 50

        # 車體移動控制
        if action == 'forward': robot.moveforward(speed, 0)
        elif action == 'backward': robot.movebackward(speed, 0)
        elif action == 'left': robot.turnLeft(speed, 0)
        elif action == 'right': robot.turnRight(speed, 0)
        elif action == 'stop': robot.t_stop(0)

        # 雲台控制（維持角度累加，初始為 pan:90, tilt:0）
        elif action == 'pan_left':
            pan_angle = min(pan_angle + 5, 180)
            robot.set_servo_angle(10, pan_angle, 0)
        elif action == 'pan_right':
            pan_angle = max(pan_angle - 5, 0)
            robot.set_servo_angle(10, pan_angle, 0)
        elif action == 'tilt_up':
            tilt_angle = max(tilt_angle - 5, 0)
            robot.set_servo_angle(9, tilt_angle, 0)
        elif action == 'tilt_down':
            tilt_angle = min(tilt_angle + 5, 180)
            robot.set_servo_angle(9, tilt_angle, 0)

        print(f"收到指令: {action}, 當前雲台 pan={pan_angle}, tilt={tilt_angle}")
        return jsonify(status="success", action=action, pan=pan_angle, tilt=tilt_angle)
    except Exception as e:
        print("Control error:", e)
        return jsonify(status="error", message=str(e))


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)