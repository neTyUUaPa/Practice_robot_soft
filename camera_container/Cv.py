import cv2
from flask import Flask, Response

class CameraController:
    def __init__(self):
        self.camera = None
        #self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        #self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    def start_camera(self):
        """Включает камеру и возвращает объект захвата"""
        if self.camera is None:
            self.camera = cv2.VideoCapture(0)
            if not self.camera.isOpened():
                print("Ошибка: Не удалось включить камеру")
                return None
            print("Камера работает, а ты нет")
        return self.camera

    def stop_camera(self):
        """Выключает камеру и освобождает ресурсы"""
        if self.camera is not None:
            self.camera.release()
            cv2.destroyAllWindows()
            self.camera = None
            print("Камера выключена, а ты все еще не работаешь")

    def generate_frames(self):
        """Генератор для отправки кадров с камеры"""
        # Инициализация словаря ArUco
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())
        while True:
            ret, frame = self.camera.read()
            if not ret:
                break

            # Преобразование кадра в градации серого
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Обнаружение маркеров
            corners, ids, _ = detector.detectMarkers(gray)

            # Если маркеры обнаружены, рисуем их
            if ids is not None:
                print(f"Обнаружен маркер ID: {ids.flatten()}")
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            # Преобразование кадра в JPEG для передачи через HTTP
            _, buffer = cv2.imencode('.jpg', frame)

            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')



# Настройка Flask
app = Flask(__name__)
camera_controller = CameraController()

@app.route('/video_feed')
def video_feed():
    """Трансляция видео через HTTP"""
    camera = camera_controller.start_camera()
    if camera is None:
        return "Ошибка: Камера не работает", 500
    return Response(
        camera_controller.generate_frames(),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )

if __name__ == "__main__":
    # Запуск Flask-сервера
    app.run(host='0.0.0.0', port=5000, threaded=True)