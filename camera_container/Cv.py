import cv2


class CameraController:
    def __init__(self):
        self.camera = None

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


def detect_aruco_markers():
    """Основная функция для обнаружения маркеров"""
    controller = CameraController()
    camera = controller.start_camera()

    if camera is None:
        return

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())

    try:
        while True:
            ret, frame = camera.read()
            if not ret:
                print("Ошибка чтения кадра")
                break

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = detector.detectMarkers(gray)

            if ids is not None:
                print(f"Обнаружен маркер ID: {ids.flatten()}")
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            cv2.imshow("ArUco Detection", frame)

            if cv2.waitKey(1) == ord("q"):
                break

    finally:
        controller.stop_camera()


if __name__ == "__main__":
    detect_aruco_markers()