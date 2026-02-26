import cv2
import numpy as np

MARKER_SIZE = 175  # размер маркера в мм

def GetBoardData(frame, camera_matrix, dist_coeffs, detector, left_bottom, left_top, right_top, right_bottom):
    # получаем координаты угловых маркеров и их id
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)
    if ids is None:
        return None, None, None, None
    # находим центры угловых маркеров в пикселях
    img_centers = []
    corner_ids = [left_bottom, left_top, right_top, right_bottom]
    for corner_id in corner_ids:
        if corner_id in ids:
            idx = np.where(ids == corner_id)[0][0]
            center = np.mean(corners[idx][0], axis=0)
            img_centers.append(center)

    if len(img_centers) != 4:  # TODO: graceful degradation (если камера считала не все углы доски, стоит все равно попробовать вычислить размер)
        return None, None, None, None
    img_centers = np.array(img_centers, dtype=np.float32)
    # размеры доски в пикселях (как полусумма векторов) - при небольших углах должно работать корректно
    pixel_width = (np.linalg.norm(img_centers[3] - img_centers[0]) + np.linalg.norm(img_centers[2] - img_centers[1])) / 2
    pixel_height = (np.linalg.norm(img_centers[1] - img_centers[0]) + np.linalg.norm(img_centers[2] - img_centers[3])) / 2
    # 3d-координаты углов маркера в его собственной системе координат
    half = MARKER_SIZE / 2.0
    marker_obj_points = np.array([
        [-half, -half, 0],
        [ half, -half, 0],
        [ half,  half, 0],
        [-half,  half, 0]
    ], dtype=np.float32)
    # 3d-позиции центров угловых маркеров
    positions = {}
    for corner_id in corner_ids:
        if corner_id in ids:
            idx = np.where(ids == corner_id)[0][0]
            marker_corners = corners[idx][0]
            success, rvec, tvec = cv2.solvePnP(marker_obj_points, marker_corners, camera_matrix, dist_coeffs)
            if success:
                positions[corner_id] = tvec.flatten()

    if len(positions) != 4:
        return None, None, None, None
    # координаты центров маркеров
    A = positions[left_bottom]
    B = positions[left_top]
    C = positions[right_top]
    D = positions[right_bottom]
    # центр доски и расстояние до него
    center = (A + B + C + D) / 4
    depth = np.linalg.norm(center)
    v_center = center / depth  # направление на центр
    # нормаль к плоскости доски через векторное произведение диагоналей
    diag1 = C - A
    diag2 = D - B
    normal = np.cross(diag1, diag2)
    normal = normal / np.linalg.norm(normal)
    # косинус угла между направлением на центр и нормалью (нужен для коррекции перспективы)
    cos_angle = np.abs(np.dot(v_center, normal))
    # фокусное расстояние камеры
    focal_length = (camera_matrix[0,0] + camera_matrix[1,1]) / 2
    # реальные размеры доски с поправкой на угол наклона
    width = (pixel_width * depth) / (focal_length * cos_angle)
    height = (pixel_height * depth) / (focal_length * cos_angle)

    return depth, width, height, positions

def GetRobotPosition(robot_pos, corner_positions, left_bottom, left_top, right_bottom, board_width, board_height):
    # координаты угловых маркеров в 3d
    lb = corner_positions[left_bottom]
    lt = corner_positions[left_top]
    rb = corner_positions[right_bottom]
    # строим оси доски: x - от левого нижнего к правому нижнему, y - от левого нижнего к левому верхнему
    v_x = rb - lb
    v_y = lt - lb
    # делаем оси перпендикулярными
    v_x = v_x / np.linalg.norm(v_x)
    v_y = v_y - np.dot(v_y, v_x) * v_x
    v_y = v_y / np.linalg.norm(v_y)
    # вектор от левого нижнего угла до робота
    vec = robot_pos - lb
    # проекция на оси доски (координаты робота в мм)
    x = np.dot(vec, v_x)
    y = np.dot(vec, v_y)
    return x, y

def main():
    data = np.load('/Users/vmn/Downloads/ориентация_по_доске/calibration_final_final.npz') # результаты калибровки камеры
    camera_matrix = data['camera_matrix']
    dist_coeffs = data['dist_coeffs']
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
    detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
    # id маркеров (соответствуют файлам 1.png, 2.png, 3.png, 4.png, 5.png)
    left_bottom = 1
    left_top = 2
    right_top = 3
    right_bottom = 4
    robot = 5

    frame = cv2.imread('/Users/vmn/Downloads/ориентация_по_доске/фото_ноут1.jpg')
    if frame is None:
        print("Ошибка при чтении файла")
        return

    distance, width, height, corner_positions = GetBoardData(frame, camera_matrix, dist_coeffs, detector, left_bottom, left_top, right_top, right_bottom)
    if distance is not None:
        print(f"Расстояние до доски: {distance:.0f} мм")
        print(f"Размеры доски: {width:.0f} x {height:.0f} мм")
        # ищем маркер робота
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)
        if ids is not None and robot in ids:
            robot_idx = np.where(ids == robot)[0][0]

            # 3d-координаты углов маркера робота
            half = MARKER_SIZE / 2.0
            marker_obj_points = np.array([
                [-half, -half, 0],
                [ half, -half, 0],
                [ half,  half, 0],
                [-half,  half, 0]
            ], dtype=np.float32)

            robot_corners = corners[robot_idx][0]
            success, rvec_robot, tvec_robot = cv2.solvePnP(marker_obj_points, robot_corners, camera_matrix, dist_coeffs)
            if success:
                robot_pos = tvec_robot.flatten()
                robot_x, robot_y = GetRobotPosition(robot_pos, corner_positions, left_bottom, left_top, right_bottom, width, height)
                print(f"Робот: x={robot_x:.0f} мм, y={robot_y:.0f} мм")
            else:
                print("Не удалось определить позицию робота")
        else:
            print("Не удалось обнаружить робота")
    else:
        print("Не удалось обнаружить доску")

if __name__ == "__main__":
    main()
