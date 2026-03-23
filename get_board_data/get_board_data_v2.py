import cv2
import numpy as np
import time

MARKER_SIZE = 0.065  # размер маркера в м
SMOOTH_ALPHA = 0.9   # коэффициент сглаживания доски

# глобальные сглаженные параметры доски
SMOOTHED_ORIGIN = None
SMOOTHED_X = None
SMOOTHED_Y = None
SMOOTHED_NORMAL = None


def SmoothBoard(origin, board_x, board_y, board_normal):
    global SMOOTHED_ORIGIN, SMOOTHED_X, SMOOTHED_Y, SMOOTHED_NORMAL
    if SMOOTHED_ORIGIN is None:
        SMOOTHED_ORIGIN = origin
        SMOOTHED_X = board_x
        SMOOTHED_Y = board_y
        SMOOTHED_NORMAL = board_normal
        return origin, board_x, board_y, board_normal

    SMOOTHED_ORIGIN = SMOOTH_ALPHA * SMOOTHED_ORIGIN + (1 - SMOOTH_ALPHA) * origin
    SMOOTHED_X = SMOOTH_ALPHA * SMOOTHED_X + (1 - SMOOTH_ALPHA) * board_x
    SMOOTHED_Y = SMOOTH_ALPHA * SMOOTHED_Y + (1 - SMOOTH_ALPHA) * board_y
    SMOOTHED_NORMAL = SMOOTH_ALPHA * SMOOTHED_NORMAL + (1 - SMOOTH_ALPHA) * board_normal

    SMOOTHED_X = SMOOTHED_X / np.linalg.norm(SMOOTHED_X)
    SMOOTHED_Y = SMOOTHED_Y / np.linalg.norm(SMOOTHED_Y)
    SMOOTHED_NORMAL = SMOOTHED_NORMAL / np.linalg.norm(SMOOTHED_NORMAL)

    return SMOOTHED_ORIGIN, SMOOTHED_X, SMOOTHED_Y, SMOOTHED_NORMAL


def GetBoardData(frame, camera_matrix, dist_coeffs, detector, left_bottom, left_top, right_top, right_bottom):
    # получаем координаты угловых маркеров и их id
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)
    if ids is not None:
        print(f"Найденные ID маркеров: {ids.flatten().tolist()}")
    else:
        return None, None, None, None, None, None

    # 3d-координаты углов маркера в его собственной системе координат
    half = MARKER_SIZE / 2.0
    marker_obj_points = np.array([[-half, -half, 0], [half, -half, 0], [half, half, 0], [-half, half, 0]], dtype=np.float32)
    corner_ids = [left_bottom, left_top, right_top, right_bottom]

    # 3d-позиции центров угловых маркеров
    positions = {}
    for corner_id in corner_ids:
        if corner_id in ids:
            idx = np.where(ids == corner_id)[0][0]
            marker_corners = corners[idx][0]
            success, rvec, tvec = cv2.solvePnP(marker_obj_points, marker_corners, camera_matrix, dist_coeffs)
            if success:
                positions[corner_id] = tvec.flatten()

    # если найдено меньше 4 маркеров доски — не можем построить плоскость
    if len(positions) < 4:
        return None, None, None, None, None, None

    origin = positions[left_bottom]

    # ищем точку для оси X
    if right_bottom in positions and left_bottom in positions:
        x_ref = positions[right_bottom]
    elif right_top in positions and left_top in positions:
        x_ref = positions[right_top]
    else:
        x_ref = list(positions.values())[1]

    # ищем точку для оси Y
    if left_top in positions and left_bottom in positions:
        y_ref = positions[left_top]
    elif right_top in positions and right_bottom in positions:
        y_ref = positions[right_top]
    else:
        y_ref = list(positions.values())[2]

    v_x = x_ref - origin
    board_x = v_x / np.linalg.norm(v_x)
    v_y = y_ref - origin
    v_y = v_y - np.dot(v_y, board_x) * board_x
    board_y = v_y / np.linalg.norm(v_y)
    board_normal = np.cross(board_x, board_y)
    board_normal = board_normal / np.linalg.norm(board_normal)

    # сглаживание положения доски
    board_origin, board_x, board_y, board_normal = SmoothBoard(origin, board_x, board_y, board_normal)
    # оценка размеров доски
    widths = []
    heights = []

    if left_bottom in positions and right_bottom in positions:
        widths.append(np.linalg.norm(positions[left_bottom] - positions[right_bottom]))
    if left_top in positions and right_top in positions:
        widths.append(np.linalg.norm(positions[left_top] - positions[right_top]))
    if left_bottom in positions and left_top in positions:
        heights.append(np.linalg.norm(positions[left_bottom] - positions[left_top]))
    if right_bottom in positions and right_top in positions:
        heights.append(np.linalg.norm(positions[right_bottom] - positions[right_top]))

    width = sum(widths) / len(widths) if len(widths) > 0 else 0
    height = sum(heights) / len(heights) if len(heights) > 0 else 0

    return board_origin, board_x, board_y, board_normal, width, height


def DetectRobot(frame, camera_matrix, dist_coeffs, detector, robot):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)
    if ids is None or robot not in ids:
        return None, None

    idx = np.where(ids == robot)[0][0]
    robot_corners = corners[idx][0]
    half = MARKER_SIZE / 2.0
    marker_obj_points = np.array([[-half, -half, 0], [half, -half, 0], [half, half, 0], [-half, half, 0]], dtype=np.float32)

    success, rvec_robot, tvec_robot = cv2.solvePnP(marker_obj_points, robot_corners, camera_matrix, dist_coeffs)
    if not success:
        return None, None

    return rvec_robot, tvec_robot.flatten()


def GetRobotState(robot_pos, rvec_robot, board_origin, board_x, board_y):
    vec = robot_pos - board_origin
    x = np.dot(vec, board_x)
    y = np.dot(vec, board_y)
    R, _ = cv2.Rodrigues(rvec_robot)
    robot_forward = R[:, 0]
    theta = np.arctan2(np.dot(robot_forward, board_y), np.dot(robot_forward, board_x))
    return x, y, theta


def main():
    # data = np.load('../camera_set_up/calibration_result.npz') # результаты калибровки камеры
    data = np.load('calibration_result6.npz') # результаты калибровки камеры
    camera_matrix = data['camera_matrix']
    dist_coeffs = data['dist_coeffs']
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())

    # id маркеров (соответствуют файлам 1.png, 2.png, 3.png, 4.png, 7.png)
    left_bottom = 1
    left_top = 2
    right_top = 3
    right_bottom = 4
    robot = 7

    # camera_index = 0
    # cap = cv2.VideoCapture(camera_index)
    # cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
    # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 2160)
    # time.sleep(2)
    # ok, frame = cap.read()
    # print(f"Размер кадра: {frame.shape}")
    # timestamp = int(time.time() * 1000)
    # path = f"/Users/vmn/Downloads/saved/shot_{timestamp}.png"
    # print(timestamp)
    # cv2.imwrite(path, frame)
    # cap.release()

    image_path = '/Users/vmn/Downloads/saved/shot_1773312585795.png'
    frame = cv2.imread(image_path)
    print(f"Размер кадра: {frame.shape}")
    if frame is None:
        print("Ошибка при чтении файла")
        return

    board_origin, board_x, board_y, board_normal, width, height = GetBoardData(frame, camera_matrix, dist_coeffs, detector, left_bottom, left_top, right_top, right_bottom)
    if board_origin is None:
        print("Не удалось обнаружить доску")
        return

    print(f"Размеры доски: {width:.4f} x {height:.4f} м")

    # ищем маркер робота
    rvec_robot, robot_pos = DetectRobot(frame, camera_matrix, dist_coeffs, detector, robot)
    if robot_pos is not None:
        robot_x, robot_y, robot_theta = GetRobotState(robot_pos, rvec_robot, board_origin, board_x, board_y)
        print(f"Координаты робота: x={robot_x:.4f} м, y={robot_y:.4f} м, theta={robot_theta:.4f}")
    else:
        print("Не удалось обнаружить робота")


if __name__ == "__main__":
    main()