import cv2
import numpy as np
import time
from itertools import combinations

MARKER_SIZE = 0.065  # размер маркера в м

def GetBoardData(frame, camera_matrix, dist_coeffs, detector, left_bottom, left_top, right_top, right_bottom, robot):
    # получаем координаты угловых маркеров и их id
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)
    if ids is not None:
        print(f"Найденные ID маркеров: {ids.flatten().tolist()}")
    if ids is None:
        return None, None, None, None, None
    # находим центры угловых маркеров в пикселях
    img_centers = []
    corner_ids = [left_bottom, left_top, right_top, right_bottom]
    for corner_id in corner_ids:
        if corner_id in ids:
            idx = np.where(ids == corner_id)[0][0]
            center = np.mean(corners[idx][0], axis=0)
            img_centers.append(center)

    if len(img_centers) < 3:
        return None, None, None, None, None
    img_centers = np.array(img_centers, dtype=np.float32)
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

    points_list = list(positions.values())
    distances_to_plane = []
    for triple in combinations(points_list, 3):
        p1, p2, p3 = triple
        v1 = p2 - p1
        v2 = p3 - p1
        normal = np.cross(v1, v2)
        norm_length = np.linalg.norm(normal)
        normal = normal / norm_length  
        distance = abs(np.dot(normal, p1))
        distances_to_plane.append(distance)

    lengths = []
    heights = []

    if left_bottom in positions and right_bottom in positions:
        lengths.append(np.linalg.norm(positions[left_bottom] - positions[right_bottom]))
    if left_top in positions and right_top in positions:
        lengths.append(np.linalg.norm(positions[left_top] - positions[right_top]))
    if left_bottom in positions and left_top in positions:
        heights.append(np.linalg.norm(positions[left_bottom] - positions[left_top]))
    if right_bottom in positions and right_top in positions:
        heights.append(np.linalg.norm(positions[right_bottom] - positions[right_top]))

    # print(lengths)
    # print(heights)

    width = sum(lengths) / len(lengths)
    height = sum(heights) / len(heights)
    depth = np.mean(distances_to_plane)

    robot_pos = None
    if robot in ids:
        robot_idx = np.where(ids == robot)[0][0]
        robot_corners = corners[robot_idx][0]
        success, rvec_robot, tvec_robot = cv2.solvePnP(marker_obj_points, robot_corners, camera_matrix, dist_coeffs)
        if success:
            robot_pos = tvec_robot.flatten()

    return depth, width, height, positions, robot_pos

def HelpGetRobotPosition(robot_pos, corner_positions, left_bottom, left_top, right_bottom):
    if left_bottom not in corner_positions or right_bottom not in corner_positions or left_top not in corner_positions:
        return None, None
    lb = corner_positions[left_bottom]
    vec = robot_pos - lb
    rb = corner_positions[right_bottom]
    v_x = rb - lb
    if (np.linalg.norm(v_x) == 0):
        v_x = 0
    else:
        v_x = v_x / np.linalg.norm(v_x)
    x = np.dot(vec, v_x)
    lt = corner_positions[left_top]
    v_y = lt - lb
    v_y = v_y - np.dot(v_y, v_x) * v_x
    if (np.linalg.norm(v_y) == 0):
        v_y = 0
    else:
        v_y = v_y / np.linalg.norm(v_y)
    y = np.dot(vec, v_y)
    return x, y

def GetRobotPosition(robot_pos, corner_positions, left_bottom, left_top, right_bottom, right_top, board_width, board_height):
    x_candidates = []
    y_candidates = []
    x, y = HelpGetRobotPosition(robot_pos, corner_positions, left_bottom, left_top, right_bottom)
    if x is not None and y is not None:
        x_candidates.append(x)
        y_candidates.append(y)
    x, y = HelpGetRobotPosition(robot_pos, corner_positions, left_top, left_bottom, right_top)
    if x is not None and y is not None:
        x_candidates.append(x)
        y_candidates.append(board_height - y)
    x, y = HelpGetRobotPosition(robot_pos, corner_positions, right_bottom, right_top, left_bottom)
    if x is not None and y is not None:
        x_candidates.append(board_width - x)
        y_candidates.append(y)
    x, y = HelpGetRobotPosition(robot_pos, corner_positions, right_top, right_bottom, left_top)
    if x is not None and y is not None:
        x_candidates.append(board_width - x)
        y_candidates.append(board_height - y)

    return np.mean(x_candidates), np.mean(y_candidates)

def main():
    data = np.load('../camera_set_up/calibration_result.npz') # результаты калибровки камеры
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
    # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    # time.sleep(2)
    # ok, frame = cap.read()
    # print(f"Размер кадра: {frame.shape}")
    # cap.release()

    image_path = 'фото.jpg'
    frame = cv2.imread(image_path)

    # new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (w, h), 0, (w, h))
    # undistorted = cv2.undistort(frame, camera_matrix, dist_coeffs, None, new_camera_matrix)
    # cv2.imshow('Undistorted', undistorted)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    # frame_viz = frame.copy()
    # gray = cv2.cvtColor(frame_viz, cv2.COLOR_BGR2GRAY)
    # corners_viz, ids_viz, _ = detector.detectMarkers(gray)
    # if ids_viz is not None:
    #     cv2.aruco.drawDetectedMarkers(frame_viz, corners_viz, ids_viz)
    # # кадр с маркерами
    # cv2.imshow('кадр с маркерами', frame_viz)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    if frame is None:
        print("Ошибка при чтении файла")
        return

    distance, width, height, corner_positions, robot_pos = GetBoardData(frame, camera_matrix, dist_coeffs, detector, left_bottom, left_top, right_top, right_bottom, robot)
    if distance is not None:
        print(f"Расстояние до доски: {distance:.4f} м")
        print(f"Размеры доски: {width:.4f} x {height:.4f} м")
        # ищем маркер робота
        if robot_pos is not None:
            robot_x, robot_y = GetRobotPosition(robot_pos, corner_positions, left_bottom, left_top, right_bottom, right_top, width, height)
            print(f"Координаты робота: x={robot_x:.4f} м, y={robot_y:.4f} м")
        else:
            print("Не удалось обнаружить робота")
    else:
        print("Не удалось обнаружить доску")

if __name__ == "__main__":
    main()
