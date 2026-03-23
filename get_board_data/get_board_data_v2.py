import cv2
import numpy as np
import time

MARKER_SIZE = 0.065  # marker size in mm
SMOOTH_ALPHA = 0.9   # the smoothing coefficient of the board

class SmoothBoard:
    def __init__(self, alpha):
        self.alpha = alpha
        self.origin = None
        self.x = None
        self.y = None
        self.normal = None

    def update(self, origin, board_x, board_y, board_normal):
        if self.origin is None:
            self.origin = origin
            self.x = board_x
            self.y = board_y
            self.normal = board_normal
            return origin, board_x, board_y, board_normal
        
        self.origin = self.alpha * self.origin + (1 - self.alpha) * origin
        self.x = self.alpha * self.x + (1 - self.alpha) * board_x
        self.y = self.alpha * self.y + (1 - self.alpha) * board_y
        self.normal = self.alpha * self.normal + (1 - self.alpha) * board_normal

        self.x = self.x / np.linalg.norm(self.x)
        self.y = self.y / np.linalg.norm(self.y)
        self.normal = self.normal / np.linalg.norm(self.normal)
        
        return self.origin, self.x, self.y, self.normal


def GetBoardData(frame, camera_matrix, dist_coeffs, detector, left_bottom, left_top, right_top, right_bottom):
    # get the coordinates of the corner markers and their IDs
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)
    if ids is not None:
        print(f"Найденные ID маркеров: {ids.flatten().tolist()}")
    else:
        return None, None, None, None, None, None

    # 3d coordinates of the marker's corners in its own coordinate system
    half = MARKER_SIZE / 2.0
    marker_obj_points = np.array([[-half, -half, 0], [half, -half, 0], [half, half, 0], [-half, half, 0]], dtype=np.float32)
    corner_ids = [left_bottom, left_top, right_top, right_bottom]

    # 3d positions of the corner marker centers
    positions = {}
    for corner_id in corner_ids:
        if corner_id in ids:
            idx = np.where(ids == corner_id)[0][0]
            marker_corners = corners[idx][0]
            success, rvec, tvec = cv2.solvePnP(marker_obj_points, marker_corners, camera_matrix, dist_coeffs)
            if success:
                positions[corner_id] = tvec.flatten()

    origin = positions[left_bottom]

    x_ref = positions[right_bottom]
    y_ref = positions[left_top]

    v_x = x_ref - origin
    board_x = v_x / np.linalg.norm(v_x)
    v_y = y_ref - origin
    v_y = v_y - np.dot(v_y, board_x) * board_x
    board_y = v_y / np.linalg.norm(v_y)
    board_normal = np.cross(board_x, board_y)
    board_normal = board_normal / np.linalg.norm(board_normal)

    # smoothing the board position
    smooth_board = SmoothBoard(SMOOTH_ALPHA)
    board_origin, board_x, board_y, board_normal = smooth_board.update(origin, board_x, board_y, board_normal)
    # estimating board sizes
    widths = []
    heights = []

    widths.append(np.linalg.norm(positions[left_bottom] - positions[right_bottom]))
    widths.append(np.linalg.norm(positions[left_top] - positions[right_top]))
    heights.append(np.linalg.norm(positions[left_bottom] - positions[left_top]))
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

def Visualize(frame, detector, robot_pos, robot_theta, robot_x, robot_y, camera_matrix, dist_coeffs):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)
    # draw all the markers found
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

    if robot_pos is not None:
        # robot's position
        robot_2d, _ = cv2.projectPoints(robot_pos.reshape(1,3), np.zeros((3,1)), np.zeros((3,1)), camera_matrix, dist_coeffs)
        robot_2d = tuple(robot_2d[0][0].astype(int))
        # circle around the robot
        cv2.circle(frame, robot_2d, 30, (0, 255, 0), 2)
        # arrow
        arrow_end = robot_pos + np.array([0.05*np.cos(robot_theta), 0.05*np.sin(robot_theta), 0])
        arrow_2d, _ = cv2.projectPoints(arrow_end.reshape(1,3), np.zeros((3,1)), np.zeros((3,1)), camera_matrix, dist_coeffs)
        cv2.arrowedLine(frame, robot_2d, tuple(arrow_2d[0][0].astype(int)), (0,255,255), 2)
        # text with coordinates
        cv2.putText(frame, f"x={robot_x:.3f} y={robot_y:.3f} th={robot_theta:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)

    cv2.imshow('Result', frame)
    cv2.waitKey(0)

def main():
    data = np.load('calibration_result6.yaml') # результаты калибровки камеры
    camera_matrix = data['camera_matrix']
    dist_coeffs = data['dist_coeffs']
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())

    # markers' ids
    left_bottom = 1
    left_top = 2
    right_top = 3
    right_bottom = 4
    robot = 7

    camera_index = 0
    cap = cv2.VideoCapture(camera_index)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 2160)
    time.sleep(2)
    ok, frame = cap.read()
    cap.release()

    print(f"Размер кадра: {frame.shape}")
    if frame is None:
        print("Ошибка при чтении файла")
        return

    board_origin, board_x, board_y, board_normal, width, height = GetBoardData(frame, camera_matrix, dist_coeffs, detector, left_bottom, left_top, right_top, right_bottom)
    if board_origin is None:
        print("Не удалось обнаружить доску")
        return

    print(f"Размеры доски: {width:.4f} x {height:.4f} м")

    # looking for a robot marker
    rvec_robot, robot_pos = DetectRobot(frame, camera_matrix, dist_coeffs, detector, robot)
    if robot_pos is not None:
        robot_x, robot_y, robot_theta = GetRobotState(robot_pos, rvec_robot, board_origin, board_x, board_y)
        print(f"Координаты робота: x={robot_x:.4f} м, y={robot_y:.4f} м, theta={robot_theta:.4f}")
    else:
        print("Не удалось обнаружить робота")

    Visualize(frame, detector, robot_pos, robot_theta, robot_x, robot_y, camera_matrix, dist_coeffs)

if __name__ == "__main__":
    main()
