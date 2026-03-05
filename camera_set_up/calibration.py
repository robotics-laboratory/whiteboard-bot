import cv2
import numpy as np
import os

DICT = cv2.aruco.DICT_6X6_250
CELLS_X, CELLS_Y = 7,10
CELL_WIDTH, MARKER_WIDTH = 0.049, 0.022

DICT = cv2.aruco.getPredefinedDictionary(DICT)
BOARD = cv2.aruco.CharucoBoard((CELLS_X, CELLS_Y), CELL_WIDTH, MARKER_WIDTH, DICT)
BOARD.setLegacyPattern(True)

image_folder = "/Users/vmn/Downloads/фото2"
image_extensions = ['.jpg', '.jpeg', '.png', '.bmp', '.tiff']
image_files = [f for f in os.listdir(image_folder) 
               if os.path.splitext(f)[1].lower() in image_extensions]
image_files.sort()

# Целевое разрешение
TARGET_WIDTH = 1942
TARGET_HEIGHT = 1296

all_corners = []
all_ids = []
current = 0
skip = 1

for img_file in image_files:
    current += 1
    if current % skip: 
        continue
    
    img_path = os.path.join(image_folder, img_file)
    frame = cv2.imread(img_path)
    if frame is None:
        continue

    if frame.shape[1] != TARGET_WIDTH or frame.shape[0] != TARGET_HEIGHT:
        frame = cv2.resize(frame, (TARGET_WIDTH, TARGET_HEIGHT))

    height, width = frame.shape[:2]
    
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aru_corners, aru_ids, _ = cv2.aruco.detectMarkers(gray, DICT)
    
    char_corners = None
    char_ids = None
    
    if aru_corners:
        ok, char_corners, char_ids = cv2.aruco.interpolateCornersCharuco(aru_corners, aru_ids, gray, BOARD)

    good_frame = aru_corners and char_ids is not None

    if good_frame:
        all_corners.append(char_corners)
        all_ids.append(char_ids)

print(f"Число кадров: {len(all_corners)}")
rms, matrix, dist_coefs, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
    all_corners, all_ids, BOARD, (TARGET_WIDTH, TARGET_HEIGHT), None, None
)

print("RMS:", rms)
print("CAMERA MATRIX:\n", matrix.tolist())
print("DISTORTION COEFFICIENTS:\n", dist_coefs.ravel().tolist())
np.savez('calibration_result.npz', camera_matrix=matrix, dist_coeffs=dist_coefs, rms=rms)
