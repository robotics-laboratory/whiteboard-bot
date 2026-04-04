import cv2
import os

DICT = cv2.aruco.DICT_6X6_250
CELLS_X, CELLS_Y = 7,10
CELL_WIDTH, MARKER_WIDTH = 0.049, 0.022

DICT = cv2.aruco.getPredefinedDictionary(DICT)
BOARD = cv2.aruco.CharucoBoard((CELLS_X, CELLS_Y), CELL_WIDTH, MARKER_WIDTH, DICT)
BOARD.setLegacyPattern(True)

image_folder = "path"
image_extensions = ['.jpg', '.jpeg', '.png', '.bmp', '.tiff']
image_files = [f for f in os.listdir(image_folder) 
               if os.path.splitext(f)[1].lower() in image_extensions]
image_files.sort()

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

    height, width = frame.shape[:2]
    
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aru_corners, aru_ids, _ = cv2.aruco.detectMarkers(gray, DICT)
    
    char_corners = None
    char_ids = None
    
    if aru_corners:
        ok, char_corners, char_ids = cv2.aruco.interpolateCornersCharuco(aru_corners, aru_ids, gray, BOARD)

    good_frame = aru_corners and char_ids is not None and len(char_corners) > 6

    if good_frame:
        all_corners.append(char_corners)
        all_ids.append(char_ids)

print(f"Число кадров: {len(all_corners)}")
rms, matrix, dist_coefs, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
    all_corners, all_ids, BOARD, (height, width), None, None
)

print("RMS:", rms)
print("CAMERA MATRIX:\n", matrix.tolist())
print("DISTORTION COEFFICIENTS:\n", dist_coefs.ravel().tolist())

calibration_data = {
    'calib_size': [width, height],
    'matrix': matrix.tolist(),
    'distortion': dist_coefs.ravel().tolist(),
    'rms': float(rms)
}

# with open('calibration_result6.yaml', 'w') as f:
#     yaml.dump(calibration_data, f, default_flow_style=None)
