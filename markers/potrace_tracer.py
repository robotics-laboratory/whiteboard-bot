import math
import time
import turtle
from pathlib import Path

import potrace
from PIL import Image

# part I: generate polylines
input_img = Path("") # write the path to your file
timestamp = str(int(time.time() * 1000))
output_txt = input_img.parent / f"{timestamp}_potrace_lines.txt"

img = Image.open(input_img).convert('L')
img = Image.eval(img, lambda x: 255 - x)

bitmap = potrace.Bitmap(img, blacklevel=0.5)
path = bitmap.trace(turdsize=2, alphamax=1, opticurve=False)

polylines = []
for curve in path:
    polyline = []
    polyline.append((curve.start_point.x, curve.start_point.y))
    for segment in curve.segments:
        if segment.is_corner:
            if hasattr(segment, 'c'):
                polyline.append((segment.c.x, segment.c.y))
            polyline.append((segment.end_point.x, segment.end_point.y))
        else:
            polyline.append((segment.end_point.x, segment.end_point.y))
    
    polylines.append(polyline)

# with open(output_txt, "w") as f:
#     for polyline in polylines:
#         # (x1, y1); (x2, y2); (x3, y3)
#         points_str = "; ".join([f"({x:.2f}, {y:.2f})" for x, y in polyline])
#         f.write(points_str + "\n")

print(f"Всего {len(polylines)} ломаных")

# part II: generating commands for robot
current_x, current_y = 0.0, 0.0
current_angle = 0.0  # angle in degrees

def normalize_angle(angle):
    angle = angle % 360
    if angle > 180:
        angle -= 360
    return angle

def angle_to_target(x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1
    return math.degrees(math.atan2(dy, dx))

def distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

# list of commands for robot:
# 1) up
# 2) down
# 3) forward x
# 4) rotate w
commands = []

# start with "up"
commands.append("up")

for polyline in polylines:
    if not polyline:
        continue

    # go to the first dot of the polyline
    target_x, target_y = polyline[0]

    # rotate to the goal dot
    target_angle = angle_to_target(current_x, current_y, target_x, target_y)
    angle_diff = normalize_angle(target_angle - current_angle)
    if abs(angle_diff) > 0.01:
        commands.append(f"rotate {angle_diff:.2f}")
        current_angle = target_angle

    dist = distance(current_x, current_y, target_x, target_y)
    if dist > 0.01:
        commands.append(f"forward {dist:.2f}")
        current_x, current_y = target_x, target_y

    commands.append("down")

    # go to remaining dots
    for i in range(1, len(polyline)):
        target_x, target_y = polyline[i]

        # rotate to the next dot
        target_angle = angle_to_target(current_x, current_y, target_x, target_y)
        angle_diff = normalize_angle(target_angle - current_angle)
        if abs(angle_diff) > 0.01:
            commands.append(f"rotate {angle_diff:.2f}")
            current_angle = target_angle

        # go to the next dot
        dist = distance(current_x, current_y, target_x, target_y)
        if dist > 0.01:
            commands.append(f"forward {dist:.2f}")
            current_x, current_y = target_x, target_y

    commands.append("up")

# for cmd in commands:
#     print(cmd)

# check robot's plan with turtle
t = turtle.Turtle()
t.speed(0)
t.penup()
t.goto(-200, 200)
for cmd in commands:
    if cmd == "up":
        t.penup()
    elif cmd == "down":
        t.pendown()
    elif cmd.startswith("forward"):
        dist = float(cmd.split()[1])
        t.forward(dist)
    elif cmd.startswith("rotate"):
        angle = float(cmd.split()[1])
        t.right(angle)

turtle.done()
