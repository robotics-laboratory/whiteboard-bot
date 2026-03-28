import argparse
import keyboard
from socket import socket, AF_INET, SOCK_STREAM

parser = argparse.ArgumentParser(prog='pult')
parser.add_argument('ip')
args = parser.parse_args()

right_mod = 0.01
left_mod = 0.01

with socket(AF_INET, SOCK_STREAM) as client:
    client.connect((args.ip, 8080))
    right_speed = 0
    left_speed = 0
    while True:
        event = keyboard.read_event()
        print(event, right_speed, left_speed)
        if event.event_type == keyboard.KEY_DOWN:
            if event.name == "down" :
                right_speed -= right_mod
                left_speed -= left_mod
            elif event.name == "up":
                right_speed += right_mod
                left_speed += left_mod
            elif event.name == "left":
                left_speed += left_mod
                right_speed -= right_mod
            elif event.name == "right":
                right_speed += right_mod
                left_speed -= left_mod
            elif event.name == "r":
                right_speed = 0
                left_speed = 0
        last_left = left_speed
        last_right = right_speed
        s = f"L{left_speed}\n"
        print(client.sendall(s.encode('utf-8')))
        s = f"R{right_speed}\n"
        print(client.sendall(s.encode('utf-8')))

