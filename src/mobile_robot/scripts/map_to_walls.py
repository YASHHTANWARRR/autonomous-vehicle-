#!/usr/bin/env python3
import cv2
import numpy as np
from pathlib import Path
import math

MAP_FILE = "maps/track_map.pgm"
MAP_RES = 0.05
WALL_HEIGHT = 1.5
WALL_THICKNESS = 0.2

img = cv2.imread(MAP_FILE, 0)
assert img is not None, "Map not found"

_, bw = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY_INV)
contours, _ = cv2.findContours(bw, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

out = Path("models/generated_walls/model.sdf")
out.parent.mkdir(parents=True, exist_ok=True)

with open(out, "w") as f:
    f.write('<sdf version="1.9">\n')
    f.write('<model name="generated_walls">\n')
    f.write('<static>true</static>\n')

    for i, c in enumerate(contours):
        for j in range(len(c)-1):
            p1 = c[j][0]
            p2 = c[j+1][0]

            dx = (p2[0] - p1[0]) * MAP_RES
            dy = (p2[1] - p1[1]) * MAP_RES
            length = math.hypot(dx, dy)
            if length < 0.05:
                continue

            x = (p1[0] + p2[0]) * 0.5 * MAP_RES
            y = (p1[1] + p2[1]) * 0.5 * MAP_RES
            yaw = math.atan2(dy, dx)

            f.write(f"""
<link name="wall_{i}_{j}">
  <pose>{x:.3f} {y:.3f} {WALL_HEIGHT/2} 0 0 {yaw:.3f}</pose>
  <collision name="collision">
    <geometry>
      <box>
        <size>{length:.3f} {WALL_THICKNESS} {WALL_HEIGHT}</size>
      </box>
    </geometry>
  </collision>
  <visual name="visual">
    <geometry>
      <box>
        <size>{length:.3f} {WALL_THICKNESS} {WALL_HEIGHT}</size>
      </box>
    </geometry>
    <material>
      <ambient>0.3 0.3 0.3 1</ambient>
    </material>
  </visual>
</link>
""")

    f.write('</model>\n</sdf>\n')

print("✔ Gazebo wall model generated")

