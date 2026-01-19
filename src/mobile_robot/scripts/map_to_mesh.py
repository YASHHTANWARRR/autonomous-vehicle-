#!/usr/bin/env python3
import cv2
import shapely.geometry as geom
import shapely.ops as ops
import trimesh

MAP = "maps/track_map.pgm"
RES = 0.05
HEIGHT = 1.5

img = cv2.imread(MAP, 0)
if img is None:
    raise RuntimeError("Could not load track_map.pgm")

_, bw = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY_INV)

contours, _ = cv2.findContours(bw, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

polys = []
for c in contours:
    pts = [(p[0][0] * RES, p[0][1] * RES) for p in c]
    if len(pts) > 3:
        polys.append(geom.Polygon(pts))

walls = ops.unary_union(polys)

mesh = trimesh.creation.extrude_polygon(walls, HEIGHT)
mesh.export("models/generated_walls/track_walls.stl")

print("✔ track_walls.stl generated")
