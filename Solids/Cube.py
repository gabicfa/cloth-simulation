import numpy as np
from Solids.Triangle import Triangle
from Solids.Solid import Solid

class Cube(Solid):
    def __init__(self, center=[0.0, 0.0, 0.0], size=1.0):
        super().__init__(center)
        self.size = size
        self.center = np.array(center)

        hs = size / 2.0

        # Define vertices for the cube centered at the origin
        vertices = [
            center + np.array([-hs, -hs, -hs]),
            center + np.array([hs, -hs, -hs]),
            center + np.array([hs, hs, -hs]),
            center + np.array([-hs, hs, -hs]),
            center + np.array([-hs, -hs, hs]),
            center + np.array([hs, -hs, hs]),
            center + np.array([hs, hs, hs]),
            center + np.array([-hs, hs, hs])
        ]

        self.vertices= vertices

        # Define triangles for each face of the cube
        # Vertices are specified in counter-clockwise order
        self.triangles = [
            Triangle(self.vertices[0], self.vertices[3], self.vertices[1]), Triangle(self.vertices[1], self.vertices[3], self.vertices[2]),  # Front face
            Triangle(self.vertices[5], self.vertices[6], self.vertices[4]), Triangle(self.vertices[4], self.vertices[6], self.vertices[7]),  # Back face
            Triangle(self.vertices[4], self.vertices[7], self.vertices[0]), Triangle(self.vertices[0], self.vertices[7], self.vertices[3]),  # Left face
            Triangle(self.vertices[1], self.vertices[2], self.vertices[5]), Triangle(self.vertices[5], self.vertices[2], self.vertices[6]),  # Right face
            Triangle(self.vertices[4], self.vertices[0], self.vertices[5]), Triangle(self.vertices[5], self.vertices[0], self.vertices[1]),  # Bottom face
            Triangle(self.vertices[3], self.vertices[7], self.vertices[2]), Triangle(self.vertices[2], self.vertices[7], self.vertices[6])   # Top face
        ]

        # Calculate the minimum and maximum coordinates of the cube
        self.min_coords = np.min(self.vertices, axis=0)
        self.max_coords = np.max(self.vertices, axis=0)

    def translate(self, v):
        self.center += v
        for vertex in self.vertices:
            vertex += v
        for triangle in self.triangles:
            triangle.translate(v)

        self.min_coords += v
        self.max_coords += v

    def rotate(self, axis, angle):
        rotation_matrix = super().rotation_matrix(axis, angle)
        self.center = np.dot(rotation_matrix, self.center)
        self.vertices = [np.dot(rotation_matrix, vertex) for vertex in self.vertices]
        for triangle in self.triangles:
            triangle.rotate(axis, angle)

        self.min_coords = np.min(self.vertices, axis=0)
        self.max_coords = np.max(self.vertices, axis=0)

    def scale(self, factor):
        factor = np.array(factor)
        self.center *= factor
        self.size *= factor
        for vertex in self.vertices:
            vertex *= factor
        for triangle in self.triangles:
            triangle.scale(factor)

        self.min_coords *= factor
        self.max_coords *= factor

    def point_inside(self, p, margin=0.01):
        min_coords_with_margin = self.min_coords - margin
        max_coords_with_margin = self.max_coords + margin

        return np.all(min_coords_with_margin <= p) and np.all(p <= max_coords_with_margin)