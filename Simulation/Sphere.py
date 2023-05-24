import numpy as np
from Simulation.Triangle import Triangle
from Simulation.Solid import Solid

class Sphere(Solid):
    def __init__(self, center=[0.0, 0.0, 0.0], radius=0.5):
        super().__init__(center)

        self.radius = radius

        # Create initial icosahedron
        phi = (1 + np.sqrt(5)) / 2  # golden ratio
        u = 1 / np.sqrt(phi**2 + 1)  # coordinate on unit sphere
        v = phi * u  # coordinate on unit sphere

        vertices = [
            np.array([-u, v, 0]),
            np.array([u, v, 0]),
            np.array([-u, -v, 0]),
            np.array([u, -v, 0]),
            np.array([0, -u, v]),
            np.array([0, u, v]),
            np.array([0, -u, -v]),
            np.array([0, u, -v]),
            np.array([v, 0, -u]),
            np.array([v, 0, u]),
            np.array([-v, 0, -u]),
            np.array([-v, 0, u])
        ]

        # Scale vertices by radius and move to center
        self.vertices = [center + radius * vertex for vertex in vertices]

        # Define triangles (each face of the icosahedron) by their vertex indices
        triangles_indices = [
            (0, 11, 5), (0, 5, 1), (0, 1, 7), (0, 7, 10), (0, 10, 11),
            (1, 5, 9), (5, 11, 4), (11, 10, 2), (10, 7, 6), (7, 1, 8),
            (3, 9, 4), (3, 4, 2), (3, 2, 6), (3, 6, 8), (3, 8, 9),
            (4, 9, 5), (2, 4, 11), (6, 2, 10), (8, 6, 7), (9, 8, 1)
        ]

        # Generate triangle objects
        self.triangles = [
            Triangle(self.vertices[i], self.vertices[j], self.vertices[k])
            for (i, j, k) in triangles_indices
        ]

    def translate(self, translation):
        self.center += translation
        self.vertices = [vertex + translation for vertex in self.vertices]
        for triangle in self.triangles:
            triangle.translate(translation)

    def rotate(self, axis, angle):
        rotation_matrix = super().rotation_matrix(axis, angle)
        self.center = rotation_matrix @ self.center
        self.vertices = [rotation_matrix @ vertex for vertex in self.vertices]
        for triangle in self.triangles:
            triangle.rotate(axis, angle)

    def scale(self, factor):
        self.radius *= factor
        self.vertices = [self.center + factor * (vertex - self.center) for vertex in self.vertices]
        for triangle in self.triangles:
            triangle.scale([factor, factor, factor])

    def point_inside(self, p, margin=0.8):
        distance = np.linalg.norm(self.center - p)
        result = distance <= (self.radius + margin)
        return result