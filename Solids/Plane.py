import numpy as np
from Solids.Solid import Solid
from Solids.Triangle import Triangle

class Plane(Solid):
    def __init__(self, center=[0.0, 0.0, 0.0], width=1.0, height=1.0, normal=[0.0, 0.0, 1.0]):
        super().__init__(center)

        self.center = np.array(center)
        self.normal = np.array(normal) / np.linalg.norm(normal)
        self.width = width
        self.height = height

        # Arbitrary perpendicular direction to the normal to be 'width' direction
        self.width_direction = np.array([-self.normal[1], self.normal[0], 0.0])
        if np.linalg.norm(self.width_direction) < 1e-10:  # the normal was parallel to z-axis
            self.width_direction = np.array([1.0, 0.0, 0.0])  # choose x-axis as width direction
        self.width_direction /= np.linalg.norm(self.width_direction)  # normalize

        # The 'height' direction is perpendicular to both the normal and the 'width' direction
        self.height_direction = np.cross(self.normal, self.width_direction)

        hw = self.width / 2.0
        hh = self.height / 2.0

        vertices = [
            self.center - hw * self.width_direction - hh * self.height_direction,
            self.center + hw * self.width_direction - hh * self.height_direction,
            self.center + hw * self.width_direction + hh * self.height_direction,
            self.center - hw * self.width_direction + hh * self.height_direction
        ]

        self.triangles = [
            Triangle(vertices[0], vertices[1], vertices[2]),
            Triangle(vertices[0], vertices[2], vertices[3])
        ]

    def translate(self, v):
        self.center += v
        for triangle in self.triangles:
            triangle.translate(v)

    def rotate(self, axis, angle):
        rotation_matrix = super().rotation_matrix(axis, angle)
        self.center = np.dot(rotation_matrix, self.center)
        self.normal = np.dot(rotation_matrix, self.normal)
        self.width_direction = np.dot(rotation_matrix, self.width_direction)
        self.height_direction = np.dot(rotation_matrix, self.height_direction)
        for triangle in self.triangles:
            triangle.rotate(axis, angle)

    def scale(self, factor):
        # uniform scaling
        self.width *= factor
        self.height *= factor

        # Adjust the vertices of the triangles accordingly
        hw = self.width / 2.0
        hh = self.height / 2.0
        vertices = [
            self.center - hw * self.width_direction - hh * self.height_direction,
            self.center + hw * self.width_direction - hh * self.height_direction,
            self.center + hw * self.width_direction + hh * self.height_direction,
            self.center - hw * self.width_direction + hh * self.height_direction
        ]
        self.triangles = [
            Triangle(vertices[0], vertices[1], vertices[2]),
            Triangle(vertices[0], vertices[2], vertices[3])
        ]

    def point_inside(self, p):
        return any(triangle.point_inside(p) for triangle in self.triangles)
