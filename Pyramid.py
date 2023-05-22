import numpy as np
from Triangle import Triangle
from Solid import Solid

class Pyramid(Solid):
    def __init__(self, center=[0.0, 0.0, 0.0], base=8, height=0.5, radius = 0.5):
        super().__init__(center)

        self.height = height
        self.base = base  # number of sides in the base
        self.center = np.array(center)

        # Compute the coordinates of the apex of the pyramid
        apex = center + np.array([0, 0, height])

        # Generate the vertices around the base of the pyramid
        self.radius = radius
        base_vertices = []
        for i in range(base):
            angle = 2 * np.pi * i / base
            x = self.radius * np.cos(angle)
            y = self.radius * np.sin(angle)
            z = 0
            base_vertices.append(center + np.array([x, y, z]))

        self.vertices = base_vertices + [apex]

        # Create triangles from the base vertices to the apex
        self.triangles = []
        for i in range(base):
            v1 = base_vertices[i]
            v2 = base_vertices[(i+1) % base]  # wrap around at the end
            self.triangles.append(Triangle(v1, v2, apex))

        # Calculate the minimum and maximum coordinates of the pyramid
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
        self.height *= factor[2]  # Assume that the z component of the factor scales the height

        for i in range(len(self.vertices)):
            self.vertices[i] *= factor

        for triangle in self.triangles:
            triangle.scale(factor)

        self.min_coords *= factor
        self.max_coords *= factor

    def point_inside(self, p, margin=0.1):
        min_coords_with_margin = self.min_coords - margin
        max_coords_with_margin = self.max_coords + margin

        return np.all(min_coords_with_margin <= p) and np.all(p <= max_coords_with_margin)