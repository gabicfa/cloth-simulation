import numpy as np
from Triangle import Triangle

class Cube:
    def __init__(self, center = [0.0,0.0,0.0], size = 1.0):
        self.size = size
        self.center = np.array(center)

        # Compute half size for convenience
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
        # Make sure to specify vertices in counter-clockwise order
        self.triangles = [
            Triangle(vertices[0], vertices[3], vertices[1]), Triangle(vertices[1], vertices[3], vertices[2]),  # Front face
            Triangle(vertices[5], vertices[6], vertices[4]), Triangle(vertices[4], vertices[6], vertices[7]),  # Back face
            Triangle(vertices[4], vertices[7], vertices[0]), Triangle(vertices[0], vertices[7], vertices[3]),  # Left face
            Triangle(vertices[1], vertices[2], vertices[5]), Triangle(vertices[5], vertices[2], vertices[6]),  # Right face
            Triangle(vertices[4], vertices[0], vertices[5]), Triangle(vertices[5], vertices[0], vertices[1]),  # Bottom face
            Triangle(vertices[3], vertices[7], vertices[2]), Triangle(vertices[2], vertices[7], vertices[6])   # Top face
        ]

        # Calculate the minimum and maximum coordinates of the cube
        self.min_coords = self.vertices[0].copy()
        self.max_coords = self.vertices[0].copy()
        for v in self.vertices:
            self.min_coords = np.minimum(self.min_coords, v)
            self.max_coords = np.maximum(self.max_coords, v)

    def rotate_point(self, point, angle, axis):
        angle = np.radians(angle)
        axis = np.array(axis)
        axis = axis/np.sqrt(np.dot(axis, axis))
        a = np.cos(angle / 2.0)
        b, c, d = -axis * np.sin(angle / 2.0)
        aa, bb, cc, dd = a * a, b * b, c * c, d * d
        bc, ad, ac, ab, bd, cd = b * c, a * d, a * c, a * b, b * d, c * d
        rotation_matrix = np.array([[aa + bb - cc - dd, 2 * (bc + ad), 2 * (bd - ac)],
                                    [2 * (bc - ad), aa + cc - bb - dd, 2 * (cd + ab)],
                                    [2 * (bd + ac), 2 * (cd - ab), aa + dd - bb - cc]])
        return np.dot(rotation_matrix, point)

    def translate(self, v):
        self.center += v
        for vertex in self.vertices:
            vertex += v
        for triangle in self.triangles:
            triangle.translate(v)

        # Update the minimum and maximum coordinates
        self.min_coords += v
        self.max_coords += v

    def rotate(self, axis, angle):
        self.center = self.rotate_point(self.center, axis, angle)
        for vertex in self.vertices:
            vertex = self.rotate_point(vertex, axis, angle)
        for triangle in self.triangles:
            triangle.rotate(axis, angle)

        # Update the minimum and maximum coordinates
        self.min_coords = self.vertices[0].copy()
        self.max_coords = self.vertices[0].copy()
        for v in self.vertices:
            self.min_coords = np.minimum(self.min_coords, v)
            self.max_coords = np.maximum(self.max_coords, v)

    def scale(self, factor):
        factor = np.array(factor)  # Convert factor to a numpy array
        self.center *= factor
        self.size *= factor
        for vertex in self.vertices:
            vertex *= factor
        for triangle in self.triangles:
            triangle.scale(factor)

        # Update the minimum and maximum coordinates
        self.min_coords *= factor
        self.max_coords *= factor

    def point_inside(self, p, margin=0.5):
        min_coords_with_margin = self.min_coords - margin
        max_coords_with_margin = self.max_coords + margin

        return np.all(min_coords_with_margin <= p) and np.all(p <= max_coords_with_margin)