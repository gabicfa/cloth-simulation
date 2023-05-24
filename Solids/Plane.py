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

        # The 'height' direction is chosen to be perpendicular to both the normal and the 'width' direction
        self.height_direction = np.cross(self.normal, self.width_direction)  # already normalized

        hw = self.width / 2.0
        hh = self.height / 2.0

        # Define vertices for the plane rectangle centered at the given point
        vertices = [
            self.center - hw * self.width_direction - hh * self.height_direction,
            self.center + hw * self.width_direction - hh * self.height_direction,
            self.center + hw * self.width_direction + hh * self.height_direction,
            self.center - hw * self.width_direction + hh * self.height_direction
        ]

        # Define triangles for the rectangle
        # Make sure to specify vertices in counter-clockwise order
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
        # We only consider non-uniform scaling along the width and height directions
        if isinstance(factor, (list, tuple, np.ndarray)) and len(factor) == 2:
            self.width *= factor[0]
            self.height *= factor[1]
        else:  # uniform scaling
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

    def point_inside(self, p, margin=0.01):
        # Assume plane_normal is the normal vector of the plane
        plane_normal = self.triangles[0].normal
        
        # Calculate expanded vertices for each triangle
        expanded_triangles = []
        for triangle in self.triangles:
            expanded_vertices = [vertex + margin * plane_normal for vertex in triangle.vertices]
            expanded_triangle = Triangle(*expanded_vertices)
            expanded_triangles.append(expanded_triangle)
            
        # Check if the point is inside the expanded triangles
        return any(triangle.point_inside(p) for triangle in expanded_triangles)

