import numpy as np

from Solids.Solid import Solid
class Triangle:
    def __init__(self, p1 =[0.0, 0.0, 0.0], p2 = [1.0, 0.0, 0.0], p3 = [0.0, 1.0, 0.0]):
        self.p1 = np.array(p1)
        self.p2 = np.array(p2)
        self.p3 = np.array(p3)
        self.vertices = [self.p1, self.p2, self.p3]
        self.normal = self.calculate_normal()

    def calculate_normal(self):
        # Compute the normal using cross product
        edge1 = self.p2 - self.p1
        edge2 = self.p3 - self.p1
        normal = np.cross(edge1, edge2)
        normal /= np.linalg.norm(normal)
        return normal

    def point_inside(self, p):
        # Check if point p lies inside the triangle using barycentric coordinates
        v0 = self.p2 - self.p1
        v1 = self.p3 - self.p1
        v2 = p - self.p1

        dot00 = np.dot(v0, v0)
        dot01 = np.dot(v0, v1)
        dot02 = np.dot(v0, v2)
        dot11 = np.dot(v1, v1)
        dot12 = np.dot(v1, v2)

        # Compute barycentric coordinates
        invDenom = 1 / (dot00 * dot11 - dot01 * dot01)
        u = (dot11 * dot02 - dot01 * dot12) * invDenom
        v = (dot00 * dot12 - dot01 * dot02) * invDenom

        # Check if point is in triangle
        return (u >= 0) and (v >= 0) and (u + v < 1)

    def rotate(self, axis, angle):
        R = Solid.rotation_matrix(axis, angle)
        self.p1 = np.dot(R, self.p1)
        self.p2 = np.dot(R, self.p2)
        self.p3 = np.dot(R, self.p3)
        self.vertices = [self.p1, self.p2, self.p3]
        self.normal = self.calculate_normal() 

    
    def translate(self, vector):
        self.p1 += np.array(vector)
        self.p2 += np.array(vector)
        self.p3 += np.array(vector)
        self.vertices = [self.p1, self.p2, self.p3]
        self.normal = self.calculate_normal()
    
    def scale(self, factor):
        self.p1 *= np.array(factor)
        self.p2 *= np.array(factor)
        self.p3 *= np.array(factor)
        self.vertices = [self.p1, self.p2, self.p3]
        self.normal = self.calculate_normal()