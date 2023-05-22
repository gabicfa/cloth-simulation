from abc import ABC, abstractmethod
import numpy as np

class Solid(ABC):
    def __init__(self, center):
        self.center = np.array(center)
        self.vertices = []
        self.triangles = []

    @abstractmethod
    def point_inside(self, p, margin):
        pass

    @staticmethod
    def rotation_matrix(axis, angle):
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
        return rotation_matrix