import numpy as np

class Particle:
    def __init__(self, mass, position, is_fixed = False):
        self.mass = mass
        self.position = position
        self.velocity = np.zeros_like(position)
        self.force = np.zeros_like(position)
        self.initial_position = np.copy(position)
        self.is_fixed = is_fixed

class Spring:
    def __init__(self, particle1, particle2, stiffness):
        self.particle1 = particle1
        self.particle2 = particle2
        self.stiffness = stiffness
        self.rest_length = np.linalg.norm(particle2.position - particle1.position)
class Triple:
    def __init__(self, p1, p2, p3):
        self.p1 = p1
        self.p2 = p2
        self.p3 = p3
        vector1 = p2.initial_position - p1.initial_position
        vector2 = p3.initial_position - p2.initial_position
        cosine_angle = np.dot(vector1, vector2) / (np.linalg.norm(vector1) * np.linalg.norm(vector2))
        self.initial_angle = np.arccos(cosine_angle)

class Quadruple:
    def __init__(self, p1, p2, p3, p4):
        self.p1 = p1
        self.p2 = p2
        self.p3 = p3
        self.p4 = p4
        self.L_shear = np.linalg.norm((self.p2.position - self.p1.position) + (self.p4.position - self.p3.position))