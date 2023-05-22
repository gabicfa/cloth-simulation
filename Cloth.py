from ClothElements import Particle, Spring, Quadruple, Triple
from scipy.sparse import lil_matrix, identity
from scipy.sparse.linalg import cg
import numpy as np
from Solid import Solid
class Cloth:
    def __init__(self, num_particles_x = 20, num_particles_y = 20, cloth_width = 3, cloth_height = 1):
        self.particles = []
        self.springs = []
        self.springs_dict = {}
        self.triples = []
        self.quadruples = []

        # self.damping = 1
        # self.mass = 1
        # self.stiffness = 20 * num_particles_x * num_particles_y 
        # self.shearing_stiffness = 10
        # self.bending_stiffness = 5
        # self.rotation_angle_x = 90
        # self.friction_coefficient = 10
        # self.restitution_coefficient = 0.1

        self.damping = 1
        self.mass = 1
        self.stiffness = 20 * num_particles_x * num_particles_y 
        self.shearing_stiffness = 10
        self.bending_stiffness = 5
        self.friction_coefficient = 10
        self.restitution_coefficient = 0.1

        # --------- SETUP --------- #

        # Create particles
        for j in range(num_particles_y):
            for i in range(num_particles_x):
                is_fixed = False
                # if (j == num_particles_y-1) :
                #     is_fixed = True
                position = np.array([cloth_width * i / (num_particles_x - 1), cloth_height * j / (num_particles_y - 1), 0.0])
                particle = Particle(self.mass, position, is_fixed)
                self.particles.append(particle)
    
        # Create springs
        for j in range(num_particles_y):
            for i in range(num_particles_x):
                # Structural springs
                if i < num_particles_x - 1:
                    particle1 = self.particles[i + j * num_particles_x]
                    particle2 = self.particles[(i + 1) + j * num_particles_x]
                    spring = Spring(particle1, particle2, self.stiffness)
                    self.add_spring(particle1, particle2, spring)
                if j < num_particles_y - 1:
                    particle1 = self.particles[i + j * num_particles_x]
                    particle2 = self.particles[i + (j + 1) * num_particles_x]
                    spring = Spring(particle1, particle2, self.stiffness)
                    self.add_spring(particle1, particle2, spring)
                
                # Diagonal springs
                if i < num_particles_x - 1 and j < num_particles_y - 1:
                    # Add a spring between the current particle and the one diagonally down-right from it
                    particle1 = self.particles[i + j * num_particles_x]
                    particle2 = self.particles[(i + 1) + (j + 1) * num_particles_x]
                    spring = Spring(particle1, particle2, self.stiffness)
                    self.add_spring(particle1, particle2, spring)

                if i > 0 and j < num_particles_y - 1:
                    # Add a spring between the current particle and the one diagonally down-left from it
                    particle1 = self.particles[i + j * num_particles_x]
                    particle2 = self.particles[(i - 1) + (j + 1) * num_particles_x]
                    spring = Spring(particle1, particle2, self.stiffness)
                    self.add_spring(particle1, particle2, spring)

        #Create Triples
        for j in range(num_particles_y):
            for i in range(num_particles_x - 2):
                triple = Triple(self.particles[i + j * num_particles_x], 
                        self.particles[(i + 1) + j * num_particles_x], 
                        self.particles[(i + 2) + j * num_particles_x])
                self.triples.append(triple)
        
        #Create Quadruples
        for j in range(num_particles_y - 1):
            for i in range(num_particles_x - 1):
                quadruple = Quadruple(self.particles[i + j * num_particles_x], 
                                      self.particles[(i + 1) + j * num_particles_x],
                                      self.particles[i + (j + 1) * num_particles_x],
                                      self.particles[(i + 1) + (j + 1) * num_particles_x])
                self.quadruples.append(quadruple)
    
    def add_spring(self, particle1, particle2, spring):
        self.springs.append(spring)
        key = frozenset([particle1, particle2])
        self.springs_dict[key] = spring
    
    def find_spring(self, particle1, particle2):
        key = frozenset([particle1, particle2])
        if key in self.springs_dict:
            return self.springs_dict[key]
        else:
            return None
    
    def translate(self, vector):
        vector = np.array(vector)
        for particle in self.particles:
            particle.position += vector

    def rotate(self, axis, angle):
        axis = np.array(axis)
        rotation_matrix = Solid.rotation_matrix(axis, angle)

        for particle in self.particles:
            particle.position = np.dot(rotation_matrix, particle.position)

    # --------- FORCES --------- #

    def apply_shearing_forces(self, quadruple):
        p1, p2, p3, p4 = quadruple.p1, quadruple.p2, quadruple.p3, quadruple.p4
        L_shear = quadruple.L_shear
        C_shear = np.linalg.norm((p2.position - p1.position) + (p4.position - p3.position)) - L_shear
        C_dot_shear = np.linalg.norm((p2.velocity - p1.velocity) + (p4.velocity - p3.velocity))

        grad_C_shear_p1 = -(p2.position - p1.position + p4.position - p3.position) / L_shear
        grad_C_shear_p2 = (p2.position - p1.position + p4.position - p3.position) / L_shear
        grad_C_shear_p3 = -(p2.position - p1.position + p4.position - p3.position) / L_shear
        grad_C_shear_p4 = (p2.position - p1.position + p4.position - p3.position) / L_shear
        p1.force += -self.shearing_stiffness * C_shear * grad_C_shear_p1 - self.damping * C_dot_shear * grad_C_shear_p1
        p2.force += -self.shearing_stiffness * C_shear * grad_C_shear_p2 - self.damping * C_dot_shear * grad_C_shear_p2
        p3.force += -self.shearing_stiffness * C_shear * grad_C_shear_p3 - self.damping * C_dot_shear * grad_C_shear_p3
        p4.force += -self.shearing_stiffness * C_shear * grad_C_shear_p4 - self.damping * C_dot_shear * grad_C_shear_p4

    def apply_bending_forces(self, triple):
        particle1 = triple.p1
        particle2 = triple.p2
        particle3 = triple.p3
        C_dot_bending = np.linalg.norm(particle2.velocity - particle1.velocity - particle3.velocity)
        v1 = particle2.position - particle1.position
        v2 = particle3.position - particle2.position
        bending_force = self.bending_stiffness * (v1 - v2)
        particle1.force -= bending_force - self.damping * C_dot_bending
        particle3.force += bending_force - self.damping * C_dot_bending

    def apply_stretch_forces(self, spring):
        current_length = np.linalg.norm(spring.particle1.position - spring.particle2.position)
        C_dot_stretch = np.linalg.norm(spring.particle1.velocity - spring.particle2.velocity)

        stretch_force = spring.stiffness * (current_length - spring.rest_length)
        force_direction = (spring.particle1.position - spring.particle2.position) / current_length
        norm = np.linalg.norm(force_direction)
        if norm > 0 :  # Check norm before normalization
            force_direction /= norm
        
        spring.particle1.force -= stretch_force * force_direction - self.damping * C_dot_stretch * force_direction
        spring.particle2.force += stretch_force * force_direction - self.damping * C_dot_stretch * force_direction

    def apply_external_forces(self):
        gravity = np.array([0, -9.81, 0])  # Define the gravitational force vector
        for particle in self.particles:
            particle.force += particle.mass * gravity

            # Apply drag force
            drag_coefficient = 0.3
            drag_force = -drag_coefficient * particle.velocity
            particle.force += drag_force

    # --------- CONSTRAINTS --------- #

    def stretching_constraint(self, spring):
        current_length = np.linalg.norm(spring.particle1.position - spring.particle2.position)
        return current_length - spring.rest_length

    def bending_constraint(self, triple):
        # Calculate the current angle between the two springs
        spring1 = self.find_spring(triple.p1, triple.p2)
        spring2 = self.find_spring(triple.p2, triple.p3)
        vector1 = spring1.particle2.position - spring1.particle1.position
        vector2 = spring2.particle2.position - spring2.particle1.position
        cosine_angle = np.dot(vector1, vector2) / (np.linalg.norm(vector1) * np.linalg.norm(vector2))
        current_angle = np.arccos(cosine_angle)
        # Assume a rest angle of 0 (i.e., the springs are initially straight)
        rest_angle = 0
        return current_angle - rest_angle

    def shearing_constraint(self, quadruple):
        particle1 = quadruple.p1
        particle2 = quadruple.p2
        particle3 = quadruple.p3
        particle4 = quadruple.p4
        # Calculate the current length of the diagonal
        diagonal1 = particle2.position - particle1.position
        diagonal2 = particle4.position - particle3.position
        current_length = np.linalg.norm(diagonal1 - diagonal2)
        # Assume a rest length equal to the initial length of the diagonal
        rest_length = np.linalg.norm(particle2.initial_position - particle1.initial_position)
        # Return the difference between the current length and the rest length
        return current_length - rest_length

    def update_constraints(self):
        for spring in self.springs:
            stretching = self.stretching_constraint(spring)
            if stretching > 0 :
                self.apply_stretch_forces(spring)

        for triple in self.triples:
            bending = self.bending_constraint(triple)
            if bending > 0 :
                self.apply_bending_forces(triple)

        for quadruple in self.quadruples:
            shearing = self.shearing_constraint(quadruple)
            if shearing > 0 :
                self.apply_shearing_forces(quadruple)

    # --------- INTEGRATION --------- #
    def integrate_particles(self, delta_time):
        num_particles = len(self.particles)
        M = lil_matrix((3*num_particles, 3*num_particles))
        for i, particle in enumerate(self.particles):
            M[3*i:3*i+3, 3*i:3*i+3] = particle.mass * identity(3)

        # Calculate the current state vector
        x = np.zeros(3 * num_particles)
        v = np.zeros(3 * num_particles)
        for i, particle in enumerate(self.particles):
            x[3*i:3*i+3] = particle.position
            v[3*i:3*i+3] = particle.velocity

        # Calculate the force vector
        F = np.zeros(3 * num_particles)
        for i, particle in enumerate(self.particles):
            F[3*i:3*i+3] = particle.force

        # Calculate the Jacobian of the force
        J = self.calculate_jacobian()

        # Form the linear system and solve for the next state
        A = M - delta_time * J
        b = M @ v + delta_time * F
        next_v, _ = cg(A, b)
        next_x = x + delta_time * next_v

        # Update the particle positions and velocities
        for i, particle in enumerate(self.particles):
            if not particle.is_fixed : 
                particle.velocity = next_v[3*i:3*i+3]
                particle.position = next_x[3*i:3*i+3]
            particle.force = np.zeros(3)

    def calculate_jacobian(self):
        num_particles = len(self.particles)
        J = lil_matrix((3 * num_particles, 3 * num_particles))
        epsilon = 1e-5  # Small change for finite difference

        # Iterate over all pairs of particles
        # for i in range(num_particles):
        #     for j in range(num_particles):
        #         # Calculate the change in force due to a small change in position
        #         original_position_i = self.particles[i].position.copy()
        #         self.particles[i].position += epsilon
        #         F_plus = np.zeros(3 * num_particles)
        #         for k, particle in enumerate(self.particles):
        #             F_plus[3*k:3*k+3] = particle.force
        #         self.particles[i].position = original_position_i  # Reset the position

        #         # Calculate the original force
        #         F = np.zeros(3 * num_particles)
        #         for k, particle in enumerate(self.particles):
        #             F[3*k:3*k+3] = particle.force

        #         # Calculate the derivative of the force with respect to position
        #         dF_dx = (F_plus - F) / epsilon
        #         J[3*i:3*i+3, 3*j:3*j+3] = dF_dx[3*j:3*j+3]

        return J

    def integrate_particles1(self, delta_time):
        for particle in self.particles:
            if not particle.is_fixed :
                acceleration = particle.force / particle.mass
                particle.velocity += acceleration * delta_time
                particle.position += particle.velocity * delta_time
            particle.force = np.zeros(3)
    
    # --------- COLLISION --------- #

    def handle_collisions_with_solid(self, solid, threshold=0.01):
        for particle in self.particles:
            if solid.point_inside(particle.position):
                for triangle in solid.triangles:
                    # Distance from the particle to the plane of the triangle
                    plane_distance = np.dot(triangle.normal, particle.position - triangle.p1)
                    if -threshold < plane_distance < threshold:  # If the particle is near the plane of the triangle
                        # If the particle is inside the triangle boundaries
                        if triangle.point_inside(particle.position):
                            # Correct the position of the particle if it's within the threshold distance to the plane
                            if plane_distance > 0:
                                particle.position += (threshold - plane_distance) * triangle.normal 
                            else:
                                particle.position -= (threshold + plane_distance) * triangle.normal 
                            
                            # Reverse the velocity component normal to the plane and apply damping
                            velocity_projection = np.dot(particle.velocity, triangle.normal)
                            if velocity_projection < 0:
                                particle.velocity -= (1 + self.restitution_coefficient) * velocity_projection * triangle.normal  
                            
                            # Compute the tangential component of the velocity
                            tangent_velocity =  particle.velocity - velocity_projection * triangle.normal 
                            
                            # Apply friction in the opposite direction of the tangential velocity
                            friction_force = -self.friction_coefficient * tangent_velocity  
                            particle.force += friction_force  # Add friction force to the particle
    
    # --------- SIMULATION --------- #

    def update_simulation(self, delta_time):
        self.apply_external_forces()
        self.update_constraints()
        self.integrate_particles(delta_time)