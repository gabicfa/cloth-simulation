from ClothElements import Particle, Spring, Quadruple, Triple
import numpy as np

class Cloth:
    def __init__(self, num_particles_x = 10, num_particles_y = 10, cloth_width = 2.0, cloth_height = 2.0):
        self.particles = []
        self.springs = []
        self.springs_dict = {}
        self.triples = []
        self.quadruples = []

        self.rotation_angle_x = 90

        # --------- SETUP --------- #

        # Create particles
        for j in range(num_particles_y):
            for i in range(num_particles_x):
                position = np.array([cloth_width * i / (num_particles_x - 1), cloth_height * j / (num_particles_y - 1), 0.0])
                # Apply rotation transformation
                rotated_position = np.array([
                    position[0],
                    position[1] * np.cos(np.deg2rad(self.rotation_angle_x)) - position[2] * np.sin(np.deg2rad(self.rotation_angle_x)),
                    position[1] * np.sin(np.deg2rad(self.rotation_angle_x)) + position[2] * np.cos(np.deg2rad(self.rotation_angle_x))
                ])
                position = rotated_position
                mass = 0.1
                particle = Particle(mass, position)
                self.particles.append(particle)
    
        # Create springs
        stiffness = 10
        for j in range(num_particles_y):
            for i in range(num_particles_x):
                # Structural springs
                if i < num_particles_x - 1:
                    particle1 = self.particles[i + j * num_particles_x]
                    particle2 = self.particles[(i + 1) + j * num_particles_x]
                    spring = Spring(particle1, particle2, stiffness)
                    self.add_spring(particle1, particle2, spring)
                if j < num_particles_y - 1:
                    particle1 = self.particles[i + j * num_particles_x]
                    particle2 = self.particles[i + (j + 1) * num_particles_x]
                    spring = Spring(particle1, particle2, stiffness)
                    self.add_spring(particle1, particle2, spring)
                
                # Diagonal springs
                if i < num_particles_x - 1 and j < num_particles_y - 1:
                    # Add a spring between the current particle and the one diagonally down-right from it
                    particle1 = self.particles[i + j * num_particles_x]
                    particle2 = self.particles[(i + 1) + (j + 1) * num_particles_x]
                    spring = Spring(particle1, particle2, stiffness)
                    self.add_spring(particle1, particle2, spring)

                if i > 0 and j < num_particles_y - 1:
                    # Add a spring between the current particle and the one diagonally down-left from it
                    particle1 = self.particles[i + j * num_particles_x]
                    particle2 = self.particles[(i - 1) + (j + 1) * num_particles_x]
                    spring = Spring(particle1, particle2, stiffness=1)
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

    # --------- FORCES --------- #

    def apply_shearing_forces(self, quadruple, shearing_stiffness):
        p1, p2, p3, p4 = quadruple.p1, quadruple.p2, quadruple.p3, quadruple.p4
        L_shear = quadruple.L_shear
        C_shear = np.linalg.norm((p2.position - p1.position) + (p4.position - p3.position)) - L_shear
        grad_C_shear_p1 = -(p2.position - p1.position + p4.position - p3.position) / L_shear
        grad_C_shear_p2 = (p2.position - p1.position + p4.position - p3.position) / L_shear
        grad_C_shear_p3 = -(p2.position - p1.position + p4.position - p3.position) / L_shear
        grad_C_shear_p4 = (p2.position - p1.position + p4.position - p3.position) / L_shear
        p1.force += -shearing_stiffness * C_shear * grad_C_shear_p1
        p2.force += -shearing_stiffness * C_shear * grad_C_shear_p2
        p3.force += -shearing_stiffness * C_shear * grad_C_shear_p3
        p4.force += -shearing_stiffness * C_shear * grad_C_shear_p4
    
    def apply_bending_forces(self, triple, bending_stiffness):
        particle1 = triple.p1
        particle2 = triple.p2
        particle3 = triple.p3
        v1 = particle2.position - particle1.position
        v2 = particle3.position - particle2.position
        bending_force = bending_stiffness * (v1 - v2)
        particle1.force -= bending_force
        particle3.force += bending_force
    
    def apply_stretch_forces(self, spring):
        current_length = np.linalg.norm(spring.particle1.position - spring.particle2.position)
        stretch_force = spring.stiffness * (current_length - spring.rest_length)
        force_direction = (spring.particle1.position - spring.particle2.position) / current_length
        spring.particle1.force -= stretch_force * force_direction
        spring.particle2.force += stretch_force * force_direction

    def apply_external_forces(self):
        gravity = np.array([0, -9.81, 0])  # Define the gravitational force vector
        for particle in self.particles:
            particle.force += particle.mass * gravity

    # --------- COLLISION --------- #

    def handle_collisions(self):
        ground_level = -1.0
        damping_factor = 0.1  # Adjust damping factor as needed

        sphere_center = np.array([1.0, -0.5, 1.0])  # The center of the sphere
        sphere_radius = 0.5  # The radius of the sphere

        for particle in self.particles:
            if particle.position[1] < ground_level:
                # Handle collision with ground
                particle.position[1] = ground_level
                # Apply damping to all velocity components
                particle.velocity *= damping_factor
                particle.velocity[1] = -particle.velocity[1]
            # NOT WORKING - TO DO
            elif np.linalg.norm(particle.position - sphere_center) < sphere_radius:
                damping_factor = 0.0  # Adjust damping factor as needed
                # Handle collision with sphere
                direction = (particle.position - sphere_center) / np.linalg.norm(particle.position - sphere_center)
                particle.position = sphere_center + direction * sphere_radius
                # Apply damping to the velocity component in the direction of the normal
                particle.velocity -= (1 + damping_factor) * np.dot(particle.velocity, direction) * direction
                particle.velocity -= (1 + damping_factor) * np.dot(particle.velocity, direction) * direction

    
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
                self.apply_bending_forces(triple, 1)

        for quadruple in self.quadruples:
            shearing = self.shearing_constraint(quadruple)
            if shearing > 0 :
                self.apply_shearing_forces(quadruple, 1)

    # --------- INTEGRATION --------- #

    def integrate_particles(self, delta_time):
        for particle in self.particles:
            acceleration = particle.force / particle.mass
            particle.velocity += acceleration * delta_time
            particle.position += particle.velocity * delta_time
            particle.force = np.zeros(3)

    def update_simulation(self, delta_time):
        self.apply_external_forces()
        self.handle_collisions()
        self.update_constraints()
        self.integrate_particles(delta_time)
