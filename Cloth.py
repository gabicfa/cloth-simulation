import numpy as np

class Particle:
    def __init__(self, position, velocity, mass):
        self.position = position
        self.velocity = velocity
        self.mass = mass
        self.force = np.zeros(3)  # Placeholder for external forces

class Spring:
    def __init__(self, particle1, particle2, rest_length, stiffness):
        self.particle1 = particle1
        self.particle2 = particle2
        self.rest_length = rest_length
        self.stiffness = stiffness

class Cloth:
    def __init__(self):
        self.particles = []  # List of particles
        self.springs = []  # List of springs
        self.rotation_angle_x = 30

        # Set up initial positions, velocities, masses, and springs
        # Add code here to create particles and springs for the cloth

        # Example code to create a grid of particles connected by springs
        num_particles_x = 10
        num_particles_y = 10
        cloth_width = 2.0
        cloth_height = 2.0
        rest_length_x = cloth_width / (num_particles_x - 1)
        rest_length_y = cloth_height / (num_particles_y - 1)

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
                velocity = np.zeros(3)
                mass = 0.1
                particle = Particle(rotated_position, velocity, mass)
                self.particles.append(particle)
    

        # Create springs
        for j in range(num_particles_y):
            for i in range(num_particles_x):
                # Structural springs
                if i < num_particles_x - 1:
                    particle1 = self.particles[i + j * num_particles_x]
                    particle2 = self.particles[(i + 1) + j * num_particles_x]
                    spring = Spring(particle1, particle2, rest_length_x, stiffness=0.2)
                    self.springs.append(spring)
                if j < num_particles_y - 1:
                    particle1 = self.particles[i + j * num_particles_x]
                    particle2 = self.particles[i + (j + 1) * num_particles_x]
                    spring = Spring(particle1, particle2, rest_length_y, stiffness=0.2)
                    self.springs.append(spring)

    def apply_spring_forces(self):
        gravity = np.array([0, -9.81, 0])  # Define the gravitational force vector

        for particle in self.particles:
            # Apply gravity as an external force to each particle
            particle.force += particle.mass * gravity

        # Adding the spring forces
        for spring in self.springs:
            # Calculate the spring force according to Hooke's law
            displacement = spring.particle1.position - spring.particle2.position
            distance = np.linalg.norm(displacement)
            direction = displacement / distance if distance != 0 else 0  # to avoid division by zero
            spring_force_magnitude = spring.stiffness * (spring.rest_length - distance)
            spring_force = spring_force_magnitude * direction

            # Apply the spring force to each particle connected by the spring
            spring.particle1.force -= spring_force
            spring.particle2.force += spring_force

    def handle_collisions(self):
        ground_level = -0.5
        damping_factor = 0.4  # Adjust damping factor as needed

        for particle in self.particles:
            if particle.position[1] < ground_level:
                # Handle collision with ground
                particle.position[1] = ground_level
                # Apply damping to all velocity components
                particle.velocity *= damping_factor
                particle.velocity[1] = -particle.velocity[1]

    def update_constraints(self):
        stretching_limit = 1.1  # Allow springs to stretch to 110% of their rest length
        for spring in self.springs:
            p1 = spring.particle1
            p2 = spring.particle2
            distance = np.linalg.norm(p2.position - p1.position)
            if distance > spring.rest_length * stretching_limit:
                correction = (distance - spring.rest_length * stretching_limit) / 2.0
                direction = (p2.position - p1.position) / distance
                p1.position += direction * correction
                p2.position -= direction * correction

    def integrate_particles(self, delta_time):
        for particle in self.particles:
            # Calculate acceleration based on the applied forces
            acceleration = particle.force / particle.mass

            # Update velocity using the acceleration
            particle.velocity += acceleration * delta_time

            # Update position using the new velocity
            particle.position += particle.velocity * delta_time
            # Reset the force for the next iteration
            particle.force = np.zeros(3)

    def update_simulation(self, delta_time):
        self.apply_spring_forces()
        self.handle_collisions()
        self.update_constraints()
        self.integrate_particles(delta_time)
