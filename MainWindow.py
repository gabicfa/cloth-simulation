import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QOpenGLWidget
from PyQt5.QtCore import Qt
from OpenGL.GL import *
from OpenGL.GLU import *
from Cloth import Cloth
from Sphere import Sphere
from Triangle import Triangle
from Cube import Cube

class GLWidget(QOpenGLWidget):
    def initializeGL(self):
        glClearColor(1.0, 1.0, 1.0, 1.0)  # Set clear color to white
        glEnable(GL_DEPTH_TEST)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45.0, self.width() / self.height(), 0.1, 100.0)
    
    def drawCube(self):
        # Draw faces
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)
        glColor3f(0.5, 0.5, 0.5)  # Red color for faces
        glBegin(GL_TRIANGLES)
        for triangle in self.cube.triangles:
            for vertex in triangle.vertices:
                glVertex3f(*vertex)
        glEnd()

        # Draw edges
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE)
        glColor3f(0, 0, 0)  # Green color for edges
        glBegin(GL_TRIANGLES)
        for triangle in self.cube.triangles:
            for vertex in triangle.vertices:
                glVertex3f(*vertex)
        glEnd()
        

    def drawTriangle(self):
        glColor3f(0.0, 1.0, 0.0)
        glBegin(GL_TRIANGLES)
        glVertex3f(self.triangle.p1[0], self.triangle.p1[1]-0.05, self.triangle.p1[2])
        glVertex3f(self.triangle.p2[0], self.triangle.p2[1]-0.05, self.triangle.p2[2])
        glVertex3f(self.triangle.p3[0], self.triangle.p3[1]-0.05, self.triangle.p3[2])
        glEnd()
    
    def drawSphere(self):
        glColor3f(1.0, 0.0, 0.0) 
        sphere_radius = self.sphere.radius - 0.05
        sphere_slices = 50
        sphere_stacks = 50
        sphere_quad = gluNewQuadric()
        glPushMatrix()
        glTranslatef(self.sphere.center[0], self.sphere.center[1], self.sphere.center[2])
        gluSphere(sphere_quad, sphere_radius, sphere_slices, sphere_stacks)
        glPopMatrix()

    def paintGL(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        # gluLookAt(-3.0, 0.0, 0.0, 1.0, 0.0, 0, 0, 1, 0)
        gluLookAt(3.0, 1.0, 4, 0.5, 0.0, 0, 0, 1, 0)

        glPointSize(7.0)  # Set the size of the points
        glBegin(GL_POINTS)
        glColor3f(0.0, 0.0, 0.0)  # Set the color of the particles
        for particle in self.cloth.particles:
            if particle.is_fixed : 
                glColor3f(0.0, 0.0, 1.0)  # Set the color of the particles
            else : 
                glColor3f(0.0, 0.0, 0.0)  # Set the color of the particles
            position = particle.position
            glVertex3f(position[0], position[1], position[2])
        glEnd()

        
        # self.drawSphere()
        # self.drawTriangle()

        # glPolygonMode(GL_FRONT_AND_BACK, GL_LINE)
        
        self.drawCube()

        # glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)

        glLineWidth(2.0)
        glBegin(GL_LINES)
        glColor3f(0.0, 0.0, 0.0)  # Set the color of the strings
        for spring in self.cloth.springs:
            p1 = spring.particle1.position
            p2 = spring.particle2.position
            glVertex3f(p1[0], p1[1], p1[2])
            glVertex3f(p2[0], p2[1], p2[2])
        glEnd()

        # # Render the floor plane
        # glColor3f(0.5, 0.5, 0.5)  # Set the color of the floor
        # glBegin(GL_QUADS)
        # glVertex3f(-2.5, -1.05, -2.5)  # Bottom left
        # glVertex3f(2.5, -1.05, -2.5)  # Bottom right
        # glVertex3f(2.5, -1.05, 2.5)  # Top right
        # glVertex3f(-2.5, -1.05, 2.5)  # Top left
        # glEnd()

        glFlush()

    def timerEvent(self, event):
        # Update the simulation
        delta_time = 0.001  # Adjust the time step as needed
        # self.cloth.handle_collisions_with_sphere(self.sphere)
        # self.cloth.handle_collisions_with_triangle(self.triangle)
        self.cloth.handle_collisions_with_cube(self.cube) 
        self.cloth.update_simulation(delta_time)
        # Trigger the rendering
        self.update()


class MainWindow(QMainWindow):
    def __init__(self, cloth, sphere, triangle, cube):
        super().__init__()
        self.setWindowTitle("Cloth Simulation")
        self.setFixedSize(800, 600)
        self.gl_widget = GLWidget(self)
        self.gl_widget.cloth = cloth
        self.gl_widget.sphere = sphere
        self.gl_widget.triangle = triangle
        self.gl_widget.cube = cube
        self.setCentralWidget(self.gl_widget)
        # Set up the timer to trigger the update periodically
        timer_interval = 1  # Adjust the interval in milliseconds as needed
        self.timer_id = self.startTimer(timer_interval)
    
    def timerEvent(self, event):
        # Forward the timer event to the GLWidget
        self.gl_widget.timerEvent(event)

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Escape:
            self.close()


if __name__ == "__main__":
    app = QApplication(sys.argv)

    # Create an instance of ClothSimulation
    cloth = Cloth()
    sphere = Sphere()
    triangle = Triangle()
    cube = Cube()
    triangle.rotate(-90, [1,0,0])
    triangle.translate([0.15,-0.2,0.2])
    triangle.scale([0.5,0.5,0.5])

    
    cube.translate([1.0,-0.5,1.0])
    cube.scale([0.5,0.5,0.5])
    # Create an instance of MainWindow and pass the ClothSimulation instance
    window = MainWindow(cloth, sphere, triangle, cube)

    window.show()

    # Start the application event loop
    sys.exit(app.exec_())
