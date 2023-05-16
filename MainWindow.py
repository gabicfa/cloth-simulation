import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QOpenGLWidget
from PyQt5.QtGui import QOpenGLVersionProfile, QSurfaceFormat
from PyQt5.QtCore import Qt
from OpenGL.GL import *
from OpenGL.GLU import *
from Cloth import ClothSimulation

class GLWidget(QOpenGLWidget):
    def initializeGL(self):
        glClearColor(1.0, 1.0, 1.0, 1.0)  # Set clear color to white
        glEnable(GL_DEPTH_TEST)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45.0, self.width() / self.height(), 0.1, 100.0)

    def paintGL(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        gluLookAt(0, 0, 5, 0, 0, 0, 0, 1, 0)

        # Render the cloth simulation
        # glPushMatrix()
        # glTranslatef(-cloth_width / 2.0, -cloth_height / 2.0, 0.0)  # Translate to the center of the cloth

        glPointSize(7.0)  # Set the size of the points
        glBegin(GL_POINTS)
        glColor3f(0.0, 0.0, 0.0)  # Set the color of the particles

        for particle in self.cloth_simulation.particles:
            position = particle.position
            glVertex3f(position[0], position[1], position[2])
        glEnd()

        glLineWidth(2.0)
        glBegin(GL_LINES)
        glColor3f(0.0, 0.0, 0.0)  # Set the color of the strings
        for spring in self.cloth_simulation.springs:
            p1 = spring.particle1.position
            p2 = spring.particle2.position
            glVertex3f(p1[0], p1[1], p1[2])
            glVertex3f(p2[0], p2[1], p2[2])
        glEnd()

        # Render the floor plane
        glColor3f(0.5, 0.5, 0.5)  # Set the color of the floor
        glBegin(GL_QUADS)
        glVertex3f(-2.5, -0.5, -2.5)  # Bottom left
        glVertex3f(2.5, -0.5, -2.5)  # Bottom right
        glVertex3f(2.5, -0.5, 2.5)  # Top right
        glVertex3f(-2.5, -0.5, 2.5)  # Top left
        glEnd()

        glFlush()

    def timerEvent(self, event):
        # Update the simulation
        delta_time = 0.01  # Adjust the time step as needed
        self.cloth_simulation.update_simulation(delta_time)

        # Trigger the rendering
        self.update()


class MainWindow(QMainWindow):
    def __init__(self, cloth_simulation):
        super().__init__()
        self.setWindowTitle("Cloth Simulation")
        self.setFixedSize(800, 600)
        self.gl_widget = GLWidget(self)
        self.gl_widget.cloth_simulation = cloth_simulation
        self.setCentralWidget(self.gl_widget)
        # Set up the timer to trigger the update periodically
        timer_interval = 10  # Adjust the interval in milliseconds as needed
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
    cloth_simulation = ClothSimulation()

    # Create an instance of MainWindow and pass the ClothSimulation instance
    window = MainWindow(cloth_simulation)
    window.show()

    # Start the application event loop
    sys.exit(app.exec_())
