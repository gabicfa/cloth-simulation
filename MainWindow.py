import sys

from Cloth import Cloth
from Sphere import Sphere
from Cube import Cube
from Pyramid import Pyramid
from Plane import Plane

from PyQt5.QtWidgets import QApplication, QMainWindow, QOpenGLWidget
from PyQt5.QtCore import Qt
from OpenGL.GL import *
from OpenGL.GLU import *


class GLWidget(QOpenGLWidget):
    def initializeGL(self):
        glClearColor(1.0, 1.0, 1.0, 1.0)  # Set clear color to white
        glEnable(GL_DEPTH_TEST)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45.0, self.width() / self.height(), 0.1, 100.0)
    
    def drawSolid(self, solid):
        # Draw faces
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)
        glColor3f(0.5, 0.5, 0.5)
        glBegin(GL_TRIANGLES)
        for triangle in solid.triangles:
            for vertex in triangle.vertices:
                glVertex3f(*vertex)
        glEnd()

        # Draw edges
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE)
        glColor3f(0, 0, 0)
        glBegin(GL_TRIANGLES)
        for triangle in solid.triangles:
            for vertex in triangle.vertices:
                glVertex3f(*vertex)
        glEnd()
    
    def paintGL(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        # gluLookAt(-3.0, 0.0, 0.0, 1.0, 0.0, 0, 0, 1, 0)
        gluLookAt(0.5, 0.0, 4.0, 0.5, -1.0, 0, 0, 1, 0)

        #Cloth particles
        glPointSize(7.0) 
        glBegin(GL_POINTS)
        glColor3f(0.0, 0.0, 0.0)
        for particle in self.cloth.particles:
            if particle.is_fixed : 
                glColor3f(0.0, 0.0, 1.0)
            else : 
                glColor3f(0.0, 0.0, 0.0)
            position = particle.position
            glVertex3f(position[0], position[1], position[2])
        glEnd()

        #Cloth Springs
        glLineWidth(2.0)
        glBegin(GL_LINES)
        glColor3f(0.0, 0.0, 0.0)  # Set the color of the strings
        for spring in self.cloth.springs:
            p1 = spring.particle1.position
            p2 = spring.particle2.position
            glVertex3f(p1[0], p1[1], p1[2])
            glVertex3f(p2[0], p2[1], p2[2])
        glEnd()


        for solid in self.solids:
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE) 
            self.drawSolid(solid)
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)

        glFlush()

    def timerEvent(self, event):
        # Update the simulation
        delta_time = 0.001  # Time step
        for solid in self.solids:
            self.cloth.handle_collisions_with_solid(solid) 
        self.cloth.update_simulation(delta_time)
        self.update()


class MainWindow(QMainWindow):
    def __init__(self, cloth, solids):
        super().__init__()
        self.setWindowTitle("Cloth Simulation")
        self.setFixedSize(800, 600)

        self.gl_widget = GLWidget(self)
        self.gl_widget.cloth = cloth
        self.gl_widget.solids = solids

        self.setCentralWidget(self.gl_widget)
        timer_interval = 1
        self.timer_id = self.startTimer(timer_interval)
    
    def timerEvent(self, event):
        self.gl_widget.timerEvent(event)

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Escape:
            self.close()


if __name__ == "__main__":
    app = QApplication(sys.argv)

    cloth = Cloth()
    cloth.rotate([1,0,0], 90)
    cloth.translate([-1.0,-0.45,0.0])
    
    sphere = Sphere()
    cube = Cube()
    pyramid = Pyramid()
    plane = Plane()

    pyramid.rotate([1,0,0], -90)
    pyramid.translate([1.25,-1.05,0.5])

    cube.scale([0.5,0.5,0.5])
    cube.translate([-0.2,-0.8,0.5])
    
    sphere.scale(0.7)
    sphere.translate([0.5,-0.75,0.5])
    
    plane.rotate([1,0,0], -90)
    plane.scale(8.0)
    plane.translate([0.5, -1.07,0.0])
    
    solids = [sphere, cube, pyramid, plane]
    window = MainWindow(cloth, solids)
    window.show()

    # Start the application event loop
    sys.exit(app.exec_())
