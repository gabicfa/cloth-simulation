#!/usr/bin/env python

import sys
sys.path.append('../')  # Adds the parent directory to Python's module search path

import numpy as np
from PyQt5.QtCore import *
from PyQt5.QtWidgets import QApplication, QMainWindow, QOpenGLWidget, QVBoxLayout
from PyQt5 import uic

from Simulation.Cloth import Cloth
from Simulation.Sphere import Sphere
from Simulation.Cube import Cube
from Simulation.Pyramid import Pyramid
from Simulation.Plane import Plane

from OpenGL.GL import *
from OpenGL.GLU import *

AXIS_DICT = {
    "X" : [1,0,0],
    "Y" : [0,1,0],
    "Z" : [0,0,1]
}

class SimulationWidget(QOpenGLWidget):
    def initializeGL(self):
        glClearColor(1.0, 1.0, 1.0, 1.0)  # Set clear color to white
        glEnable(GL_DEPTH_TEST)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45.0, self.width() / self.height(), 0.1, 100.0)
        
        # Camera properties
        self.cam_position = np.array([0.5, 0.0, 4.0])
        self.cam_target = np.array([0.5, -1.0, 0.0])
        self.cam_up_vector = np.array([0.0, 1.0, 0.0])
        
        self.rotate_speed = 0.01
        self.move_speed = 0.05
    
    def mousePressEvent(self, event):
        self.mouse_last_position = event.pos()
    
    def wheelEvent(self, event):
        delta = event.angleDelta().y() / 120  # The vertical scroll amount
        self.cam_position += (self.cam_target - self.cam_position) * self.move_speed * delta
        self.update()

    def mouseMoveEvent(self, event):
        dx = event.x() - self.mouse_last_position.x()
        dy = event.y() - self.mouse_last_position.y()

        self.cam_target += self.rotate_speed * np.array([-dx, dy, 0])
        self.mouse_last_position = event.pos()

        self.update()

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
        gluLookAt(*(list(self.cam_position) + list(self.cam_target) + list(self.cam_up_vector)))

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

    def update_simulation(self):
        # Update the simulation
        delta_time = 0.001  # Time step
        for solid in self.solids:
            self.cloth.handle_collisions_with_solid(solid) 
        self.cloth.update_simulation(delta_time)
        self.update()

    def timerEvent(self, event):
        self.update_simulation()

class MainWindow(QMainWindow):
    def __init__(self, *args, **kwargs):
        super(QMainWindow, self).__init__(*args, **kwargs)

        uic.loadUi('mainwindow.ui', self)

        # Create your SimulationWidget and assign the solids and cloth instances
        self.simulation_widget = SimulationWidget(self.simulationWidget)
        self.simulation_widget.solids = []
        self.timer = QTimer()

        self.plane = Plane()
        self.plane.rotate([1,0,0], -90)
        self.plane.scale(8.0)
        self.plane.translate([0.5, -1.07,0.0])

        self.timer.timeout.connect(self.simulation_widget.update_simulation)
        
        self.collision_object.currentTextChanged.connect(self.update_collision_object_fields)

        # Set the focus policy for the simulation widget
        self.simulation_widget.setFocusPolicy(Qt.StrongFocus)
        self.simulation_widget.setFocus()  # Explicitly set the initial focus to the simulation widget

        self.set_values_simulation()
        self.simulation_widget.solids.append(self.plane)
        self.add_solid_check_box.toggled.connect(self.toggle_combobox)
        self.add_floor_check_box.toggled.connect(self.toggle_floor)

        # sphere = Sphere()
        # cube = Cube()
        # pyramid = Pyramid()
        
        # pyramid.rotate([1,0,0], -90)
        # pyramid.translate([1.25,-1.05,0.5])

        # cube.scale([0.5,0.5,0.5])
        # cube.translate([-0.2,-0.8,0.5])

        # sphere.scale(0.7)
        # sphere.translate([0.5,-0.75,0.5])

        layout = QVBoxLayout(self.simulationWidget)
        layout.addWidget(self.simulation_widget)

        self.set_cloth_default_button.clicked.connect(self.set_default)
        self.simulate_button.clicked.connect(self.start_simulation)
        self.stop_simulation_button.clicked.connect(self.stop_simulation)
        self.restart_simulation_button.clicked.connect(self.restart_simulation)

    def mousePressEvent(self, event):
        self.simulation_widget.mousePressEvent(event)

    def mouseMoveEvent(self, event):
        self.simulation_widget.mouseMoveEvent(event)
    
    def timerEvent(self, event):
        self.simulation_widget.update_simulation()

    def keyPressEvent(self, event):
        # Move the camera depending on which key is pressed
        if event.key() == Qt.Key_Escape:
            self.close()
        if event.key() == Qt.Key_Up:
            self.simulation_widget.cam_position += self.simulation_widget.move_speed * self.simulation_widget.cam_up_vector
        elif event.key() == Qt.Key_Down:
            self.simulation_widget.cam_position -= self.simulation_widget.move_speed * self.simulation_widget.cam_up_vector
        elif event.key() == Qt.Key_Left:
            self.simulation_widget.cam_position -= np.cross(self.simulation_widget.cam_target - self.simulation_widget.cam_position, self.simulation_widget.cam_up_vector) * self.simulation_widget.move_speed
        elif event.key() == Qt.Key_Right:
            self.simulation_widget.cam_position += np.cross(self.simulation_widget.cam_target - self.simulation_widget.cam_position, self.simulation_widget.cam_up_vector) * self.simulation_widget.move_speed

        self.simulation_widget.update()
    
    def set_default(self):

        self.num_particles_x.setValue(5)
        self.num_particles_y.setValue(5)
        self.cloth_width.setValue(1.0)
        self.cloth_height.setValue(1.0)

        self.rotate_cloth_angle.setValue(0)
        self.rotate_cloth_axis.setCurrentText("X")

        self.translate_x.setValue(0.0)
        self.translate_y.setValue(0.0)
        self.translate_z.setValue(0.0)

        self.add_solid_check_box.setChecked(True)
        self.add_floor_check_box.setChecked(True)
        self.collision_object.setCurrentText("Cube")

        self.cube_size.setValue(1.0)
        self.cube_rotate_angle.setValue(0)
        self.cube_axis.setCurrentText("X")

        self.cube_translate_x.setValue(0.5)
        self.cube_translate_y.setValue(-0.8)
        self.cube_translate_z.setValue(0.5)
        self.cube_scale_factor.setValue(0.5)
        
        self.update_collision_object_fields()


    def set_values_simulation(self):
        num_particles_x = self.num_particles_x.value()
        num_particles_y = self.num_particles_y.value()
        cloth_width = self.cloth_width.value()
        cloth_height = self.cloth_height.value()
        rotate_cloth_angle = self.rotate_cloth_angle.value()
        rotate_cloth_axis = AXIS_DICT[self.rotate_cloth_axis.currentText()]
        translate_x = self.translate_x.value()
        translate_y = self.translate_y.value()
        translate_z = self.translate_z.value()

        cloth = Cloth(num_particles_x, num_particles_y, cloth_width, cloth_height)
        cloth.rotate(rotate_cloth_axis, rotate_cloth_angle)
        cloth.translate([translate_x,translate_y,translate_z])

        current_item = self.collision_object.currentText()
        if current_item == "Cube":
            cube_size = self.cube_size.value()
            cube_rotate_angle = self.cube_rotate_angle.value()
            cube_axis = AXIS_DICT[self.cube_axis.currentText()]

            cube_translate_x = self.cube_translate_x.value()
            cube_translate_y = self.cube_translate_y.value()
            cube_translate_z = self.cube_translate_z.value()
            cube_scale_factor = self.cube_scale_factor.value()

            cube = Cube([0,0,0], cube_size)
            cube.rotate(cube_axis, cube_rotate_angle)
            cube.translate([cube_translate_x,cube_translate_y,cube_translate_z])
            cube.scale([cube_scale_factor,cube_scale_factor,cube_scale_factor])

            self.simulation_widget.solids = [cube]

        self.simulation_widget.cloth = cloth

    def toggle_combobox(self, checked):
        self.collision_object.setEnabled(checked)
        if checked:
            self.update_collision_object_fields()
        else :
            self.cube_group_box.setVisible(False)
            self.sphere_group_box.setVisible(False)
            self.cone_group_box.setVisible(False)
    
    def toggle_floor(self, checked):
        if checked and self.plane not in self.simulation_widget.solids:
            self.simulation_widget.solids.append(self.plane)
        elif not checked and self.plane in self.simulation_widget.solids:
            self.simulation_widget.solids.remove(self.plane)


    def update_collision_object_fields(self):
        current_item = self.collision_object.currentText()
        if current_item == "Cube":
            self.cube_group_box.setVisible(True)
            self.sphere_group_box.setVisible(False)
            self.cone_group_box.setVisible(False)
        elif current_item == "Sphere":
            self.cube_group_box.setVisible(False)
            self.sphere_group_box.setVisible(True)
            self.cone_group_box.setVisible(False)
        elif current_item == "Cone":
            self.cube_group_box.setVisible(False)
            self.sphere_group_box.setVisible(False)
            self.cone_group_box.setVisible(True)

    def start_simulation(self):
        timer_interval = 1
        self.timer.start(timer_interval)

    def stop_simulation(self):
        if self.timer.isActive():
            self.timer.stop()
    
    def restart_simulation(self):
        self.stop_simulation()
        self.set_values_simulation()
        if self.add_floor_check_box.isChecked() and self.plane not in self.simulation_widget.solids:
            self.simulation_widget.solids.append(self.plane)
        self.start_simulation()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    widget = MainWindow()
    widget.show()
    sys.exit(app.exec_())
