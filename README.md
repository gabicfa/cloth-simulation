# Cloth Simulation

This repository contains a Python implementation of a cloth simulation using the Baraff and Witkin method. The simulation utilizes the PyQt5 library for creating a graphical user interface (GUI) to interact with the simulation.

## Code Structure

The code consists of the following files:

- `main.py`: This file contains the main entry point for the application and sets up the GUI using PyQt5. It also handles user interactions, such as mouse and keyboard events, and connects them to the corresponding simulation functions.

- `Simulation/Cloth.py`: This file contains the implementation of the Cloth class, which represents the cloth object in the simulation. It contains methods for initializing the cloth, applying forces, handling constraints, and integrating the particle motion.

- `Simulation/ClothElement.py`: This file contains the implementation of the Particle, Spring, Triple, and Quadruple classes, which are used in the cloth simulation.

- `Solids/Solid.py`: This file contains the abstract Solid class, which serves as a base class for different solid objects in the simulation. It provides methods for handling solids' properties and transformations.

- `Solids/Triangle.py`: This file contains the Triangle class, which represents a triangular solid object in the simulation. It provides methods for calculating the normal, checking if a point is inside the triangle, and applying transformations such as rotation, translation, and scaling.

- `Solids/Plane.py`: This file contains the Plane class, which represents a plane solid object in the simulation. It inherits from the Solid class and provides methods for handling plane-specific properties and transformations.

- `Solids/Cube.py`: This file contains the Cube class, which represents a cube solid object in the simulation. It also inherits from the Solid class and provides methods for handling cube-specific properties and transformations.

- `Solids/Sphere.py`: This file contains the Sphere class, which represents a sphere solid object in the simulation. It inherits from the Solid class and provides methods for handling sphere-specific properties and transformations.

- `Solids/Pyramid.py`: This file contains the Pyramid class, which represents a pyramid solid object in the simulation. It inherits from the Solid class and provides methods for handling pyramid-specific properties and transformations.

## Dependencies

The following libraries are required to run the code:

- PyQt5: Used for creating the GUI and handling user interactions.
- NumPy: Used for mathematical operations and vector calculations.
- OpenGL: Used for rendering the simulation scene.


Make sure these libraries are installed before running the code.

## GUI Interaction

The GUI provides various options for configuring and interacting with the cloth simulation. Here are the main features:

- Simulation Parameters: You can adjust the number of particles in the cloth, cloth dimensions, and other simulation parameters using the sliders and input fields in the GUI.

- Camera Controls:
  - Use the arrow keys to move the camera around the simulation scene.
  - Click and drag with the left mouse button to rotate the camera around the scene.
  - Scroll the mouse wheel to zoom in and out of the scene.

- Adding Solids: You can add different solid objects, such as a cube, sphere, or pyramid, to the simulation scene. Adjust the parameters for each solid in the corresponding sections of the GUI.

- Simulation Control: Use the buttons provided to start, stop, or restart the simulation.

## How to Run

To run the code, make sure you have Python installed on your system along with the required libraries mentioned above. Then follow these steps:

1. Clone the repository to your local machine.
2. Open a terminal or command prompt and navigate to the repository directory.
3. Run the following command to start the simulation:

```
   python main.py
```

The GUI window will appear, and you can interact with the simulation using the controls provided.

## References

This implementation is based on the Baraff and Witkin method for cloth simulation, as described in their paper "Large Steps in Cloth Simulation".