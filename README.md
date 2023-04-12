# Ray-Tracing-and-Rendering
This project was built in two parts:
* **Ray-Tracing:** Implementating a basic ray-tracer using computation kernels (functions passed as arguments to functions). Given a ray, the code traverses the scene graph, computes intersections, and (under appropriate conditions) invokes the computation kernel with the intersection info. 
* **Rendring using OpenGL:**  Generating a room scene which can be navigated in a walk-through fashion by writing the OpenGL analogs of many of the methods used in the ray-tracing part. Emphasis is placed both upon the implemention of OpenGL's basic capabilities (e.g. shading. lighting, transparency, materials properties, etc.) and their use in generating more involved effects (e.g. shadows, reflections.)

## Getting Started
After you copy the provided files to your directory, the first thing to do is compile the program. To do this, you will first have to compile the JPEG library and then compile the assignment3 executable.

### On a Windows Machine
Begin by double-clicking on Assignments.sln to open the workspace in Microsoft Visual Studios.
* Compile the Assignment3 executable by right-clicking on "Assignment3" and selecting "Build". (If the `JPEG.lib`, `Image.lib`, `Ray.lib`, and `Util.lib` libraries have not already been compiled, they will be compiled first.)
* The executable Assignment3.exe is compiled in Release mode for the 64-bit architecture and will be placed in the root directory.

### On a Linux Machine
* Type make `-f Makefile3` to compile the Assignment3 executable. (If the `libImage.a`, `libRay.a`, and `libUtil.a` libraries have not already been compiled, they will be compiled first.) This assumes that JPEG libraries have already been installed on your machine. (If it hasn't been installed yet, you can install it by typing `sudo apt-get install libjpeg-dev`.)
* The executable Assignment3 is compiled in Release mode and will be placed in the root directory.
* If you are having trouble linking either to the gl libraries or the glu libraries, you should install the associated packages: `sudo apt-get install libgl1-mesa-dev libglu1-mesa-dev`, and the glut package: `sudo apt-get install freeglut3-dev`

## How the Executable Works
The executable takes in as a mandatory arguments the input (`.ray`) .ray file name. Additionally, you can also pass in the dimensions of the viewing window and the complexity of the tesselation for objects like the sphere, the cylinder, and the cone. (Specifically, this specifies the resolution, e.g. the number of angular samples.) It is invoked from the command line with:

`% Assignment3 --in in.ray --width w --height h --cplx c`

Feel free to add new arguments to deal with the new functionalities you are implementing. Just make sure they are documented.
