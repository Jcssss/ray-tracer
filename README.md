# Ray Tracer
Built as a series of projects for my class (CSC 305 - Intro to Computer Graphics), the ray tracer was developed through multiple iterations incorporating new components at each step.

## About
The Ray Tracer was built in 4 iterations:
1. Version 1 involved the ray tracing of a simple scene with a single light and a sphere. The program incorporated both perspecive and orthographic views along with ambient, specular, and diffuse shading components.
2. Version 2 added multiple light sources and objects to the scene, requiring the addition of shadows, reflection, and refraction. The program also used Perlin Noise to create texture on a few of the objects in the scene.
3. Version 3 used larger inputs, and required the implementation of an AABB tree to efficiently output the intended scene.
4. Finally, Version 4 added a rasterization component, that required implementation of a Vertex Shader, Fragment Shader, and Blending Shader, to render the image with both Per-Vertex Shading and Flat Shading. Additionally, we also added the ability to rotate the object around it's baycenter to create a gif.
