// C++ include
#include <iostream>
#include <string>
#include <vector>

// Utilities for the Assignment
#include "raster.h"

#include <gif.h>
#include <fstream>
 
#include <Eigen/Geometry>
// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"

using namespace std;
using namespace Eigen;

//Image height
const int H = 480;

//Camera settings
const double near_plane = 1.5;       //AKA focal length
const double far_plane = near_plane * 100;
const double field_of_view = 0.7854; //45 degrees
const double aspect_ratio = 1.5;
const bool is_perspective = true;
const Vector3d camera_position(0, 0, 3);
const Vector3d camera_gaze(0, 0, -1);
const Vector3d camera_top(0, 1, 0);

//Object
const std::string data_dir = DATA_DIR;
const std::string mesh_filename(data_dir + "bunny.off");
MatrixXd vertices; // n x 3 matrix (n points)
MatrixXi facets;   // m x 3 matrix (m triangles)

//Material for the object
const Vector3d obj_diffuse_color(0.5, 0.5, 0.5);
const Vector3d obj_specular_color(0.2, 0.2, 0.2);
const double obj_specular_exponent = 256.0;

//Lights
std::vector<Vector3d> light_positions;
std::vector<Vector3d> light_colors;
//Ambient light
const Vector3d ambient_light(0.3, 0.3, 0.3);

//Fills the different arrays
void setup_scene()
{
    //Loads file
    std::ifstream in(mesh_filename);
    if (!in.good())
    {
        std::cerr << "Invalid file " << mesh_filename << std::endl;
        exit(1);
    }
    std::string token;
    in >> token;
    int nv, nf, ne;
    in >> nv >> nf >> ne;
    vertices.resize(nv, 3);
    facets.resize(nf, 3);
    for (int i = 0; i < nv; ++i)
    {
        in >> vertices(i, 0) >> vertices(i, 1) >> vertices(i, 2);
    }
    for (int i = 0; i < nf; ++i)
    {
        int s;
        in >> s >> facets(i, 0) >> facets(i, 1) >> facets(i, 2);
        assert(s == 3);
    }

    //Lights
    light_positions.emplace_back(8, 8, 0);
    light_colors.emplace_back(16, 16, 16);

    light_positions.emplace_back(6, -8, 0);
    light_colors.emplace_back(16, 16, 16);

    light_positions.emplace_back(4, 8, 0);
    light_colors.emplace_back(16, 16, 16);

    light_positions.emplace_back(2, -8, 0);
    light_colors.emplace_back(16, 16, 16);

    light_positions.emplace_back(0, 8, 0);
    light_colors.emplace_back(16, 16, 16);

    light_positions.emplace_back(-2, -8, 0);
    light_colors.emplace_back(16, 16, 16);

    light_positions.emplace_back(-4, 8, 0);
    light_colors.emplace_back(16, 16, 16);
}

void build_uniform(UniformAttributes &uniform)
{
    //TODO: setup uniform
    double image_y = near_plane * tan(field_of_view / 2); //TODO: compute the correct pixels size
    double image_x = aspect_ratio * image_y; //TODO: compute the correct pixels size

    //TODO: setup camera, compute w, u, v
    Vector3d w = -camera_gaze.normalized();
    Vector3d u = (camera_top.cross(w)).normalized();
    Vector3d v = (w.cross(u)).normalized();
    Vector3d e = camera_position;

    //TODO: compute the camera transformation
    Matrix4d worldToCamera;
    worldToCamera << u[0], v[0], w[0], e[0],
                     u[1], v[1], w[1], e[1],
                     u[2], v[2], w[2], e[2],
                     0, 0, 0, 1;

    //TODO: setup projection matrix
    Matrix4d cameraToCanon;
    double r = image_x;
    double l = -r;
    double t = image_y;
    double b = -t;
    double n = -near_plane;
    double f = -far_plane;
    cameraToCanon << 2/(r-l), 0, 0, -(r+l)/(r-l),
                     0, 2/(t-b), 0, -(t+b)/(t-b),
                     0, 0, 2/(n-f), -(n+f)/(n-f),
                     0, 0, 0, 1;

    uniform.transformation = cameraToCanon * worldToCamera.inverse();

    Matrix4d P;
    if (is_perspective)
    {
        //TODO setup perspective camera
        Matrix4d perspective;
        perspective << n, 0, 0, 0,
                       0, n, 0, 0,
                       0, 0, n+f, -f * n,
                       0, 0, 1, 0;

        Matrix4d flipZ;
        flipZ << 1, 0, 0, 0,
                 0, 1, 0, 0,
                 0, 0, -1, 0,
                 0, 0, 0, 1;     
        
        uniform.transformation = flipZ * cameraToCanon * perspective * worldToCamera.inverse();
    }
    else
    {
    } 
}

void simple_render(Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> &frameBuffer)
{
    UniformAttributes uniform;
    build_uniform(uniform);
    Program program;

    program.VertexShader = [](const VertexAttributes &va, const UniformAttributes &uniform) {
        //TODO: fill the shader
        VertexAttributes temp = va;
        temp.position = uniform.transformation * temp.position;
        return temp;
    };

    program.FragmentShader = [](const VertexAttributes &va, const UniformAttributes &uniform) {
        //TODO: fill the shader
        return FragmentAttributes(1, 0, 0);
    };

    program.BlendingShader = [](const FragmentAttributes &fa, const FrameBufferAttributes &previous) {
        //TODO: fill the shader
        return FrameBufferAttributes(fa.color[0], fa.color[1], fa.color[2], fa.color[3]);
    };

    std::vector<VertexAttributes> vertex_attributes;

    //TODO: build the vertex attributes from vertices and facets
    for (int i = 0; i < facets.rows(); ++i) {
        Vector3i f = facets.row(i);

        Vector3d v = vertices.row(f[0]);
        vertex_attributes.push_back(VertexAttributes(v[0], v[1], v[2]));

        v = vertices.row(f[1]);
        vertex_attributes.push_back(VertexAttributes(v[0], v[1], v[2]));

        v = vertices.row(f[2]);
        vertex_attributes.push_back(VertexAttributes(v[0], v[1], v[2]));
    }

    rasterize_triangles(program, uniform, vertex_attributes, frameBuffer);
}

Matrix4d compute_rotation(const double alpha)
{
    //TODO: Compute the rotation matrix of angle alpha on the y axis around the object barycenter
    double x;
    double z;
    for (int i = 0; i < vertices.rows(); ++i) {
        x += vertices.row(i)[0];
        z += vertices.row(i)[2];
    }

    x = x/vertices.rows();
    z = z/vertices.rows();

    Matrix4d translateToOrigin;
    translateToOrigin << 1, 0, 0, -x,
                         0, 1, 0, 0,
                         0, 0, 1, -z,
                         0, 0, 0, 1;
    
    Matrix4d rotate;
    rotate << cos(alpha), 0, sin(alpha), 0,
           0, 1, 0, 0,
           -sin(alpha), 0, cos(alpha), 0,
           0, 0, 0, 1;

    Matrix4d translateFromOrigin;
    translateFromOrigin << 1, 0, 0, x,
                         0, 1, 0, 0,
                         0, 0, 1, z,
                         0, 0, 0, 1;

    return translateFromOrigin * rotate * translateToOrigin;
}

void wireframe_render(const double alpha, Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> &frameBuffer)
{
    UniformAttributes uniform;
    build_uniform(uniform);
    Program program;

    uniform.rotation = compute_rotation(alpha);

    program.VertexShader = [](const VertexAttributes &va, const UniformAttributes &uniform) {
        //TODO: fill the shader
        VertexAttributes temp = va;
        temp.position = uniform.transformation * uniform.rotation * temp.position;
        return temp;
    };

    program.FragmentShader = [](const VertexAttributes &va, const UniformAttributes &uniform) {
        //TODO: fill the shader
        return FragmentAttributes(1, 0, 0);
    };

    program.BlendingShader = [](const FragmentAttributes &fa, const FrameBufferAttributes &previous) {
        //TODO: fill the shader
        return FrameBufferAttributes(fa.color[0], fa.color[1], fa.color[2], fa.color[3]);
    };

    std::vector<VertexAttributes> vertex_attributes;

    //TODO: generate the vertex attributes for the edges and rasterize the lines
    //TODO: use the transformation matrix

    for (int i = 0; i < facets.rows(); ++i) {
        Vector3i f = facets.row(i);

        Vector3d v1 = vertices.row(f[0]);
        Vector3d v2 = vertices.row(f[1]);
        Vector3d v3 = vertices.row(f[2]);

        vertex_attributes.push_back(VertexAttributes(v1[0], v1[1], v1[2]));
        vertex_attributes.push_back(VertexAttributes(v2[0], v2[1], v2[2]));

        vertex_attributes.push_back(VertexAttributes(v2[0], v2[1], v2[2]));
        vertex_attributes.push_back(VertexAttributes(v3[0], v3[1], v3[2]));

        vertex_attributes.push_back(VertexAttributes(v3[0], v3[1], v3[2]));
        vertex_attributes.push_back(VertexAttributes(v1[0], v1[1], v1[2]));

    }

    rasterize_lines(program, uniform, vertex_attributes, 0.5, frameBuffer);
}

void get_shading_program(Program &program)
{
    program.VertexShader = [](const VertexAttributes &va, const UniformAttributes &uniform) {
        //TODO: transform the position and the normal
        VertexAttributes temp = va;
        temp.position = uniform.rotation * temp.position;
        temp.normal = uniform.rotation * temp.normal;

        //TODO: compute the correct lighting
        Vector3d p(temp.position[0], temp.position[1], temp.position[2]);
        Vector3d N(temp.normal[0], temp.normal[1], temp.normal[2]);

        // Punctual lights contribution (direct lighting)
        const Vector4d spec_color(obj_specular_color[0], obj_specular_color[1], obj_specular_color[2], 0);
        Vector4d lights_color(0, 0, 0, 0);
        for (int i = 0; i < light_positions.size(); ++i)
        {
            // const Vector4d light_origin(light_positions[i][0], light_positions[i][1], light_positions[i][2], 0);
            // const Vector4d new_light_position = uniform.transformation * light_origin;
            // const Vector3d light_position(new_light_position[0], new_light_position[1], new_light_position[2]);

            const Vector3d &light_position = light_positions[i];
            Vector4d light_color(light_colors[i][0], light_colors[i][1], light_colors[i][2], 0);

            const Vector3d Li = (light_position - p).normalized();

            Vector4d diff_color(obj_diffuse_color[0], obj_diffuse_color[0], obj_diffuse_color[0], 0);

            // TODO: Add shading parameters
            const Vector3d l = (light_position - p).normalized();
            const Vector3d v = (camera_position - p).normalized();
            const Vector3d h = (v + l).normalized();
            const Vector3d n = N;

            // Diffuse contribution
            const Vector4d diffuse = diff_color * std::max(l.dot(n), 0.0);

            // Specular contribution, use obj_specular_color
            const Vector4d specular = spec_color * pow(std::max(0., n.dot(h)), obj_specular_exponent);

            // Attenuate lights according to the squared distance to the lights
            const Vector3d D = light_position - p;
            lights_color += (diffuse + specular).cwiseProduct(light_color) / D.squaredNorm();
        }

        temp.color = uniform.color + lights_color; 
        temp.position = uniform.transformation * temp.position;
        temp.normal = (uniform.transformation).inverse().transpose() * temp.normal;

        return temp;
    };

    program.FragmentShader = [](const VertexAttributes &va, const UniformAttributes &uniform) {
        //TODO: create the correct fragment
        FragmentAttributes temp(va.color[0], va.color[1], va.color[2], 1);
        temp.position = va.position;
        return temp;
    };

    program.BlendingShader = [](const FragmentAttributes &fa, const FrameBufferAttributes &previous) {
        //TODO: implement the depth check
        if (fa.position[2] > previous.depth)
        {
            FrameBufferAttributes out(fa.color[0], fa.color[1], fa.color[2], 1);
            out.depth = fa.position[2];
            return out;
        }
        else
        {
            return previous;
        }
        //return FrameBufferAttributes(fa.color[0], fa.color[1], fa.color[2], fa.color[3]);
    };
}

void flat_shading(const double alpha, Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> &frameBuffer)
{
    UniformAttributes uniform;
    build_uniform(uniform);
    Program program;
    get_shading_program(program);
    uniform.rotation = compute_rotation(alpha);

    std::vector<VertexAttributes> vertex_attributes;
    
    //TODO: compute the normals
    for (int i = 0; i < facets.rows(); ++i) {
        Vector3i f = facets.row(i);

        Vector3d v1 = vertices.row(f[0]);
        Vector3d v2 = vertices.row(f[1]);
        Vector3d v3 = vertices.row(f[2]);

        Vector3d cross_product = -(v3 - v2).cross(v3 - v1).normalized();
        Vector4d normal(cross_product[0], cross_product[1], cross_product[2], 0);

        VertexAttributes v1_attributes(v1[0], v1[1], v1[2]);
        VertexAttributes v2_attributes(v2[0], v2[1], v2[2]);
        VertexAttributes v3_attributes(v3[0], v3[1], v3[2]);

        v1_attributes.normal = normal;
        v2_attributes.normal = normal;
        v3_attributes.normal = normal;

        vertex_attributes.push_back(v1_attributes);
        vertex_attributes.push_back(v2_attributes);
        vertex_attributes.push_back(v3_attributes);
    }

    //TODO: set material colors
    uniform.color << 0.25, 0.25, 0.25, 1;

    rasterize_triangles(program, uniform, vertex_attributes, frameBuffer);
}

Eigen::Vector4d avg (const vector<Vector4d> list) 
{
    Vector4d total(0, 0, 0, 0);
    for (int i = 0; i < list.size(); ++i) {
        total = total + list[i];
    }

    total = (total / list.size()).normalized();
    return total;
}

void pv_shading(const double alpha, Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> &frameBuffer)
{
    UniformAttributes uniform;
    build_uniform(uniform);
    Program program;
    get_shading_program(program);

    uniform.rotation = compute_rotation(alpha);

    //TODO: compute the vertex normals as vertex normal average
    std::vector<std::vector<Vector4d>> vertex_normals(vertices.size());

    for (int j = 0; j < facets.rows(); ++j) {
        
        Vector3i f = facets.row(j);
        Vector3d v1 = vertices.row(f[0]);
        Vector3d v2 = vertices.row(f[1]);
        Vector3d v3 = vertices.row(f[2]);

        Vector3d cross_product = -(v3 - v2).cross(v3 - v1).normalized();
        Vector4d normal(cross_product[0], cross_product[1], cross_product[2], 0);
        
        vertex_normals[f[0]].push_back(normal);
        vertex_normals[f[1]].push_back(normal);
        vertex_normals[f[2]].push_back(normal);

    }

    std::vector<VertexAttributes> vertex_attributes;
    //TODO: create vertex attributes
    for (int i = 0; i < facets.rows(); ++i) {
        Vector3i f = facets.row(i);

        Vector3d v1 = vertices.row(f[0]);
        Vector3d v2 = vertices.row(f[1]);
        Vector3d v3 = vertices.row(f[2]);

        VertexAttributes v1_attributes(v1[0], v1[1], v1[2]);
        VertexAttributes v2_attributes(v2[0], v2[1], v2[2]);
        VertexAttributes v3_attributes(v3[0], v3[1], v3[2]);

        v1_attributes.normal = avg(vertex_normals[f[0]]);
        v2_attributes.normal = avg(vertex_normals[f[1]]);
        v3_attributes.normal = avg(vertex_normals[f[2]]);

        vertex_attributes.push_back(v1_attributes);
        vertex_attributes.push_back(v2_attributes);
        vertex_attributes.push_back(v3_attributes);
    }

    //TODO: set material colors
    uniform.color << 0.25, 0.25, 0.25, 1;

    rasterize_triangles(program, uniform, vertex_attributes, frameBuffer);
}

int main(int argc, char *argv[])
{
    setup_scene();

    int W = H * aspect_ratio;
    Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> frameBuffer(W, H);
    vector<uint8_t> image;

    frameBuffer.setConstant(FrameBufferAttributes());
    simple_render(frameBuffer);
    framebuffer_to_uint8(frameBuffer, image);
    stbi_write_png("simple_perspective.png", frameBuffer.rows(), frameBuffer.cols(), 4, image.data(), frameBuffer.rows() * 4);

    frameBuffer.setConstant(FrameBufferAttributes());
    wireframe_render(0.0, frameBuffer);
    framebuffer_to_uint8(frameBuffer, image);
    stbi_write_png("wireframe_perspective.png", frameBuffer.rows(), frameBuffer.cols(), 4, image.data(), frameBuffer.rows() * 4);

    frameBuffer.setConstant(FrameBufferAttributes());
    flat_shading(0.0, frameBuffer);
    framebuffer_to_uint8(frameBuffer, image);
    stbi_write_png("flat_shading_perspective.png", frameBuffer.rows(), frameBuffer.cols(), 4, image.data(), frameBuffer.rows() * 4);

    frameBuffer.setConstant(FrameBufferAttributes());
    pv_shading(0, frameBuffer);
    framebuffer_to_uint8(frameBuffer, image);
    stbi_write_png("pv_shading_perspective.png", frameBuffer.rows(), frameBuffer.cols(), 4, image.data(), frameBuffer.rows() * 4);

    //TODO: add the animation
    int delay = 25;
    GifWriter g;

    GifBegin(&g, "flat_shading_perspective.gif", frameBuffer.rows(), frameBuffer.cols(), delay);
    for (double i = 0; i < 6.28; i += 0.30)
    {
        frameBuffer.setConstant(FrameBufferAttributes());
        flat_shading(i, frameBuffer);
        framebuffer_to_uint8(frameBuffer, image);
        GifWriteFrame(&g, image.data(), frameBuffer.rows(), frameBuffer.cols(), delay);
    }
    GifEnd(&g);

    GifBegin(&g, "wireframe_perspective.gif", frameBuffer.rows(), frameBuffer.cols(), delay);
    for (double i = 0; i < 6.28; i += 0.30)
    {
        frameBuffer.setConstant(FrameBufferAttributes());
        wireframe_render(i, frameBuffer);
        framebuffer_to_uint8(frameBuffer, image);
        GifWriteFrame(&g, image.data(), frameBuffer.rows(), frameBuffer.cols(), delay);
    }
    GifEnd(&g);

    GifBegin(&g, "pv_shading_perspective.gif", frameBuffer.rows(), frameBuffer.cols(), delay);
    for (double i = 0; i < 6.28; i += 0.30)
    {
        frameBuffer.setConstant(FrameBufferAttributes());
        pv_shading(i, frameBuffer);
        framebuffer_to_uint8(frameBuffer, image);
        GifWriteFrame(&g, image.data(), frameBuffer.rows(), frameBuffer.cols(), delay);
    }
    GifEnd(&g);

    return 0;
}
