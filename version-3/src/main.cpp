////////////////////////////////////////////////////////////////////////////////
// C++ include
#include <iostream>
#include <string>
#include <vector>
#include <limits>
#include <fstream>
#include <algorithm>
#include <numeric>

// Utilities for the Assignment
#include "utils.h"

// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"

// Shortcut to avoid Eigen:: everywhere, DO NOT USE IN .h
using namespace Eigen;

////////////////////////////////////////////////////////////////////////////////
// Class to store tree
////////////////////////////////////////////////////////////////////////////////

class AABBTree
{
public:
    class Node
    {
    public:
        AlignedBox3d bbox;
        int parent;   // Index of the parent node (-1 for root)
        int left;     // Index of the left child (-1 for a leaf)
        int right;    // Index of the right child (-1 for a leaf)
        int triangle; // Index of the node triangle (-1 for internal nodes)
    };

    std::vector<Node> nodes;
    int root = 0;

    AABBTree() = default;                           // Default empty constructor
    AABBTree(const MatrixXd &V, const MatrixXi &F); // Build a BVH from an existing mesh

private:
    // builds the bvh recursively
    int build_recursive(const MatrixXd &V, const MatrixXi &F, const MatrixXd &centroids, int from, int to, int parent, std::vector<int> &triangles);
};

void print_node (AABBTree::Node node_to_print, int index) {
    std::cout << "Node_Index: " << index << std::endl;
    std::cout << "Parent: " << node_to_print.parent << std::endl;
    std::cout << "Left: " << node_to_print.left << std::endl;
    std::cout << "Right: " << node_to_print.right << std::endl;
    std::cout << "Triangle: " << node_to_print.triangle << std::endl;
}

////////////////////////////////////////////////////////////////////////////////
// Scene setup, global variables
////////////////////////////////////////////////////////////////////////////////
const std::string data_dir = DATA_DIR;
const std::string filename("raytrace.png");
const std::string mesh_filename(data_dir + "bunny.off");

//Camera settings
const double focal_length = 2;
const double field_of_view = 0.7854; //45 degrees
const bool is_perspective = true;
const Vector3d camera_position(0, 0, 2);

// Triangle Mesh
MatrixXd vertices; // n x 3 matrix (n points)
MatrixXi facets;   // m x 3 matrix (m triangles)
AABBTree bvh;

//Maximum number of recursive calls
const int max_bounce = 5;

//Material for the object, same material for all objects
const Vector4d obj_ambient_color(0.0, 0.5, 0.0, 0);
const Vector4d obj_diffuse_color(0.5, 0.5, 0.5, 0);
const Vector4d obj_specular_color(0.2, 0.2, 0.2, 0);
const double obj_specular_exponent = 256.0;
const Vector4d obj_reflection_color(0.7, 0.7, 0.7, 0);

// Precomputed (or otherwise) gradient vectors at each grid node
const int grid_size = 20;
std::vector<std::vector<Vector2d>> grid;

// Objects
std::vector<Vector3d> sphere_centers;
std::vector<double> sphere_radii;
std::vector<Matrix3d> parallelograms;

//Lights
std::vector<Vector3d> light_positions;
std::vector<Vector4d> light_colors;

//Ambient light
const Vector4d ambient_light(0.2, 0.2, 0.2, 0);

//Fills the different arrays
void setup_scene()
{
    //Loads file
    std::ifstream in(mesh_filename);
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

    //setup tree
    bvh = AABBTree(vertices, facets);

    //Spheres
    // sphere_centers.emplace_back(10, 0, 1);
    // sphere_radii.emplace_back(1);

    // sphere_centers.emplace_back(7, 0.05, -1);
    // sphere_radii.emplace_back(1);

    sphere_centers.emplace_back(4, 0.1, 1);
    sphere_radii.emplace_back(1);

    sphere_centers.emplace_back(1, 0.2, -1);
    sphere_radii.emplace_back(1);

    sphere_centers.emplace_back(-2, 0.4, 1);
    sphere_radii.emplace_back(1);

    sphere_centers.emplace_back(-5, 0.8, -1);
    sphere_radii.emplace_back(1);

    // sphere_centers.emplace_back(-8, 1.6, 1);
    // sphere_radii.emplace_back(1);

    //Parallelograms
    parallelograms.emplace_back();
    parallelograms.back() << -100, 100, -100,
        -1.25, 0, -1.2,
        -100, -100, 100;

    //Lights
    light_positions.emplace_back(8, 8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(6, -8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(4, 8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(2, -8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(0, 8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(-2, -8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(-4, 8, 0);
    light_colors.emplace_back(16, 16, 16, 0);
}

////////////////////////////////////////////////////////////////////////////////
// BVH Code
////////////////////////////////////////////////////////////////////////////////

AlignedBox3d bbox_from_triangle(const Vector3d &a, const Vector3d &b, const Vector3d &c)
{
    AlignedBox3d box;
    box.extend(a);
    box.extend(b);
    box.extend(c);
    return box;
}

AABBTree::AABBTree(const MatrixXd &V, const MatrixXi &F)
{
    // Compute the centroids of all the triangles in the input mesh
    MatrixXd centroids(F.rows(), V.cols());
    centroids.setZero();
    for (int i = 0; i < F.rows(); ++i)
    {
        for (int k = 0; k < F.cols(); ++k)
        {
            centroids.row(i) += V.row(F(i, k));
        }
        centroids.row(i) /= F.cols();
    }

    //Vector containing the list of triangle indices
    std::vector<int> triangles(F.rows());
    std::iota(triangles.begin(), triangles.end(), 0);

    root = build_recursive(V, F, centroids, 0, triangles.size(), -1, triangles);
}

int AABBTree::build_recursive(const MatrixXd &V, const MatrixXi &F, const MatrixXd &centroids, int from, int to, int parent, std::vector<int> &triangles)
{
    // Scene is empty, so is the aabb tree
    if (to - from == 0)
    {
        return -1;
    }

    // If there is only 1 triangle left, then we are at a leaf
    if (to - from == 1)
    {
        //TODO create leaf node and return correct left index
        Node leaf;
        leaf.left = -1;
        leaf.right = -1;
        leaf.parent = parent;
        leaf.triangle = triangles[from];

        Vector3d v1 = V.row(F(leaf.triangle, 0));
        Vector3d v2 = V.row(F(leaf.triangle, 1));
        Vector3d v3 = V.row(F(leaf.triangle, 2));
        leaf.bbox = bbox_from_triangle(v1, v2, v3);

        nodes.emplace_back(leaf);
        return nodes.size() - 1;
    }

    AlignedBox3d centroid_box;

    //TODO Use AlignedBox3d to find the box around the current centroids
    for (int i = from; i < to; ++i) {
        Vector3d center(centroids(i, 0), centroids(i, 1), centroids(i, 2));
        centroid_box.extend(center);
    }

    // Diagonal of the box
    Vector3d extent = centroid_box.diagonal();

    //TODO find the largest dimension
    int longest_dim = 2;
    if (extent[0] >= extent[1] && extent[0] >= extent[2]) {
        longest_dim = 0;
    } else if (extent[1] >= extent[0] && extent[1] >= extent[2]) {
        longest_dim = 1;
    }

    // TODO sort centroids along the longest dimension
    std::sort(triangles.begin() + from, triangles.begin() + to, [&](int f1, int f2) {
        //TODO sort the **triangles** along the centroid largest dimension
        // return true if triangle f1 comes before triangle f2
        Vector3d c1 = centroids.row(f1);
        Vector3d c2 = centroids.row(f2);
        return c1[longest_dim] < c2[longest_dim];
    });

    //TODO Create a new internal node and do a recursive call to build the left and right part of the tree
    //TODO finally return the correct index

    // // compute the halfway point to split at
    int half = (from + to) / 2;

    // create the new node, add it to the list and get index
    Node new_node;
    nodes.emplace_back(new_node);
    int node_index = nodes.size() - 1;

    // Find the bounding box for the node
    for (int i = from; i < to; i++) {
        int triangle = triangles[i];
        Vector3d v1 = V.row(F(triangle, 0));
        Vector3d v2 = V.row(F(triangle, 1));
        Vector3d v3 = V.row(F(triangle, 2));
        nodes[node_index].bbox.extend(bbox_from_triangle(v1, v2, v3));
    }

    // Find the parent, triangle, and left/right node indexes
    nodes[node_index].parent = parent;
    nodes[node_index].triangle = -1;
    int left = build_recursive(V, F, centroids, from, half, node_index, triangles);;
    int right = build_recursive(V, F, centroids, half, to, node_index, triangles);;

    nodes[node_index].left = left;
    nodes[node_index].right = right;

    return node_index;
}

////////////////////////////////////////////////////////////////////////////////
// Intersection code
////////////////////////////////////////////////////////////////////////////////

double ray_triangle_intersection(const Vector3d &ray_origin, const Vector3d &ray_direction, const Vector3d &a, const Vector3d &b, const Vector3d &c, Vector3d &p, Vector3d &N)
{
    // TODO
    // Compute whether the ray intersects the given triangle.
    // If you have done the parallelogram case, this should be very similar to it.
    const Vector3d triangle_origin = a;
    const Vector3d triangle_u = b - a;
    const Vector3d triangle_v = c - a;

    MatrixXd coefficients(3, 3);
    coefficients(0, 0) = triangle_u[0];
    coefficients(1, 0) = triangle_u[1];
    coefficients(2, 0) = triangle_u[2];
    coefficients(0, 1) = triangle_v[0];
    coefficients(1, 1) = triangle_v[1];
    coefficients(2, 1) = triangle_v[2];
    coefficients(0, 2) = -ray_direction[0];
    coefficients(1, 2) = -ray_direction[1];
    coefficients(2, 2) = -ray_direction[2];

    Vector3d answers = ray_origin - triangle_origin;

    coefficients = coefficients.inverse();
    Vector3d solution = coefficients * answers;
    const double u = solution[0];
    const double v = solution[1];
    const double t = solution[2];

    if (u >= 0 && v >= 0 && t >= 0 && u + v <= 1)
    {
        p = triangle_origin + u * triangle_u + v * triangle_v;
        N = triangle_u.cross(triangle_v).normalized();

        return t;
    }

    return -1;
}

//Compute the intersection between a ray and a sphere, return -1 if no intersection
double ray_sphere_intersection(const Vector3d &ray_origin, const Vector3d &ray_direction, int index, Vector3d &p, Vector3d &N)
{
    // TODO, implement the intersection between the ray and the sphere at index index.
    //return t or -1 if no intersection

    const Vector3d sphere_center = sphere_centers[index];
    const double sphere_radius = sphere_radii[index];

    double t = -1;

    const double term_1 = ray_direction.dot(ray_direction);
    const double term_2 = 2 * (ray_origin.dot(ray_direction) - sphere_center.dot(ray_direction));
    const double term_3 = (ray_origin - sphere_center).dot(ray_origin - sphere_center) - sphere_radius * sphere_radius;
    const double desc = term_2 * term_2 - 4 * term_1 * term_3;

    if (desc < 0)
    {   
        return -1;
    }
    else
    {

        t = (-term_2 - sqrt(desc)) / (2 * term_1);
        if (t < 0) {
            t = (-term_2 + sqrt(desc)) / (2 * term_1);
            if (t < 0) {
                return -1;
            }
        }

        //TODO set the correct intersection point, update p to the correct value
        p = ray_origin + t * ray_direction;
        N = (p - sphere_center).normalized();

        return t;
    }

    return -1;
}

// Compute the intersection between a ray and a paralleogram, return -1 if no intersection
double ray_parallelogram_intersection(const Vector3d &ray_origin, const Vector3d &ray_direction, int index, Vector3d &p, Vector3d &N)
{
    // TODO, implement the intersection between the ray and the parallelogram at index index.
    //return t or -1 if no intersection

    const Vector3d pgram_origin = parallelograms[index].col(0);
    const Vector3d A = parallelograms[index].col(1);
    const Vector3d B = parallelograms[index].col(2);
    const Vector3d pgram_u = A - pgram_origin;
    const Vector3d pgram_v = B - pgram_origin;

    MatrixXd coefficients(3, 3);
    coefficients(0, 0) = pgram_u[0];
    coefficients(1, 0) = pgram_u[1];
    coefficients(2, 0) = pgram_u[2];
    coefficients(0, 1) = pgram_v[0];
    coefficients(1, 1) = pgram_v[1];
    coefficients(2, 1) = pgram_v[2];
    coefficients(0, 2) = -ray_direction[0];
    coefficients(1, 2) = -ray_direction[1];
    coefficients(2, 2) = -ray_direction[2];

    Vector3d answers = ray_origin - pgram_origin;

    coefficients = coefficients.inverse();
    Vector3d solution = coefficients * answers;
    const double u = solution[0];
    const double v = solution[1];
    const double t = solution[2];

    if (u >= 0 && v >= 0 && t >= 0 && u <= 1 && v <= 1)
    {
        p = pgram_origin + u * pgram_u + v * pgram_v;
        N = -pgram_u.cross(pgram_v).normalized();

        return t;
    }

    return -1;
}

bool ray_box_intersection(const Vector3d &ray_origin, const Vector3d &ray_direction, const AlignedBox3d &box)
{
    // TODO
    // Compute whether the ray intersects the given box.
    // we are not testing with the real surface here anyway.
    Vector3d min_corner = box.min();
    Vector3d max_corner = box.max();

    float t1 = (min_corner[0] - ray_origin[0]) / ray_direction[0];
    float t2 = (max_corner[0] - ray_origin[0]) / ray_direction[0];
    float t3 = (min_corner[1] - ray_origin[1]) / ray_direction[1];
    float t4 = (max_corner[1] - ray_origin[1]) / ray_direction[1];
    float t5 = (min_corner[2] - ray_origin[2]) / ray_direction[2];
    float t6 = (max_corner[2] - ray_origin[2]) / ray_direction[2];

    float tmin = std::max(std::max(std::min(t1, t2), std::min(t3, t4)), std::min(t5, t6));
    float tmax = std::min(std::min(std::max(t1, t2), std::max(t3, t4)), std::max(t5, t6));

    return !(tmax < 0) && !(tmin > tmax);
}

std::vector<int> get_leaves (const Vector3d &ray_origin, const Vector3d &ray_direction, int &node_index) {
    
    std::vector<int> leaves;
    AABBTree::Node cur_node = bvh.nodes[node_index];

    if (cur_node.triangle != -1) {
        leaves.emplace_back(cur_node.triangle);
        return leaves;
    }
    
    bool intersects = ray_box_intersection(ray_origin, ray_direction, cur_node.bbox);
    
    std::vector<int> left_leaves;
    std::vector<int> right_leaves;

    if (intersects && cur_node.left != -1) {
        left_leaves = get_leaves(ray_origin, ray_direction, cur_node.left);
        for (int num: left_leaves) {
            leaves.emplace_back(num);
        }
    } 

    if (intersects && cur_node.right != -1) {
        right_leaves = get_leaves(ray_origin, ray_direction, cur_node.right);
        for (int num: right_leaves) {
            leaves.emplace_back(num);
        }
    } 

    return leaves;
} 

//Finds the closest intersecting object returns its index
//In case of intersection it writes into p and N (intersection point and normals)
bool find_nearest_object(const Vector3d &ray_origin, const Vector3d &ray_direction, Vector3d &p, Vector3d &N)
{
    Vector3d tmp_p, tmp_N;
    int closest_index = -1;
    double closest_t = std::numeric_limits<double>::max(); //closest t is "+ infinity"

    for (int i = 0; i < sphere_centers.size(); ++i)
    {
        //returns t and writes on tmp_p and tmp_N
        const double t = ray_sphere_intersection(ray_origin, ray_direction, i, tmp_p, tmp_N);
        //We have intersection
        if (t >= 0)
        {
            //The point is before our current closest t
            if (t < closest_t)
            {
                closest_index = i;
                closest_t = t;
                p = tmp_p;
                N = tmp_N;
            }
        }
    }

    for (int i = 0; i < parallelograms.size(); ++i)
    {
        //returns t and writes on tmp_p and tmp_N
        const double t = ray_parallelogram_intersection(ray_origin, ray_direction, i, tmp_p, tmp_N);
        //We have intersection
        if (t >= 0)
        {
            //The point is before our current closest t
            if (t < closest_t)
            {
                closest_index = sphere_centers.size() + i;
                closest_t = t;
                p = tmp_p;
                N = tmp_N;
            }
        }
    }

    // TODO
    // Method (1): Traverse every triangle and return the closest hit.
    // for (int i = 0; i < facets.rows(); ++i)
    // {
    //     Vector3d v1 = vertices.row(facets(i, 0));
    //     Vector3d v2 = vertices.row(facets(i, 1));
    //     Vector3d v3 = vertices.row(facets(i, 2));

    //     const double t = ray_triangle_intersection(ray_origin, ray_direction, v1, v2, v3, tmp_p, tmp_N);
    //     //We have intersection
    //     if (t >= 0)
    //     {
    //         //The point is before our current closest t
    //         if (t < closest_t)
    //         {
    //             closest_index = i;
    //             closest_t = t;
    //             p = tmp_p;
    //             N = tmp_N;
    //         }
    //     }
    // }

    // Method (2): Traverse the BVH tree and test the intersection with a
    // triangles at the leaf nodes that intersects the input ray.
    std::vector<int> leaves = get_leaves(ray_origin, ray_direction, bvh.root);
    for (int i: leaves) {
        Vector3d v1 = vertices.row(facets(i, 0));
        Vector3d v2 = vertices.row(facets(i, 1));
        Vector3d v3 = vertices.row(facets(i, 2));

        const double t = ray_triangle_intersection(ray_origin, ray_direction, v1, v2, v3, tmp_p, tmp_N);
        //We have intersection
        if (t >= 0)
        {
            //The point is before our current closest t
            if (t < closest_t)
            {
                closest_index = i;
                closest_t = t;
                p = tmp_p;
                N = tmp_N;
            }
        }
    }

    return closest_index != -1;
}

////////////////////////////////////////////////////////////////////////////////
// Raytracer code
////////////////////////////////////////////////////////////////////////////////

//Checks if the light is visible
bool is_light_visible(const Vector3d &ray_origin, const Vector3d &ray_direction, const Vector3d &light_position, const int &object_num)
{

    // TODO: Determine if the light is visible here
    // Use find_nearest_object
    Vector3d p, N;
    const bool nearest_object = find_nearest_object(ray_origin, ray_direction, p, N);

    return !nearest_object;// || dist_to_object > dist_to_light ;
}

Vector4d shoot_ray(const Vector3d &ray_origin, const Vector3d &ray_direction, int max_bounce)
{
    //Intersection point and normal, these are output of find_nearest_object
    Vector3d p, N;

    const bool nearest_object = find_nearest_object(ray_origin, ray_direction, p, N);

    if (!nearest_object)
    {
        // Return a transparent color
        return Vector4d(0, 0, 0, 0);
    }

    // Ambient light contribution
    const Vector4d ambient_color = obj_ambient_color.array() * ambient_light.array();

    // Punctual lights contribution (direct lighting)
    Vector4d lights_color(0, 0, 0, 0);
    for (int i = 0; i < light_positions.size(); ++i)
    {
        const Vector3d &light_position = light_positions[i];
        const Vector4d &light_color = light_colors[i];

        const Vector3d Li = (light_position - p).normalized();

        // TODO: Shoot a shadow ray to determine if the light should affect the intersection point and call is_light_visible
        if (!is_light_visible(p + 0.001 * Li, Li, light_position, nearest_object)) {
            continue;
        }

        Vector4d diff_color = obj_diffuse_color;

        // Diffuse contribution
        const Vector4d diffuse = diff_color * std::max(Li.dot(N), 0.0);

        // Specular contribution
        const Vector3d Hi = (Li - ray_direction).normalized();
        const Vector4d specular = obj_specular_color * std::pow(std::max(N.dot(Hi), 0.0), obj_specular_exponent);
        // Vector3d specular(0, 0, 0);

        // Attenuate lights according to the squared distance to the lights
        const Vector3d D = light_position - p;
        lights_color += (diffuse + specular).cwiseProduct(light_color) / D.squaredNorm();
    }

    Vector4d refl_color = obj_reflection_color;
    
    // TODO: Compute the color of the reflected ray and add its contribution to the current point color.
    // use refl_color
    Vector4d reflection_color(0, 0, 0, 0);

    if (max_bounce > 0) {
        const Vector3d d = (p - ray_origin).normalized();
        const Vector3d n = N;

        const Vector3d r = (d - 2 * d.dot(n) * n).normalized();
        const Vector4d ray_color = shoot_ray(p + 0.001 * r, r, max_bounce - 1);

        reflection_color[0] = ray_color[0] * refl_color[0];
        reflection_color[1] = ray_color[1] * refl_color[1];
        reflection_color[2] = ray_color[2] * refl_color[2];
        reflection_color[3] = ray_color[3] * refl_color[3];
    }

    // Rendering equation
    Vector4d C = ambient_color + lights_color + reflection_color;

    //Set alpha to 1
    C(3) = 1;

    return C;
}

////////////////////////////////////////////////////////////////////////////////

void raytrace_scene()
{
    std::cout << "Simple ray tracer." << std::endl;

    int w = 640;
    int h = 480;
    MatrixXd R = MatrixXd::Zero(w, h);
    MatrixXd G = MatrixXd::Zero(w, h);
    MatrixXd B = MatrixXd::Zero(w, h);
    MatrixXd A = MatrixXd::Zero(w, h); // Store the alpha mask

    // The camera always points in the direction -z
    // The sensor grid is at a distance 'focal_length' from the camera center,
    // and covers an viewing angle given by 'field_of_view'.
    double aspect_ratio = double(w) / double(h);
    //TODO
    double image_y = focal_length * tan(field_of_view / 2);;
    double image_x = aspect_ratio * image_y;

    // The pixel grid through which we shoot rays is at a distance 'focal_length'
    const Vector3d image_origin(-image_x, image_y, camera_position[2] - focal_length);
    const Vector3d x_displacement(2.0 / w * image_x, 0, 0);
    const Vector3d y_displacement(0, -2.0 / h * image_y, 0);

    for (unsigned i = 0; i < w; ++i)
    {
        for (unsigned j = 0; j < h; ++j)
        {
            const Vector3d pixel_center = image_origin + (i + 0.5) * x_displacement + (j + 0.5) * y_displacement;

            // Prepare the ray
            Vector3d ray_origin;
            Vector3d ray_direction;

            if (is_perspective)
            {
                // Perspective camera
                ray_origin = camera_position;
                ray_direction = (pixel_center - camera_position).normalized();
            }
            else
            {
                // Orthographic camera
                ray_origin = pixel_center;
                ray_direction = Vector3d(0, 0, -1);
            }

            const Vector4d C = shoot_ray(ray_origin, ray_direction, max_bounce);
            R(i, j) = C(0);
            G(i, j) = C(1);
            B(i, j) = C(2);
            A(i, j) = C(3);
        }
    }

    // Save to png
    write_matrix_to_png(R, G, B, A, filename);
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char *argv[])
{
    setup_scene();

    raytrace_scene();
    return 0;
}
