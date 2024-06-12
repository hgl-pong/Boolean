#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/intersect.hpp>
#include <array>
#include "stb_image_write.h"

// A basic OBJ file loader
class ObjLoader {
public:
    std::vector<glm::vec3> vertices;
    std::vector<glm::vec3> normals;
    //std::vector<glm::vec2> uvs;
    std::vector<std::array<int, 9>> triangle_indices;

    // Load an OBJ file
    bool load_obj(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Failed to open file " << filename << std::endl;
            return false;
        }

        std::vector<glm::vec3> temp_vertices;
        std::vector<glm::vec3> temp_normals;
        std::vector<glm::vec2> temp_uvs;

        std::string line;
        while (std::getline(file, line)) {
            std::istringstream iss(line);
            std::string prefix;
            iss >> prefix;

            if (prefix == "v") {
                glm::vec3 v;
                iss >> v.x >> v.y >> v.z;
                temp_vertices.push_back(v);
            }
            else if (prefix == "vn") {
                glm::vec3 vn;
                iss >> vn.x >> vn.y >> vn.z;
                temp_normals.push_back(vn);
            }
            else if (prefix == "vt") {
                glm::vec2 vt;
                iss >> vt.x >> vt.y;
                temp_uvs.push_back(vt);
            }
            else if (prefix == "f") {
                std::array<int, 9> face;
                char slash;
                for (int i = 0; i < 3; ++i) {
                    iss >> face[3 * i] >> slash >> face[3 * i + 1] >> slash >> face[3 * i + 2];
                }
                triangle_indices.push_back(face);
            }
        }

        // Convert the triangle indices to vertex and normal indices
        for (const auto& indices : triangle_indices) {
            vertices.push_back(temp_vertices[indices[0] - 1]);
            vertices.push_back(temp_vertices[indices[3] - 1]);
            vertices.push_back(temp_vertices[indices[6] - 1]);
            normals.push_back(temp_normals[indices[2] - 1]);
            normals.push_back(temp_normals[indices[5] - 1]);
            normals.push_back(temp_normals[indices[8] - 1]);
            //if (!temp_uvs.empty()) {
            //    uvs.push_back(temp_uvs[indices[1] - 1]);
            //    uvs.push_back(temp_uvs[indices[4] - 1]);
            //    uvs.push_back(temp_uvs[indices[7] - 1]);
            //}
        }

        // Compute model center and radius
        glm::vec3 min_coords(std::numeric_limits<float>::max());
        glm::vec3 max_coords(std::numeric_limits<float>::lowest());
        for (const auto& v : vertices) {
            min_coords = glm::min(min_coords, v);
            max_coords = glm::max(max_coords, v);
        }
        const auto center = (min_coords + max_coords) / 2.0f;
        float radius = 0.0f;
        for (const auto& v : vertices) {
            radius = std::max(radius, glm::length(v - center));
        }
        std::cout << "Loaded " << filename << " (" << vertices.size() << " vertices, " << triangle_indices.size() << " triangles)" << std::endl;
        std::cout << "Model center: " << center.x << ", " << center.y << ", " << center.z << std::endl;
        std::cout << "Model radius: " << radius << std::endl;
        return true;
    }
};

// Map the vertices in the given OBJ file to a 2D texture space using a simple box projection
class UvMapper {
public:
    // Map the vertices to 2D texture coordinates
    std::vector<glm::vec2> map_vertices(const ObjLoader& obj_loader) {
        std::vector<glm::vec2> tex_coords;
        for (const auto& v : obj_loader.vertices) {
            //const float u = (ver.x - min_coords_.x) / (max_coords_.x - min_coords_.x);
            //const float v = (ver.y - min_coords_.y) / (max_coords_.y - min_coords_.y);
            //
            float pi = acos(-1.0f);
            float theta = atan2(v.z, v.x);
            float phi = atan2(sqrt(v.x * v.x + v.z * v.z), v.y);

            const float ux = (theta + pi) / (2 * pi);
            const float uy = phi / pi;

            tex_coords.emplace_back(ux, uy);
        }
        return tex_coords;
    }
    // Compute the bounding box of the given vertices and set the projection parameters accordingly
    void set_projection(const std::vector<glm::vec3>& vertices) {
        min_coords_ = glm::vec3(std::numeric_limits<float>::max());
        max_coords_ = glm::vec3(std::numeric_limits<float>::lowest());
        for (const auto& v : vertices) {
            min_coords_ = glm::min(min_coords_, v);
            max_coords_ = glm::max(max_coords_, v);
        }
    }
private:
    glm::vec3 min_coords_;
    glm::vec3 max_coords_;
};

// A function to create a PNG image from an array of RGB pixel values
void save_png_image(const std::string& filename, const int width, const int height, const unsigned char* data) {
    stbi_write_png(filename.c_str(), width, height, 3, data, 0);
}

namespace glm {
    static float	intersect_ray_triangle(vec3 P1, vec3 P2, vec3 P3, vec3 O, vec3 D)
    {
        vec3 e1 = P2 - P1;
        vec3 e2 = P3 - P1;
        vec3 pvec = cross(D, e2);
        float det = dot(e1, pvec);

        if (det < 1e-8 && det > -1e-8) {
            return INFINITY;
        }

        float inv_det = 1 / det;
        vec3 tvec = O - P1;
        float u = dot(tvec, pvec) * inv_det;
        if (u < 0 || u > 1) {
            return INFINITY;
        }

        vec3 qvec = cross(tvec, e1);
        float v = dot(D, qvec) * inv_det;
        if (v < 0 || u + v > 1) {
            return INFINITY;
        }
        return dot(e2, qvec) * inv_det;
    }
}

int main() {
    // Load the OBJ file
    ObjLoader obj_loader;
    if (!obj_loader.load_obj("beshon.obj")) {
        return 1;
    }
    // Map the vertices to texture space
    UvMapper uv_mapper;
    uv_mapper.set_projection(obj_loader.vertices);
    const auto tex_coords = uv_mapper.map_vertices(obj_loader);

    // Compute the texture dimensions and allocate memory for the texture data
    const int texture_width = 1024;
    const int texture_height = 1024;
    const int num_pixels = texture_width * texture_height;
    std::vector<unsigned char> texture_data(num_pixels * 3);

    // Sample the texture by rendering the triangles with the vertex colors
    for (std::size_t i = 0; i < obj_loader.triangle_indices.size(); ++i) {
        const auto& indices = obj_loader.triangle_indices[i];
        const auto& v0 = obj_loader.vertices[indices[0] - 1];
        const auto& v1 = obj_loader.vertices[indices[3] - 1];
        const auto& v2 = obj_loader.vertices[indices[6] - 1];
        const auto& uv0 = tex_coords[3 * i];
        const auto& uv1 = tex_coords[3 * i + 1];
        const auto& uv2 = tex_coords[3 * i + 2];
        const auto e1 = v1 - v0;
        const auto e2 = v2 - v0;
        const auto n = glm::normalize(glm::cross(e1, e2));
        // Compute the bounding box of the triangle in texture space
        const float u_min = std::min({ uv0.x, uv1.x, uv2.x });
        const float u_max = std::max({ uv0.x, uv1.x, uv2.x });
        const float v_min = std::min({ uv0.y, uv1.y, uv2.y });
        const float v_max = std::max({ uv0.y, uv1.y, uv2.y });
        const int u_min_pixel = static_cast<int>(u_min * texture_width);
        const int u_max_pixel = static_cast<int>(u_max * texture_width);
        const int v_min_pixel = static_cast<int>(v_min * texture_height);
        const int v_max_pixel = static_cast<int>(v_max * texture_height);

        // Render the triangle by sampling the texture at each pixel within the bounding box
        for (int y = v_min_pixel; y <= v_max_pixel; ++y) {
            for (int x = u_min_pixel; x <= u_max_pixel; ++x) {
                const float u = (x + 0.5f) / texture_width;
                const float v = (y + 0.5f) / texture_height;
                if (glm::intersect_ray_triangle(
                    glm::vec3(u, v, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f),
                    v0, e1, e2/*, n, glm::vec3(0.0f),
                    glm::vec3(0.0f), glm::vec3(0.0f)*/)) {
                    const auto color = obj_loader.vertices[indices[0] - 1];
                    const int index = (y * texture_width + x) * 3;
                    texture_data[index] = static_cast<unsigned char>(color.r * 255.0f);
                    texture_data[index + 1] = static_cast<unsigned char>(color.g * 255.0f);
                    texture_data[index + 2] = static_cast<unsigned char>(color.b * 255.0f);
                }
            }
        }
    }

    // Save the texture image to a PNG file
    save_png_image("texture.png", texture_width, texture_height, texture_data.data());

    return 0;
}
