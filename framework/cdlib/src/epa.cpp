#include "epa.hpp"

namespace cdlib {
    std::optional<CollisionData> EPA::get_collision_data() {
        // Get the current face normals
        auto [face_normals, min_triangle] = get_face_normals(faces);

        glm::vec3 min_normal;
        float min_distance = std::numeric_limits<float>::max();

        while (min_distance == std::numeric_limits<float>::max()) {
            // Get the normal and distance of the closest triangle to the origin
            min_normal = face_normals[min_triangle].normal;
            min_distance = face_normals[min_triangle].distance;

            // Get the support point (on the Minkowski difference) in the direction of the normal
            auto support = collider_1->support(min_normal) - collider_2->support(-min_normal);
            float distance = dot(min_normal, support);

            // Check if the support point is far enough from the current min triangle
            if (abs(distance - min_distance) > EPA_EPSILON) {
                min_distance = std::numeric_limits<float>::max();
                std::vector<Edge> uniqueEdges;

                // Iterate through the faces of the polytope and retrieve a list of unique edges
                for (size_t i = 0; i < face_normals.size(); i++) {
                    // If the normal and support vector are facing the same direction
                    if (dot(face_normals[i].normal, support) > dot(face_normals[i].normal, polytope[faces[i*3]])) {
                        size_t face_idx = i * 3;

                        // Add the current triangle's edges to the list of unique edges
                        add_unique_edge(uniqueEdges, face_idx, face_idx + 1);
                        add_unique_edge(uniqueEdges, face_idx + 1, face_idx + 2);
                        add_unique_edge(uniqueEdges, face_idx + 2, face_idx);

                        // Update the faces
                        faces[face_idx + 2] = faces.back(); faces.pop_back();
                        faces[face_idx + 1] = faces.back(); faces.pop_back();
                        faces[face_idx] = faces.back(); faces.pop_back();

                        // Update the face normals
                        face_normals[i] = face_normals.back();
                        face_normals.pop_back();

                        // Decrement the counter because we removed a face
                        i--;
                    }
                }

                // Create a list of new faces using the unique edges and the new support vertex
                std::vector<size_t> new_faces;
                for (const auto [index_1, index_2] : uniqueEdges) {
                    new_faces.push_back(index_1);
                    new_faces.push_back(index_2);
                    new_faces.push_back(polytope.size()); // The new support vertex
                }

                polytope.push_back(support); // Add the new support vertex

                // Get the new face normals and the closest triangle to the origin with the newly added support vertex
                auto [new_face_normals, new_min_triangle] = get_face_normals(new_faces);

                float old_min_distance = std::numeric_limits<float>::max();
                for (size_t i = 0; i < face_normals.size(); i++) {
                    if (face_normals[i].distance < old_min_distance) {
                        old_min_distance = face_normals[i].distance;
                        min_triangle = i;
                    }
                }

                if (new_face_normals[new_min_triangle].distance < old_min_distance) {
                    min_triangle = new_min_triangle + face_normals.size();
                }

                faces.insert(faces.end(), new_faces.begin(), new_faces.end());
                face_normals.insert(face_normals.end(), new_face_normals.begin(), new_face_normals.end());
            }
        }

        CollisionData collision_data;
        collision_data.normal = min_normal;
        collision_data.depth = min_distance + EPA_EPSILON;
        return collision_data;
    }

    FaceNormalsData EPA::get_face_normals(const std::vector<size_t>& face_vector) const {
        std::vector<FaceNormal> face_normals;
        size_t minTriangle = 0;
        float minDistance = std::numeric_limits<float>::max();

        // Iterate through the faces of the polytope
        for (auto i = face_vector.cbegin(); i != face_vector.cend();) {
            // Get the vertices of the triangle
            const glm::vec3& a = polytope[*i++];
            const glm::vec3& b = polytope[*i++];
            const glm::vec3& c = polytope[*i++];

            // Calculate the normal and distance of the triangle
            glm::vec3 normal = normalize(cross(b - a, c - a));
            float distance = dot(normal, a);

            // Flip the normal and distance if the normal is facing the wrong way
            if (distance < 0) {
                normal = -normal;
                distance = -distance;
            }

            // Add the normal and distance to the list of face normals
            face_normals.emplace_back(normal, distance);

            // Check if the triangle is the closest to the origin
            if (distance < minDistance) {
                minTriangle = face_normals.size() - 1;
                minDistance = distance;
            }
        }

        return { std::move(face_normals), minTriangle };
    }

    void EPA::add_unique_edge(std::vector<Edge>& edges, const size_t a, const size_t b) const {
        // A neighboring face edge is in reverse order, due to the winding order of the triangle
        // We want to only store unique edges as to later repair the faces with the newly added support vertex
        if (const auto reverse_edge = std::ranges::find(edges, Edge{faces[b], faces[a]}); reverse_edge == edges.end()) {
            edges.emplace_back(faces[a], faces[b]);
        } else {
            edges.erase(reverse_edge);
        }
    }
}