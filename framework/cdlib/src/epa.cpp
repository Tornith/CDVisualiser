#include "epa.hpp"

#include <algorithm>
#include <iostream>
#include <optional>
#include <ostream>
#include <unordered_map>

namespace cdlib {
    // Unique Edge struct
    struct UniqueEdge {
        size_t a;
        size_t b;

        bool operator==(const UniqueEdge& other) const {
            return (a == other.a && b == other.b) || (a == other.b && b == other.a);
        }
    };

    // Hash function for UniqueEdge
    struct UniqueEdgeHash {
        std::size_t operator()(const UniqueEdge& edge) const {
            return std::hash<size_t>()(edge.a) ^ std::hash<size_t>()(edge.b);
        }
    };

    std::pair<std::vector<glm::vec4>, size_t> get_face_normals(const std::vector<glm::vec3>& polytope, const std::vector<std::array<size_t, 3>>& faces) {
        std::vector<glm::vec4> normals;
        auto min_distance = std::numeric_limits<float>::max();
        size_t min_index = 0;

        for (const auto face : faces) {
            const auto a = polytope[face[0]];
            const auto b = polytope[face[1]];
            const auto c = polytope[face[2]];

            const auto normal = normalize(cross(b - a, c - a));
            const auto distance = dot(normal, a);
            const auto sign = glm::sign(distance);
            normals.emplace_back(normal * sign, std::abs(distance));

            if (std::abs(distance) < min_distance) {
                min_index = normals.size() - 1;
                min_distance = std::abs(distance);
            }
        }

        return {normals, min_index};
    }

    CollisionData EPA::get_collision_data() {
        // Get starting minimum face
        glm::vec3 min_normal;
        std::optional<float> min_distance = std::nullopt;

        bool repeated_point = false;

        while (!min_distance.has_value()) {
            auto [normals, min_index] = get_face_normals(polytope, faces);

            min_normal = glm::vec3(normals[min_index]);
            min_distance = normals[min_index].w;

            if (repeated_point) {
                break;
            }

            // Get support in the point of the minimal normal
            glm::vec3 support_1 = collider_1->support(min_normal);
            glm::vec3 support_2 = collider_2->support(-min_normal);

            glm::vec3 new_point = support_1 - support_2;

            // If the new point is already in the polytope, we must have reached the closest face
            if (std::ranges::find(polytope, new_point) != polytope.end()) {
                repeated_point = true;
                continue;
            }

            // If the difference in distances is less than epsilon, then we got the same point as before and the face is the closest
            if (std::abs(dot(new_point, min_normal) - min_distance.value()) < EPA_EPSILON) {
                break;
            }

            // Otherwise we continue and reset the minimum distance
            min_distance = std::nullopt;

            // Filter out faces that are facing in the same direction as the new point
            std::vector<glm::vec4> old_normals;
            std::vector<std::array<size_t, 3>> old_faces;

            std::vector<std::array<size_t, 3>> new_faces;
            std::unordered_map<UniqueEdge, size_t, UniqueEdgeHash> edge_counter;
            for (size_t i = 0; i < faces.size(); i++) {
                if (dot(glm::vec3(normals[i]), new_point) - normals[i].w <= 0) {
                    old_normals.push_back(normals[i]);
                    old_faces.push_back(faces[i]);
                    continue;
                }

                for (size_t j = 0; j < 3; j++) {
                    const auto edge = UniqueEdge{faces[i][j], faces[i][(j + 1) % 3]};
                    ++edge_counter[edge];
                }
            }

            std::vector<UniqueEdge> unique_edges;
            for (const auto& [edge, count] : edge_counter) {
                if (count == 1) unique_edges.push_back(edge);
            }

            // For each of the endpoints of the unique edges, create a new face with the new point
            for (const auto& [a, b] : unique_edges) {
                new_faces.push_back({a, b, polytope.size()});
            }

            polytope.push_back(new_point);

            // Concatenate the old and new arrays
            faces = old_faces;
            faces.insert(faces.end(), new_faces.begin(), new_faces.end());
        }

        return {
            true,
            min_normal,
            min_distance.value() + 0.001f
        };
    }
}
