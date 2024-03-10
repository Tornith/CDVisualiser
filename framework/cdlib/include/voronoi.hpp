#pragma once
#include <memory>
#include <optional>
#include <unordered_set>
#include <vector>
#include <glm/glm.hpp>

namespace cdlib{
    namespace Voronoi{
        struct Feature {
            std::vector<std::shared_ptr<Feature>> neighbours{};
        };

        struct Face : Feature{
            // Face neighbours are edges
            glm::vec3 normal;
            glm::vec3 point;

            Face(const glm::vec3& normal, const glm::vec3& point)
                : normal(normal),
                  point(point) {
            }
        };

        struct Edge : Feature{
            // Edge neighbours are faces and vertices
            glm::vec3 point_a;
            glm::vec3 point_b;
            glm::vec3 normal;

            Edge(const glm::vec3& point_a, const glm::vec3& point_b, const glm::vec3& normal)
                : point_a(point_a),
                  point_b(point_b),
                  normal(normal) {
            }
        };

        struct Vertex : Feature{
            // Vertex neighbours are edges
            glm::vec3 position;

            explicit Vertex(const glm::vec3& position)
                : position(position) {
            }
        };

        inline std::optional<glm::vec3> get_voronoi_plane(const Face& face, const Edge& edge){
            // Check if they are neighbours
            if (std::ranges::find(face.neighbours, std::make_shared<Feature>(edge)) != face.neighbours.end()){
                return face.normal;
            }
        }

        inline std::optional<glm::vec3> get_voronoi_plane(const Edge& edge, const Vertex& vertex){
            // Check if they are neighbours
            if (std::ranges::find(edge.neighbours, std::make_shared<Feature>(vertex)) != edge.neighbours.end()){
                return edge.normal;
            }
        }
    }
}