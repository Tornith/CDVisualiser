#pragma once
#include <algorithm>
#include <memory>
#include <optional>
#include <stdexcept>
#include <variant>
#include <vector>
#include <glm/glm.hpp>

namespace cdlib::Voronoi {
    struct Plane {
        glm::vec3 normal;
        float d;

        Plane(const glm::vec3& normal, float d)
            : normal(normal),
              d(d) {
        }

        Plane(const glm::vec3& normal, const glm::vec3& point)
            : normal(normal),
              d(-dot(normal, point)) {
        }

        friend bool operator==(const Plane& lhs, const Plane& rhs) = default;
        friend bool operator!=(const Plane& lhs, const Plane& rhs) = default;

        [[nodiscard]] glm::vec4 to_vec4() const {
            return {normal, d};
        }

        [[nodiscard]] float distance_to(const glm::vec3& point) const {
            return dot(normal, point) + d;
        }

        [[nodiscard]] bool is_above(const glm::vec3& point) const {
            return distance_to(point) > 0;
        }

        [[nodiscard]] float get_intersection_parameter(const glm::vec3& point_a, const glm::vec3& point_b) const;
    };

    struct Feature {
        virtual ~Feature() = default;

        std::vector<std::shared_ptr<Feature>> neighbours{};

        friend bool operator==(const Feature& lhs, const Feature& rhs) = default;
        friend bool operator!=(const Feature& lhs, const Feature& rhs) = default;

        void add_neighbour(const std::shared_ptr<Feature>& neighbour){
            neighbours.push_back(neighbour);
        }

        void add_neighbours(const std::vector<std::shared_ptr<Feature>>& new_neighbours){
            neighbours.insert(neighbours.end(), new_neighbours.begin(), new_neighbours.end());
        }

        void clear_neighbours(){
            neighbours.clear();
        }

        [[nodiscard]] virtual bool in_voronoi_region(const glm::vec3& point) const = 0;
    };

    struct Face final : Feature{
        Plane plane;

        Face(const glm::vec3& normal, const glm::vec3& point)
            : plane({normal, point}){
        }

        Face(const glm::vec3& normal, float d)
            : plane({normal, d}){
        }

        explicit Face(const Plane& plane)
            : plane(plane) {
        }

        friend bool operator==(const Face& lhs, const Face& rhs) = default;
        friend bool operator!=(const Face& lhs, const Face& rhs) = default;

        [[nodiscard]] bool in_voronoi_region(const glm::vec3& point) const override;
    };

    struct Edge final : Feature{
        // Edge neighbours are faces and vertices
        glm::vec3 point_a;
        glm::vec3 point_b;

        Edge(const glm::vec3& point_a, const glm::vec3& point_b)
            : point_a(point_a),
              point_b(point_b) {
        }

        friend bool operator==(const Edge& lhs, const Edge& rhs) = default;
        friend bool operator!=(const Edge& lhs, const Edge& rhs) = default;

        [[nodiscard]] bool in_voronoi_region(const glm::vec3& point) const override;
    };

    struct Vertex final : Feature{
        // Vertex neighbours are edges
        glm::vec3 position;

        explicit Vertex(const glm::vec3& position)
            : position(position) {
        }

        friend bool operator==(const Vertex& lhs, const Vertex& rhs) = default;
        friend bool operator!=(const Vertex& lhs, const Vertex& rhs) = default;

        [[nodiscard]] bool in_voronoi_region(const glm::vec3& point) const override;
    };

    struct VoronoiObject {
        std::vector<std::shared_ptr<Vertex>> vertices;
        std::vector<std::shared_ptr<Edge>> edges;
        std::vector<std::shared_ptr<Face>> faces;
    };

    struct VoronoiPlane final : Plane {
        // The face/vertex that the plane is associated with
        std::variant<std::shared_ptr<Face>, std::shared_ptr<Vertex>> feature;
        // The edge that the plane is associated with
        std::shared_ptr<Edge> edge;
    };

    struct ClipData {
        bool is_clipped;
        float lambda_l;
        float lambda_h;
        std::shared_ptr<Feature> neighbour_l;
        std::shared_ptr<Feature> neighbour_h;
    };


    [[nodiscard]] inline VoronoiPlane get_voronoi_plane(const std::shared_ptr<Vertex>& vertex, const std::shared_ptr<Edge>& edge, bool inverse_normal = false);
    [[nodiscard]] inline VoronoiPlane get_voronoi_plane(const std::shared_ptr<Face>& face, const std::shared_ptr<Edge>& edge, bool inverse_normal = false);

    [[nodiscard]] std::optional<VoronoiPlane> get_voronoi_plane_safe(const std::shared_ptr<Feature>& feature, const std::shared_ptr<Edge>& edge, bool inverse_normal = false);

    template <typename T> requires std::is_base_of_v<Feature, T>
    [[nodiscard]] bool in_voronoi_region(const std::shared_ptr<T>& feature, const glm::vec3& point) {
        return std::ranges::all_of(feature->neighbours, [&feature, &point](const std::shared_ptr<Feature>& edge){
            const auto casted_edge = std::dynamic_pointer_cast<Edge>(edge);
            if (!casted_edge){
                throw std::runtime_error("Feature does not have a valid edge neighbour");
            }
            return get_voronoi_plane(feature, casted_edge).is_above(point);
        });
    }

    [[nodiscard]] inline ClipData clip_edge(const std::shared_ptr<Feature>& feature, const std::shared_ptr<Edge>& edge);
}
