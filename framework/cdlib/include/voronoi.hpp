#pragma once
#include <algorithm>
#include <memory>
#include <optional>
#include <stdexcept>
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

        [[nodiscard]] bool is_above(const glm::vec3& point) const {
            return dot(normal, point) + d >= 0;
        }
    };

    struct Feature {
        virtual ~Feature() = default;

        std::vector<std::shared_ptr<Feature>> neighbours{};

        friend bool operator==(const Feature& lhs, const Feature& rhs) = default;
        friend bool operator!=(const Feature& lhs, const Feature& rhs) = default;

        void add_neighbour(const std::shared_ptr<Feature>& neighbour){
            neighbours.push_back(neighbour);
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


    [[nodiscard]] inline Plane get_voronoi_plane(const Vertex& vertex, const Edge& edge);
    [[nodiscard]] inline Plane get_voronoi_plane(const Face& face, const Edge& edge);

    [[nodiscard]] std::optional<Plane> get_voronoi_plane_safe(auto feature, const Edge& edge);

    template <typename T> requires std::is_base_of_v<Feature, T>
    [[nodiscard]] bool in_voronoi_region(const T& feature, const glm::vec3& point) {
        return std::ranges::all_of(feature.neighbours, [&feature, &point](const std::shared_ptr<Feature>& edge){
            const auto casted_edge = std::dynamic_pointer_cast<Edge>(edge);
            if (!casted_edge){
                throw std::runtime_error(T::class_name + " neighbour is not an edge");
            }
            return get_voronoi_plane(feature, *casted_edge).is_above(point);
        });
    }
}
