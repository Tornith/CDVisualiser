#pragma once
#include <memory>
#include <vector>
#include <glm/glm.hpp>

/**
 * @file convex_polyhedron.hpp
 * @brief A class that represents a convex polyhedron with a half-edge DCEL data structure (Doubly Connected Edge List)
 */

namespace cdlib {
    struct Plane {
        glm::vec3 normal;
        float d;

        Plane() = default;

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

        friend bool operator==(const Feature& lhs, const Feature& rhs) = default;
        friend bool operator!=(const Feature& lhs, const Feature& rhs) = default;

        [[nodiscard]] virtual std::vector<std::shared_ptr<Feature>> get_neighbours() const = 0;
    };

    struct HalfEdge;

    struct Face final : Feature {
        Plane plane;
        std::vector<std::shared_ptr<HalfEdge>> edges;

        explicit Face(const Plane& plane)
            : plane(plane) {
        }

        Face(const glm::vec3& normal, const float d)
            : plane(normal, d) {
        }

        friend bool operator==(const Face& lhs, const Face& rhs) = default;
        friend bool operator!=(const Face& lhs, const Face& rhs) = default;

        [[nodiscard]] std::vector<std::shared_ptr<Feature>> get_neighbours() const override;
    };

    struct Vertex final : Feature {
        glm::vec3 position;
        std::vector<std::shared_ptr<HalfEdge>> edges;

        explicit Vertex(const glm::vec3& position)
            : position(position) {
        }

        friend bool operator==(const Vertex& lhs, const Vertex& rhs) = default;
        friend bool operator!=(const Vertex& lhs, const Vertex& rhs) = default;

        [[nodiscard]] std::vector<std::shared_ptr<Feature>> get_neighbours() const override;
    };

    struct HalfEdge final : Feature {
        std::shared_ptr<Vertex> start;
        std::shared_ptr<Vertex> end;
        std::shared_ptr<HalfEdge> twin;
        std::shared_ptr<HalfEdge> next;
        std::shared_ptr<HalfEdge> prev;
        std::shared_ptr<Face> face;

        HalfEdge(const std::shared_ptr<Vertex>& start, const std::shared_ptr<Vertex>& end)
            : start(start),
              end(end) {
        }

        friend bool operator==(const HalfEdge& lhs, const HalfEdge& rhs) = default;
        friend bool operator!=(const HalfEdge& lhs, const HalfEdge& rhs) = default;

        [[nodiscard]] std::vector<std::shared_ptr<Feature>> get_neighbours() const override;
    };

    struct ConvexPolyhedron {
        std::vector<std::shared_ptr<Vertex>> vertices;
        std::vector<std::shared_ptr<Face>> faces;
        std::vector<std::shared_ptr<HalfEdge>> hedges;

        ConvexPolyhedron() = default;

        [[nodiscard]] static std::shared_ptr<ConvexPolyhedron> build_dcel(const std::vector<glm::vec3>& vertices, const std::vector<std::vector<size_t>>& faces);
    };
}