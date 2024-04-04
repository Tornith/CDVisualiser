#pragma once
#include <algorithm>
#include <iostream>
#include <memory>
#include <optional>
#include <variant>
#include <glm/glm.hpp>

#include "convex_polyhedron.hpp"

namespace cdlib::Voronoi {
    template <typename T>
    concept IsFeature = std::is_base_of_v<Feature, T>;

    struct VoronoiPlane final : Plane {
        // The face/vertex that the plane is associated with
        std::shared_ptr<Feature> feature;
        // The edge that the plane is associated with
        std::shared_ptr<HalfEdge> edge;

        // Negation operator
        [[nodiscard]] VoronoiPlane operator-() const {
            Plane inverted_plane = Plane::operator-();
            return {inverted_plane, feature, edge};
        }
    };

    [[nodiscard]] inline VoronoiPlane get_voronoi_plane(const std::shared_ptr<Vertex>& vertex, const std::shared_ptr<HalfEdge>& edge);
    [[nodiscard]] inline VoronoiPlane get_voronoi_plane(const std::shared_ptr<Face>& face, const std::shared_ptr<HalfEdge>& edge);

    [[nodiscard]] std::optional<VoronoiPlane> get_voronoi_plane_safe(const std::shared_ptr<Feature>& feature, const std::shared_ptr<Feature>& neighbour);

    template <typename T> requires IsFeature<T>
    [[nodiscard]] std::pair<bool, std::optional<VoronoiPlane>> in_voronoi_region(const std::shared_ptr<T>& feature, const glm::vec3& point) {
        const auto neighbours = feature->get_neighbours();
        for (const auto& neighbour : neighbours) {
            const auto plane = get_voronoi_plane_safe(feature, neighbour);
            if (!plane) {
                std::cerr << "Feature does not have a valid edge neighbour" << std::endl;
                return {false, std::nullopt};
            }
            if (!plane->is_above(point)) {
                return {false, plane};
            }
        }
        return {true, std::nullopt};
    }

    template <>
    [[nodiscard]] inline std::pair<bool, std::optional<VoronoiPlane>> in_voronoi_region<Face>(const std::shared_ptr<Face>& feature, const glm::vec3& point) {
        // Run the same code as the general case and check if the point is above the face itself
        if (auto base = in_voronoi_region<Feature>(feature, point); !base.first) {
            return base;
        }
        if (!feature->plane.is_above(point)) {
            return {false, VoronoiPlane{feature->plane, feature, nullptr}};
        }
        return {true, std::nullopt};
    }

    template <typename T> requires IsFeature<T>
    [[nodiscard]] std::optional<VoronoiPlane> find_maximally_violating_voronoi_plane(const std::shared_ptr<T>& feature, const glm::vec3& point) {
        const auto neighbours = feature->get_neighbours();
        auto max_plane = std::optional<VoronoiPlane>{};
        auto max_distance = std::numeric_limits<float>::max();

        for (const auto& neighbour : neighbours) {
            const auto plane = get_voronoi_plane_safe(feature, neighbour);
            if (!plane) {
                std::cerr << "Feature does not have a valid edge neighbour" << std::endl;
                continue;
            }
            const auto distance = plane->distance_to(point);
            if (distance < 0 && distance < max_distance) {
                max_distance = distance;
                max_plane = plane;
            }
        }

        return max_plane;
    }
}
