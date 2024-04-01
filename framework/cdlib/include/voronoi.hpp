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
        std::variant<std::shared_ptr<Face>, std::shared_ptr<Vertex>> feature;
        // The edge that the plane is associated with
        std::shared_ptr<HalfEdge> edge;

        // Negation operator
        [[nodiscard]] VoronoiPlane operator-() const {
            Plane inverted_plane = Plane::operator-();
            return {inverted_plane, feature, edge};
        }
    };

    struct ClipData {
        bool is_clipped;
        float lambda_l;
        float lambda_h;
        std::shared_ptr<Feature> neighbour_l;
        std::shared_ptr<Feature> neighbour_h;
    };

    [[nodiscard]] inline VoronoiPlane get_voronoi_plane(const std::shared_ptr<Vertex>& vertex, const std::shared_ptr<HalfEdge>& edge);
    [[nodiscard]] inline VoronoiPlane get_voronoi_plane(const std::shared_ptr<Face>& face, const std::shared_ptr<HalfEdge>& edge);

    [[nodiscard]] std::optional<VoronoiPlane> get_voronoi_plane_safe(const std::shared_ptr<Feature>& feature_1, const std::shared_ptr<Feature>& feature_2, bool invert = false);

    template <typename T> requires IsFeature<T>
    [[nodiscard]] bool in_voronoi_region(const std::shared_ptr<T>& feature, const glm::vec3& point) {
        const auto neighbours = feature->get_neighbours();
        return std::ranges::all_of(neighbours, [&feature, &point](const std::shared_ptr<Feature>& neighbour){
            const auto plane = get_voronoi_plane_safe(feature, neighbour);
            if (!plane){
                std::cerr << "Feature does not have a valid edge neighbour" << std::endl;
                return false;
            }
            return plane->is_above(point);
        });
    }

    template <>
    [[nodiscard]] inline bool in_voronoi_region<Face>(const std::shared_ptr<Face>& feature, const glm::vec3& point) {
        // Run the same code as the general case and check if the point is above the face itself
        const auto base = in_voronoi_region<Feature>(feature, point);
        const auto above = feature->plane.is_above(point);
        return base && above;
    }

    template <>
    [[nodiscard]] inline bool in_voronoi_region<HalfEdge>(const std::shared_ptr<HalfEdge>& feature, const glm::vec3& point) {
        const auto neighbours = feature->get_neighbours();
        return std::ranges::all_of(neighbours, [&feature, &point](const std::shared_ptr<Feature>& neighbour){
            // If the neighbour is twin's face, then use the twin instead
            const auto twin = feature->twin;
            const auto plane = get_voronoi_plane_safe(neighbour == twin->face ? feature->twin : feature, neighbour, true);
            if (!plane){
                std::cerr << "Feature does not have a valid edge neighbour" << std::endl;
                return false;
            }
            return plane->is_above(point);
        });
    }

    [[nodiscard]] ClipData clip_edge(const std::shared_ptr<Feature>& feature, const std::shared_ptr<HalfEdge>& edge);
}
