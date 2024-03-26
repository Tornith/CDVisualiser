#pragma once
#include <algorithm>
#include <iostream>
#include <memory>
#include <optional>
#include <variant>
#include <glm/glm.hpp>

#include "convex_polyhedron.hpp"

namespace cdlib::Voronoi {
    struct VoronoiPlane final : Plane {
        // The face/vertex that the plane is associated with
        std::variant<std::shared_ptr<Face>, std::shared_ptr<Vertex>> feature;
        // The edge that the plane is associated with
        std::shared_ptr<HalfEdge> edge;
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

    [[nodiscard]] std::optional<VoronoiPlane> get_voronoi_plane_safe(const std::shared_ptr<Feature>& feature, const std::shared_ptr<HalfEdge>& edge);
    [[nodiscard]] std::optional<VoronoiPlane> get_voronoi_plane_safe(const std::shared_ptr<Feature>& feature_1, const std::shared_ptr<Feature>& feature_2);

    template <typename T> requires std::is_base_of_v<Feature, T>
    [[nodiscard]] bool in_voronoi_region(const std::shared_ptr<T>& feature_1, const glm::vec3& point) {
        return std::ranges::all_of(feature_1->get_neighbours(), [&feature_1, &point](const std::shared_ptr<Feature>& feature_2){
            const auto plane = get_voronoi_plane_safe(feature_1, feature_2);
            if (!plane){
                std::cerr << "Feature does not have a valid edge neighbour" << std::endl;
                return false;
            }
            return plane->is_above(point);
        });
    }

    [[nodiscard]] inline ClipData clip_edge(const std::shared_ptr<Feature>& feature, const std::shared_ptr<HalfEdge>& edge);
}
