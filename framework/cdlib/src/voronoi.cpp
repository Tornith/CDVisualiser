#include "voronoi.hpp"

#include <algorithm>
#include <stdexcept>

namespace cdlib::Voronoi {
    VoronoiPlane get_voronoi_plane(const std::shared_ptr<Vertex>& vertex, const std::shared_ptr<HalfEdge>& edge) {
        const auto other = edge->start == vertex ? *edge->end : *edge->start;
        const glm::vec3 edge_direction = normalize(vertex->position - other.position);
        const glm::vec3 face_normal = normalize(edge_direction);

        return {Plane(face_normal, vertex->position), vertex, edge};
    }

    VoronoiPlane get_voronoi_plane(const std::shared_ptr<Face>& face, const std::shared_ptr<HalfEdge>& edge) {
        const glm::vec3 edge_direction = normalize(edge->end->position - edge->start->position);
        const glm::vec3 face_normal = normalize(face->plane.normal);

        // The voronoi plane is perpendicular to the face normal and the edge direction
        const glm::vec3 plane_normal = normalize(cross(face_normal, edge_direction));
        return {Plane(plane_normal, edge->start->position), face, edge};
    }

    std::optional<VoronoiPlane> get_voronoi_plane_safe(const std::shared_ptr<Feature>& feature_1, const std::shared_ptr<Feature>& feature_2, bool invert) {
        // Get which feature is the edge and which is the other
        const auto first = std::dynamic_pointer_cast<HalfEdge>(feature_1);
        const auto second = std::dynamic_pointer_cast<HalfEdge>(feature_2);

        if ((!first && !second) || (first && second)){
            return std::nullopt;
        }

        const auto edge = first ? first : second;
        const auto feature = first ? feature_2 : feature_1;

        if (const auto casted_face = std::dynamic_pointer_cast<Face>(feature)){
            const auto plane = get_voronoi_plane(casted_face, edge);
            return invert ? -plane : plane;
        }

        if (const auto casted_vertex = std::dynamic_pointer_cast<Vertex>(feature)){
            const auto plane = get_voronoi_plane(casted_vertex, edge);
            return invert ? -plane : plane;
        }

        return std::nullopt;
    }

    ClipData clip_edge(const std::shared_ptr<Feature>& feature, const std::shared_ptr<HalfEdge>& edge) {
        auto is_clipped = true;
        float lambda_l = 0;
        float lambda_h = 1;
        std::shared_ptr<Feature> neighbour_l = nullptr;
        std::shared_ptr<Feature> neighbour_h = nullptr;

        const auto [head, tail] = std::pair{edge->start->position, edge->end->position};

        // For all feature-edge voronoi plane
        for (const auto& neighbour : feature->get_neighbours()){
            const auto voronoi_plane = get_voronoi_plane_safe(feature, neighbour);
            if (!voronoi_plane){
                continue;
            }

            // Get the distance of the edge points to the plane
            const float distance_head = voronoi_plane->distance_to(head);
            const float distance_tail = voronoi_plane->distance_to(tail);

            // Case 1: Both the points are below the plane => no intersection, the neighbouring feature is 'closer'
            if (distance_head < 0 && distance_tail < 0){
                is_clipped = false;
                neighbour_l = neighbour;
                neighbour_h = neighbour;
                break;
            }

            // Case 2: Tail is below the plane, head is above the plane
            if (distance_head > 0 && distance_tail < 0){
                const float lambda = distance_tail / (distance_tail - distance_head);
                if (lambda > lambda_l){
                    lambda_l = lambda;
                    neighbour_l = neighbour;
                    // Check if the lower bound is greater than the upper bound
                    if (lambda_l > lambda_h){
                        is_clipped = false;
                        break;
                    }
                }
            }

            // Case 3: Head is below the plane, tail is above the plane
            else if (distance_head < 0 && distance_tail > 0){
                const float lambda = distance_tail / (distance_tail - distance_head);
                if (lambda < lambda_h){
                    lambda_h = lambda;
                    neighbour_h = neighbour;
                    // Check if the lower bound is greater than the upper bound
                    if (lambda_l > lambda_h){
                        is_clipped = false;
                        break;
                    }
                }
            }
        }

        return {is_clipped, lambda_l, lambda_h, neighbour_l, neighbour_h};
    }


}
