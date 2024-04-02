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

    std::optional<VoronoiPlane> get_voronoi_plane_safe(const std::shared_ptr<Feature>& feature, const std::shared_ptr<Feature>& neighbour) {
        // Get which feature is the edge and which is the other
        auto feature_edge = std::dynamic_pointer_cast<HalfEdge>(feature);
        const auto neighbour_edge = std::dynamic_pointer_cast<HalfEdge>(neighbour);

        // If neither or both are edges, return nothing
        if ((!feature_edge && !neighbour_edge) || (feature_edge && neighbour_edge)){
            return std::nullopt;
        }

        // If feature is an edge set the invert flag
        const auto invert = static_cast<bool>(feature_edge);

        // If the feature is an edge and the neighbour is a face, swap for twin if the neighbour is the twin's face
        if (feature_edge && std::dynamic_pointer_cast<Face>(neighbour) == feature_edge->twin->face){
            feature_edge = feature_edge->twin;
        }

        const auto edge = feature_edge ? feature_edge : neighbour_edge;
        const auto other = feature_edge ? neighbour : feature;

        if (const auto casted_face = std::dynamic_pointer_cast<Face>(other)){
            const auto plane = get_voronoi_plane(casted_face, edge);
            return invert ? -plane : plane;
        }

        if (const auto casted_vertex = std::dynamic_pointer_cast<Vertex>(other)){
            const auto plane = get_voronoi_plane(casted_vertex, edge);
            return invert ? -plane : plane;
        }

        return std::nullopt;
    }

    ClipData clip_edge(const std::shared_ptr<HalfEdge>& edge, const std::shared_ptr<Feature>& feature) {
        auto is_clipped = true;
        float lambda_l = 0;
        float lambda_h = 1;
        std::shared_ptr<Feature> neighbour_l = nullptr;
        std::shared_ptr<Feature> neighbour_h = nullptr;

        // The original paper calculates the points in reverse order
        const auto head = edge->end->position;
        const auto tail = edge->start->position;

        // For all voronoi planes of the feature
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
            if (distance_head >= 0 && distance_tail < 0){
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
            else if (distance_head < 0 && distance_tail >= 0){
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
