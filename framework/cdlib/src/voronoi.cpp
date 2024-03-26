#include "voronoi.hpp"

#include <algorithm>
#include <stdexcept>

namespace cdlib::Voronoi {
    VoronoiPlane get_voronoi_plane(const std::shared_ptr<Vertex>& vertex, const std::shared_ptr<HalfEdge>& edge) {
        const glm::vec3 face_normal = normalize(edge->end->position - edge->start->position);

        return {Plane(face_normal, vertex->position), vertex, edge};
    }

    VoronoiPlane get_voronoi_plane(const std::shared_ptr<Face>& face, const std::shared_ptr<HalfEdge>& edge) {
        const glm::vec3 edge_direction = normalize(edge->end->position - edge->start->position);
        const glm::vec3 face_normal = normalize(face->plane.normal);

        // The voronoi plane is perpendicular to the face normal and the edge direction
        const glm::vec3 plane_normal = normalize(cross(face_normal, edge_direction));

        return {Plane(plane_normal, dot(plane_normal, face->plane.normal)), face, edge};
    }

    std::optional<VoronoiPlane> get_voronoi_plane_safe(const std::shared_ptr<Feature>& feature, const std::shared_ptr<HalfEdge>& edge) {
        // If the feature is an edge, return nullopt
        if (std::dynamic_pointer_cast<HalfEdge>(feature)){
            return std::nullopt;
        }

        const auto neighbours = feature->get_neighbours();

        auto it = std::ranges::find_if(neighbours, [&edge](const auto& neighbour){
            return *neighbour == *edge;
        });

        if (it == neighbours.end()){
            return std::nullopt;
        }

        if (const auto casted_face = std::dynamic_pointer_cast<Face>(feature)){
            return get_voronoi_plane(casted_face, edge);
        }

        if (const auto casted_vertex = std::dynamic_pointer_cast<Vertex>(feature)){
            return get_voronoi_plane(casted_vertex, edge);
        }

        return std::nullopt;
    }

    std::optional<VoronoiPlane> get_voronoi_plane_safe(const std::shared_ptr<Feature>& feature_1, const std::shared_ptr<Feature>& feature_2) {
        // Get which feature is the edge and which is the other
        const auto first = std::dynamic_pointer_cast<HalfEdge>(feature_1);
        const auto second = std::dynamic_pointer_cast<HalfEdge>(feature_2);

        if ((!first && !second) || (first && second)){
            return std::nullopt;
        }

        return first ? get_voronoi_plane_safe(feature_2, first) : get_voronoi_plane_safe(feature_1, second);
    }

    // bool Face::in_voronoi_region(const glm::vec3& point) const {
    //     const auto shared_face = std::make_shared<Face>(*this);
    //     return Voronoi::in_voronoi_region(shared_face, point);
    // }
    //
    // bool Vertex::in_voronoi_region(const glm::vec3& point) const {
    //     const auto shared_vertex = std::make_shared<Vertex>(*this);
    //     return Voronoi::in_voronoi_region(shared_vertex, point);
    // }
    //
    // bool Edge::in_voronoi_region(const glm::vec3& point) const {
    //     // Create a shared_ptr from the edge
    //     const auto shared_edge = std::make_shared<Edge>(*this);
    //
    //     // For all the neightbours of the edge, get their voronoi planes
    //     // and check if the point is in the positive half-space of all of them
    //     return std::ranges::all_of(neighbours, [&point, &shared_edge](const std::shared_ptr<Feature>& neighbour){
    //         // Attempt to cast to either a face or a vertex
    //         if (const auto casted_face = std::dynamic_pointer_cast<Face>(neighbour)){
    //             auto a = get_voronoi_plane(casted_face, shared_edge, true); // Inverse the normal, because we want the plane to be facing towards the edge
    //             return a.is_above(point);
    //         }
    //         // If it's not a face, it must be a vertex
    //         if (const auto casted_vertex = std::dynamic_pointer_cast<Vertex>(neighbour)){
    //             auto a = get_voronoi_plane(casted_vertex, shared_edge, true); // Inverse the normal, because we want the plane to be facing towards the edge
    //             return a.is_above(point);
    //         }
    //         throw std::runtime_error("Invalid neighbour type");
    //     });
    // }

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
