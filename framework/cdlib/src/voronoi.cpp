#include "voronoi.hpp"

#include <algorithm>
#include <stdexcept>

namespace cdlib::Voronoi {
    VoronoiPlane get_voronoi_plane(const std::shared_ptr<Vertex>& vertex, const std::shared_ptr<Edge>& edge, const bool inverse_normal) {
        // Get the other vertex of the edge
        const glm::vec3 other_vertex = vertex->position == edge->point_a ? edge->point_b : edge->point_a;
        glm::vec3 face_normal = glm::normalize(vertex->position - other_vertex);

        return {Plane(inverse_normal ? -face_normal : face_normal, vertex->position), vertex, edge};
    }

    VoronoiPlane get_voronoi_plane(const std::shared_ptr<Face>& face, const std::shared_ptr<Edge>& edge, const bool inverse_normal) {
        const glm::vec3 edge_direction = glm::normalize(edge->point_b - edge->point_a);
        const glm::vec3 face_normal = glm::normalize(face->plane.normal);

        // TODO: Redo all of this
        // A different edge from the face's neighbours
        const auto it = std::ranges::find_if(face->neighbours, [&edge](const auto& neighbour){
            return *neighbour != *edge;
        });

        // Cast the iterator to a pointer to the edge
        if (it == face->neighbours.end()){
            throw std::runtime_error("Edge not found in face neighbours");
        }

        const auto other_edge = std::dynamic_pointer_cast<Edge>(*it);

        // Get the vertex, from the neighbour edge, that is not part of the edge
        const glm::vec3 other_vertex = other_edge->point_a == edge->point_a || other_edge->point_a == edge->point_b ? other_edge->point_b : other_edge->point_a;

        // The voronoi plane is perpendicular to the face normal and the edge direction
        glm::vec3 plane_normal = normalize(cross(face_normal, edge_direction));

        // Flip the normal if it's pointing in the wrong direction
        if (dot(plane_normal, other_vertex - edge->point_a) < 0){
            plane_normal = -plane_normal;
        }

        if (inverse_normal){
            plane_normal = -plane_normal;
        }

        return {Plane(plane_normal, edge->point_a), face, edge};
    }

    std::optional<VoronoiPlane> get_voronoi_plane_safe(const std::shared_ptr<Feature>& feature, const std::shared_ptr<Edge>& edge, const bool inverse_normal) {
        // If the feature is an edge, return nullopt
        if (std::dynamic_pointer_cast<Edge>(feature)){
            return std::nullopt;
        }

        auto it = std::ranges::find_if(feature->neighbours, [&edge](const auto& neighbour){
            return *neighbour == *edge;
        });

        if (it == feature->neighbours.end()){
            return std::nullopt;
        }

        if (const auto casted_face = std::dynamic_pointer_cast<Face>(feature)){
            return get_voronoi_plane(casted_face, edge, inverse_normal);
        }

        if (const auto casted_vertex = std::dynamic_pointer_cast<Vertex>(feature)){
            return get_voronoi_plane(casted_vertex, edge, inverse_normal);
        }

        return std::nullopt;
    }

    bool Face::in_voronoi_region(const glm::vec3& point) const {
        const auto shared_face = std::make_shared<Face>(*this);
        return Voronoi::in_voronoi_region(shared_face, point);
    }

    bool Vertex::in_voronoi_region(const glm::vec3& point) const {
        const auto shared_vertex = std::make_shared<Vertex>(*this);
        return Voronoi::in_voronoi_region(shared_vertex, point);
    }

    bool Edge::in_voronoi_region(const glm::vec3& point) const {
        // Create a shared_ptr from the edge
        const auto shared_edge = std::make_shared<Edge>(*this);

        // For all the neightbours of the edge, get their voronoi planes
        // and check if the point is in the positive half-space of all of them
        return std::ranges::all_of(neighbours, [&point, &shared_edge](const std::shared_ptr<Feature>& neighbour){
            // Attempt to cast to either a face or a vertex
            if (const auto casted_face = std::dynamic_pointer_cast<Face>(neighbour)){
                auto a = get_voronoi_plane(casted_face, shared_edge, true); // Inverse the normal, because we want the plane to be facing towards the edge
                return a.is_above(point);
            }
            // If it's not a face, it must be a vertex
            if (const auto casted_vertex = std::dynamic_pointer_cast<Vertex>(neighbour)){
                auto a = get_voronoi_plane(casted_vertex, shared_edge, true); // Inverse the normal, because we want the plane to be facing towards the edge
                return a.is_above(point);
            }
            throw std::runtime_error("Invalid neighbour type");
        });
    }

    float Plane::get_intersection_parameter(const glm::vec3& point_a, const glm::vec3& point_b) const {
        const float distance_a = dot(normal, point_a) + d;
        const float distance_b = dot(normal, point_b) + d;

        return distance_a / (distance_a - distance_b);
    }

    ClipData clip_edge(const std::shared_ptr<Feature>& feature, const std::shared_ptr<Edge>& edge) {
        auto is_clipped = true;
        float lambda_l = 0;
        float lambda_h = 1;
        std::shared_ptr<Feature> neighbour_l = nullptr;
        std::shared_ptr<Feature> neighbour_h = nullptr;

        const auto [head, tail] = std::pair{edge->point_a, edge->point_b};

        // For all feature-edge voronoi plane
        for (const auto& neighbour : feature->neighbours){
            const auto voronoi_plane = get_voronoi_plane_safe(feature, edge, true);
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
