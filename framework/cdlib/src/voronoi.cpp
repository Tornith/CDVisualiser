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
}
