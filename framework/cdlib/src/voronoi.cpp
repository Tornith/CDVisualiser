#include "voronoi.hpp"

#include <algorithm>
#include <stdexcept>

namespace cdlib::Voronoi {
    Plane get_voronoi_plane(const Vertex& vertex, const Edge& edge) {
        // Get the other vertex of the edge
        const glm::vec3 other_vertex = vertex.position == edge.point_a ? edge.point_b : edge.point_a;
        glm::vec3 face_normal = normalize(vertex.position - other_vertex);

        return {face_normal, vertex.position};
    }

    Plane get_voronoi_plane(const Face& face, const Edge& edge) {
        // TODO: Make sure the winding is correct
        const glm::vec3 edge_direction = normalize(edge.point_b - edge.point_a);
        const glm::vec3 face_normal = normalize(face.plane.normal);

        glm::vec3 plane_normal = normalize(cross(face_normal, edge_direction));

        return {plane_normal, edge.point_a};
    }

    std::optional<Plane> get_voronoi_plane_safe(auto feature, const Edge& edge) {
        auto it = std::ranges::find_if(feature.neighbours, [&edge](const auto& neighbour){
            return *neighbour == edge;
        });

        if (it == feature.neighbours.end()){
            return std::nullopt;
        }

        return get_voronoi_plane(feature, edge);
    }

    bool Face::in_voronoi_region(const glm::vec3& point) const {
        return Voronoi::in_voronoi_region(*this, point);
    }

    bool Vertex::in_voronoi_region(const glm::vec3& point) const {
        return Voronoi::in_voronoi_region(*this, point);
    }

    bool Edge::in_voronoi_region(const glm::vec3& point) const {
        // For all the neightbours of the edge, get their voronoi planes
        // and check if the point is in the positive half-space of all of them
        return std::ranges::all_of(neighbours, [&point, this](const std::shared_ptr<Feature>& neighbour){
            // Attempt to cast to either a face or a vertex
            if (const auto casted_face = std::dynamic_pointer_cast<Face>(neighbour)){
                return get_voronoi_plane(*casted_face, *this).is_above(point);
            }
            // If it's not a face, it must be a vertex
            if (const auto casted_vertex = std::dynamic_pointer_cast<Vertex>(neighbour)){
                return get_voronoi_plane(*casted_vertex, *this).is_above(point);
            }
            throw std::runtime_error("Invalid neighbour type");
        });
    }
}
