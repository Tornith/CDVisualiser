#include "convex_polyhedron.hpp"

#include <algorithm>
#include <iostream>
#include <unordered_map>

// Specialize the hash function for the pair of size_t
template<>
struct std::hash<std::pair<size_t, size_t>> {
    [[nodiscard]] size_t operator()(const std::pair<size_t, size_t>& pair) const noexcept {
        return pair.first ^ pair.second;
    }
};

namespace cdlib {
    float Plane::get_intersection_parameter(const glm::vec3& point_a, const glm::vec3& point_b) const {
        const float distance_a = dot(normal, point_a) + d;
        const float distance_b = dot(normal, point_b) + d;

        return distance_a / (distance_a - distance_b);
    }

    std::shared_ptr<ConvexPolyhedron> ConvexPolyhedron::build_dcel(const std::vector<glm::vec3>& vertices, const std::vector<std::vector<size_t>>& faces) {
        const auto polyhedron = std::make_shared<ConvexPolyhedron>();
        const auto edge_count = vertices.size() + faces.size() - 2; // Euler characteristic: V - E + F = 2 => E = V + F - 2

        polyhedron->vertices.reserve(vertices.size());
        polyhedron->faces.reserve(faces.size());
        polyhedron->hedges.reserve(edge_count * 2); // Each edge has a twin

        // Twin edge map
        std::unordered_map<std::pair<size_t, size_t>, HalfEdgeP> edge_map;

        // Create vertices
        for (const auto& vertex : vertices) {
            const auto vertex_ptr = std::make_shared<Vertex>(vertex);
            vertex_ptr->polyhedron = polyhedron;
            polyhedron->vertices.emplace_back(vertex_ptr);
        }

        // Create faces and edges
        for (const auto& face_indices : faces) {
            std::vector<HalfEdgeP> face_edges;
            face_edges.reserve(face_indices.size());

            // Create edges
            for (size_t i = 0; i < face_indices.size(); i++) {
                const auto i1 = face_indices[i];
                const auto i2 = face_indices[(i + 1) % face_indices.size()];

                const auto edge = std::make_shared<HalfEdge>(polyhedron->vertices[i1], polyhedron->vertices[i2]);
                edge_map[{i1, i2}] = edge;
                edge->polyhedron = polyhedron;
                face_edges.emplace_back(edge);
                polyhedron->hedges.emplace_back(edge);

                // Add the edge to the start vertex
                polyhedron->vertices[i1]->edges.emplace_back(edge);
            }

            // Create face
            const auto v1 = vertices[face_indices[1]] - vertices[face_indices[0]];
            const auto v2 = vertices[face_indices[2]] - vertices[face_indices[0]];
            const auto c = cross(v1, v2);
            const auto normal = normalize(c);
            const auto face = std::make_shared<Face>(Plane(normal, vertices[face_indices[0]]));
            face->polyhedron = polyhedron;
            face->edges = face_edges;

            // Set the face of the edges and the next and prev pointers
            for (size_t i = 0; i < face_edges.size(); i++) {
                const auto edge = face_edges[i];
                edge->face = face;
                edge->next = face_edges[(i + 1) % face_edges.size()];
                edge->prev = face_edges[(i + face_edges.size() - 1) % face_edges.size()];
            }

            // Add the face to the polyhedron
            polyhedron->faces.emplace_back(face);
        }

        // Set the twin pointers
        for (const auto& [edge_pair, edge] : edge_map) {
            const auto& [i1, i2] = edge_pair;
            const auto twin_edge = edge_map[{i2, i1}];
            if (!twin_edge) {
                std::cerr << "Twin edge not found" << std::endl;
                continue;
            }
            edge->twin = twin_edge;
            twin_edge->twin = edge;
        }

        // Return the polyhedron
        return polyhedron;
    }

    std::vector<FeatureP> Face::get_base_neighbours() const {
        return {edges.begin(), edges.end()};
    }

    std::vector<FeatureP> Vertex::get_base_neighbours() const {
        return {edges.begin(), edges.end()};
    }

    std::vector<FeatureP> HalfEdge::get_base_neighbours() const {
        return {start, end, face, twin->face};
    }

    std::vector<FaceP> HalfEdge::get_neighbour_faces() const {
        return {face, twin->face};
    }

    std::vector<VertexP> HalfEdge::get_neighbour_vertices() const {
        return {start, end};
    }

    HalfEdgeP HalfEdge::create(const glm::vec3& start, const glm::vec3& end) {
        const auto polyhedron = std::make_shared<ConvexPolyhedron>();
        polyhedron->set_transform(glm::mat4(1.0f));

        const auto start_v = std::make_shared<Vertex>(start);
        const auto end_v = std::make_shared<Vertex>(end);

        start_v->polyhedron = polyhedron;
        end_v->polyhedron = polyhedron;

        const auto edge = std::make_shared<HalfEdge>(start_v, end_v);
        const auto twin = std::make_shared<HalfEdge>(edge->end, edge->start);

        edge->twin = twin;
        twin->twin = edge;
        edge->polyhedron = polyhedron;
        twin->polyhedron = polyhedron;

        return edge;
    }

    float distance_point_line(const glm::vec3& point, const glm::vec3& a, const glm::vec3& b) {
        const auto ab = b - a;
        const auto ap = point - a;
        const auto bp = point - b;

        const auto e = dot(ap, ab);
        if (e <= 0.0f) {
            return length(ap);
        }

        const auto f = dot(ab, ab);
        if (e >= f) {
            return length(bp);
        }

        return length(cross(ab, ap)) / length(ab);
    }

    float distance_point_plane(const glm::vec3& point, const Plane& plane) {
        return dot(plane.normal, point) + plane.d;
    }

    float Vertex::distance_to(const std::shared_ptr<Feature>& other) const
    {
        if (const auto vertex = std::dynamic_pointer_cast<Vertex>(other)) {
            return distance(get_position(), vertex->get_position());
        }
        if (const auto edge = std::dynamic_pointer_cast<HalfEdge>(other)) {
            return distance_point_line(get_position(), edge->start->get_position(), edge->end->get_position());
        }
        if (const auto face = std::dynamic_pointer_cast<Face>(other)) {
            return distance_point_plane(get_position(), face->get_plane());
        }
        return std::numeric_limits<float>::lowest();;
    }

    float HalfEdge::distance_to(const std::shared_ptr<Feature>& other) const
    {
        if (const auto vertex = std::dynamic_pointer_cast<Vertex>(other)) {
            return distance_point_line(vertex->get_position(), start->get_position(), end->get_position());
        }
        if (const auto edge = std::dynamic_pointer_cast<HalfEdge>(other)) {
            return std::numeric_limits<float>::lowest(); // TODO: Implement
        }
        if (const auto face = std::dynamic_pointer_cast<Face>(other)) {
            return std::numeric_limits<float>::lowest(); // TODO: Implement
        }
        return std::numeric_limits<float>::lowest();;
    }

    float Face::distance_to(const std::shared_ptr<Feature>& other) const
    {
        if (const auto vertex = std::dynamic_pointer_cast<Vertex>(other)) {
            return distance_point_plane(vertex->get_position(), get_plane());
        }
        if (const auto edge = std::dynamic_pointer_cast<HalfEdge>(other)) {
            return std::numeric_limits<float>::lowest(); // TODO: Implement
        }
        if (const auto face = std::dynamic_pointer_cast<Face>(other)) {
            return std::numeric_limits<float>::lowest(); // TODO: Implement
        }
        return std::numeric_limits<float>::lowest();;
    }

    std::vector<std::shared_ptr<Vertex>> Face::get_vertices() const
    {
        const auto vertices = std::ranges::transform_view(edges, [](const auto& edge) { return edge->start; });
        return {vertices.begin(), vertices.end()};
    }
}
