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

        const auto t = dot(ap, ab) / dot(ab, ab);
        const auto clamped_t = glm::clamp(t, 0.0f, 1.0f);
        const auto projection = a + ab * clamped_t;
        return distance(point, projection);
    }

    float distance_point_triangle(const glm::vec3& point, const glm::vec3& a, const glm::vec3& b, const glm::vec3& c) {
        // Project point to the plane of the triangle
        const auto normal = normalize(cross(b - a, c - a));
        const auto d = -dot(normal, a);
        const auto projected_point = point - normal * (dot(normal, point) + d);

        // Calculate the barycentric coordinates
        const auto v0 = b - a;
        const auto v1 = c - a;
        const auto v2 = projected_point - a;

        const auto den = v0.x * v1.y - v1.x * v0.y;
        const auto v = (v2.x * v1.y - v1.x * v2.y) / den;
        const auto w = (v0.x * v2.y - v2.x * v0.y) / den;
        const auto u = 1.0f - v - w;

        // Check if the point is inside the triangle
        if (v >= 0 && w >= 0 && u >= 0) {
            return distance(projected_point, point);
        }

        // Calculate the distance to the edges
        const auto d0 = distance_point_line(point, a, b);
        const auto d1 = distance_point_line(point, b, c);
        const auto d2 = distance_point_line(point, c, a);

        return std::min({d0, d1, d2});
    }

    float distance_point_face(const glm::vec3& point, const std::vector<glm::vec3>& vertices) {
        // Split the face into triangles and calculate the distance to each of them
        float min_distance = std::numeric_limits<float>::max();
        for (size_t i = 1; i < vertices.size() - 1; i++) {
            const auto distance = distance_point_triangle(point, vertices[0], vertices[i], vertices[i + 1]);
            min_distance = std::min(min_distance, distance);
        }
        return min_distance;
    }

    float distance_line_line(const glm::vec3& a1, const glm::vec3& a2, const glm::vec3& b1, const glm::vec3& b2) {
        const auto e1 = a2 - a1;
        const auto e2 = b2 - b1;

        const auto n = cross(e1, e2);

        // Check if the lines are parallel
        if (length(n) < 1e-6f) {
            return distance_point_line(a1, b1, b2);
        }

        const auto d = abs(glm::dot(n, a1 - b1)) / length(n);

        // Calculate the parameters for clamp
        const auto t1 = dot(cross(e2, n), b1 - a1) / dot(n, n);
        const auto t2 = dot(cross(e1, n), b1 - a1) / dot(n, n);

        // Clamp the parameters to the line segment
        const auto t1_clamped = glm::clamp(t1, 0.0f, 1.0f);
        const auto t2_clamped = glm::clamp(t2, 0.0f, 1.0f);

        const auto p1 = a1 + e1 * t1_clamped;
        const auto p2 = b1 + e2 * t2_clamped;

        return distance(p1, p2);
    }

    float distance_line_triangle(const glm::vec3& a, const glm::vec3& b, const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2) {
        // Compute distance between the line and each of the edges
        const auto d0 = distance_line_line(a, b, v0, v1);
        const auto d1 = distance_line_line(a, b, v1, v2);
        const auto d2 = distance_line_line(a, b, v2, v0);

        // Compute distance between endpoints and the triangle
        const auto d3 = distance_point_triangle(a, v0, v1, v2);
        const auto d4 = distance_point_triangle(b, v0, v1, v2);

        // Return the minimum distance
        return std::min({d0, d1, d2, d3, d4});
    }

    float distance_line_face(const glm::vec3& a, const glm::vec3& b, const std::vector<glm::vec3>& vertices) {
        // Split the face into triangles and calculate the distance to each of them
        float min_distance = std::numeric_limits<float>::max();
        for (size_t i = 1; i < vertices.size() - 1; i++) {
            const auto distance = distance_line_triangle(a, b, vertices[0], vertices[i], vertices[i + 1]);
            min_distance = std::min(min_distance, distance);
        }
        return min_distance;
    }

    float distance_triangle_triangle(const glm::vec3& a0, const glm::vec3& a1, const glm::vec3& a2, const glm::vec3& b0, const glm::vec3& b1, const glm::vec3& b2) {
        auto distances = std::vector<float>();
        const auto triangle_a = std::vector{a0, a1, a2};
        const auto triangle_b = std::vector{b0, b1, b2};

        // Compute the distance between each pair of edges
        for (size_t i = 0; i < 3; i++) {
            for (size_t j = 0; j < 3; j++) {
                const auto distance = distance_line_line(triangle_a[i], triangle_a[(i + 1) % 3], triangle_b[j], triangle_b[(j + 1) % 3]);
                distances.push_back(distance);
            }
        }

        // Compute the distance between the vertices and the opposite triangle
        for (size_t i = 0; i < 3; i++) {
            const auto distance = distance_point_triangle(triangle_a[i], b0, b1, b2);
            distances.push_back(distance);
        }
        for (size_t i = 0; i < 3; i++) {
            const auto distance = distance_point_triangle(triangle_b[i], a0, a1, a2);
            distances.push_back(distance);
        }

        return *std::ranges::min_element(distances);
    }

    float distance_face_face(const std::vector<glm::vec3>& vertices_a, const std::vector<glm::vec3>& vertices_b) {
        // Split the faces into triangles and calculate the distance to each pair of triangles
        float min_distance = std::numeric_limits<float>::max();
        for (size_t i = 1; i < vertices_a.size() - 1; i++) {
            for (size_t j = 1; j < vertices_b.size() - 1; j++) {
                const auto distance = distance_triangle_triangle(vertices_a[0], vertices_a[i], vertices_a[i + 1], vertices_b[0], vertices_b[j], vertices_b[j + 1]);
                min_distance = std::min(min_distance, distance);
            }
        }
        return min_distance;
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
            return distance_point_face(get_position(), face->get_vertex_positions());
        }
        return std::numeric_limits<float>::max();
    }

    float HalfEdge::distance_to(const std::shared_ptr<Feature>& other) const
    {
        if (const auto vertex = std::dynamic_pointer_cast<Vertex>(other)) {
            return distance_point_line(vertex->get_position(), start->get_position(), end->get_position());
        }
        if (const auto edge = std::dynamic_pointer_cast<HalfEdge>(other)) {
            return distance_line_line(start->get_position(), end->get_position(), edge->start->get_position(), edge->end->get_position());
        }
        if (const auto face = std::dynamic_pointer_cast<Face>(other)) {
            return distance_line_face(start->get_position(), end->get_position(), face->get_vertex_positions());
        }
        return std::numeric_limits<float>::max();
    }

    float Face::distance_to(const std::shared_ptr<Feature>& other) const
    {
        if (const auto vertex = std::dynamic_pointer_cast<Vertex>(other)) {
            return distance_point_face(vertex->get_position(), get_vertex_positions());
        }
        if (const auto edge = std::dynamic_pointer_cast<HalfEdge>(other)) {
            return distance_line_face(edge->start->get_position(), edge->end->get_position(), get_vertex_positions());
        }
        if (const auto face = std::dynamic_pointer_cast<Face>(other)) {
            // Very expensive operation but V-clip never uses this
            return distance_face_face(get_vertex_positions(), face->get_vertex_positions());
        }
        return std::numeric_limits<float>::max();
    }

    std::vector<std::shared_ptr<Vertex>> Face::get_vertices() const
    {
        const auto vertices = std::ranges::transform_view(edges, [](const auto& edge) { return edge->start; });
        return {vertices.begin(), vertices.end()};
    }

    std::vector<glm::vec3> Face::get_vertex_positions() const {
        const auto vertices = get_vertices();
        const auto vertex_positions_view = std::ranges::transform_view(vertices, [](const VertexP& vertex) { return vertex->get_position(); });
        return std::vector<glm::vec3>{vertex_positions_view.begin(), vertex_positions_view.end()};
    }

    std::string ConvexPolyhedron::get_debug_data() const {
        // Print out all vertices in format "[(x, y, z), (x, y, z), ...]" without the float f suffix
        auto vertices_str = std::string{"Vertices: ["};
        for (const auto& vertex : vertices) {
            vertices_str += "(" + std::to_string(vertex->get_position().x) + ", " + std::to_string(vertex->get_position().y) + ", " + std::to_string(vertex->get_position().z) + "), ";
        }

        // Print out all edges in format "[(P[i] - P[j]) * t + P[i], (P[k] - P[l]) * t + P[k], ...]" where i, j, k, ... are indices of vertices indexed from 1 (i.e. shifted by one)
        auto edges_str = std::string{"Edges: ["};
        for (const auto& edge : hedges) {
            edges_str += "(P[" + std::to_string(std::distance(vertices.begin(), std::find(vertices.begin(), vertices.end(), edge->start)) + 1) + "] - P[" + std::to_string(std::distance(vertices.begin(), std::find(vertices.begin(), vertices.end(), edge->end)) + 1) + "]) \\cdot t + P[" + std::to_string(std::distance(vertices.begin(), std::find(vertices.begin(), vertices.end(), edge->end)) + 1) + "], ";
        }

        // Combine the strings and return
        vertices_str.pop_back();
        vertices_str.pop_back();
        vertices_str += "]";
        edges_str.pop_back();
        edges_str.pop_back();
        edges_str += "]";

        return vertices_str + "\n" + edges_str;
    }

    std::string Face::to_string() const {
        const auto plane_ws = get_plane();
        // Format the vertices as [(x, y, z), (x, y, z), ...] without the float f suffix
        auto vertices_string = std::string{"["};
        for (const auto& vertex : get_vertices()) {
            const auto vertex_position = vertex->get_position();
            vertices_string += "(" + std::to_string(vertex_position.x) + ", " + std::to_string(vertex_position.y) + ", " + std::to_string(vertex_position.z) + "), ";
        }
        vertices_string.pop_back();
        vertices_string.pop_back();
        vertices_string += "]";
        return "Face(plane: " + plane_ws.to_string() + ", vertices: " + vertices_string + ")";
    }

    std::string Vertex::to_string() const {
        const auto position = get_position();
        return "Vertex(position: (" + std::to_string(position.x) + ", " + std::to_string(position.y) + ", " + std::to_string(position.z) + "))";
    }

    std::string HalfEdge::to_string() const {
        const auto start_position = start->get_position();
        const auto end_position = end->get_position();
        return "HalfEdge(start: (" + std::to_string(start_position.x) + ", " + std::to_string(start_position.y) + ", " + std::to_string(start_position.z) + "), end: (" + std::to_string(end_position.x) + ", " + std::to_string(end_position.y) + ", " + std::to_string(end_position.z) + "))";
    }
}
