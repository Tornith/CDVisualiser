#pragma once
#include <array>
#include <memory>
#include <ranges>
#include <string>
#include <vector>
#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtx/string_cast.hpp>

/**
 * @file convex_polyhedron.hpp
 * @brief A class that represents a convex polyhedron with a half-edge DCEL data structure (Doubly Connected Edge List)
 */

namespace cdlib {
    struct Simplex {
    private:
        std::array<glm::vec3, 4> _points;
        size_t _size;

    public:
        Simplex() : _points({}), _size(0) {}

        explicit Simplex(const std::vector<glm::vec3>& points) : _points({}) {
            for (const auto& point : points) {
                insert(point);
            }
            _size = std::min(points.size(), static_cast<size_t>(4));
        }

        [[nodiscard]] size_t size() const {
            return _size;
        }

        [[nodiscard]] const glm::vec3& operator[](const size_t index) const {
            return _points[index];
        }

        void insert(const glm::vec3& point) {
            _points = {point, _points[0], _points[1], _points[2]};
            _size = std::min(_size + 1, static_cast<size_t>(4));
        }

        void reorder(const std::initializer_list<size_t> indices) {
            std::array<glm::vec3, 4> newPoints {};
            size_t newSize = 0;

            for (const auto index : indices) {
                if (index < _size) {
                    newPoints[newSize++] = _points[index];
                }
            }

            _points = newPoints;
            _size = newSize;
        }

        Simplex& operator=(std::initializer_list<glm::vec3> list){
            for (const auto& point : list){
                insert(point);
            }
            return *this;
        }

        [[nodiscard]] auto begin() const {
            return _points.begin();
        }

        [[nodiscard]] auto end() const {
            return _points.begin() + static_cast<int>(_size);
        }

        void clear() {
            _size = 0;
            _points = {};
        }
    };

    struct Plane {
        glm::vec3 normal;
        float d;

        Plane() = default;

        Plane(const glm::vec3& normal, float d)
            : normal(normal),
              d(d) {
        }

        explicit Plane(const glm::vec4& vec)
            : normal(vec),
              d(vec.w) {
        }

        Plane(const glm::vec3& normal, const glm::vec3& point)
            : normal(normal),
              d(dot(normal, point)) {
        }

        friend bool operator==(const Plane& lhs, const Plane& rhs) = default;
        friend bool operator!=(const Plane& lhs, const Plane& rhs) = default;

        [[nodiscard]] glm::vec4 to_vec4() const {
            return {normal, d};
        }

        [[nodiscard]] float distance_to(const glm::vec3& point) const {
            return dot(normal, point) - d;
        }

        [[nodiscard]] bool is_above(const glm::vec3& point) const {
            return distance_to(point) >= 0;
        }

        [[nodiscard]] float get_intersection_parameter(const glm::vec3& point_a, const glm::vec3& point_b) const;

        [[nodiscard]] Plane operator-() const {
            return {-normal, -d};
        }

        // Transformation by a 4x4 matrix
        [[nodiscard]] Plane operator*(const glm::mat4& matrix) const {
            const auto normal = glm::vec3(matrix * glm::vec4(this->normal, 0.0));
            const auto point = glm::vec3(matrix * glm::vec4(this->normal * this->d, 1.0));
            return {normal, point};
        }

        friend Plane operator*(const glm::mat4& matrix, const Plane& plane) {
            return plane * matrix;
        }

        [[nodiscard]] std::string to_string() const {
            return "Plane(normal: (" + std::to_string(normal.x) + ", " + std::to_string(normal.y) + ", " + std::to_string(normal.z) + "), d: " + std::to_string(d) + ")";
        }
    };

    // Forward declarations
    struct Vertex;
    struct Face;
    struct HalfEdge;

    // Convex polyhedron class
    struct ConvexPolyhedron {
    private:
        std::vector<std::shared_ptr<Vertex>> vertices;
        std::vector<std::shared_ptr<Face>> faces;
        std::vector<std::shared_ptr<HalfEdge>> hedges;

        glm::mat4 transform = glm::mat4(1.0);

    public:
        ConvexPolyhedron() = default;

        [[nodiscard]] static std::shared_ptr<ConvexPolyhedron> build_dcel(const std::vector<glm::vec3>& vertices, const std::vector<std::vector<size_t>>& faces);

        [[nodiscard]] std::vector<std::shared_ptr<Vertex>> get_vertices() const {
            return vertices;
        }

        [[nodiscard]] std::vector<std::shared_ptr<Face>> get_faces() const {
            return faces;
        }

        [[nodiscard]] std::vector<std::shared_ptr<HalfEdge>> get_half_edges() const {
            return hedges;
        }

        [[nodiscard]] std::shared_ptr<Vertex> get_vertex(const size_t index) const {
            return vertices[index];
        }

        [[nodiscard]] std::shared_ptr<Face> get_face(const size_t index) const {
            return faces[index];
        }

        [[nodiscard]] std::shared_ptr<HalfEdge> get_half_edge(const size_t index) const {
            return hedges[index];
        }

        void add_vertex(const std::shared_ptr<Vertex>& vertex) {
            vertices.push_back(vertex);
        }

        void add_face(const std::shared_ptr<Face>& face) {
            faces.push_back(face);
        }

        void add_half_edge(const std::shared_ptr<HalfEdge>& hedge) {
            hedges.push_back(hedge);
        }

        void set_transform(const glm::mat4& transform) {
            this->transform = transform;
        }

        [[nodiscard]] glm::mat4 get_transform() const {
            return transform;
        }

        [[nodiscard]] std::string get_debug_data() const;
    };

    struct Feature {
        std::shared_ptr<ConvexPolyhedron> polyhedron;
        virtual ~Feature() = default;

        friend bool operator==(const Feature& lhs, const Feature& rhs) = default;
        friend bool operator!=(const Feature& lhs, const Feature& rhs) = default;

        template <typename T = Feature> requires std::is_base_of_v<Feature, T>
        [[nodiscard]] std::vector<std::shared_ptr<T>> get_neighbours() const {
            const auto neighbours = get_base_neighbours();

            // If T is a Feature, we don't need to cast
            if constexpr (std::is_same_v<T, Feature>) {
                return neighbours;
            }

            // Otherwise cast the neighbours to T
            std::vector<std::shared_ptr<T>> casted_neighbours;
            casted_neighbours.reserve(neighbours.size());
            for (const auto& neighbour : neighbours) {
                casted_neighbours.emplace_back(std::dynamic_pointer_cast<T>(neighbour));
            }

            return casted_neighbours;
        }

        [[nodiscard]] virtual float distance_to(const std::shared_ptr<Feature>& other) const = 0;

        [[nodiscard]] virtual std::string to_string() const = 0;

    protected:
        [[nodiscard]] virtual std::vector<std::shared_ptr<Feature>> get_base_neighbours() const = 0;
    };

    struct HalfEdge;

    struct Face final : Feature {
    private:
        Plane plane;
        std::vector<std::shared_ptr<HalfEdge>> edges;

    public:
        explicit Face(const Plane& plane)
            : plane(plane) {
        }

        Face(const glm::vec3& normal, const float d)
            : plane(normal, d) {
        }

        [[nodiscard]] Plane get_plane() const {
            return polyhedron->get_transform() * plane;
        }

        [[nodiscard]] Plane get_local_plane() const {
            return plane;
        }

        [[nodiscard]] std::vector<std::shared_ptr<HalfEdge>> get_edges() const {
            return edges;
        }

        [[nodiscard]] std::vector<std::shared_ptr<Vertex>> get_vertices() const;

        [[nodiscard]] std::vector<glm::vec3> get_vertex_positions() const;

        friend bool operator==(const Face& lhs, const Face& rhs) = default;
        friend bool operator!=(const Face& lhs, const Face& rhs) = default;

        [[nodiscard]] float distance_to(const std::shared_ptr<Feature>& other) const override;

        [[nodiscard]] std::string to_string() const override;

    protected:
        [[nodiscard]] std::vector<std::shared_ptr<Feature>> get_base_neighbours() const override;

        friend struct ConvexPolyhedron;
    };

    struct Vertex final : Feature {
    private:
        glm::vec3 position;
        std::vector<std::shared_ptr<HalfEdge>> edges;

    public:
        explicit Vertex(const glm::vec3& position)
            : position(position) {
        }

        [[nodiscard]] glm::vec3 get_position() const {
            // Multiplied by the polyhedron->transform matrix
            return polyhedron->get_transform() * glm::vec4(position, 1.0);
        }

        [[nodiscard]] glm::vec3 get_local_position() const {
            return position;
        }

        [[nodiscard]] std::vector<std::shared_ptr<HalfEdge>> get_edges() const {
            return edges;
        }

        friend bool operator==(const Vertex& lhs, const Vertex& rhs) = default;
        friend bool operator!=(const Vertex& lhs, const Vertex& rhs) = default;

        [[nodiscard]] float distance_to(const std::shared_ptr<Feature>& other) const override;

        [[nodiscard]] std::string to_string() const override;

    protected:
        [[nodiscard]] std::vector<std::shared_ptr<Feature>> get_base_neighbours() const override;

        friend struct ConvexPolyhedron;
    };

    struct HalfEdge final : Feature {
        std::shared_ptr<Vertex> start;
        std::shared_ptr<Vertex> end;
        std::shared_ptr<HalfEdge> twin;
        std::shared_ptr<HalfEdge> next;
        std::shared_ptr<HalfEdge> prev;
        std::shared_ptr<Face> face;

        HalfEdge(const std::shared_ptr<Vertex>& start, const std::shared_ptr<Vertex>& end)
            : start(start),
              end(end) {
        }

        friend bool operator==(const HalfEdge& lhs, const HalfEdge& rhs) = default;
        friend bool operator!=(const HalfEdge& lhs, const HalfEdge& rhs) = default;

        [[nodiscard]] glm::vec3 get_direction() const {
            return end->get_position() - start->get_position();
        }

        [[nodiscard]] std::vector<std::shared_ptr<Face>> get_neighbour_faces() const;
        [[nodiscard]] std::vector<std::shared_ptr<Vertex>> get_neighbour_vertices() const;

        static std::shared_ptr<HalfEdge> create(const glm::vec3& start, const glm::vec3& end);

        [[nodiscard]] float distance_to(const std::shared_ptr<Feature>& other) const override;

        [[nodiscard]] std::string to_string() const override;

    protected:
        [[nodiscard]] std::vector<std::shared_ptr<Feature>> get_base_neighbours() const override;
    };

    // Pointer aliases
    using FeatureP = std::shared_ptr<Feature>;
    using VertexP = std::shared_ptr<Vertex>;
    using FaceP = std::shared_ptr<Face>;
    using HalfEdgeP = std::shared_ptr<HalfEdge>;

    // Concepts
    template <typename T>
    concept IsFeature = std::is_base_of_v<Feature, T>;

    [[nodiscard]] inline float feature_distance(const FeatureP& feature_1, const FeatureP& feature_2) {
        return feature_1->distance_to(feature_2);
    }
}