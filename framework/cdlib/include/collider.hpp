#pragma once

#include <ranges>
#include <vector>
#include <glm/glm.hpp>
#include "convex_polyhedron.hpp"

namespace cdlib {
    class Collider {
    protected:
        std::shared_ptr<ConvexPolyhedron> shape;
        std::pair<glm::vec3, glm::vec3> aabb;

        std::vector<glm::vec3> cached_global_vertices;
        bool is_cache_valid{ false };

    public:
        Collider() = default;

        explicit Collider(const std::shared_ptr<ConvexPolyhedron>& shape) : shape(shape) {
            aabb = calculate_aabb(get_global_vertices());
        }

        Collider(const std::shared_ptr<ConvexPolyhedron>& shape, const glm::mat4& transform) : shape(shape) {
            aabb = calculate_aabb(get_global_vertices());
            shape->set_transform(transform);
        }

        virtual ~Collider() = default;

        Collider(const Collider& other) = default;
        Collider(Collider&& other) noexcept : shape(std::move(other.shape)), aabb(other.aabb) {}

        Collider& operator=(const Collider& other) {
            if (this == &other)
                return *this;
            shape = other.shape;
            aabb = other.aabb;
            return *this;
        }

        Collider& operator=(Collider&& other) noexcept {
            if (this == &other)
                return *this;
            shape = std::move(other.shape);
            aabb = other.aabb;
            return *this;
        }

        [[nodiscard]] std::vector<VertexP> get_vertices() const {
            return shape->get_vertices();
        }

        [[nodiscard]] std::vector<glm::vec3> get_global_vertices() {
            if (is_cache_valid) {
                return cached_global_vertices;
            }

            const auto view = std::ranges::views::transform(shape->get_vertices(), [this](const VertexP& vertex) {
                return vertex->get_position();
            });

            is_cache_valid = true;
            cached_global_vertices = std::vector(view.begin(), view.end());

            return cached_global_vertices;
        }

        virtual void set_shape(const std::shared_ptr<ConvexPolyhedron>& shape) {
            Collider::shape = shape;
            is_cache_valid = false;
        }

        [[nodiscard]] std::shared_ptr<ConvexPolyhedron> get_shape() const {
            return shape;
        }

        [[nodiscard]] glm::vec3 get_global_vertex(const size_t index) const {
            return shape->get_vertex(index)->get_position();
        }

        [[nodiscard]] glm::mat4 get_transform() const {
            return shape->get_transform();
        }

        void set_transform(const glm::mat4& transform) {
            shape->set_transform(transform);
            is_cache_valid = false;
        }

        [[nodiscard]] virtual glm::vec3 support(const glm::vec3& direction) const = 0;

        void set_aabb(const std::pair<glm::vec3, glm::vec3>& aabb) {
            Collider::aabb = aabb;
        }

        [[nodiscard]] const std::pair<glm::vec3, glm::vec3>& get_aabb() const {
            return aabb;
        }

        void update_aabb() {
            aabb = calculate_aabb(shape->get_vertices());
        }

        static std::pair<glm::vec3, glm::vec3> calculate_aabb(const std::vector<glm::vec3>& vertices) {
            glm::vec3 min = vertices[0];
            glm::vec3 max = vertices[0];

            for (const auto& vertex : vertices) {
                min = glm::min(min, vertex);
                max = glm::max(max, vertex);
            }

            return { min, max };
        }

        static std::pair<glm::vec3, glm::vec3> calculate_aabb(const std::vector<VertexP>& vertices) {
            glm::vec3 min = vertices[0]->get_position();
            glm::vec3 max = vertices[0]->get_position();

            for (const auto& vertex : vertices) {
                min = glm::min(min, vertex->get_position());
                max = glm::max(max, vertex->get_position());
            }

            return { min, max };
        }

        [[nodiscard]] bool is_valid() const {
            if (shape == nullptr) {
                return false;
            }
            const auto is_empty = shape->get_vertices().empty();
            return !is_empty;
        }
    };

    class ConvexCollider final : public Collider {
    public:
        ConvexCollider() = default;

        explicit ConvexCollider(const std::shared_ptr<ConvexPolyhedron>& shape)
            : Collider(shape) {
        }

        ConvexCollider(const std::shared_ptr<ConvexPolyhedron>& shape, const glm::mat4& transform)
            : Collider(shape, transform) {
        }

        ~ConvexCollider() override = default;

        ConvexCollider(const ConvexCollider& other) = default;

        ConvexCollider(ConvexCollider&& other) noexcept : Collider(std::move(other)) {
        }

        ConvexCollider& operator=(const ConvexCollider& other) {
            if (this == &other)
                return *this;
            Collider::operator=(other);
            return *this;
        }

        ConvexCollider& operator=(ConvexCollider&& other) noexcept {
            if (this == &other)
                return *this;
            Collider::operator=(std::move(other));
            return *this;
        }

        [[nodiscard]] glm::vec3 support(const glm::vec3& direction) const override {
            const auto rotated_direction = glm::vec3(shape->get_transform() * glm::vec4(direction, 0.0f));

            // Find the vertex that is the furthest in the given direction
            float max_dot = dot(shape->get_vertex(0)->get_position(), rotated_direction);
            size_t max_index = 0;

            // Start loop at 1 as we've already calculated for vertex 0
            for (size_t i = 1; i < shape->get_vertices().size(); i++) {
                const float dot = glm::dot(shape->get_vertex(i)->get_position(), rotated_direction);
                if (dot > max_dot) {
                    max_dot = dot;
                    max_index = i;
                }
            }
            return shape->get_vertex(max_index)->get_position();
        }
    };

    class RayCollider final : public Collider
    {
        constexpr static float RAY_LENGTH = 100.0f;

        glm::vec3 direction{};
        glm::vec3 origin{};
    public:
        RayCollider() = default;

        RayCollider(const glm::vec3& origin, const glm::vec3& direction) : direction(normalize(direction) * RAY_LENGTH), origin(origin) {
            shape = create_ray_shape();
        }

        ~RayCollider() override = default;

        RayCollider(const RayCollider& other) = default;
        RayCollider(RayCollider&& other) noexcept : Collider(std::move(other)), direction(other.direction), origin(other.origin) {}

        std::shared_ptr<ConvexPolyhedron> create_ray_shape() {
            const auto polyhedron = std::make_shared<ConvexPolyhedron>();
            polyhedron->set_transform(glm::mat4(1.0f));

            const auto start_v = std::make_shared<Vertex>(origin);
            const auto end_v = std::make_shared<Vertex>(origin + direction);

            start_v->polyhedron = polyhedron;
            end_v->polyhedron = polyhedron;

            polyhedron->add_vertex(start_v);
            polyhedron->add_vertex(end_v);

            const auto edge = std::make_shared<HalfEdge>(start_v, end_v);
            const auto twin = std::make_shared<HalfEdge>(edge->end, edge->start);

            edge->twin = twin;
            twin->twin = edge;
            edge->polyhedron = polyhedron;
            twin->polyhedron = polyhedron;

            polyhedron->add_half_edge(edge);
            polyhedron->add_half_edge(twin);

            return polyhedron;
        }

        [[nodiscard]] glm::vec3 support(const glm::vec3& direction) const override
        {
            if (dot(direction, this->direction) > 0) {
                return origin + this->direction;
            }
            return origin;
        }
    };
}