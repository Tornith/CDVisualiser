#pragma once

#include <numeric>
#include <ranges>
#include <vector>
#include <glm/glm.hpp>
#include "convex_polyhedron.hpp"

namespace cdlib {
    class Collider {
    protected:
        std::shared_ptr<ConvexPolyhedron> shape;
        glm::mat4 transform{};
        std::pair<glm::vec3, glm::vec3> aabb;

        std::vector<glm::vec3> cached_global_vertices;
        bool is_cache_valid{ false };

    public:
        Collider() = default;

        explicit Collider(const std::shared_ptr<ConvexPolyhedron>& shape) : shape(shape) {
            aabb = calculate_aabb(get_global_vertices());
            transform = glm::mat4(1.0);
        }

        Collider(const std::shared_ptr<ConvexPolyhedron>& shape, const glm::mat4& transform) : shape(shape), transform(transform) {
            aabb = calculate_aabb(get_global_vertices());
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

        [[nodiscard]] std::vector<std::shared_ptr<Vertex>> get_local_vertices() const {
            return shape->vertices;
        }

        [[nodiscard]] const glm::mat4& get_transform_matrix() const {
            return transform;
        }

        void set_transform_matrix(const glm::mat4& transform) {
            is_cache_valid = false;
            Collider::transform = transform;
        }

        [[nodiscard]] std::vector<glm::vec3> get_global_vertices() {
            if (is_cache_valid) {
                return cached_global_vertices;
            }

            const auto view = std::ranges::views::transform(shape->vertices, [this](const auto& vertex) {
                return glm::vec3(transform * glm::vec4(vertex->position, 1.0));
            });

            is_cache_valid = true;
            cached_global_vertices = std::vector(view.begin(), view.end());

            return cached_global_vertices;
        }

        [[nodiscard]] glm::vec3 get_global_vertex(const size_t index) const {
            if (is_cache_valid) {
                return cached_global_vertices[index];
            }
            return transform * glm::vec4(shape->vertices[index]->position, 1.f);
        }

        virtual void set_shape(const std::shared_ptr<ConvexPolyhedron>& shape) {
            Collider::shape = shape;
            is_cache_valid = false;
        }

        [[nodiscard]] virtual glm::vec3 support(const glm::vec3& direction) const = 0;

        [[nodiscard]] glm::vec3 global_support(const glm::vec3& direction) const {
            return transform * glm::vec4(support(direction), 1.0);
        }

        void set_aabb(const std::pair<glm::vec3, glm::vec3>& aabb) {
            Collider::aabb = aabb;
        }

        [[nodiscard]] const std::pair<glm::vec3, glm::vec3>& get_aabb() const {
            return aabb;
        }

        void update_aabb() {
            aabb = calculate_aabb(shape->vertices);
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

        static std::pair<glm::vec3, glm::vec3> calculate_aabb(const std::vector<std::shared_ptr<Vertex>>& vertices) {
            glm::vec3 min = vertices[0]->position;
            glm::vec3 max = vertices[0]->position;

            for (const auto& vertex : vertices) {
                min = glm::min(min, vertex->position);
                max = glm::max(max, vertex->position);
            }

            return { min, max };
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
            // Find the vertex that is the furthest in the given direction
            float max_dot = glm::dot(shape->vertices[0]->position, direction);
            size_t max_index = 0;

            // Start loop at 1 as we've already calculated for vertex 0
            for (size_t i = 1; i < shape->vertices.size(); i++) {
                const float dot = glm::dot(shape->vertices[i]->position, direction);
                if (dot > max_dot) {
                    max_dot = dot;
                    max_index = i;
                }
            }
            return shape->vertices[max_index]->position;
        }
    };
}