#pragma once

#include <numeric>
#include <vector>
#include <glm/glm.hpp>
namespace cdlib {
    class Collider {
    protected:
        std::vector<glm::vec3> vertices;
        glm::mat4 transform{};
        std::pair<glm::vec3, glm::vec3> aabb;

        std::vector<glm::vec3> cached_global_vertices;
        bool is_cache_valid{ false };

    public:
        Collider() = default;

        explicit Collider(const std::vector<glm::vec3>& vertices) : vertices(vertices) {
            aabb = calculate_aabb(get_global_vertices());
        }

        virtual ~Collider() = default;

        Collider(const Collider& other) = default;
        Collider(Collider&& other) noexcept : vertices(std::move(other.vertices)), aabb(other.aabb) {}

        Collider& operator=(const Collider& other) {
            if (this == &other)
                return *this;
            vertices = other.vertices;
            aabb = other.aabb;
            return *this;
        }

        Collider& operator=(Collider&& other) noexcept {
            if (this == &other)
                return *this;
            vertices = std::move(other.vertices);
            aabb = other.aabb;
            return *this;
        }

        [[nodiscard]] const std::vector<glm::vec3>& get_local_vertices() const {
            return vertices;
        }

        [[nodiscard]] const glm::mat4& get_transform_matrix() const {
            return transform;
        }

        void set_transform_matrix(const glm::mat4& transform) {
            is_cache_valid = false;
            Collider::transform = transform;
        }

        [[nodiscard]] const std::vector<glm::vec3>& get_global_vertices() {
            if (is_cache_valid) {
                return cached_global_vertices;
            }

            std::vector<glm::vec3> global_vertices;
            global_vertices.reserve(vertices.size());
            for (const auto& vertex : vertices) {
                global_vertices.emplace_back(transform * glm::vec4(vertex, 1.f));
            }

            is_cache_valid = true;
            cached_global_vertices = global_vertices;

            return cached_global_vertices;
        }

        [[nodiscard]] glm::vec3 get_global_vertex(const size_t index) const {
            if (is_cache_valid) {
                return cached_global_vertices[index];
            }
            return transform * glm::vec4(vertices[index], 1.f);
        }

        virtual void set_vertices(const std::vector<glm::vec3>& vertices) {
            is_cache_valid = false;
            Collider::vertices = vertices;
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
            aabb = calculate_aabb(vertices);
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
    };

    class ConvexCollider final : public Collider {
    public:
        ConvexCollider() = default;

        explicit ConvexCollider(const std::vector<glm::vec3>& vertices)
            : Collider(vertices) {
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
            float max_dot = glm::dot(vertices[0], direction);
            size_t max_index = 0;

            // Start loop at 1 as we've already calculated for vertex 0
            for (size_t i = 1; i < vertices.size(); i++) {
                const float dot = glm::dot(vertices[i], direction);
                if (dot > max_dot) {
                    max_dot = dot;
                    max_index = i;
                }
            }
            return vertices[max_index];
        }
    };
}