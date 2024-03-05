#pragma once

#include <numeric>
#include <vector>
#include <glm/glm.hpp>

class Collider {
protected:
    std::vector<glm::vec3> vertices;
    std::pair<glm::vec3, glm::vec3> aabb;

public:
    Collider() = default;

    explicit Collider(const std::vector<glm::vec3>& vertices) : vertices(vertices) {
        aabb = calculate_aabb(vertices);
    }

    Collider(const std::vector<glm::vec3>& vertices, const std::pair<glm::vec3, glm::vec3>& aabb)
        : vertices(vertices), aabb(aabb) {
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

    [[nodiscard]] const std::vector<glm::vec3>& get_vertices() const {
        return vertices;
    }

    virtual void set_vertices(const std::vector<glm::vec3>& vertices) {
        Collider::vertices = vertices;
    }

    [[nodiscard]] virtual glm::vec3 support(const glm::vec3& direction) const = 0;

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
    glm::vec3 center_of_mass{};
public:
    ConvexCollider() = default;

    explicit ConvexCollider(const std::vector<glm::vec3>& vertices) : Collider(vertices) {
        update_center_of_mass();
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

    void update_center_of_mass() {
        center_of_mass = std::accumulate(vertices.begin(), vertices.end(), glm::vec3(0.f)) / static_cast<float>(vertices.size());
    }

    void set_vertices(const std::vector<glm::vec3>& vertices) override {
        Collider::set_vertices(vertices);
        update_center_of_mass();
        update_aabb();
    }

    [[nodiscard]] glm::vec3 support(const glm::vec3& direction) const override {
        // Find the vertex that is the furthest in the given direction
        float max_dot = glm::dot(vertices[0] - center_of_mass, direction);
        size_t max_index = 0;

        // Start loop at 1 as we've already calculated for vertex 0
        for (size_t i = 1; i < vertices.size(); i++) {
            const float dot = glm::dot(vertices[i] - center_of_mass, direction);
            if (dot > max_dot) {
                max_dot = dot;
                max_index = i;
            }
        }
        return vertices[max_index];
    }
};