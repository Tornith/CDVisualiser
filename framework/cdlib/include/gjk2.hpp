#pragma once

#include <memory>
#include <utility>

#include "collider.hpp"
#include "collision_data.hpp"
#include "collision_detector.hpp"
#include "convex_polyhedron.hpp"
#include "voronoi.hpp"

namespace cdlib{
    enum GJK2State {
        INIT,
        ITERATION_1,
        ITERATION_2,
        ITERATION_3,
        ITERATION_4,
        ITERATION_5,
        EPA,
        DONE,
        ERROR
    };

    class GJK2 : public NarrowCollisionDetector {
    protected:
        GJK2State state = INIT;

        FeatureP feature_1;
        FeatureP feature_2;

    public:
        GJK2() = default;

        GJK2(std::shared_ptr<Collider> collider_1, std::shared_ptr<Collider> collider_2)
            : NarrowCollisionDetector(std::move(collider_1), std::move(collider_2)) {
        }

        GJK2(const GJK2& other) = default;

        GJK2(GJK2&& other) noexcept : NarrowCollisionDetector(std::move(other.collider_1), std::move(other.collider_2)) {}

        GJK2& operator=(const GJK2& other) {
            if (this == &other)
                return *this;
            collider_1 = other.collider_1;
            collider_2 = other.collider_2;
            return *this;
        }

        GJK2& operator=(GJK2&& other) noexcept {
            if (this == &other)
                return *this;
            collider_1 = std::move(other.collider_1);
            collider_2 = std::move(other.collider_2);
            return *this;
        }

        [[nodiscard]] GJK2State get_state() const {
            return state;
        }

        ~GJK2() override = default;

        // States
        [[nodiscard]] virtual GJK2State initialize();
        [[nodiscard]] virtual GJK2State execute_iteration_1();
        [[nodiscard]] virtual GJK2State execute_iteration_2();
        [[nodiscard]] virtual GJK2State execute_iteration_3();
        [[nodiscard]] virtual GJK2State execute_iteration_4();
        [[nodiscard]] virtual GJK2State execute_iteration_5();

        [[nodiscard]] virtual GJK2State step();

        void reset();

        [[nodiscard]] CollisionData get_collision_data() override;
    };

    class GJK2Raycast final : GJK2 {
        glm::vec3 ray_origin{};
        glm::vec3 ray_direction{};

    public:
        GJK2Raycast() = default;

        GJK2Raycast(const glm::vec3& ray_origin, const glm::vec3& ray_direction)
            : ray_origin(ray_origin),
              ray_direction(ray_direction)
        {}

        GJK2Raycast(const std::shared_ptr<Collider>& collider, const glm::vec3& ray_origin, const glm::vec3& ray_direction)
            : ray_origin(ray_origin), ray_direction(ray_direction)
        {
            collider_1 = collider;
            collider_2 = std::make_shared<RayCollider>(ray_origin, ray_direction);
        }

        GJK2Raycast(const GJK2Raycast& other) = default;
        GJK2Raycast(GJK2Raycast&& other) noexcept = default;
        GJK2Raycast& operator=(const GJK2Raycast& other) = default;
        GJK2Raycast& operator=(GJK2Raycast&& other) noexcept = default;

        [[nodiscard]] CollisionData get_collision_data() override;

        [[nodiscard]] GJK2State step() override {
            return GJK2::step();
        }
    };
}
