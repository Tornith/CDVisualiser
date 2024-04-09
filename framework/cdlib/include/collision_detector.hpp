#pragma once
#include <optional>
#include <utility>

#include "collider.hpp"
#include "collision_data.hpp"

namespace cdlib {
    // Abstract class for different collision detection methods
    class NarrowCollisionDetector {
    protected:
        std::shared_ptr<Collider> collider_1{};
        std::shared_ptr<Collider> collider_2{};
    public:
        NarrowCollisionDetector() = default;

        NarrowCollisionDetector(std::shared_ptr<Collider> collider_1, std::shared_ptr<Collider> collider_2)
            : collider_1(std::move(collider_1)),
              collider_2(std::move(collider_2)) {
        }

        virtual ~NarrowCollisionDetector() = default;

        // Detects collision between two objects
        [[nodiscard]] virtual CollisionData get_collision_data() = 0;
    };

    class BroadCollisionDetector {
    protected:
        std::vector<std::shared_ptr<Collider>> colliders;
    public:
        BroadCollisionDetector() = default;

        explicit BroadCollisionDetector(std::vector<std::shared_ptr<Collider>> colliders)
            : colliders(std::move(colliders)) {
        }

        virtual ~BroadCollisionDetector() = default;

        // Detects returns a list of pairs of colliding objects
        [[nodiscard]] virtual std::vector<std::pair<std::shared_ptr<Collider>, std::shared_ptr<Collider>>> get_collisions() = 0;
    };
}