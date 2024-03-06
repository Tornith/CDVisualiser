#pragma once
#include <optional>
#include <utility>

#include "collider.hpp"
#include "collision_data.hpp"

namespace cdlib {
    // Abstract class for different collision detection methods
    class CollisionDetector {
    protected:
        std::shared_ptr<Collider> collider_1{};
        std::shared_ptr<Collider> collider_2{};
    public:
        CollisionDetector() = default;

        CollisionDetector(std::shared_ptr<Collider> collider_1, std::shared_ptr<Collider> collider_2)
            : collider_1(std::move(collider_1)),
              collider_2(std::move(collider_2)) {
        }

        virtual ~CollisionDetector() = default;

        // Detects collision between two objects
        [[nodiscard]] virtual std::optional<CollisionData> get_collision_data() = 0;
    };
}