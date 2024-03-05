#pragma once
#include <optional>

#include "collider.hpp"
#include "collision_data.hpp"

// Abstract class for different collision detection methods
class CollisionDetector {
protected:
    const Collider* collider_1{};
    const Collider* collider_2{};
public:
    CollisionDetector() = default;

    CollisionDetector(const Collider* collider_1, const Collider* collider_2)
        : collider_1(collider_1),
          collider_2(collider_2) {
    }

    virtual ~CollisionDetector() = default;

    // Detects collision between two objects
    [[nodiscard]] virtual std::optional<cdlib::CollisionData> get_collision_data() = 0;
};