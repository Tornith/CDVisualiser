#pragma once

namespace cdlib {
    struct CollisionData {
        glm::vec3 point_1{}; // Point on the first object
        glm::vec3 point_2{}; // Point on the second object
        glm::vec3 normal{};  // Normal of the collision
        float depth{};       // Depth of the collision
    };
}