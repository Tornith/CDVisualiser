#pragma once

namespace cdlib {
    struct CollisionData {
        glm::vec3 normal{};  // Normal of the collision
        float depth{};       // Depth of the collision
    };
}