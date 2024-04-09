#pragma once

namespace cdlib {
    struct CollisionData {
        bool is_colliding{ false };
        glm::vec3 normal{};  // Normal of the collision
        float depth{};       // Depth of the collision

        // Features
        FeatureP feature_1;
        FeatureP feature_2;
    };
}