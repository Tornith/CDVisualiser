#pragma once

namespace cdlib {
    struct CollisionData {
        glm::vec3 normal{};  // Normal of the collision
        float depth{};       // Depth of the collision

        // Features
        std::shared_ptr<Feature> feature_1;
        std::shared_ptr<Feature> feature_2;
    };
}