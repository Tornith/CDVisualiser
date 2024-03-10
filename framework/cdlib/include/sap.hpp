#pragma once
#include <array>
#include <memory>

#include "collision_detector.hpp"

namespace cdlib {
    struct Endpoint {
        std::shared_ptr<Collider> collider;
        Endpoint *next, *prev;
        float value;
        bool is_min;
    };

    class SAP : BroadCollisionDetector {
    protected:
        std::array<std::vector<Endpoint>, 3> endpoints;
        std::vector<std::pair<std::shared_ptr<Collider>, std::shared_ptr<Collider>>> collisions;
    public:
        SAP() = default;

        explicit SAP(const std::vector<std::shared_ptr<Collider>>& colliders)
            : BroadCollisionDetector(colliders) {
            create_endpoints();
        }

        void create_endpoints();
        void sort_endpoints();

        void update_endpoints(std::vector<std::shared_ptr<Collider>> changed_colliders);

        [[nodiscard]] std::vector<std::pair<std::shared_ptr<Collider>, std::shared_ptr<Collider>>> get_collisions() override;
    };
}
