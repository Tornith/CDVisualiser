#pragma once
#include <array>
#include <memory>
#include <unordered_map>

#include "aabb.hpp"
#include "collision_detector.hpp"

namespace cdlib {
    struct Endpoint {
        ColliderP collider;
        std::array<std::shared_ptr<Endpoint>, 3> prev;
        std::array<std::shared_ptr<Endpoint>, 3> next;
        glm::vec3 value;
        bool is_min;

        std::shared_ptr<Endpoint> other_endpoint;

        [[nodiscard]] AABB get_aabb() const {
            return AABB(is_min ? value : other_endpoint->value, is_min ? other_endpoint->value : value);
        }
    };

    using EndpointP = std::shared_ptr<Endpoint>;

    class SAP final : BroadCollisionDetector {
    protected:
        std::array<EndpointP, 3> list_heads = {nullptr, nullptr, nullptr};
        std::unordered_map<ColliderP, std::pair<EndpointP, EndpointP>> endpoint_map;
        CollisionSet collisions;
    public:
        SAP() = default;

        explicit SAP(const std::set<ColliderP>& colliders)
            : BroadCollisionDetector(colliders) {
            for (const auto& collider : colliders) {
                insert(collider);
            }
        }

        void move_endpoint(size_t axis, const EndpointP& endpoint, const EndpointP& dest);
        void create_endpoints(const ColliderP& collider);
        void update_endpoints(const ColliderP& collider);

        std::pair<EndpointP, EndpointP> get_endpoints(const ColliderP& collider);

        void insert(const ColliderP& collider) override;
        void remove(const ColliderP& collider) override;

        [[nodiscard]] CollisionSet get_collisions() override;



        // Debug print lists
        void print_lists() const;
        void print_map() const;
    };
}
