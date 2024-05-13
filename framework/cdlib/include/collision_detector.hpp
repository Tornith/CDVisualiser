#pragma once
#include <optional>
#include <set>
#include <unordered_set>
#include <utility>

#include "collider.hpp"
#include "collision_data.hpp"

namespace cdlib {
    // Abstract class for different collision detection methods
    class NarrowCollisionDetector {
    protected:
        ColliderP collider_1{};
        ColliderP collider_2{};
    public:
        NarrowCollisionDetector() = default;

        NarrowCollisionDetector(ColliderP collider_1, ColliderP collider_2)
            : collider_1(std::move(collider_1)),
              collider_2(std::move(collider_2)) {
        }

        virtual ~NarrowCollisionDetector() = default;

        // Detects collision between two objects
        [[nodiscard]] virtual CollisionData get_collision_data() = 0;
    };

    struct CollisionPair {
        ColliderP collider_1;
        ColliderP collider_2;

        bool operator==(const CollisionPair& other) const {
            return (collider_1 == other.collider_1 && collider_2 == other.collider_2) ||
                   (collider_1 == other.collider_2 && collider_2 == other.collider_1);
        }

        [[nodiscard]] bool contains(const ColliderP& collider) const {
            return collider_1 == collider || collider_2 == collider;
        }

        // Hash function for CollisionPair
        struct Hash {
            std::size_t operator()(const CollisionPair& pair) const {
                const std::size_t h1 = std::hash<ColliderP>()(pair.collider_1);
                const std::size_t h2 = std::hash<ColliderP>()(pair.collider_2);
                return h1 + h2 + (h1 ^ h2);
            }
        };
    };

    using CollisionSet = std::unordered_set<CollisionPair, CollisionPair::Hash>;

    class BroadCollisionDetector {
    protected:
        std::set<ColliderP> colliders;
    public:
        BroadCollisionDetector() = default;

        explicit BroadCollisionDetector(std::set<ColliderP> colliders)
            : colliders(std::move(colliders)) {
        }

        virtual ~BroadCollisionDetector() = default;

        virtual void insert(const ColliderP& collider) = 0;
        virtual void remove(const ColliderP& collider) = 0;

        // Detects returns a list of pairs of colliding objects
        [[nodiscard]] virtual CollisionSet get_collisions() = 0;
    };
}