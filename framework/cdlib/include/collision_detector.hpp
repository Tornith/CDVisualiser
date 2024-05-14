#pragma once
#include <optional>
#include <set>
#include <unordered_set>
#include <utility>

#include "collider.hpp"
#include "collision_data.hpp"

namespace cdlib {
    struct RayCastResult {
        bool hit;
        ColliderP collider;
        glm::vec3 position;
        glm::vec3 normal;
        float distance;

        bool operator<(const RayCastResult& other) const {
            if (hit != other.hit) return hit < other.hit;
            if (collider != other.collider) return collider < other.collider;
            if (position != other.position) return length(position) < length(other.position);
            if (normal != other.normal) return length(normal) < length(other.normal);
            return distance < other.distance;
        }

        friend bool operator==(const RayCastResult& lhs, const RayCastResult& rhs) {
            return lhs.hit == rhs.hit
                   && lhs.collider == rhs.collider
                   && lhs.position == rhs.position
                   && lhs.normal == rhs.normal
                   && lhs.distance == rhs.distance;
        }

        friend bool operator!=(const RayCastResult& lhs, const RayCastResult& rhs) {
            return !(lhs == rhs);
        }

        struct Hash {
            std::size_t operator()(const RayCastResult& r) const {
                const std::size_t h1 = std::hash<bool>{}(r.hit);
                const std::size_t h2 = r.collider ? std::hash<ColliderP>{}(r.collider) : 0;
                const std::size_t h3 = std::hash<float>{}(r.position.x + r.position.y + r.position.z);
                const std::size_t h4 = std::hash<float>{}(r.normal.x + r.normal.y + r.normal.z);
                const std::size_t h5 = std::hash<float>{}(r.distance);

                return h1 ^ h2 << 1 ^ h3 << 2 ^ h4 << 3 ^ h5;
            }
        };
    };

    using RayCastResultSet = std::unordered_set<RayCastResult, RayCastResult::Hash>;

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

        // Raycast
        static RayCastResult raycast(const Ray& ray, const ColliderP& collider) {
            return {};
        };
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

        [[nodiscard]] virtual RayCastResultSet raycast(const Ray& ray) = 0;
        [[nodiscard]] virtual RayCastResultSet raycast(const glm::vec3& origin, const glm::vec3& direction) {
            return raycast(Ray(origin, direction));
        }
    };
}