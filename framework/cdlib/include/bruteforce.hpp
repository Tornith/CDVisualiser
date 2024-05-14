#pragma once
#include "collision_detector.hpp"

namespace cdlib{
    class NarrowBruteforce final : public NarrowCollisionDetector {
    public:
        NarrowBruteforce() = default;

        NarrowBruteforce(ColliderP collider_1, ColliderP collider_2)
            : NarrowCollisionDetector(std::move(collider_1), std::move(collider_2)) {
        }

        NarrowBruteforce(const NarrowBruteforce& other) = default;

        NarrowBruteforce(NarrowBruteforce&& other) noexcept
            : NarrowCollisionDetector(other.collider_1, other.collider_2) {
        }

        NarrowBruteforce& operator=(const NarrowBruteforce& other) {
            if (this == &other)
                return *this;
            collider_1 = other.collider_1;
            collider_2 = other.collider_2;
            return *this;
        }

        NarrowBruteforce& operator=(NarrowBruteforce&& other) noexcept {
            if (this == &other)
                return *this;
            collider_1 = other.collider_1;
            collider_2 = other.collider_2;
            return *this;
        }

        ~NarrowBruteforce() override = default;

        [[nodiscard]] std::pair<FeatureP, FeatureP> get_closest_features() const;
        [[nodiscard]] bool is_colliding() const;

        CollisionData get_collision_data() override;

        static std::set<RayCastResult> raycast(const Ray& ray, const std::set<ColliderP>& colliders);
        static RayCastResult raycast(const Ray& ray, const ColliderP& collider);
    };

    class BroadBruteforce final : public BroadCollisionDetector {
    public:
        BroadBruteforce() = default;

        explicit BroadBruteforce(std::set<ColliderP> colliders)
            : BroadCollisionDetector(std::move(colliders)) {
        }

        BroadBruteforce(const BroadBruteforce& other) = default;

        BroadBruteforce(BroadBruteforce&& other) noexcept
            : BroadCollisionDetector(other.colliders) {
        }

        BroadBruteforce& operator=(const BroadBruteforce& other) {
            if (this == &other)
                return *this;
            colliders = other.colliders;
            return *this;
        }

        BroadBruteforce& operator=(BroadBruteforce&& other) noexcept {
            if (this == &other)
                return *this;
            colliders = other.colliders;
            return *this;
        }

        ~BroadBruteforce() override = default;

        void insert(const ColliderP& collider) override;
        void remove(const ColliderP& collider) override;

        [[nodiscard]] CollisionSet get_collisions() override;

        [[nodiscard]] std::set<ColliderP> raycast(const Ray& ray) override;
    };
}
