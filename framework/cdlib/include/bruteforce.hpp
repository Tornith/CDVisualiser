#pragma once
#include "collision_detector.hpp"

namespace cdlib{
    class Bruteforce : public NarrowCollisionDetector {
    public:
        Bruteforce() = default;

        Bruteforce(std::shared_ptr<Collider> collider_1, std::shared_ptr<Collider> collider_2)
            : NarrowCollisionDetector(std::move(collider_1), std::move(collider_2)) {
        }

        Bruteforce(const Bruteforce& other) = default;

        Bruteforce(Bruteforce&& other) noexcept
            : NarrowCollisionDetector(other.collider_1, other.collider_2) {
        }

        Bruteforce& operator=(const Bruteforce& other) {
            if (this == &other)
                return *this;
            collider_1 = other.collider_1;
            collider_2 = other.collider_2;
            return *this;
        }

        Bruteforce& operator=(Bruteforce&& other) noexcept {
            if (this == &other)
                return *this;
            collider_1 = other.collider_1;
            collider_2 = other.collider_2;
            return *this;
        }

        ~Bruteforce() override = default;

        [[nodiscard]] std::pair<FeatureP, FeatureP> get_closest_features() const;
        [[nodiscard]] bool is_colliding() const;

        CollisionData get_collision_data() override;
    };
}
