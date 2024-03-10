#pragma once

#include <memory>
#include <optional>

#include "collider.hpp"
#include "collision_data.hpp"
#include "collision_detector.hpp"

namespace cdlib{
    enum VClipState {
        VCLIP_VERTEX_VERTEX,
        VCLIP_VERTEX_EDGE,
        VCLIP_EDGE_EDGE,
        VCLIP_VERTEX_FACE,
        VCLIP_EDGE_FACE
    };

    class VClip : NarrowCollisionDetector {
    public:
        VClip() = default;

        VClip(std::shared_ptr<Collider> collider_1, std::shared_ptr<Collider> collider_2)
            : NarrowCollisionDetector(std::move(collider_1), std::move(collider_2)) {
        }

        VClip(const VClip& other) = default;

        VClip(VClip&& other) noexcept
            : NarrowCollisionDetector(other.collider_1, other.collider_2) {
        }

        VClip& operator=(const VClip& other) {
            if (this == &other)
                return *this;
            collider_1 = other.collider_1;
            collider_2 = other.collider_2;
            return *this;
        }

        VClip& operator=(VClip&& other) noexcept {
            if (this == &other)
                return *this;
            collider_1 = other.collider_1;
            collider_2 = other.collider_2;
            return *this;
        }

        ~VClip() override = default;

        bool clip_edge() {

        }

        [[nodiscard]] std::optional<CollisionData> get_collision_data() override;
    };
}
