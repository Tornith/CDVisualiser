#pragma once

#include <memory>
#include <optional>

#include "collider.hpp"
#include "collision_data.hpp"
#include "collision_detector.hpp"
#include "convex_polyhedron.hpp"

namespace cdlib{
    enum VClipState {
        VCLIP_VERTEX_VERTEX,
        VCLIP_VERTEX_EDGE,
        VCLIP_EDGE_EDGE,
        VCLIP_VERTEX_FACE,
        VCLIP_EDGE_FACE
    };

    struct ClipData {
        bool is_clipped;
        float lambda_l;
        float lambda_h;
        std::shared_ptr<Feature> neighbour_l;
        std::shared_ptr<Feature> neighbour_h;
    };

    class VClip : NarrowCollisionDetector {
        VClipState state = VCLIP_VERTEX_VERTEX;
        std::shared_ptr<Feature> feature_1;
        std::shared_ptr<Feature> feature_2;
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

        [[nodiscard]] std::optional<CollisionData> get_collision_data() override;
    };

    [[nodiscard]] ClipData clip_edge(const std::shared_ptr<HalfEdge>& edge, const std::shared_ptr<Feature>& feature);

    void post_clip_derivative_check(const ClipData& clip_data, const std::shared_ptr<HalfEdge>& edge, const std::shared_ptr<Feature>& feature);
}
