#pragma once

#include <memory>
#include <optional>

#include "collider.hpp"
#include "collision_data.hpp"
#include "collision_detector.hpp"
#include "convex_polyhedron.hpp"
#include "voronoi.hpp"

namespace cdlib{
    enum VClipState {
        INIT,
        PENETRATION,
        CONTINUE,
        DONE
    };

    struct ClipData {
        bool is_clipped;
        float lambda_l;
        float lambda_h;
        std::shared_ptr<Feature> neighbour_l;
        std::shared_ptr<Feature> neighbour_h;

        glm::vec3 point_l{};
        glm::vec3 point_h{};

        ClipData(glm::vec3 start, glm::vec3 end, bool is_clipped, float lambda_l, float lambda_h, const std::shared_ptr<Feature>& neighbour_l,
            const std::shared_ptr<Feature>& neighbour_h)
            : is_clipped(is_clipped),
              lambda_l(lambda_l),
              lambda_h(lambda_h),
              neighbour_l(neighbour_l),
              neighbour_h(neighbour_h)
        {
            const auto direction = end - start;
            point_l = start + lambda_l * direction;
            point_h = start + lambda_h * direction;
        }
    };

    class VClip : NarrowCollisionDetector {
        VClipState state = INIT;

        std::shared_ptr<Feature> feature_1;
        std::shared_ptr<Feature> feature_2;

        uint_fast8_t primary_feature_index = 0;
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

        template<typename T = Feature> requires Voronoi::IsFeature<T>
        [[nodiscard]] const std::shared_ptr<T>& primary_feature() const {
            const auto primary = primary_feature_index == 0 ? feature_1 : feature_2;
            return std::dynamic_pointer_cast<T>(primary);
        }

        template<typename T = Feature> requires Voronoi::IsFeature<T>
        [[nodiscard]] const std::shared_ptr<T>& secondary_feature() const {
            const auto secondary = primary_feature_index == 0 ? feature_2 : feature_1;
            return std::dynamic_pointer_cast<T>(secondary);
        }

        template<typename T> requires Voronoi::IsFeature<T>
        [[nodiscard]] const std::shared_ptr<T>& get_feature() const {
            if (std::dynamic_pointer_cast<T>(feature_1)) {
                return std::dynamic_pointer_cast<T>(feature_1);
            } else if (std::dynamic_pointer_cast<T>(feature_2)) {
                return std::dynamic_pointer_cast<T>(feature_2);
            }
            return nullptr;
        }

        void set_primary_feature(const std::shared_ptr<Feature>& feature) {
            if (primary_feature_index == 0) {
                feature_1 = feature;
            } else {
                feature_2 = feature;
            }
        }

        void set_secondary_feature(const std::shared_ptr<Feature>& feature) {
            if (primary_feature_index == 0) {
                feature_2 = feature;
            } else {
                feature_1 = feature;
            }
        }

        template<typename T> requires Voronoi::IsFeature<T>
        void set_feature(const std::shared_ptr<Feature>& feature) {
            if (std::dynamic_pointer_cast<T>(feature_1)) {
                feature_1 = feature;
            } else {
                feature_2 = feature;
            }
        }

        template<typename T> requires Voronoi::IsFeature<T>
        void set_feature_as_primary() {
            if (std::dynamic_pointer_cast<T>(feature_1)) {
                primary_feature_index = 0;
            } else {
                primary_feature_index = 1;
            }
        }

        [[nodiscard]] VClipState get_state() const {
            return state;
        }

        ~VClip() override = default;

        bool post_clip_derivative_update(const ClipData& clip_data, float derivative_l, float derivative_h);
        bool post_clip_derivative_check(const ClipData& clip_data);

        [[nodiscard]] VClipState handle_local_minimum();

        // States
        [[nodiscard]] VClipState execute_vertex_vertex();
        [[nodiscard]] VClipState execute_vertex_edge();
        [[nodiscard]] VClipState execute_vertex_face();
        [[nodiscard]] VClipState execute_edge_edge();
        [[nodiscard]] VClipState execute_edge_face();

        [[nodiscard]] std::optional<CollisionData> get_collision_data() override;
    };

    [[nodiscard]] ClipData clip_edge(const std::shared_ptr<HalfEdge>& edge, const std::shared_ptr<Feature>& feature);
}
