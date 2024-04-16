#pragma once

#include <memory>
#include <optional>
#include <utility>

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
        DONE,
        ERROR
    };

    struct ClipData {
        bool is_clipped;
        float lambda_l;
        float lambda_h;
        FeatureP neighbour_l;
        FeatureP neighbour_h;

        HalfEdgeP clipped_edge;
        FeatureP clipping_feature;

        glm::vec3 point_l{};
        glm::vec3 point_h{};

        ClipData(glm::vec3 start, glm::vec3 end, bool is_clipped, float lambda_l, float lambda_h,
            FeatureP neighbour_l, FeatureP neighbour_h, HalfEdgeP clipped_edge, FeatureP clipping_feature)
            : is_clipped(is_clipped),
              lambda_l(lambda_l),
              lambda_h(lambda_h),
              neighbour_l(std::move(neighbour_l)),
              neighbour_h(std::move(neighbour_h)),
              clipped_edge(std::move(clipped_edge)),
              clipping_feature(std::move(clipping_feature))
        {
            const auto direction = start - end;
            point_l = end + lambda_l * direction;
            point_h = end + lambda_h * direction;
        }

        [[nodiscard]] bool simply_excluded() const {
            return !is_clipped && neighbour_l == neighbour_h;
        }

        [[nodiscard]] bool compoundly_excluded() const {
            return !is_clipped && neighbour_l != neighbour_h;
        }
    };

    class VClip : public NarrowCollisionDetector {
    protected:
        VClipState state = INIT;

        FeatureP feature_1;
        FeatureP feature_2;

        uint_fast8_t primary_feature_index = 0;

    public:
        VClip() = default;

        VClip(std::shared_ptr<Collider> collider_1, std::shared_ptr<Collider> collider_2)
            : NarrowCollisionDetector(std::move(collider_1), std::move(collider_2)) {
        }

        VClip(const VClip& other) = default;

        VClip(VClip&& other) noexcept : NarrowCollisionDetector(std::move(other.collider_1), std::move(other.collider_2)) {}

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
            collider_1 = std::move(other.collider_1);
            collider_2 = std::move(other.collider_2);
            return *this;
        }

        template<typename T = Feature> requires IsFeature<T>
        [[nodiscard]] std::shared_ptr<T> primary_feature() const {
            const auto primary = primary_feature_index == 0 ? feature_1 : feature_2;
            return std::dynamic_pointer_cast<T>(primary);
        }

        template<typename T = Feature> requires IsFeature<T>
        [[nodiscard]] std::shared_ptr<T> secondary_feature() const {
            const auto secondary = primary_feature_index == 0 ? feature_2 : feature_1;
            return std::dynamic_pointer_cast<T>(secondary);
        }

        template<typename T = Feature> requires IsFeature<T>
        [[nodiscard]] std::shared_ptr<T> get_other_feature(const FeatureP& feature) const {
            if (feature == feature_1) {
                return std::dynamic_pointer_cast<T>(feature_2);
            } if (feature == feature_2) {
                return std::dynamic_pointer_cast<T>(feature_1);
            }
            return nullptr;
        }

        template<typename T> requires IsFeature<T>
        [[nodiscard]] std::shared_ptr<T> get_feature() const {
            if (std::dynamic_pointer_cast<T>(feature_1)) {
                return std::dynamic_pointer_cast<T>(feature_1);
            } else if (std::dynamic_pointer_cast<T>(feature_2)) {
                return std::dynamic_pointer_cast<T>(feature_2);
            }
            return nullptr;
        }

        void set_primary_feature(const FeatureP& feature) {
            if (primary_feature_index == 0) {
                feature_1 = feature;
            } else {
                feature_2 = feature;
            }
        }

        void set_secondary_feature(const FeatureP& feature) {
            if (primary_feature_index == 0) {
                feature_2 = feature;
            } else {
                feature_1 = feature;
            }
        }

        template<typename T> requires IsFeature<T>
        void set_feature(const FeatureP& feature) {
            if (std::dynamic_pointer_cast<T>(feature_1)) {
                feature_1 = feature;
            } else {
                feature_2 = feature;
            }
        }

        void set_feature(const FeatureP& set_feature, const FeatureP& replaced_feature) {
            if (feature_1 == replaced_feature) {
                feature_1 = set_feature;
            } else if (feature_2 == replaced_feature) {
                feature_2 = set_feature;
            } else {
                std::cerr << "Feature not found" << std::endl;
            }
        }

        template<typename T> requires IsFeature<T>
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

        static std::optional<float> distance_derivative_sign(glm::vec3 edge_point, const HalfEdgeP& edge, const VertexP& vertex);
        static std::optional<float> distance_derivative_sign(glm::vec3 edge_point, const HalfEdgeP& edge, const FaceP& face);

        static std::optional<float> distance_derivative_sign(glm::vec3 edge_point, const HalfEdgeP& edge, const FeatureP& feature) {
            if (std::dynamic_pointer_cast<Vertex>(feature)) {
                return distance_derivative_sign(edge_point, edge, std::dynamic_pointer_cast<Vertex>(feature));
            } if (std::dynamic_pointer_cast<Face>(feature)) {
                return distance_derivative_sign(edge_point, edge, std::dynamic_pointer_cast<Face>(feature));
            }
            return std::nullopt;
        }

        static std::optional<float> distance_derivative_sign(const float lambda, const HalfEdgeP& edge, const FeatureP& feature) {
            const auto edge_point = edge->start->get_position() + lambda * edge->get_direction();
            return distance_derivative_sign(edge_point, edge, feature);
        }

        static std::optional<FeatureP> post_clip_derivative_update(const ClipData& clip_data, std::optional<float> derivative_l, std::optional<float> derivative_h);
        static std::optional<FeatureP> post_clip_derivative_check(const ClipData& clip_data);

        [[nodiscard]] VClipState handle_local_minimum();

        [[nodiscard]] static FeatureP closest_face_vertex_to_edge(const HalfEdgeP& clipped_edge, const FaceP& face);

        // States
        [[nodiscard]] virtual VClipState initialize();
        [[nodiscard]] virtual VClipState execute_vertex_vertex();
        [[nodiscard]] virtual VClipState execute_vertex_edge();
        [[nodiscard]] virtual VClipState execute_vertex_face();
        [[nodiscard]] virtual VClipState execute_edge_edge();
        [[nodiscard]] virtual VClipState execute_edge_face();

        [[nodiscard]] virtual VClipState step();

        void reset();

        [[nodiscard]] CollisionData get_collision_data() override;

        [[nodiscard]] bool debug_brute_force(const FeatureP& expected_feature_1, const FeatureP& expected_feature_2) const;
    };

    [[nodiscard]] ClipData clip_edge(const HalfEdgeP& clipped_edge, const FeatureP& feature);

    template<typename T> requires IsFeature<T>
    [[nodiscard]] ClipData clip_edge(const HalfEdgeP& clipped_edge, const FeatureP& feature,
        const std::vector<std::shared_ptr<T>>& neighbours, const std::optional<ClipData>& previous_clip_data = std::nullopt) {

        auto is_clipped = true;
        float lambda_l = previous_clip_data.has_value() ? previous_clip_data->lambda_l : 0;
        float lambda_h = previous_clip_data.has_value() ? previous_clip_data->lambda_h : 1;
        FeatureP neighbour_l = previous_clip_data.has_value() ? previous_clip_data->neighbour_l : nullptr;
        FeatureP neighbour_h = previous_clip_data.has_value() ? previous_clip_data->neighbour_h : nullptr;

        // The original paper calculates the points in reverse order
        const auto head = clipped_edge->end->get_position();
        const auto tail = clipped_edge->start->get_position();

        // For all voronoi planes of the feature
        for (const auto& neighbour : neighbours){
            const auto voronoi_plane = Voronoi::get_voronoi_plane_safe(feature, neighbour);
            if (!voronoi_plane){
                continue;
            }

            // Get the distance of the edge points to the plane
            const float distance_head = voronoi_plane->distance_to(head);
            const float distance_tail = voronoi_plane->distance_to(tail);

            // Case 1: Both the points are below the plane => no intersection, the neighbouring feature is 'closer'
            if (distance_head < 0 && distance_tail < 0){
                is_clipped = false;
                neighbour_l = neighbour;
                neighbour_h = neighbour;
                break;
            }

            // Case 2: Tail is below the plane, head is above the plane
            if (distance_head >= 0 && distance_tail < 0){
                const float lambda = distance_tail / (distance_tail - distance_head);
                if (lambda > lambda_l){
                    lambda_l = lambda;
                    neighbour_l = neighbour;
                    // Check if the lower bound is greater than the upper bound
                    if (lambda_l > lambda_h){
                        is_clipped = false;
                        break;
                    }
                }
            }

            // Case 3: Head is below the plane, tail is above the plane
            else if (distance_head < 0 && distance_tail >= 0){
                const float lambda = distance_tail / (distance_tail - distance_head);
                if (lambda < lambda_h){
                    lambda_h = lambda;
                    neighbour_h = neighbour;
                    // Check if the lower bound is greater than the upper bound
                    if (lambda_l > lambda_h){
                        is_clipped = false;
                        break;
                    }
                }
            }
        }

        return {head, tail, is_clipped, lambda_l, lambda_h, neighbour_l, neighbour_h, clipped_edge, feature};
    }

    class VClipRaycast final : VClip {
        glm::vec3 ray_origin{};
        glm::vec3 ray_direction{};

    public:
        VClipRaycast() = default;

        VClipRaycast(const glm::vec3& ray_origin, const glm::vec3& ray_direction)
            : ray_origin(ray_origin),
              ray_direction(ray_direction)
        {}

        VClipRaycast(const std::shared_ptr<Collider>& collider, const glm::vec3& ray_origin, const glm::vec3& ray_direction)
            : ray_origin(ray_origin), ray_direction(ray_direction)
        {
            collider_1 = collider;
            collider_2 = std::make_shared<RayCollider>(ray_origin, ray_direction);
        }

        VClipRaycast(const VClipRaycast& other) = default;
        VClipRaycast(VClipRaycast&& other) noexcept = default;
        VClipRaycast& operator=(const VClipRaycast& other) = default;
        VClipRaycast& operator=(VClipRaycast&& other) noexcept = default;

        [[nodiscard]] CollisionData get_collision_data() override;

        [[nodiscard]] VClipState step() override {
            return VClip::step();
        }

        [[nodiscard]] VClipState initialize() override;
        [[nodiscard]] VClipState execute_vertex_edge() override;
        [[nodiscard]] VClipState execute_edge_edge() override;
        [[nodiscard]] VClipState execute_edge_face() override;
    };
}
