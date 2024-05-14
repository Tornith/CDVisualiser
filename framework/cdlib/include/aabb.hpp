#pragma once
#include <optional>
#include <glm/vec3.hpp>

#include "ray.hpp"

namespace cdlib {
    struct AABBRayCastResult {
        bool hit;
        float t_min;
        float t_max;
        glm::vec3 t_min_normal;
        glm::vec3 t_max_normal;
    };

    struct AABB {
        glm::vec3 min;
        glm::vec3 max;

        bool operator==(const AABB& other) const {
            return min == other.min && max == other.max;
        }

        [[nodiscard]] bool intersects(const AABB& other) const {
            return min.x <= other.max.x && max.x >= other.min.x &&
                   min.y <= other.max.y && max.y >= other.min.y &&
                   min.z <= other.max.z && max.z >= other.min.z;
        }

        [[nodiscard]] float get_surface_area() const {
            const glm::vec3 diff = max - min;
            return 2.0f * (diff.x * diff.y + diff.x * diff.z + diff.y * diff.z);
        }

        [[nodiscard]] glm::vec3 get_center() const {
            return (min + max) / 2.0f;
        }

        [[nodiscard]] glm::vec3 get_extent() const {
            return max - min;
        }

        [[nodiscard]] bool contains(const glm::vec3& point) const {
            return point.x >= min.x && point.x <= max.x &&
                   point.y >= min.y && point.y <= max.y &&
                   point.z >= min.z && point.z <= max.z;
        }

        [[nodiscard]] bool contains(const AABB& other) const {
            return min.x <= other.min.x && max.x >= other.max.x &&
                   min.y <= other.min.y && max.y >= other.max.y &&
                   min.z <= other.min.z && max.z >= other.max.z;
        }

        [[nodiscard]] AABB merge(const AABB& other) const {
            return {glm::min(min, other.min), glm::max(max, other.max)};
        }

        [[nodiscard]] AABB merge(const glm::vec3& point) const {
            return {glm::min(min, point), glm::max(max, point)};
        }

        [[nodiscard]] AABBRayCastResult raycast(const glm::vec3& from, const glm::vec3& to, float t_min, float t_max) const {
            const auto direction = to - from;
            const auto inv_direction = 1.0f / direction;

            auto t_min_axis = -1;
            auto t_max_axis = -1;

            for (auto axis = 0; axis < 3; ++axis) {
                auto t0 = (min[axis] - from[axis]) * inv_direction[axis];
                auto t1 = (max[axis] - from[axis]) * inv_direction[axis];

                if (inv_direction[axis] < 0.0f) {
                    std::swap(t0, t1);
                }

                if (t0 > t_min) {
                    t_min = t0;
                    t_min_axis = axis;
                }

                if (t1 < t_max) {
                    t_max = t1;
                    t_max_axis = axis;
                }

                if (t_max <= t_min) {
                    return {
                        false,
                        std::numeric_limits<float>::max(),
                        std::numeric_limits<float>::max(),
                        glm::vec3(0.0f),
                        glm::vec3(0.0f)
                    };
                }
            }

            // Check if we have encountered a hit
            if (t_min_axis == -1 || t_max_axis == -1) {
                return {
                    false,
                    t_min,
                    t_max,
                    glm::vec3(0.0f),
                    glm::vec3(0.0f)
                };
            }

            // Convert axes to normals
            auto t_min_normal = glm::vec3(0.0f);
            auto t_max_normal = glm::vec3(0.0f);

            t_min_normal[t_min_axis] = direction[t_min_axis] < 0.0f ? 1.0f : -1.0f;
            t_max_normal[t_max_axis] = direction[t_max_axis] < 0.0f ? 1.0f : -1.0f;

            return {true, t_min, t_max, t_min_normal, t_max_normal};
        }

        [[nodiscard]] AABBRayCastResult raycast(const Ray& ray) const {
            return raycast(ray.from(), ray.to(), 0.0f, 1.0f);
        }

        [[nodiscard]] AABB fatten(const glm::vec3& margin) const {
            return {min - margin, max + margin};
        }

        [[nodiscard]] AABB shrink(const glm::vec3& margin) const {
            return {min + margin, max - margin};
        }

        [[nodiscard]] AABB displace(const glm::vec3& displacement) const {
            glm::vec3 min = this->min;
            glm::vec3 max = this->max;

            if (displacement.x > 0.0f) {
                max.x += displacement.x;
            } else {
                min.x += displacement.x;
            }

            if (displacement.y > 0.0f) {
                max.y += displacement.y;
            } else {
                min.y += displacement.y;
            }

            if (displacement.z > 0.0f) {
                max.z += displacement.z;
            } else {
                min.z += displacement.z;
            }

            return {min, max};
        }
    };
};