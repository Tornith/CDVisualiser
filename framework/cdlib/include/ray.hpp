#pragma once
#include <glm/glm.hpp>
#include <glm/vec3.hpp>
#include <glm/ext/scalar_constants.hpp>

namespace cdlib{
    struct Ray{
        glm::vec3 origin;
        glm::vec3 direction;
        float distance;

        Ray(const glm::vec3& origin, const glm::vec3& direction, const float distance = 1000.0f) : origin(origin), direction(normalize(direction)), distance(distance) {}

        [[nodiscard]] glm::vec3 from() const {
            return origin;
        }

        [[nodiscard]] glm::vec3 to() const {
            return origin + direction * distance;
        }

        [[nodiscard]] Ray clip(const float t_min, const float t_max) const {
            const auto new_origin = origin + direction * t_min * distance;
            const auto new_end = origin + direction * t_max * distance;
            return from_two_points(new_origin, new_end);
        }

        static Ray from_two_points(const glm::vec3& from, const glm::vec3& to) {
            const auto new_direction = to - from;
            const auto length = glm::length(new_direction);
            return {from, new_direction, length};
        }

        static bool intersects_triangle(const Ray& ray, const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, glm::vec3& intersection_point) {
            const glm::vec3 edge1 = b - a;
            const glm::vec3 edge2 = c - a;
            const glm::vec3 h = glm::cross(ray.direction, edge2);
            const float a_dot_h = glm::dot(edge1, h);

            // This means the ray is parallel to the triangle.
            if (a_dot_h > -glm::epsilon<float>() && a_dot_h < glm::epsilon<float>()) {
                return false;
            }

            const float f = 1.0f / a_dot_h;
            const glm::vec3 s = ray.origin - a;
            const float u = f * glm::dot(s, h);

            // Intersection lies outside of the triangle.
            if (u < 0.0f || u > 1.0f) {
                return false;
            }

            const glm::vec3 q = glm::cross(s, edge1);
            const float v = f * glm::dot(ray.direction, q);

            // Intersection lies outside of the triangle.
            if (v < 0.0f || u + v > 1.0f) {
                return false;
            }

            // Compute the intersection point.
            const float t = f * glm::dot(edge2, q);
            if (t > glm::epsilon<float>()) {  // ray intersection
                intersection_point = ray.origin + ray.direction * t;
                return true;
            }

            // No hit, ray intersection point lies behind the ray origin.
            return false;
        }
    };
}
