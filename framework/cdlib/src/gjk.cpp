#include "gjk.hpp"

#include <stdexcept>

#include "epa.hpp"

namespace cdlib {
    bool GJK::is_colliding() {
        // Reset the simplex
        simplex.clear();

        // Get the first arbitrary direction
        direction = normalize(collider_1->get_vertices()[0] - collider_2->get_vertices()[0]);

        for (int i = 0; i < MAX_ITERATIONS; i++) {
            // Get the support points
            glm::vec3 a = collider_1->support(direction);
            glm::vec3 b = collider_2->support(-direction);

            // Get the new point
            glm::vec3 new_point = a - b;

            if (simplex.size() < 2) {
                // If the simplex is empty or has only one point, we can add the new point
                simplex.push_back(new_point);
                direction = next_direction().first;
                continue;
            }

            // Check if we've passed the origin
            if (!passed_origin(direction, new_point)) {
                return false;
            }

            // Add the new point to the simplex
            simplex.push_back(new_point);

            // If the origin is in the simplex, the objects are colliding
            if (origin_in_simplex()) {
                return true;
            }

            // Get the next direction
            auto [new_direction, dropped_index] = next_direction();
            direction = new_direction;

            if (dropped_index.has_value()) {
                simplex.erase(simplex.begin() + dropped_index.value());
            }
        }

        // If we reached the maximum number of iterations, the result is inconclusive
        return false;
    }

    glm::vec3 GJK::get_line_normal(const glm::vec3& a, const glm::vec3& b, const glm::vec3& o) {
        // Get a vector, that is perpendicular to the line, and is facing the o point
        glm::vec3 ab = b - a;
        glm::vec3 ao = o - a;

        // Project the ao vector onto the ab vector
        float t = glm::dot(ao, ab) / glm::dot(ab, ab);
        glm::vec3 proj = ab * t;

        // Subtract the projected vector from the ao vector to get the normal
        glm::vec3 normal = ao - proj;
        return glm::normalize(normal);
    }

    glm::vec3 GJK::get_face_normal(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, const glm::vec3& o) {
        // The normal of the face is the perpendicular vector to the face, facing the o point
        glm::vec3 ab = b - a;
        glm::vec3 ac = c - a;
        glm::vec3 ao = o - a;
        glm::vec3 normal = glm::normalize(glm::cross(ab, ac));
        if (glm::dot(normal, ao) < 0) {
            return -normal;
        }
        return normal;
    }


    bool GJK::origin_in_simplex() {
        // If the simplex is a point, line or triangle we skip
        if (simplex.size() < 4) {
            return false;
        }

        // Helper lambda function to check if the origin is on the same side of the face as the fourth point
        auto same_side = [](const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, const glm::vec3& d) {
            glm::vec3 normal = glm::cross(b - a, c - a);
            auto dotD = glm::dot(normal, d - a);
            auto dotO = glm::dot(normal, glm::vec3(0.f) - a);
            return dotD * dotO > 0; // If the signs match, the result is positive and therefore on the same side
        };

        return same_side(simplex[0], simplex[1], simplex[2], simplex[3]) &&
            same_side(simplex[1], simplex[2], simplex[3], simplex[0]) &&
            same_side(simplex[2], simplex[3], simplex[0], simplex[1]) &&
            same_side(simplex[3], simplex[0], simplex[1], simplex[2]);
    }

    float GJK::line_point_distance(const glm::vec3&a, const glm::vec3&b, const glm::vec3&o) {
        // Get the normal of the line
        glm::vec3 normal = get_line_normal(a, b, o);
        glm::vec3 ao = o - a;

        return glm::dot(normal, ao);
    }

    float GJK::triangle_point_distance(const glm::vec3&a, const glm::vec3&b, const glm::vec3&c, const glm::vec3&o) {
        // Get the normal of the triangle
        glm::vec3 normal = get_face_normal(a, b, c, o);
        glm::vec3 ao = o - a;

        return glm::dot(normal, ao);
    }


    // Sanity check, if we crossed the origin
    bool GJK::passed_origin(const glm::vec3& normal, const glm::vec3& point) {
        return glm::dot(normal, point) >= 0;
    }

    std::pair<glm::vec3, std::optional<long long>> GJK::next_direction() const {
        // If the simplex is a point, return the opposite of the point
        if (simplex.size() == 1) {
            return { -simplex[0], std::nullopt };
        }

        // If the simplex is a line, return the normal of the line
        if (simplex.size() == 2) {
            return { get_line_normal(simplex[0], simplex[1]), std::nullopt };
        }

        // If the simplex is a triangle, return the normal of the triangle
        if (simplex.size() == 3) {
            return { get_face_normal(simplex[0], simplex[1], simplex[2]), std::nullopt };
        }

        // Otherwise the simplex is a tetrahedron, we need to evolve the simplex properly

        // Edges, coming out of the newest point D
        auto ad = simplex[3] - simplex[0];
        auto bd = simplex[3] - simplex[1];
        auto cd = simplex[3] - simplex[2];
        auto d0 = -simplex[3];

        // Get the normals of the faces
        auto abdN = cross(ad, bd);
        auto bcdN = cross(bd, cd);
        auto cadN = cross(cd, ad);

        // Check if the origin is on the same side of the faces as the point D
        if (dot(abdN, d0) > 0) {
            // The origin is on the outside of the face ABD
            // We can remove the point C
            return { abdN, 2 };
        }
        if (dot(bcdN, d0) > 0) {
            // The origin is on the outside of the face BCD
            // We can remove the point A
            return { bcdN, 0 };
        }
        if (dot(cadN, d0) > 0) {
            // The origin is on the outside of the face CAD
            // We can remove the point B
            return { cadN, 1 };
        }
        throw std::runtime_error("This should never happen, since we check for the origin in simplex earlier");
    }

    /**
     * GJK with EPA
     */

    std::optional<CollisionData> GJKEPA::get_collision_data() {
        if (!is_colliding()) {
            return std::nullopt;
        }

        // If the objects are colliding, we can use the EPA algorithm to get the collision data
        EPA epa(collider_1, collider_2, simplex);
        return epa.get_collision_data();
    }

    /**
     * SteppableGJK
     */

    void SteppableGJK::step_by_step_init() {
        step_by_step_initialized = true;
        step_by_step_gjk_finished = false;
        current_state = SteppableGJKState::UNINITIALIZED;
        iteration = 0;
        simplex_obj_1.clear();
        simplex_obj_2.clear();
    }

    SteppableGJKState SteppableGJK::step_by_step_next() {
        if (get_finished()) {
            return SteppableGJKState::FINISHED;
        }
        if (!step_by_step_initialized) {
            return SteppableGJKState::UNINITIALIZED;
        }
        const auto last_state = current_state;

        iteration_step();

        return last_state;
    }


    void SteppableGJK::preiteration() {
        // Reset the simplex
        simplex.clear();

        // Get the first arbitrary direction
        direction = glm::normalize(collider_1->get_vertices()[0] - collider_2->get_vertices()[0]);

        current_state = SteppableGJKState::ITERATION_1;
    }

    void SteppableGJK::iteration_substep_1() {
        // Get the support points
        current_point_a = collider_1->support(direction);
        current_point_b = collider_2->support(-direction);

        // Get the new point
        current_new_point = current_point_a - current_point_b;

        if (simplex.size() < 2) {
            // If the simplex is empty or has only one point, we can add the new point
            simplex.push_back(current_new_point);
            simplex_obj_1.push_back(current_point_a);
            simplex_obj_2.push_back(current_point_b);

            direction = next_direction().first;

            current_state = SteppableGJKState::ITERATION_1; // Go back to the first substep
            iteration++; // Increase the iteration
            return;
        }

        current_state = SteppableGJKState::ITERATION_2;
    }

    void SteppableGJK::iteration_substep_2() {
        if (!passed_origin(direction, current_new_point)) {
            step_by_step_gjk_finished = true;
            result = false;
            return;
        }

        current_state = SteppableGJKState::ITERATION_3;
    }

    void SteppableGJK::iteration_substep_3() {
        simplex.push_back(current_new_point);
        simplex_obj_1.push_back(current_point_a);
        simplex_obj_2.push_back(current_point_b);

        current_state = SteppableGJKState::ITERATION_4;
    }

    void SteppableGJK::iteration_substep_4() {
        if (origin_in_simplex()) {
            step_by_step_gjk_finished = true;
            result = true;
            return;
        }

        current_state = SteppableGJKState::ITERATION_5;
    }

    void SteppableGJK::iteration_substep_5() {
        auto [new_direction, dropped_index] = next_direction();
        direction = new_direction;

        if (dropped_index.has_value()) {
            // Using the simplex_map (which maps what points correspond to a given minkowski difference point)
            // we can remove the points from the simplex_obj_1 and simplex_obj_2
            auto [a, b] = simplex_map[simplex[dropped_index.value()]];
            // Check if there are no other references to the object points in the simplex_map
            if (std::count(simplex_obj_1.begin(), simplex_obj_1.end(), a) == 1) {
                simplex_obj_1.erase(std::ranges::find(simplex_obj_1, a));
            }
            if (std::count(simplex_obj_2.begin(), simplex_obj_2.end(), b) == 1) {
                simplex_obj_2.erase(std::ranges::find(simplex_obj_2, b));
            }

            // Remove the point from the main simplex
            simplex.erase(simplex.begin() + dropped_index.value());
        }

        // Go back to the first substep
        current_state = SteppableGJKState::ITERATION_1;
        iteration++;
    }

    void SteppableGJK::iteration_step() {
        if (current_state == SteppableGJKState::UNINITIALIZED) {
            preiteration();
            return;
        }
        if (current_state == SteppableGJKState::ITERATION_1) {
            iteration_substep_1();
            return;
        }
        if (current_state == SteppableGJKState::ITERATION_2) {
            iteration_substep_2();
            return;
        }
        if (current_state == SteppableGJKState::ITERATION_3) {
            iteration_substep_3();
            return;
        }
        if (current_state == SteppableGJKState::ITERATION_4) {
            iteration_substep_4();
            return;
        }
        if (current_state == SteppableGJKState::ITERATION_5) {
            iteration_substep_5();
            return;
        }

        if (iteration >= MAX_ITERATIONS) {
            step_by_step_gjk_finished = true;
            result = false;
        }
    }

    void SteppableGJK::evaluate() {
        // Perform the GJK algorithm without any step-by-step visualization
        step_by_step_init();
        while (!get_finished()) {
            iteration_step();
        }
    }

    /**
     * SteppableGJKEPA
     */

    void SteppableGJKEPA::iteration_step() {
        if (step_by_step_gjk_finished) {
            iteration_substep_epa();
            return;
        }
        SteppableGJK::iteration_step();
    }

    void SteppableGJKEPA::step_by_step_init() {
        SteppableGJK::step_by_step_init();
        step_by_step_epa_finished = false;
    }

    void SteppableGJKEPA::iteration_substep_epa() {
        step_by_step_epa_finished = true;

        if (!result) {
            // If the GJK algorithm didn't find a collision, we can't run the EPA algorithm
            collision_data = std::nullopt;
            return;
        }

        EPA epa(collider_1, collider_2, simplex);
        collision_data = epa.get_collision_data();
    }
}
