#include "gjk2.hpp"
#include "epa.hpp"

namespace cdlib
{
    /********************
     * GJK
     ********************/

    glm::vec3 GJK::get_support_point(const glm::vec3& direction) const
    {
        return collider_1->support(direction) - collider_2->support(-direction);
    }

    GJKState GJK::initialize()
    {
        // Check if the colliders are valid
        if (!collider_1->is_valid() && !collider_2->is_valid()){
            std::cerr << "GJK: Invalid colliders" << std::endl;
            return GJKState::ERROR;
        }

        // Get an arbitrary direction
        direction = glm::vec3(1.0f, 0.0f, 0.0f);
        auto support_point = get_support_point(direction);

        // Insert the support point into the simplex and look in the direction of the origin
        insert_simplex_point(support_point);
        direction = normalize(-support_point);
        return GJKState::CONTINUE;
    }

    GJKState GJK::execute_iteration()
    {
        // Get a new support point
        const auto support_point = get_support_point(direction);

        if(!passed_origin(support_point))
        {
            return GJKState::NO_COLLISION;
        }

        insert_simplex_point(support_point);

        return update_simplex();
    }

    GJKState GJK::step()
    {
        if (state == GJKState::ERROR){
            return GJKState::ERROR;
        }
        if (state == GJKState::INIT){
            return initialize();
        }
        return execute_iteration();
    }

    void GJK::reset()
    {
        simplex.clear();
        direction = glm::vec3(0.0f);
        state = GJKState::INIT;
    }

    CollisionData GJK::calculate_collision_data() {
        return {state == GJKState::COLLISION};
    }

    CollisionData GJK::get_collision_data()
    {
        auto iteration = 0;
        while (state == GJKState::CONTINUE || state == GJKState::UPDATE_SIMPLEX || state == GJKState::INIT){
            state = step();
            if (iteration++ > 10000){
                std::cerr << "GJK: Iteration limit reached" << std::endl;
                return {false};
            }
        }
        return calculate_collision_data();
    }

    bool GJK::passed_origin(const glm::vec3& support) const
    {
        return dot(support, direction) >= 0;
    }

    void GJK::set_simplex_indices(const std::initializer_list<size_t> indices)
    {
        simplex.reorder(indices);
    }

    void GJK::insert_simplex_point(const glm::vec3& point)
    {
        simplex.insert(point);
    }

    GJKState GJK::update_simplex()
    {
        switch (simplex.size()){
            case 2:
                return update_line_simplex();
            case 3:
                return update_triangle_simplex();
            case 4:
                return update_tetrahedron_simplex();
            default:
                std::cerr << "GJK: Invalid simplex size" << std::endl;
                return GJKState::ERROR;
        }
    }

    GJKState GJK::update_line_simplex()
    {
        const auto ab = simplex[1] - simplex[0];
        const auto ao = -simplex[0];

        // If the origin lies in a region between the two points (i.e. is closest to the line segment, not to the points)
        if (dot(ab, ao) > 0) {
            direction = cross(cross(ab, ao), ab);
        }

        // If the origin is closest to the first point
        else {
            set_simplex_indices({0});
            direction = ao;
        }

        return GJKState::CONTINUE;
    }

    GJKState GJK::update_triangle_simplex()
    {
        // Check the "projected" origin against the voronoi regions of a triangle
        // Since the newest point added was A, then origin cannot lie in direction of the BC edge.
        // The point can then either:
        // - Lie in the line voronoi region of AB or AC
        // - Lie in the vertex voronoi region of A
        // - Lie inside the triangle -> the point is above or below the triangle

        // Calculate the normal plane of the triangle
        const auto ab = simplex[1] - simplex[0];
        const auto ac = simplex[2] - simplex[0];
        const auto ao = -simplex[0];
        const auto n = normalize(cross(ab, ac));

        // Check the voronoi region of AC or A
        const auto acn = cross(n, ac);
        if (dot(acn, ao) > 0) {
            // Check the voronoi region of AC
            if (dot(ac, ao) > 0) {
                set_simplex_indices({0, 2});
                direction = cross(cross(ac, ao), ac); // Recalculate the normal of the line, pointing towards the origin
                return GJKState::CONTINUE;
            }
            // Otherwise we are in the vertex region of A -> return the line case for AB
            set_simplex_indices({0, 1});
            return update_line_simplex();
        }

        // Check the voronoi region of AB
        const auto abn = cross(ab, n);
        if (dot(abn, ao) > 0) {
            set_simplex_indices({0, 1});
            direction = cross(cross(ab, ao), ab); // Recalculate the normal of the line, pointing towards the origin
            return GJKState::CONTINUE;
        }

        // If both fail, we are inside the triangle -> Check if origin is above or below
        if (dot(n, ao) > 0) {
            direction = n;
        }
        else {
            set_simplex_indices({0, 2, 1});
            direction = -n;
        }

        return GJKState::CONTINUE;
    }

    GJKState GJK::update_tetrahedron_simplex()
    {
        const auto ab = simplex[1] - simplex[0];
        const auto ac = simplex[2] - simplex[0];
        const auto ad = simplex[3] - simplex[0];
        const auto ao = -simplex[0];

        const auto abc = cross(ab, ac);
        const auto acd = cross(ac, ad);
        const auto adb = cross(ad, ab);

        // Check each face of the tetrahedron (apart from bcd, since a is closer to the origin than b, c, and d)
        if (dot(abc, ao) > 0) {
            set_simplex_indices({0, 1, 2});
            return update_triangle_simplex();
        }
        if (dot(acd, ao) > 0) {
            set_simplex_indices({0, 2, 3});
            return update_triangle_simplex();
        }
        if (dot(adb, ao) > 0) {
            set_simplex_indices({0, 3, 1});
            return update_triangle_simplex();
        }

        // The origin is inside the tetrahedron
        return GJKState::COLLISION;
    }

    CollisionData GJK::raycast(const Ray& ray, const ColliderP& collider) {
        // Clip the ray against the AABB of the collider
        const auto aabb_raycast = collider->get_aabb().raycast(ray);
        if (!aabb_raycast.has_value()){
            return {false};
        }

        // We create a new ray with the clipped t values
        const auto clipped_ray = ray.clip(aabb_raycast.value().first, aabb_raycast.value().second);

        auto ray_collider = std::make_shared<RayCollider>(clipped_ray);
        auto gjk = GJK(collider, ray_collider);
        return gjk.get_collision_data();
    }

    /********************
     * GJKEPA
     ********************/

    CollisionData GJKEPA::calculate_collision_data() {
        if (state == GJKState::NO_COLLISION){
            return CollisionData{ false };
        }

        // If the objects are colliding, we can use the EPA algorithm to get the collision data
        return get_epa_data();
    }

    CollisionData GJKEPA::get_epa_data() const
    {
        EPA epa(collider_1, collider_2, simplex);
        return epa.get_collision_data();
    }

    CollisionData GJKEPA::raycast(const Ray& ray, const ColliderP& collider) {
        auto ray_collider = std::make_shared<RayCollider>(ray);
        auto gjk = GJKEPA(collider, ray_collider);
        return gjk.get_collision_data();
    }

    /********************
     * Steppable GJK
     ********************/

    CollisionData SteppableGJK::calculate_collision_data()
    {
        result = GJK::calculate_collision_data();
        return result;
    }

    GJKState SteppableGJK::step()
    {
        const auto next_state = GJK::step();
        if (next_state == GJKState::NO_COLLISION)
        {
            result = CollisionData{ false };
            state = GJKState::NO_COLLISION;
            return GJKState::NO_COLLISION;
        }
        if (state == GJKState::COLLISION){
            result = CollisionData{ true };
            state = GJKState::COLLISION;
            return GJKState::COLLISION;
        }

        state = next_state;
        return next_state;
    }

    GJKState SteppableGJK::initialize()
    {
        if (const auto base = GJK::initialize(); base != GJKState::CONTINUE){
            return base;
        }

        current_point_a = simplex_obj_1[0];
        current_point_b = simplex_obj_2[0];
        current_new_point = simplex[0];

        return GJKState::CONTINUE;
    }

    GJKState SteppableGJK::get_next_point() {
        const auto support_point = get_support_point(direction);

        current_new_point = support_point;
        current_point_a = collider_1->support(direction);
        current_point_b = collider_2->support(-direction);

        return GJKState::UPDATE_SIMPLEX;
    }

    GJKState SteppableGJK::get_next_simplex() {
        if(!passed_origin(current_new_point))
        {
            return GJKState::NO_COLLISION;
        }

        insert_simplex_point(current_new_point);

        const auto next_state = update_simplex();
        current_new_point = glm::vec3(std::numeric_limits<float>::infinity());
        current_point_a = glm::vec3(std::numeric_limits<float>::infinity());
        current_point_b = glm::vec3(std::numeric_limits<float>::infinity());
        return next_state;
    }

    GJKState SteppableGJK::execute_iteration()
    {
        if (state == GJKState::CONTINUE){
            return get_next_point();
        }
        if (state == GJKState::UPDATE_SIMPLEX){
            return get_next_simplex();
        }
        return GJKState::ERROR;
    }

    void SteppableGJK::set_simplex_indices(const std::initializer_list<size_t> indices)
    {
        GJK::set_simplex_indices(indices);
        simplex_obj_1.reorder(indices);
        simplex_obj_2.reorder(indices);
    }

    void SteppableGJK::insert_simplex_point(const glm::vec3& point)
    {
        GJK::insert_simplex_point(point);
        simplex_obj_1.insert(collider_1->support(direction));
        simplex_obj_2.insert(collider_2->support(-direction));
    }

    void SteppableGJK::reset()
    {
        GJK::reset();
        current_point_a = glm::vec3(0.f);
        current_point_b = glm::vec3(0.f);
        current_new_point = glm::vec3(0.f);

        simplex_obj_1.clear();
        simplex_obj_2.clear();

        result = CollisionData{ false };
    }

    /********************
     * Steppable GJKEPA
     ********************/

    GJKState SteppableGJK2EPA::step()
    {
        if (state == GJKState::COLLISION){
            result = get_epa_data();
            state = GJKState::EPA_FINISHED;
            return GJKState::EPA_FINISHED;
        }
        return SteppableGJK::step();
    }

    CollisionData SteppableGJK2EPA::get_epa_data() const
    {
        EPA epa(collider_1, collider_2, simplex);
        return epa.get_collision_data();
    }

    CollisionData SteppableGJK2EPA::calculate_collision_data()
    {
        if (state == GJKState::NO_COLLISION){
            result = CollisionData{ false };
            return result;
        }

        // If the objects are colliding, we can use the EPA algorithm to get the collision data
        result = get_epa_data();
        return result;
    }
}
