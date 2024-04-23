#include "gjk2.hpp"
#include "epa.hpp"

namespace cdlib
{
    /********************
     * GJK
     ********************/

    glm::vec3 GJK2::get_support_point(const glm::vec3& direction) const
    {
        return collider_1->support(direction) - collider_2->support(-direction);
    }

    GJK2State GJK2::initialize()
    {
        // Check if the colliders are valid
        if (!collider_1->is_valid() && !collider_2->is_valid()){
            std::cerr << "GJK: Invalid colliders" << std::endl;
            return GJK2State::ERROR;
        }

        // Get an arbitrary direction
        direction = normalize(collider_1->get_global_vertex(0) - collider_2->get_global_vertex(0));
        auto support_point = get_support_point(direction);

        // Insert the support point into the simplex and look in the direction of the origin
        insert_simplex_point(support_point);
        direction = normalize(-support_point);
        return GJK2State::CONTINUE;
    }

    GJK2State GJK2::execute_iteration()
    {
        // Get a new support point
        const auto support_point = get_support_point(direction);

        if(!passed_origin(support_point))
        {
            return GJK2State::NO_COLLISION;
        }

        insert_simplex_point(support_point);

        return update_simplex();
    }

    GJK2State GJK2::step()
    {
        if (state != GJK2State::INIT && state != GJK2State::CONTINUE){
            return state;
        }
        if (state == GJK2State::INIT){
            return initialize();
        }
        return execute_iteration();
    }

    void GJK2::reset()
    {
        simplex.clear();
        direction = glm::vec3(0.0f);
        state = GJK2State::INIT;
    }

    CollisionData GJK2::calculate_collision_data() {
        return {state == GJK2State::COLLISION};
    }

    CollisionData GJK2::get_collision_data()
    {
        auto iteration = 0;
        while (state == GJK2State::CONTINUE || state == GJK2State::INIT){
            state = step();
            if (iteration++ > 10000){
                std::cerr << "GJK: Iteration limit reached" << std::endl;
                break;
            }
        }
        return calculate_collision_data();
    }

    bool GJK2::passed_origin(const glm::vec3& support) const
    {
        return dot(support, direction) > 0;
    }

    void GJK2::set_simplex_indices(std::initializer_list<long long> indices)
    {
        const auto old_simplex = simplex;
        simplex.clear();
        for (const auto index : indices){
            simplex.insert(old_simplex[index]);
        }
    }

    void GJK2::insert_simplex_point(const glm::vec3& point)
    {
        simplex.insert(point);
    }

    GJK2State GJK2::update_simplex()
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
                return GJK2State::ERROR;
        }
    }

    GJK2State GJK2::update_line_simplex()
    {
        const auto ab = simplex[1] - simplex[0];
        const auto ao = -simplex[0];

        // If the origin lies in a region between the two points (i.e. is closest to the line segment, not to the points)
        if (dot(ab, ao) > 0)
        {
            direction = cross(cross(ab, ao), ab);
        }

        // If the origin is closest to the first point
        else
        {
            // simplex = {simplex[0]};
            set_simplex_indices({0});
            direction = ao;
        }

        return GJK2State::CONTINUE;
    }

    GJK2State GJK2::update_triangle_simplex()
    {
        const auto ab = simplex[1] - simplex[0];
        const auto ac = simplex[2] - simplex[0];
        const auto ao = -simplex[0];
        const auto n = cross(ab, ac);  // Normal of the triangle

        // If the origin lies in the region closest to the line segment ac
        if (dot(cross(ac, n), ao) > 0)
        {
            // The origin is "above" the line segment ac
            if (dot(ac, ao) > 0)
            {
                //simplex = {simplex[0], simplex[2]};
                set_simplex_indices({0, 2});
                direction = cross(cross(ac, ao), ac);
            }
            // The origin is "below" the line segment ac -> check line segment ab
            else
            {
                //simplex = {simplex[0], simplex[1]};
                set_simplex_indices({0, 1});
                return update_line_simplex();
            }
        }
        // Otherwise, the origin must either lie near AB or above or below the triangle
        else
        {
            if (dot(cross(ab, n), ao) > 0)
            {
                //simplex = {simplex[0], simplex[1]};
                set_simplex_indices({0, 1});
                return update_line_simplex();
            }

            // The origin is above or below the triangle
            if (dot(n, ao) > 0)
            {
                direction = n;
            }
            else
            {
                //simplex = {simplex[0], simplex[2], simplex[1]};
                set_simplex_indices({0, 2, 1});
                direction = -n;
            }
        }
        return GJK2State::CONTINUE;
    }

    GJK2State GJK2::update_tetrahedron_simplex()
    {
        const auto ab = simplex[1] - simplex[0];
        const auto ac = simplex[2] - simplex[0];
        const auto ad = simplex[3] - simplex[0];
        const auto ao = -simplex[0];

        const auto abc = cross(ab, ac);
        const auto acd = cross(ac, ad);
        const auto adb = cross(ad, ab);

        // Check each face of the tetrahedron (apart from bcd, since a is closer to the origin than b, c, and d)
        if (dot(abc, ao) > 0)
        {
            // simplex = {simplex[0], simplex[1], simplex[2]};
            set_simplex_indices({0, 1, 2});
            return update_triangle_simplex();
        }
        if (dot(acd, ao) > 0)
        {
            // simplex = {simplex[0], simplex[2], simplex[3]};
            set_simplex_indices({0, 2, 3});
            return update_triangle_simplex();
        }
        if (dot(adb, ao) > 0)
        {
            // simplex = {simplex[0], simplex[3], simplex[1]};
            set_simplex_indices({0, 3, 1});
            return update_triangle_simplex();
        }

        // The origin is inside the tetrahedron
        return GJK2State::COLLISION;
    }

    /********************
     * GJKEPA
     ********************/

    CollisionData GJK2EPA::calculate_collision_data() {
        if (state == GJK2State::NO_COLLISION){
            return CollisionData{ false };
        }

        // If the objects are colliding, we can use the EPA algorithm to get the collision data
        return get_epa_data();
    }

    CollisionData GJK2EPA::get_epa_data() const
    {
        EPA epa(collider_1, collider_2, simplex);
        return epa.get_collision_data();
    }

    /********************
     * Steppable GJK
     ********************/

    CollisionData SteppableGJK2::calculate_collision_data()
    {
        result = GJK2::calculate_collision_data();
        return result;
    }

    GJK2State SteppableGJK2::step()
    {
        const auto next_state = GJK2::step();
        if (next_state == GJK2State::NO_COLLISION)
        {
            result = CollisionData{ false };
            return GJK2State::NO_COLLISION;
        }
        if (state == GJK2State::COLLISION){
            result = CollisionData{ true };
            return GJK2State::COLLISION;
        }
        return next_state;
    }

    GJK2State SteppableGJK2::initialize()
    {
        if (const auto base = GJK2::initialize(); base != GJK2State::CONTINUE){
            return base;
        }

        current_point_a = simplex_obj_1[0];
        current_point_b = simplex_obj_2[0];
        current_new_point = simplex[0];

        return GJK2State::CONTINUE;
    }

    GJK2State SteppableGJK2::execute_iteration()
    {
        // Get a new support point
        const auto support_point = get_support_point(direction);

        current_new_point = support_point;
        current_point_a = collider_1->support(direction);
        current_point_b = collider_2->support(-direction);

        if(!passed_origin(support_point))
        {
            return GJK2State::NO_COLLISION;
        }

        insert_simplex_point(support_point);

        return update_simplex();
    }

    void SteppableGJK2::set_simplex_indices(std::initializer_list<long long> indices)
    {
        const auto old_simplex = simplex;
        const auto old_simplex_obj_1 = simplex_obj_1;
        const auto old_simplex_obj_2 = simplex_obj_2;

        simplex.clear();
        simplex_obj_1.clear();
        simplex_obj_2.clear();

        for (const auto index : indices){
            simplex.insert(old_simplex[index]);
            simplex_obj_1.insert(old_simplex_obj_1[index]);
            simplex_obj_2.insert(old_simplex_obj_2[index]);
        }
    }

    void SteppableGJK2::insert_simplex_point(const glm::vec3& point)
    {
        GJK2::insert_simplex_point(point);
        simplex_obj_1.insert(collider_1->support(direction));
        simplex_obj_2.insert(collider_2->support(-direction));
    }

    void SteppableGJK2::reset()
    {
        GJK2::reset();
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

    GJK2State SteppableGJK2EPA::step()
    {
        if (state == GJK2State::COLLISION){
            result = get_epa_data();
            return GJK2State::EPA_FINISHED;
        }
        return SteppableGJK2::step();
    }

    CollisionData SteppableGJK2EPA::get_epa_data() const
    {
        EPA epa(collider_1, collider_2, simplex);
        return epa.get_collision_data();
    }

    CollisionData SteppableGJK2EPA::calculate_collision_data()
    {
        if (state == GJK2State::NO_COLLISION){
            result = CollisionData{ false };
            return result;
        }

        // If the objects are colliding, we can use the EPA algorithm to get the collision data
        result = get_epa_data();
        return result;
    }
}
