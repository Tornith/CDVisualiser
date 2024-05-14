#pragma once

#include <memory>
#include <utility>

#include "collider.hpp"
#include "collision_data.hpp"
#include "collision_detector.hpp"
#include "convex_polyhedron.hpp"
#include "voronoi.hpp"

namespace cdlib {
    enum class GJKState {
        INIT,
        CONTINUE,
        UPDATE_SIMPLEX,
        COLLISION,
        NO_COLLISION,
        EPA_FINISHED,
        ERROR
    };

    class GJK : public NarrowCollisionDetector {
    protected:
        GJKState state = GJKState::INIT;

        Simplex simplex{};
        glm::vec3 direction{};

    public:
        GJK() = default;

        GJK(ColliderP collider_1, ColliderP collider_2)
            : NarrowCollisionDetector(std::move(collider_1), std::move(collider_2)) {
        }

        GJK(const GJK& other) = default;

        GJK(GJK&& other) noexcept : NarrowCollisionDetector(std::move(other.collider_1), std::move(other.collider_2)) {}

        GJK& operator=(const GJK& other) {
            if (this == &other)
                return *this;
            collider_1 = other.collider_1;
            collider_2 = other.collider_2;
            return *this;
        }

        GJK& operator=(GJK&& other) noexcept {
            if (this == &other)
                return *this;
            collider_1 = std::move(other.collider_1);
            collider_2 = std::move(other.collider_2);
            return *this;
        }

        [[nodiscard]] GJKState get_state() const {
            return state;
        }

        ~GJK() override = default;

        [[nodiscard]] glm::vec3 get_support_point(const glm::vec3& direction) const;

        [[nodiscard]] const Simplex& get_simplex() const {
            return simplex;
        }

        virtual void set_simplex_indices(std::initializer_list<size_t> indices);
        virtual void insert_simplex_point(const glm::vec3& point);

        // States
        [[nodiscard]] virtual GJKState initialize();
        [[nodiscard]] virtual GJKState execute_iteration();

        [[nodiscard]] virtual GJKState step();

        virtual void reset();

        [[nodiscard]] bool passed_origin(const glm::vec3& support) const;

        [[nodiscard]] GJKState update_simplex();

        [[nodiscard]] virtual GJKState update_line_simplex();
        [[nodiscard]] virtual GJKState update_triangle_simplex();
        [[nodiscard]] virtual GJKState update_tetrahedron_simplex();

        [[nodiscard]] CollisionData get_collision_data() override;
        [[nodiscard]] virtual CollisionData calculate_collision_data();

        static CollisionData raycast(const Ray& ray, const ColliderP& collider);
    };

    class GJKEPA : public GJK {
    public:
        GJKEPA() = default;

        GJKEPA(const ColliderP& collider_1, const ColliderP& collider_2)
            : GJK(collider_1, collider_2)
        {}

        GJKEPA(const GJKEPA& other) = default;
        GJKEPA(GJKEPA&& other) noexcept = default;

        [[nodiscard]] CollisionData get_epa_data() const;

        [[nodiscard]] CollisionData calculate_collision_data() override;

        static CollisionData raycast(const Ray& ray, const ColliderP& collider);
    };

    class SteppableGJK : public GJK {
    protected:
        glm::vec3 current_point_a = glm::vec3(std::numeric_limits<float>::infinity());
        glm::vec3 current_point_b = glm::vec3(std::numeric_limits<float>::infinity());
        glm::vec3 current_new_point = glm::vec3(std::numeric_limits<float>::infinity());

        Simplex simplex_obj_1;
        Simplex simplex_obj_2;

        CollisionData result{};

    public:
        SteppableGJK() = default;
        SteppableGJK(ColliderP collider_1, ColliderP collider_2) : GJK(std::move(collider_1), std::move(collider_2)) {}

        [[nodiscard]] glm::vec3 get_current_point_a() const { return current_point_a; }
        [[nodiscard]] glm::vec3 get_current_point_b() const { return current_point_b; }
        [[nodiscard]] glm::vec3 get_current_new_point() const { return current_new_point; }

        [[nodiscard]] const Simplex& get_simplex_obj_1() const { return simplex_obj_1; }
        [[nodiscard]] const Simplex& get_simplex_obj_2() const { return simplex_obj_2; }

        [[nodiscard]] glm::vec3 get_direction() const { return direction; }

        [[nodiscard]] virtual bool get_finished() const { return state == GJKState::COLLISION || state == GJKState::NO_COLLISION; }
        [[nodiscard]] bool is_colliding() const { return state == GJKState::COLLISION; }

        [[nodiscard]] virtual GJKState get_current_state() const { return state; }
        [[nodiscard]] virtual CollisionData get_result() const { return result; }

        void set_simplex_indices(std::initializer_list<size_t> indices) override;
        void insert_simplex_point(const glm::vec3& point) override;

        void reset() override;

        [[nodiscard]] GJKState step() override;
        [[nodiscard]] GJKState get_next_point();
        [[nodiscard]] GJKState get_next_simplex();

        [[nodiscard]] GJKState initialize() override;
        [[nodiscard]] GJKState execute_iteration() override;

        [[nodiscard]] CollisionData calculate_collision_data() override;
    };

    class SteppableGJK2EPA : public SteppableGJK {
    public:
        SteppableGJK2EPA() = default;
        SteppableGJK2EPA(ColliderP collider_1, ColliderP collider_2) : SteppableGJK(std::move(collider_1), std::move(collider_2)) {}

        [[nodiscard]] GJKState step() override;
        [[nodiscard]] CollisionData get_epa_data() const;
        [[nodiscard]] CollisionData calculate_collision_data() override;
        [[nodiscard]] bool get_finished() const override { return state == GJKState::NO_COLLISION || state == GJKState::EPA_FINISHED; }
    };
}
