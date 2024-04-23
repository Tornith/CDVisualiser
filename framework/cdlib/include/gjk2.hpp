#pragma once

#include <memory>
#include <utility>

#include "collider.hpp"
#include "collision_data.hpp"
#include "collision_detector.hpp"
#include "convex_polyhedron.hpp"
#include "voronoi.hpp"

namespace cdlib {
    enum class GJK2State {
        INIT,
        CONTINUE,
        COLLISION,
        NO_COLLISION,
        EPA_FINISHED,
        ERROR
    };

    class GJK2 : public NarrowCollisionDetector {
    protected:
        GJK2State state = GJK2State::INIT;

        Simplex simplex{};
        glm::vec3 direction{};

    public:
        GJK2() = default;

        GJK2(std::shared_ptr<Collider> collider_1, std::shared_ptr<Collider> collider_2)
            : NarrowCollisionDetector(std::move(collider_1), std::move(collider_2)) {
        }

        GJK2(const GJK2& other) = default;

        GJK2(GJK2&& other) noexcept : NarrowCollisionDetector(std::move(other.collider_1), std::move(other.collider_2)) {}

        GJK2& operator=(const GJK2& other) {
            if (this == &other)
                return *this;
            collider_1 = other.collider_1;
            collider_2 = other.collider_2;
            return *this;
        }

        GJK2& operator=(GJK2&& other) noexcept {
            if (this == &other)
                return *this;
            collider_1 = std::move(other.collider_1);
            collider_2 = std::move(other.collider_2);
            return *this;
        }

        [[nodiscard]] GJK2State get_state() const {
            return state;
        }

        ~GJK2() override = default;

        [[nodiscard]] glm::vec3 get_support_point(const glm::vec3& direction) const;

        [[nodiscard]] const Simplex& get_simplex() const {
            return simplex;
        }

        virtual void set_simplex_indices(std::initializer_list<long long> indices);
        virtual void insert_simplex_point(const glm::vec3& point);

        // States
        [[nodiscard]] virtual GJK2State initialize();
        [[nodiscard]] virtual GJK2State execute_iteration();

        [[nodiscard]] virtual GJK2State step();

        virtual void reset();

        [[nodiscard]] bool passed_origin(const glm::vec3& support) const;

        [[nodiscard]] GJK2State update_simplex();

        [[nodiscard]] virtual GJK2State update_line_simplex();
        [[nodiscard]] virtual GJK2State update_triangle_simplex();
        [[nodiscard]] virtual GJK2State update_tetrahedron_simplex();

        [[nodiscard]] CollisionData get_collision_data() override;
        [[nodiscard]] virtual CollisionData calculate_collision_data();
    };

    class GJK2EPA : public GJK2 {
    public:
        GJK2EPA() = default;

        GJK2EPA(const std::shared_ptr<Collider>& collider_1, const std::shared_ptr<Collider>& collider_2)
            : GJK2(collider_1, collider_2)
        {}

        GJK2EPA(const GJK2EPA& other) = default;
        GJK2EPA(GJK2EPA&& other) noexcept = default;

        [[nodiscard]] CollisionData get_epa_data() const;

        [[nodiscard]] CollisionData calculate_collision_data() override;
    };

    class SteppableGJK2 : public GJK2 {
    protected:
        glm::vec3 current_point_a = glm::vec3(0.f);
        glm::vec3 current_point_b = glm::vec3(0.f);
        glm::vec3 current_new_point = glm::vec3(0.f);

        Simplex simplex_obj_1;
        Simplex simplex_obj_2;

        CollisionData result{};

    public:
        SteppableGJK2() = default;
        SteppableGJK2(std::shared_ptr<Collider> collider_1, std::shared_ptr<Collider> collider_2) : GJK2(std::move(collider_1), std::move(collider_2)) {}

        [[nodiscard]] glm::vec3 get_current_point_a() const { return current_point_a; }
        [[nodiscard]] glm::vec3 get_current_point_b() const { return current_point_b; }
        [[nodiscard]] glm::vec3 get_current_new_point() const { return current_new_point; }

        [[nodiscard]] const Simplex& get_simplex_obj_1() const { return simplex_obj_1; }
        [[nodiscard]] const Simplex& get_simplex_obj_2() const { return simplex_obj_2; }

        [[nodiscard]] glm::vec3 get_direction() const { return direction; }

        [[nodiscard]] virtual bool get_finished() const { return state == GJK2State::COLLISION || state == GJK2State::NO_COLLISION; }
        [[nodiscard]] bool is_colliding() const { return state == GJK2State::COLLISION; }

        [[nodiscard]] virtual GJK2State get_current_state() const { return state; }
        [[nodiscard]] virtual CollisionData get_result() const { return result; }

        void set_simplex_indices(std::initializer_list<long long> indices) override;
        void insert_simplex_point(const glm::vec3& point) override;

        void reset() override;

        [[nodiscard]] GJK2State step() override;

        [[nodiscard]] GJK2State initialize() override;
        [[nodiscard]] GJK2State execute_iteration() override;

        [[nodiscard]] CollisionData calculate_collision_data() override;
    };

    class SteppableGJK2EPA : public SteppableGJK2 {
    public:
        SteppableGJK2EPA() = default;
        SteppableGJK2EPA(std::shared_ptr<Collider> collider_1, std::shared_ptr<Collider> collider_2) : SteppableGJK2(std::move(collider_1), std::move(collider_2)) {}

        [[nodiscard]] GJK2State step() override;
        [[nodiscard]] CollisionData get_epa_data() const;
        [[nodiscard]] CollisionData calculate_collision_data() override;
        [[nodiscard]] bool get_finished() const override { return state == GJK2State::NO_COLLISION || state == GJK2State::EPA_FINISHED; }
    };
}
