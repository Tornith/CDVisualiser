#pragma once

#include <memory>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>
#include <glm/glm.hpp>
#include <glm/gtx/hash.hpp>

#include "collider.hpp"
#include "collision_data.hpp"
#include "collision_detector.hpp"

namespace cdlib {
    class GJK : public NarrowCollisionDetector {
    protected:
        std::vector<glm::vec3> simplex;
        glm::vec3 direction{};
    public:
        GJK() = default;

        GJK(std::shared_ptr<Collider> collider_1, std::shared_ptr<Collider> collider_2) : NarrowCollisionDetector(std::move(collider_1), std::move(collider_2)) {}

        GJK(const GJK& other) = default;

        GJK(GJK&& other) noexcept
            : NarrowCollisionDetector(other.collider_1, other.collider_2),
              simplex(std::move(other.simplex)),
              direction(other.direction)
            {
        }

        GJK& operator=(const GJK& other) {
            if (this == &other)
                return *this;
            simplex = other.simplex;
            direction = other.direction;
            collider_1 = other.collider_1;
            collider_2 = other.collider_2;
            return *this;
        }

        GJK& operator=(GJK&& other) noexcept {
            if (this == &other)
                return *this;
            simplex = std::move(other.simplex);
            direction = other.direction;
            collider_1 = other.collider_1;
            collider_2 = other.collider_2;
            return *this;
        }

        ~GJK() override = default;

        virtual bool is_colliding();

        [[nodiscard]] virtual const std::vector<glm::vec3>& get_simplex() const { return simplex; }

        static glm::vec3 get_line_normal(const glm::vec3& a, const glm::vec3& b, const glm::vec3& o = glm::vec3(0.f));
        static glm::vec3 get_face_normal(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, const glm::vec3& o = glm::vec3(0.f));

        static float line_point_distance(const glm::vec3& a, const glm::vec3& b, const glm::vec3& o);
        static float triangle_point_distance(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, const glm::vec3& o);

        static bool passed_origin(const glm::vec3& normal, const glm::vec3& point);

        // First is the direction, second is the dropped point index
        [[nodiscard]] std::pair<glm::vec3, std::optional<long long>> next_direction() const;

        bool origin_in_simplex();

        [[nodiscard]] CollisionData get_collision_data() override {
            return CollisionData{is_colliding()};
        }
    public:
        constexpr static int MAX_ITERATIONS = 10000;
    };

    class GJKEPA : public GJK {
    public:
        GJKEPA() = default;

        GJKEPA(const std::shared_ptr<Collider>& collider_1, const std::shared_ptr<Collider>& collider_2) : GJK(collider_1, collider_2) {}

        explicit GJKEPA(const GJK& other) : GJK(other) {}

        explicit GJKEPA(GJK&& other) : GJK(other) {}

        [[nodiscard]] CollisionData get_collision_data() override;
    };

    // Enum, specifying the state of the steppable GJK
    enum class SteppableGJKState {
        UNINITIALIZED,
        ITERATION_1,
        ITERATION_2,
        ITERATION_3,
        ITERATION_4,
        ITERATION_5,
        EPA,
        FINISHED,
        UNDEFINED
    };

    class SteppableGJK : public GJK {
    protected:
        bool step_by_step_initialized = false;
        bool step_by_step_gjk_finished = false;

        SteppableGJKState current_state = SteppableGJKState::UNINITIALIZED;
        int iteration = 0;

        glm::vec3 current_point_a = glm::vec3(0.f);
        glm::vec3 current_point_b = glm::vec3(0.f);
        glm::vec3 current_new_point = glm::vec3(0.f);

        bool result = false;

        std::vector<glm::vec3> simplex_obj_1;
        std::vector<glm::vec3> simplex_obj_2;

        std::unordered_map<glm::vec3, std::pair<glm::vec3, glm::vec3>> simplex_map;

        void preiteration();

        void iteration_substep_1();
        void iteration_substep_2();
        void iteration_substep_3();
        void iteration_substep_4();
        void iteration_substep_5();

    public:
        SteppableGJK() = default;
        SteppableGJK(std::shared_ptr<Collider> collider_1, std::shared_ptr<Collider> collider_2) : GJK(std::move(collider_1), std::move(collider_2)) {}

        virtual void iteration_step();

        virtual void step_by_step_init();
        virtual SteppableGJKState step_by_step_next();

        void evaluate();

        // Getters
        [[nodiscard]] bool is_step_by_step_initialized() const { return step_by_step_initialized; }
        [[nodiscard]] bool is_step_by_step_finished() const { return step_by_step_gjk_finished; }
        [[nodiscard]] int current_iteration() const { return iteration; }
        [[nodiscard]] glm::vec3 get_current_point_a() const { return current_point_a; }
        [[nodiscard]] glm::vec3 get_current_point_b() const { return current_point_b; }
        [[nodiscard]] glm::vec3 get_current_new_point() const { return current_new_point; }
        [[nodiscard]] bool current_result() const { return result; }

        [[nodiscard]] const std::vector<glm::vec3>& get_simplex_obj_1() const { return simplex_obj_1; }
        [[nodiscard]] const std::vector<glm::vec3>& get_simplex_obj_2() const { return simplex_obj_2; }

        [[nodiscard]] glm::vec3 get_direction() const { return direction; }

        [[nodiscard]] virtual bool get_finished() const { return step_by_step_gjk_finished; }
        [[nodiscard]] bool is_colliding() override { return result; }

        [[nodiscard]] virtual SteppableGJKState get_current_state() const { return current_state; }

        [[nodiscard]] CollisionData get_collision_data() override {
            return CollisionData{result};
        }
    };

    class SteppableGJKEPA : public SteppableGJK {
    protected:
        bool step_by_step_epa_finished = false;
        CollisionData collision_data;

        void iteration_substep_epa();

    public:
        SteppableGJKEPA() = default;
        SteppableGJKEPA(std::shared_ptr<Collider> collider_1, std::shared_ptr<Collider> collider_2) : SteppableGJK(std::move(collider_1), std::move(collider_2)) {}

        void iteration_step() override;

        void step_by_step_init() override;

        [[nodiscard]] bool get_gjk_finished() const { return step_by_step_gjk_finished; }
        [[nodiscard]] bool get_epa_finished() const { return step_by_step_epa_finished; }
        [[nodiscard]] bool get_finished() const override { return step_by_step_gjk_finished && step_by_step_epa_finished; }
        [[nodiscard]] CollisionData get_collision_data() override { return collision_data; }
    };
}