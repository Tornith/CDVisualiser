#pragma once
#include <memory>
#include <utility>
#include <vector>
#include <array>

#include "collider.hpp"
#include "collision_data.hpp"

namespace cdlib {
    constexpr float EPA_EPSILON = 1e-6f;

    class EPA {
    protected:
        Simplex simplex;
        std::shared_ptr<Collider> collider_1{};
        std::shared_ptr<Collider> collider_2{};

        std::vector<glm::vec3> polytope;
        std::vector<std::array<size_t, 3>> faces;

    public:
        EPA() = default;

        EPA(std::shared_ptr<Collider> collider_1, std::shared_ptr<Collider> collider_2, const Simplex& simplex)
            : simplex(simplex), collider_1(std::move(collider_1)), collider_2(std::move(collider_2)) {
            polytope = std::vector(simplex.begin(), simplex.end());
            faces = {
                {0, 1, 2},
                {0, 3, 1},
                {0, 2, 3},
                {1, 3, 2}
            };
        }

        virtual ~EPA() = default;

        EPA(const EPA& other) = default;

        EPA(EPA&& other) noexcept
            : simplex(other.simplex),
              collider_1(std::move(other.collider_1)),
              collider_2(std::move(other.collider_2)),
              polytope(std::move(other.polytope)),
              faces(std::move(other.faces)) {
        }

        EPA& operator=(const EPA& other) {
            if (this == &other)
                return *this;
            simplex = other.simplex;
            collider_1 = other.collider_1;
            collider_2 = other.collider_2;
            polytope = other.polytope;
            faces = other.faces;
            return *this;
        }

        EPA& operator=(EPA&& other) noexcept {
            if (this == &other)
                return *this;
            simplex = other.simplex;
            collider_1 = other.collider_1;
            collider_2 = other.collider_2;
            polytope = std::move(other.polytope);
            faces = std::move(other.faces);
            return *this;
        }

        [[nodiscard]] virtual CollisionData get_collision_data();
    };
}
