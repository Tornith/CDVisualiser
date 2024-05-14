#pragma once
#include <functional>
#include <unordered_map>

#include "collision_detector.hpp"
#include "ray.hpp"

namespace cdlib {
    class AABBTree final : public BroadCollisionDetector {
        struct Node {
            AABB aabb;

            std::shared_ptr<Node> left;
            std::shared_ptr<Node> right;
            std::shared_ptr<Node> parent;

            ColliderP collider; // Only leaf nodes have colliders

            bool dirty;

            [[nodiscard]] bool is_leaf() const {
                return left == nullptr && right == nullptr;
            }
        };

        struct NodePair {
            std::shared_ptr<Node> node_1;
            std::shared_ptr<Node> node_2;

            struct Hash {
                [[nodiscard]] size_t operator()(const NodePair& pair) const {
                    return std::hash<std::shared_ptr<Node>>()(pair.node_1) + std::hash<std::shared_ptr<Node>>()(pair.node_2) +
                    std::hash<std::shared_ptr<Node>>()(pair.node_1) ^ std::hash<std::shared_ptr<Node>>()(pair.node_2);
                }
            };

            [[nodiscard]] bool operator==(const NodePair& other) const {
                return node_1 == other.node_1 && node_2 == other.node_2 ||
                       node_1 == other.node_2 && node_2 == other.node_1;
            }
        };

        struct AABBTreeRaycast {
            glm::vec3 from;
            glm::vec3 to;
            float max_fraction;
        };

        using NodePairSet = std::unordered_set<NodePair, NodePair::Hash>;

        using Callback = std::function<void(const std::shared_ptr<Node>&)>;
        using QueryCallback = std::function<bool(const std::shared_ptr<Node>&)>;
        using RayCallback = std::function<float(const AABBTreeRaycast&, ColliderP)>;

        const glm::vec3 fatten_margin = {0.0f, 0.0f, 0.0};
        const float displacement_multiplier = 3.0f;

        std::shared_ptr<Node> root;
        std::unordered_map<ColliderP, std::shared_ptr<Node>> collider_map;

        std::set<std::shared_ptr<Node>> move_buffer;

        std::shared_ptr<Node> test_node;

        CollisionSet collisions;

    public:
        AABBTree() = default;
        explicit AABBTree(const std::set<ColliderP>& colliders);

        AABBTree(const AABBTree& other) = default;
        AABBTree(AABBTree&& other) noexcept = default;

        AABBTree& operator=(const AABBTree& other) {
            if (this == &other)
                return *this;
            BroadCollisionDetector::operator=(other);
            root = other.root;
            collider_map = other.collider_map;
            move_buffer = other.move_buffer;
            test_node = other.test_node;
            collisions = other.collisions;
            return *this;
        }

        AABBTree& operator=(AABBTree&& other) noexcept {
            if (this == &other)
                return *this;
            BroadCollisionDetector::operator=(other);
            root = std::move(other.root);
            collider_map = std::move(other.collider_map);
            move_buffer = std::move(other.move_buffer);
            test_node = std::move(other.test_node);
            collisions = std::move(other.collisions);
            return *this;
        }

        void insert(const ColliderP& collider) override;
        void remove(const ColliderP& collider) override;
        void update(const ColliderP& collider);

        [[nodiscard]] CollisionSet get_collisions() override;

        [[nodiscard]] std::vector<std::shared_ptr<Node>> get_nodes();

        void query_point(const glm::vec3& point, const QueryCallback& callback);
        void query_aabb(const AABB& aabb, const QueryCallback& callback);
        void query_ray(const AABBTreeRaycast& input, const RayCallback& callback);

        [[nodiscard]] std::set<ColliderP> raycast(const Ray& ray) override;

    private:
        std::shared_ptr<Node> create_node(const ColliderP& collider);
        void remove_node(const std::shared_ptr<Node>& node);
        bool move_node(const std::shared_ptr<Node>& node, AABB new_aabb);

        std::shared_ptr<Node> insert_leaf(const std::shared_ptr<Node>& leaf);
        void remove_leaf(const std::shared_ptr<Node>& leaf);
        void rotate_node(const std::shared_ptr<Node>& node);
        void swap_nodes(const std::shared_ptr<Node>& node_1, const std::shared_ptr<Node>& node_2);

        void traverse(const Callback& callback);

        [[nodiscard]] float calculate_cost();

        void rebuild_tree();

        bool update_collisions(const std::shared_ptr<Node>& node);
        void clear_collisions(const ColliderP& collider = nullptr);
    };
}
