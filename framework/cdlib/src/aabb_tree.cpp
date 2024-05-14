#include "aabb_tree.hpp"

#include <stack>

namespace cdlib {
    AABBTree::AABBTree(const std::set<ColliderP>& colliders) {
        for (const auto& collider : colliders) {
            insert(collider);
        }
    }

    std::shared_ptr<AABBTree::Node> AABBTree::create_node(const ColliderP& collider) {
        auto node = std::make_shared<Node>();
        node->collider = collider;
        node->dirty = true;

        node->aabb = collider->get_aabb().fatten(fatten_margin);

        collider_map[collider] = node;

        insert_leaf(node);
        return node;
    }

    std::shared_ptr<AABBTree::Node> AABBTree::insert_leaf(const std::shared_ptr<Node>& leaf) {
        if (root == nullptr) {
            root = leaf;
            return root;
        }

        auto best_sibling = root;
        auto best_cost = leaf->aabb.merge(root->aabb).get_surface_area();

        auto stack = std::stack<std::pair<std::shared_ptr<Node>, float>>({{root, 0.0f}});
        while (!stack.empty()) {
            auto [current, inherited_cost] = stack.top();
            stack.pop();

            auto merged = current->aabb.merge(leaf->aabb);
            auto cost = merged.get_surface_area();

            auto total_cost = cost + inherited_cost;
            if (total_cost < best_cost) {
                best_cost = total_cost;
                best_sibling = current;
            }

            inherited_cost += cost - current->aabb.get_surface_area();

            float lower_bound = leaf->aabb.get_surface_area() + inherited_cost;
            if (lower_bound < best_cost) {
                if (!current->is_leaf()) {
                    stack.emplace(current->left, inherited_cost);
                    stack.emplace(current->right, inherited_cost);
                }
            }
        }

        // Create new parent
        const auto old_parent = best_sibling->parent;
        const auto new_parent = std::make_shared<Node>();
        new_parent->parent = old_parent;
        new_parent->aabb = best_sibling->aabb.merge(leaf->aabb);

        // Attach siblings to new parent
        new_parent->left = leaf;
        new_parent->right = best_sibling;
        leaf->parent = new_parent;
        best_sibling->parent = new_parent;

        if (old_parent == nullptr) {
            root = new_parent;
        }
        else {
            if (old_parent->left == best_sibling) {
                old_parent->left = new_parent;
            }
            else {
                old_parent->right = new_parent;
            }
        }

        // Update ancestors's AABB
        auto ancestor = new_parent;
        while (ancestor != nullptr) {
            const auto left = ancestor->left;
            const auto right = ancestor->right;

            ancestor->aabb = left->aabb.merge(right->aabb);

            rotate_node(ancestor);

            ancestor = ancestor->parent;
        }

        return leaf;
    }

    void AABBTree::remove_node(const std::shared_ptr<Node>& node) {
        remove_leaf(node);
        node->parent = nullptr;
        node->left = nullptr;
        node->right = nullptr;
    }

    void AABBTree::remove_leaf(const std::shared_ptr<Node>& leaf) {
        auto parent = leaf->parent;
        if (parent == nullptr) {
            root = nullptr;
            return;
        }

        auto grand_parent = parent->parent;
        auto sibling = parent->left == leaf ? parent->right : parent->left;

        // Remove parent
        parent->parent = nullptr;
        parent->left = nullptr;
        parent->right = nullptr;

        if (grand_parent == nullptr) {
            root = sibling;
            sibling->parent = nullptr;
            return;
        }

        sibling->parent = grand_parent;

        if (grand_parent->left == parent) {
            grand_parent->left = sibling;
        }
        else {
            grand_parent->right = sibling;
        }

        // Walk up the tree
        auto ancestor = grand_parent;
        while (ancestor != nullptr) {
            const auto left = ancestor->left;
            const auto right = ancestor->right;

            ancestor->aabb = left->aabb.merge(right->aabb);

            rotate_node(ancestor);

            ancestor = ancestor->parent;
        }
    }

    void AABBTree::rotate_node(const std::shared_ptr<Node>& node) {
        if (node->is_leaf()) {
            return;
        }

        auto left = node->left;
        auto right = node->right;

        float costs[4] = { 0.0f };

        if (!left->is_leaf()) {
            const auto area = left->aabb.get_surface_area();
            costs[0] = left->left->aabb.merge(right->aabb).get_surface_area() - area;
            costs[1] = left->right->aabb.merge(right->aabb).get_surface_area() - area;
        }

        if (!right->is_leaf()) {
            const auto area = right->aabb.get_surface_area();
            costs[2] = right->left->aabb.merge(left->aabb).get_surface_area() - area;
            costs[3] = right->right->aabb.merge(left->aabb).get_surface_area() - area;
        }

        int best_cost = 0;
        for (int i = 1; i < 4; ++i) {
            if (costs[i] < costs[best_cost]) {
                best_cost = i;
            }
        }

        // If the costs increases with the rotation, stop
        if (costs[best_cost] >= 0.0f) {
            return;
        }

        // Rotation cases
        switch (best_cost) {
            case 0: {
                left->right->parent = node;
                node->right = left->right;
                left->right = right;
                right->parent = left;

                left->aabb = left->left->aabb.merge(left->right->aabb);
            } break;
            case 1: {
                left->left->parent = node;
                node->right = left->left;
                left->left = right;
                right->parent = left;

                left->aabb = left->left->aabb.merge(left->right->aabb);
            } break;
            case 2: {
                right->right->parent = node;
                node->left = right->right;
                right->right = left;
                left->parent = right;

                right->aabb = right->left->aabb.merge(right->right->aabb);
            } break;
            case 3: {
                right->left->parent = node;
                node->left = right->left;
                right->left = left;
                left->parent = right;

                right->aabb = right->left->aabb.merge(right->right->aabb);
            } break;
        }
    }

    void AABBTree::swap_nodes(const std::shared_ptr<Node>& node_1, const std::shared_ptr<Node>& node_2) {
        const auto parent_1 = node_1->parent;
        const auto parent_2 = node_2->parent;

        if (parent_1 == parent_2) {
            parent_1->left = node_2;
            parent_2->right = node_1;
            return;
        }

        if (parent_1->left == node_1) {
            parent_1->left = node_2;
        }
        else {
            parent_1->right = node_2;
        }
        node_2->parent = parent_1;

        if (parent_2->left == node_2) {
            parent_2->left = node_1;
        }
        else {
            parent_2->right = node_1;
        }
        node_1->parent = parent_2;
    }

    bool AABBTree::move_node(const std::shared_ptr<Node>& node, AABB new_aabb) {
        if (node->aabb.contains(new_aabb)) {
            return false;
        }

        new_aabb = new_aabb.fatten(fatten_margin);

        remove_leaf(node);
        node->aabb = new_aabb;
        insert_leaf(node);
        node->dirty = true;

        return true;
    }

    void AABBTree::query_point(const glm::vec3& point, const QueryCallback& callback) {
        if (root == nullptr) {
            return;
        }

        auto stack = std::stack<std::shared_ptr<Node>>({root});
        while (!stack.empty()) {
            auto current = stack.top();
            stack.pop();

            if (current->aabb.contains(point)) {
                continue;
            }

            if (current->is_leaf()) {
                bool should_continue = callback(current);
                if (!should_continue) {
                    return;
                }
            }
            else {
                stack.push(current->left);
                stack.push(current->right);
            }
        }
    }

    void AABBTree::query_aabb(const AABB& aabb, const QueryCallback& callback) {
        if (root == nullptr) {
            return;
        }

        auto stack = std::stack<std::shared_ptr<Node>>({root});

        while (!stack.empty()) {
            auto current = stack.top();
            stack.pop();

            if (!current->aabb.intersects(aabb)) {
                continue;
            }

            if (current->is_leaf()) {
                bool should_continue = callback(current);
                if (!should_continue) {
                    return;
                }
            }
            else {
                stack.push(current->left);
                stack.push(current->right);
            }
        }
    }

    void AABBTree::query_ray(const AABBTreeRaycast& input, const RayCallback& callback){
        const auto from = input.from;
        const auto to = input.to;
        auto max_fraction = input.max_fraction;

        if (length(to - from) < glm::epsilon<float>()) {
            return;
        }

        auto stack = std::stack<std::shared_ptr<Node>>({root});
        while (!stack.empty()) {
            auto current = stack.top();
            stack.pop();

            if (current->is_leaf()) {
                AABBTreeRaycast sub_raycast = input;
                sub_raycast.max_fraction = max_fraction;

                auto fraction = callback(sub_raycast, current->collider);
                if (fraction == 0.0f) {
                    return;
                }
                if (fraction > 0.0f) {
                    max_fraction = fraction;
                }
            }
            else {
                std::shared_ptr<Node> left = current->left;
                std::shared_ptr<Node> right = current->right;

                auto left_distance = left->aabb.raycast(from, to, 0.0f, max_fraction);
                auto right_distance = right->aabb.raycast(from, to, 0.0f, max_fraction);

                if (right_distance.t_min < left_distance.t_min) {
                    std::swap(left, right);
                    std::swap(left_distance, right_distance);
                }

                if (!left_distance.hit) {
                    continue;
                }

                if (right_distance.hit) {
                    stack.push(right);
                }
                stack.push(left);
            }
        }
    }

    void AABBTree::traverse(const Callback& callback) {
        auto stack = std::stack<std::shared_ptr<Node>>({root});
        while (!stack.empty()) {
            auto current = stack.top();
            stack.pop();

            if (!current->is_leaf()) {
                stack.push(current->left);
                stack.push(current->right);
            }

            callback(current);
        }
    }

    float AABBTree::calculate_cost() {
        float cost = 0.0f;

        traverse([&cost](const std::shared_ptr<Node>& node) {
            cost += node->aabb.get_surface_area();
        });

        return cost;
    }

    std::vector<std::shared_ptr<AABBTree::Node>> AABBTree::get_nodes() {
        std::vector<std::shared_ptr<Node>> nodes;
        traverse([&nodes](const std::shared_ptr<Node>& node) {
            nodes.push_back(node);
        });
        return nodes;
    }

    void AABBTree::rebuild_tree() {
        // Bottom-up tree rebuild
        std::vector<std::shared_ptr<Node>> leaves;
        traverse([&leaves](const std::shared_ptr<Node>& node) {
            if (node->is_leaf()) {
                leaves.push_back(node);
            }
        });

        // Unlink all nodes
        traverse([](const std::shared_ptr<Node>& node) {
            node->parent = nullptr;
            node->left = nullptr;
            node->right = nullptr;
        });

        while (leaves.size() > 1) {
            auto min_cost = std::numeric_limits<float>::max();
            std::shared_ptr<Node> best_leaf_1 = nullptr;
            std::shared_ptr<Node> best_leaf_2 = nullptr;

            // Find the best pair of leaves to merge
            for (size_t i = 0; i < leaves.size(); ++i) {
                for (size_t j = i + 1; j < leaves.size(); ++j) {
                    const auto leaf_1 = leaves[i];
                    const auto leaf_2 = leaves[j];

                    if (const auto cost = leaf_1->aabb.merge(leaf_2->aabb).get_surface_area(); cost < min_cost) {
                        min_cost = cost;
                        best_leaf_1 = leaf_1;
                        best_leaf_2 = leaf_2;
                    }
                }
            }

            // Merge the best pair of leaves
            const auto parent = std::make_shared<Node>();
            parent->left = best_leaf_1;
            parent->right = best_leaf_2;
            parent->aabb = best_leaf_1->aabb.merge(best_leaf_2->aabb);

            best_leaf_1->parent = parent;
            best_leaf_2->parent = parent;

            // Remove the merged leaves
            leaves.erase(std::remove(leaves.begin(), leaves.end(), best_leaf_1), leaves.end());
            leaves.erase(std::remove(leaves.begin(), leaves.end(), best_leaf_2), leaves.end());

            // Add the parent to the list
            leaves.push_back(parent);
        }

        root = leaves.front();
    }

    CollisionSet AABBTree::get_collisions() {
        // Look through the move buffer
        for (const auto& node : move_buffer) {
            if (node == nullptr) {
                continue;
            }
            test_node = node;

            query_aabb(test_node->aabb, [this](const std::shared_ptr<Node>& other) {
                return update_collisions(other);
            });

            node->dirty = false;
        }

        move_buffer.clear();

        return collisions;
    }

    void AABBTree::clear_collisions(const ColliderP& collider) {
        if (collider == nullptr) {
            collisions.clear();
            return;
        }

        // Clear all collisions involving the collider
        std::erase_if(collisions, [collider](const CollisionPair& pair) {
            return pair.contains(collider);
        });
    }

    bool AABBTree::update_collisions(const std::shared_ptr<Node>& node) {
        if (node == test_node || !node->dirty || !test_node->aabb.intersects(node->aabb)) {
            return true;
        }

        if (node->is_leaf() && test_node->is_leaf()) {
            if (node->collider->get_aabb().intersects(test_node->collider->get_aabb())) {
                collisions.insert({node->collider, test_node->collider});
            }
        }

        return true;
    }

    void AABBTree::insert(const ColliderP& collider) {
        if (collider_map.contains(collider)) {
            return;
        }

        const auto node = create_node(collider);
        move_buffer.insert(node);
    }

    void AABBTree::remove(const ColliderP& collider) {
        if (!collider_map.contains(collider)) {
            return;
        }

        const auto node = collider_map[collider];
        remove_node(node);
        collider_map.erase(collider);

        move_buffer.erase(node);
    }

    void AABBTree::update(const ColliderP& collider) {
        if (!collider_map.contains(collider)) {
            return;
        }

        const auto node = collider_map[collider];

        // Clear all collisions involving the collider
        clear_collisions(collider);

        if (move_node(node, collider->get_aabb())) {
            move_buffer.insert(node);
        }
    }

    RayCastResultSet AABBTree::raycast(const Ray& ray) {
        const auto input = AABBTreeRaycast{ ray.from(), ray.to(), 1.0f};
        RayCastResultSet colliders;

        query_ray(input, [&colliders, ray](const AABBTreeRaycast& raycast, const ColliderP& collider) {
            const auto point = ray.at(raycast.max_fraction);
            colliders.insert({
                true,
                collider,
                point,
                glm::vec3(0.0f),
                distance(ray.from(), point)
            });
            return raycast.max_fraction;
        });

        return colliders;
    }
} // namespace cdlib
