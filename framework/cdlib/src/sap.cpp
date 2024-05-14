#include "sap.hpp"

#include <algorithm>
#include <iostream>

namespace cdlib {
    void SAP::create_endpoints(const ColliderP& collider) {
        const auto [min, max] = collider->get_aabb();
        const auto max_endpoint = std::make_shared<Endpoint>(
            collider,
            std::array<std::shared_ptr<Endpoint>, 3>{nullptr, nullptr, nullptr},
            std::array<std::shared_ptr<Endpoint>, 3>{nullptr, nullptr, nullptr},
            max, false
        );
        const auto min_endpoint = std::make_shared<Endpoint>(
            collider,
            std::array<std::shared_ptr<Endpoint>, 3>{nullptr, nullptr, nullptr},
            std::array<std::shared_ptr<Endpoint>, 3>{nullptr, nullptr, nullptr},
            min, true
        );

        // Set the other endpoint for each endpoint
        min_endpoint->other_endpoint = max_endpoint;
        max_endpoint->other_endpoint = min_endpoint;

        // Add the endpoints to the map
        endpoint_map[collider] = {min_endpoint, max_endpoint};
    }

    void SAP::insert(const ColliderP& collider) {
        // Retrieve the endpoints from the map, if they exist
        EndpointP min_endpoint = nullptr, max_endpoint = nullptr;
        if (!endpoint_map.contains(collider)) {
            create_endpoints(collider);
        }
        std::tie(min_endpoint, max_endpoint) = endpoint_map[collider];

        // If the list is empty, insert the min endpoint at the beginning
        if (list_heads[0] == nullptr) {
            for (int axis = 0; axis < 3; axis++) {
                list_heads[axis] = min_endpoint;
                min_endpoint->next[axis] = max_endpoint;
                max_endpoint->prev[axis] = min_endpoint;
            }
        }
        else {
            for (int axis = 0; axis < 3; axis++) {
                // Find the correct position to insert the min endpoint
                auto current = list_heads[axis];
                EndpointP prev = nullptr;

                while (current != nullptr && current->value[axis] < min_endpoint->value[axis]) {
                    prev = current;
                    current = current->next[axis];
                }

                // Insert the min endpoint
                // If the current endpoint is nullptr, we are at the end of the list
                if (current == nullptr) {
                    prev->next[axis] = min_endpoint;
                    min_endpoint->prev[axis] = prev;
                } else {
                    min_endpoint->prev[axis] = current->prev[axis];
                    min_endpoint->next[axis] = current;
                    if (current->prev[axis] != nullptr) {
                        current->prev[axis]->next[axis] = min_endpoint;
                    } else {
                        list_heads[axis] = min_endpoint;
                    }
                    current->prev[axis] = min_endpoint;
                }

                // Find the correct position to insert the max endpoint
                current = list_heads[axis];
                prev = nullptr;
                while (current != nullptr && current->value[axis] < max_endpoint->value[axis]) {
                    // Check for potential overlap
                    if (current->is_min && current->collider != collider && collider->get_aabb().intersects(current->get_aabb())) {
                        collisions.insert({collider, current->collider});
                    }

                    prev = current;
                    current = current->next[axis];
                }

                // Insert the max endpoint
                // If the current endpoint is nullptr, we are at the end of the list
                if (current == nullptr) {
                    prev->next[axis] = max_endpoint;
                    max_endpoint->prev[axis] = prev;
                } else {
                    max_endpoint->prev[axis] = current->prev[axis];
                    max_endpoint->next[axis] = current;
                    if (current->prev[axis] != nullptr) {
                        current->prev[axis]->next[axis] = max_endpoint;
                    }
                    current->prev[axis] = max_endpoint;
                }
            }
        }

        colliders.insert(collider);
    }

    void SAP::remove(const ColliderP& collider) {
        // Check if the collider is in the map
        if (!endpoint_map.contains(collider)) {
            return;
        }

        auto [min_endpoint, max_endpoint] = endpoint_map[collider];

        // Remove the endpoints from the list
        for (int axis = 0; axis < 3; axis++) {
            if (min_endpoint->prev[axis] != nullptr) {
                min_endpoint->prev[axis]->next[axis] = min_endpoint->next[axis];
            } else {
                list_heads[axis] = min_endpoint->next[axis];
            }

            if (min_endpoint->next[axis] != nullptr) {
                min_endpoint->next[axis]->prev[axis] = min_endpoint->prev[axis];
            }

            if (max_endpoint->prev[axis] != nullptr) {
                max_endpoint->prev[axis]->next[axis] = max_endpoint->next[axis];
            } else {
                list_heads[axis] = max_endpoint->next[axis];
            }

            if (max_endpoint->next[axis] != nullptr) {
                max_endpoint->next[axis]->prev[axis] = max_endpoint->prev[axis];
            }
        }

        // Remove the endpoints from the map
        endpoint_map.erase(collider);

        // Remove the pointers of the endpoints to each other
        min_endpoint->other_endpoint.reset();
        max_endpoint->other_endpoint.reset();

        // Remove the collider from the list of colliders
        colliders.erase(std::ranges::find(colliders, collider));

        // Remove the collider from the list of collisions
        std::erase_if(collisions, [collider](const CollisionPair& pair) {
            return pair.contains(collider);
        });
    }

    void SAP::update_endpoints(const ColliderP& collider) {
        // Check if the collider is in the map
        if (!endpoint_map.contains(collider)) {
            return;
        }

        auto [min_endpoint, max_endpoint] = endpoint_map[collider];

        // Update the value of the endpoints
        const auto [min, max] = collider->get_aabb();
        min_endpoint->value = min;
        max_endpoint->value = max;

        // For each axis, update the value of the endpoints
        for (int axis = 0; axis < 3; axis++) {
            EndpointP current = nullptr;
            EndpointP prev = nullptr;

            // Move min endpoint to the left
            for (current = min_endpoint->prev[axis]; current != nullptr && current->value[axis] > min_endpoint->value[axis]; current = current->prev[axis]) {
                if (!current->is_min && collider != current->collider && collider->get_aabb().intersects(current->get_aabb())) {
                    collisions.insert({collider, current->collider});
                }
            }

            if (current != min_endpoint->prev[axis]) {
                move_endpoint(axis, min_endpoint, current);
            }

            // Move max endpoint to the right
            prev = nullptr;
            for (current = max_endpoint->next[axis]; current != nullptr && current->value[axis] < max_endpoint->value[axis]; current = current->next[axis]) {
                if (current->is_min && collider != current->collider && collider->get_aabb().intersects(current->get_aabb())) {
                    collisions.insert({collider, current->collider});
                }

                prev = current;
            }

            if (current != max_endpoint->next[axis]) {
                move_endpoint(axis, min_endpoint, prev);
            }

            // Move min endpoint to the right
            prev = nullptr;
            for (current = min_endpoint->next[axis]; current != nullptr && current->value[axis] < min_endpoint->value[axis]; current = current->next[axis]) {
                if (!current->is_min) {
                    collisions.erase({collider, current->collider});
                }

                prev = current;
            }

            if (current != min_endpoint->next[axis]) {
                move_endpoint(axis, min_endpoint, prev);
            }

            // Move max endpoint to the left
            for (current = max_endpoint->prev[axis]; current != nullptr && current->value[axis] > max_endpoint->value[axis]; current = current->prev[axis]) {
                if (current->is_min) {
                    collisions.erase({collider, current->collider});
                }
            }

            if (current != max_endpoint->prev[axis]) {
                move_endpoint(axis, max_endpoint, current);
            }
        }
    }

    void SAP::move_endpoint(const size_t axis, const EndpointP& endpoint, const EndpointP& dest) {
        // Check if the endpoint is already in the correct position
        if (endpoint->prev[axis] == dest || endpoint == dest) {
            return;
        }

        // Unlink the original
        if (endpoint->prev[axis] != nullptr) {
            endpoint->prev[axis]->next[axis] = endpoint->next[axis];
        } else {
            list_heads[axis] = endpoint->next[axis];
        }

        if (endpoint->next[axis] != nullptr) {
            endpoint->next[axis]->prev[axis] = endpoint->prev[axis];
        }

        // If dest is nullptr, we are moving the endpoint to the beginning of the list
        if (dest == nullptr) {
            endpoint->prev[axis] = nullptr;
            endpoint->next[axis] = list_heads[axis];
            if (list_heads[axis] != nullptr) {
                list_heads[axis]->prev[axis] = endpoint;
            }
            list_heads[axis] = endpoint;
        } else {
            // Link the endpoint to the new position
            endpoint->prev[axis] = dest;
            endpoint->next[axis] = dest->next[axis];
            if (dest->next[axis] != nullptr) {
                dest->next[axis]->prev[axis] = endpoint;
            }
            dest->next[axis] = endpoint;
        }
    }

    std::pair<EndpointP, EndpointP> SAP::get_endpoints(const ColliderP& collider) {
        if (!endpoint_map.contains(collider)) {
            return {nullptr, nullptr};
        }
        return endpoint_map[collider];
    }

    CollisionSet SAP::get_collisions() {
        return collisions;
    }

    void SAP::print_lists() const {
        // Print the list of endpoints for each axis
        for (int axis = 0; axis < 3; axis++) {
            std::cout << "Axis " << axis << ": start -> ";
            auto current = list_heads[axis];
            while (current != nullptr) {
                // Get the number of the collider from an index of the colliders set
                const auto id = std::distance(colliders.begin(), std::ranges::find(colliders, current->collider));
                // Convert the index to character A, B, C, etc.
                const auto colliderid = static_cast<char>('A' + id);
                std::cout << colliderid << "_" << (current->is_min ? "min" : "max") << "(" << current->value[axis] << ") -> ";
                current = current->next[axis];
            }
            std::cout << "end" << std::endl;
        }

        // Print collisions
        std::cout << "Collisions: ";
        for (const auto& [collider_1, collider_2] : collisions) {
            const auto id_1 = std::distance(colliders.begin(), std::ranges::find(colliders, collider_1));
            const auto id_2 = std::distance(colliders.begin(), std::ranges::find(colliders, collider_2));
            const auto colliderid_1 = static_cast<char>('A' + id_1);
            const auto colliderid_2 = static_cast<char>('A' + id_2);
            std::cout << "(" << colliderid_1 << colliderid_2 << "), ";
        }
        std::cout << std::endl;
    }

    void SAP::print_map() const {
        // Print the map of endpoints
        for (const auto& [collider, endpoints] : endpoint_map) {
            std::cout << "Collider: " << collider << std::endl;
            std::cout << "Min: " << endpoints.first->value.x << ", " << endpoints.first->value.y << ", " << endpoints.first->value.z << std::endl;
            std::cout << "Max: " << endpoints.second->value.x << ", " << endpoints.second->value.y << ", " << endpoints.second->value.z << std::endl;
            std::cout << "Next: " << endpoints.first->next[0] << ", " << endpoints.first->next[1] << ", " << endpoints.first->next[2] << std::endl;
            std::cout << "Prev: " << endpoints.first->prev[0] << ", " << endpoints.first->prev[1] << ", " << endpoints.first->prev[2] << std::endl;
            std::cout << std::endl;
        }
    }
}
