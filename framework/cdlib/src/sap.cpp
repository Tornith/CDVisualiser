#include "sap.hpp"

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
                    if (current->is_min && current->collider != collider && collider->get_aabb().intersects(current->collider->get_aabb())) {
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

        std::cout << "Inserted collider" << std::endl;
        print_lists();
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

        // Remove the collider from the list of colliders
        colliders.erase(std::ranges::find(colliders, collider));

        // Remove the collider from the list of collisions
        std::erase_if(collisions, [collider](const CollisionPair& pair) {
            return pair.contains(collider);
        });

        std::cout << "Removed collider" << std::endl;
        print_lists();
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

        const auto walk_endpoint = [this, &collider](const EndpointP& endpoint, const int axis, const bool left) {
            EndpointP next = left ? endpoint->prev[axis] : endpoint->next[axis];

            while(next != nullptr) {
                // Check if the next endpoint is lower, or higher than the current endpoint (based on walk direction)
                if (left && next->value[axis] < endpoint->value[axis] || !left && next->value[axis] > endpoint->value[axis]) {
                    break;
                }

                // If the endpoint is a min, next is a max and we're walking left,
                // Or if the endpoint is a max, next is a min and we're walking right, check for overlap
                if ((left && endpoint->is_min && !next->is_min) || (!left && !endpoint->is_min && next->is_min)) {
                    if (next->collider != collider && collider->get_aabb().intersects(next->collider->get_aabb())) {
                        collisions.insert({collider, next->collider});
                    }
                }

                // If the endpoint is a min, next is a max and we're walking right,
                // Or if the endpoint is a max, next is a min and we're walking left, check if still overlapping
                if ((!left && endpoint->is_min && !next->is_min) || (left && !endpoint->is_min && next->is_min)) {
                    if (!collider->get_aabb().intersects(next->collider->get_aabb())) {
                        collisions.erase({collider, next->collider});
                    }
                }

                // Move to the next endpoint
                next = left ? next->prev[axis] : next->next[axis];
            }

            // Move the endpoint, if we have moved
            if (next != (left ? endpoint->prev[axis] : endpoint->next[axis])) {
                move_endpoint(axis, endpoint, next);
            }
        };

        // For each axis, update the value of the endpoints
        for (int axis = 0; axis < 3; axis++) {
            walk_endpoint(min_endpoint, axis, true);
            walk_endpoint(max_endpoint, axis, false);

            walk_endpoint(min_endpoint, axis, false);
            walk_endpoint(max_endpoint, axis, true);
        }

        std::cout << "Updated collider" << std::endl;
        print_lists();
    }

    void SAP::move_endpoint(const size_t axis, const EndpointP& endpoint, const EndpointP& dest) {
        // If the dest is nullptr, we are movint the endpoint to the beginning of the list
        if (dest == nullptr) {
            if (endpoint->prev[axis] != nullptr) {
                endpoint->prev[axis]->next[axis] = endpoint->next[axis];
            } else {
                list_heads[axis] = endpoint->next[axis];
            }
            if (endpoint->next[axis] != nullptr) {
                endpoint->next[axis]->prev[axis] = endpoint->prev[axis];
            }
            endpoint->prev[axis] = nullptr;
            endpoint->next[axis] = list_heads[axis];
            list_heads[axis]->prev[axis] = endpoint;
            list_heads[axis] = endpoint;
        } else {
            // Move the endpoint after dest
            if (endpoint->prev[axis] != nullptr) {
                endpoint->prev[axis]->next[axis] = endpoint->next[axis];
            } else {
                list_heads[axis] = endpoint->next[axis];
            }
            if (endpoint->next[axis] != nullptr) {
                endpoint->next[axis]->prev[axis] = endpoint->prev[axis];
            }
            endpoint->prev[axis] = dest;
            endpoint->next[axis] = dest->next[axis];
            if (dest->next[axis] != nullptr) {
                dest->next[axis]->prev[axis] = endpoint;
            }
            dest->next[axis] = endpoint;
        }

        std::cout << "Moved endpoint" << std::endl;
        print_lists();
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
                std::cout << (current->is_min ? "min" : "max") << "(" << current->value[axis] << ") -> ";
                current = current->next[axis];
            }
            std::cout << "end" << std::endl;
        }
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
