#include "sap.hpp"

namespace cdlib {
    void SAP::create_endpoints() {
        for (auto& collider : colliders) {
            auto aabb = collider->get_aabb();
            for (int i = 0; i < 3; i++) {
                const auto min = aabb.first[i];
                const auto max = aabb.second[i];
                endpoints[i].push_back({collider, nullptr, nullptr, min, true});
                endpoints[i].push_back({collider, nullptr, nullptr, max, false});
            }
        }

        sort_endpoints();
    }

    void SAP::sort_endpoints() {
        // Use inplace insertion sort to sort the endpoints
        for (auto& axis : endpoints) {
            for (size_t i = 1; i < axis.size(); i++) {
                auto key = axis[i];
                int j = i - 1;

                while (j >= 0 && axis[j].value > key.value) {
                    axis[j + 1] = axis[j];
                    j = j - 1;
                }
                axis[j + 1] = key;
            }
        }

        // Link the endpoints
        for (auto& axis : endpoints) {
            for (size_t i = 0; i < axis.size(); i++) {
                if (i == 0) {
                    axis[i].prev = nullptr;
                    axis[i].next = &axis[i + 1];
                } else if (i == axis.size() - 1) {
                    axis[i].prev = &axis[i - 1];
                    axis[i].next = nullptr;
                } else {
                    axis[i].prev = &axis[i - 1];
                    axis[i].next = &axis[i + 1];
                }
            }
        }
    }

    void SAP::update_endpoints(std::vector<std::shared_ptr<Collider>> changed_colliders) {
        for (auto& collider : changed_colliders) {
            // If the collider is not in the list of colliders, skip it
            auto it = std::find(colliders.begin(), colliders.end(), collider);
            if (it == colliders.end()) {
                continue;
            }
            // Otherwise
        }
    }

    std::vector<std::pair<std::shared_ptr<Collider>, std::shared_ptr<Collider>>> SAP::get_collisions() {
        sort_endpoints();
        for (int axis = 0; axis < 3; axis++) {
            for (size_t i = 0; i < endpoints[axis].size(); i++) {
                auto endpoint = endpoints[axis][i];
                if (endpoint.is_min) {
                    auto next = endpoint.next;
                    while (next != nullptr) {
                        if (next->collider != endpoint.collider) {
                            if (next->value > endpoint.value) {
                                break;
                            }
                            collisions.push_back({endpoint.collider, next->collider});
                        }
                        next = next->next;
                    }
                }
            }
        }
        return collisions;
    }
}
