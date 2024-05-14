#include "bruteforce.hpp"

#include <algorithm>

namespace cdlib {
    CollisionData NarrowBruteforce::get_collision_data() {
        const bool collision = is_colliding();

        const auto [feature_1, feature_2] = get_closest_features();
        return CollisionData(
            collision,
            glm::vec3(0.0f),
            feature_distance(feature_1, feature_2),
            feature_1,
            feature_2
        );
    }

    std::pair<FeatureP, FeatureP> NarrowBruteforce::get_closest_features() const {
        const auto vertices_1 = collider_1->get_shape()->get_vertices() | std::ranges::views::transform([](const auto& vertex) { return std::dynamic_pointer_cast<Feature>(vertex); });
        const auto vertices_2 = collider_2->get_shape()->get_vertices() | std::ranges::views::transform([](const auto& vertex) { return std::dynamic_pointer_cast<Feature>(vertex); });
        const auto edges_1 = collider_1->get_shape()->get_half_edges() | std::ranges::views::transform([](const auto& vertex) { return std::dynamic_pointer_cast<Feature>(vertex); });
        const auto edges_2 = collider_2->get_shape()->get_half_edges() | std::ranges::views::transform([](const auto& vertex) { return std::dynamic_pointer_cast<Feature>(vertex); });
        const auto faces_1 = collider_1->get_shape()->get_faces() | std::ranges::views::transform([](const auto& vertex) { return std::dynamic_pointer_cast<Feature>(vertex); });
        const auto faces_2 = collider_2->get_shape()->get_faces() | std::ranges::views::transform([](const auto& vertex) { return std::dynamic_pointer_cast<Feature>(vertex); });

        // Combine all features into one vector
        std::vector<FeatureP> features_1;
        features_1.insert(features_1.end(), vertices_1.begin(), vertices_1.end());
        features_1.insert(features_1.end(), edges_1.begin(), edges_1.end());
        features_1.insert(features_1.end(), faces_1.begin(), faces_1.end());

        std::vector<FeatureP> features_2;
        features_2.insert(features_2.end(), vertices_2.begin(), vertices_2.end());
        features_2.insert(features_2.end(), edges_2.begin(), edges_2.end());
        features_2.insert(features_2.end(), faces_2.begin(), faces_2.end());

        auto min_distance = std::numeric_limits<float>::max();
        FeatureP min_feature_1 = nullptr;
        FeatureP min_feature_2 = nullptr;
        for (const auto& feature_1 : features_1){
            for (const auto& feature_2 : features_2){
                if (feature_1 == feature_2){
                    continue;
                }

                const auto distance = feature_distance(feature_1, feature_2);
                if (distance < min_distance){
                    min_distance = distance;
                    min_feature_1 = feature_1;
                    min_feature_2 = feature_2;
                }
            }
        }

        return {min_feature_1, min_feature_2};
    }

    bool intersect_line_triangle(glm::vec3 p, glm::vec3 q, glm::vec3 a, glm::vec3 b, glm::vec3 c) {
        const auto pq = q - p;
        const auto pa = a - p;
        const auto pb = b - p;
        const auto pc = c - p;

        auto u = glm::dot(pq, glm::cross(pc, pb));
        if (u < 0.0f){
            return false;
        }
        auto v = glm::dot(pq, glm::cross(pa, pc));
        if (v < 0.0f){
            return false;
        }
        auto w = glm::dot(pq, glm::cross(pb, pa));
        if (w < 0.0f){
            return false;
        }

        // Calculate the intersection point
        const auto denom = 1.0f / (u + v + w);
        u *= denom;
        v *= denom;
        w *= denom;

        const auto intersection = u * a + v * b + w * c;
        // Calculate the parameter along the line
        const auto t = glm::dot(intersection - p, pq) / glm::dot(pq, pq);
        return t >= 0.0f && t <= 1.0f;
    }

    bool NarrowBruteforce::is_colliding() const {
        const auto vertices_1 = collider_1->get_shape()->get_vertices();
        const auto vertices_2 = collider_2->get_shape()->get_vertices();
        const auto edges_1 = collider_1->get_shape()->get_half_edges();
        const auto edges_2 = collider_2->get_shape()->get_half_edges();
        const auto faces_1 = collider_1->get_shape()->get_faces();
        const auto faces_2 = collider_2->get_shape()->get_faces();

        // Test verticies inside faces of the other object
        for (const auto& vertex_1 : vertices_1){
            auto inside = true;
            for (const auto& face_2 : faces_2){
                const auto above = face_2->get_plane().is_above(vertex_1->get_position());
                if (above){
                    inside = false;
                    break;
                }
            }
            if (inside){
                return true;
            }
        }

        for (const auto& vertex_2 : vertices_2){
            auto inside = true;
            for (const auto& face_1 : faces_1){
                const auto above = face_1->get_plane().is_above(vertex_2->get_position());
                if (above){
                    inside = false;
                    break;
                }
            }
            if (inside){
                return true;
            }
        }

        // Test edges - for each edge of object 1, check if it intersects with any face of object 2
        for (const auto& edge_1 : edges_1){
            for (const auto& face_2 : faces_2){
                if (intersect_line_triangle(
                    edge_1->start->get_position(),
                    edge_1->end->get_position(),
                    face_2->get_vertices()[0]->get_position(),
                    face_2->get_vertices()[1]->get_position(),
                    face_2->get_vertices()[2]->get_position()
                )){
                    return true;
                }
            }
        }

        // Test edges - for each edge of object 2, check if it intersects with any face of object 1
        for (const auto& edge_2 : edges_2){
            for (const auto& face_1 : faces_1){
                if (intersect_line_triangle(
                    edge_2->start->get_position(),
                    edge_2->end->get_position(),
                    face_1->get_vertices()[0]->get_position(),
                    face_1->get_vertices()[1]->get_position(),
                    face_1->get_vertices()[2]->get_position()
                )){
                    return true;
                }
            }
        }

        return false;
    }

    void BroadBruteforce::insert(const ColliderP& collider) {
        colliders.insert(collider);
    }

    void BroadBruteforce::remove(const ColliderP& collider) {
        colliders.erase(collider);
    }

    CollisionSet BroadBruteforce::get_collisions() {
        auto collisions = CollisionSet();

        for (auto it = colliders.begin(); it != colliders.end(); ++it){
            for (auto jt = std::next(it); jt != colliders.end(); ++jt){
                const auto& collider_1 = *it;
                const auto& collider_2 = *jt;
                if (collider_1->get_aabb().intersects(collider_2->get_aabb())){
                    collisions.insert({collider_1, collider_2});
                }
            }
        }

        return collisions;
    }

    std::set<ColliderP> NarrowBruteforce::raycast(const Ray& ray, const std::set<ColliderP>& colliders) {
        // Check if the ray intersects with any collider's triangle
        std::set<ColliderP> hits;
        for (const auto& collider : colliders){
            const auto result = raycast(ray, collider);
            if (result.is_colliding){
                hits.insert(collider);
            }
        }

        return hits;
    }

    CollisionData NarrowBruteforce::raycast(const Ray& ray, const ColliderP& collider) {
        const auto faces = collider->get_shape()->get_faces();
        for (const auto& face : faces){
            const auto vertices = face->get_vertices();
            const auto vertex_0 = vertices[0]->get_position();
            const auto vertex_1 = vertices[1]->get_position();
            const auto vertex_2 = vertices[2]->get_position();

            glm::vec3 point;

            if (Ray::intersects_triangle(
                ray,
                vertex_0,
                vertex_1,
                vertex_2,
                point
            )){
                return CollisionData(true);
            }
        }
        return CollisionData(false);
    }

    std::set<ColliderP> BroadBruteforce::raycast(const Ray& ray) {
        // For each collider, check if the ray intersects the collider's AABB
        std::set<ColliderP> hits;
        for (const auto& collider : colliders){
            if (collider->get_aabb().raycast(ray).has_value()){
                hits.insert(collider);
            }
        }
        return hits;
    }
}
