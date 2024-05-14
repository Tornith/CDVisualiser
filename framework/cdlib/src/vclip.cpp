#include "vclip.hpp"

#include <functional>

#include "bruteforce.hpp"
#include "voronoi.hpp"

namespace cdlib
{
    ClipData clip_edge(const HalfEdgeP& clipped_edge, const FeatureP& feature) {
        return clip_edge(clipped_edge, feature, feature->get_neighbours());
    }

    std::optional<FeatureP> VClip::post_clip_derivative_update(const ClipData& clip_data, const std::optional<float> derivative_l, const std::optional<float> derivative_h)
    {
        if (clip_data.neighbour_l != nullptr && derivative_l.has_value() && derivative_l > 0){
            return clip_data.neighbour_l;
        }
        if (clip_data.neighbour_h != nullptr && derivative_h.has_value() && derivative_h < 0){
            return clip_data.neighbour_h;
        }
        return std::nullopt;
    }

    std::optional<float> VClip::distance_derivative_sign(const glm::vec3 edge_point, const HalfEdgeP& edge, const VertexP& vertex)
    {
        // sign(D'_{E,X}(lambda) = sign(u * (e(lambda) - v)))
        //  - u = direction of the edge
        //  - e(lambda) = point on the edge (in clip_data we have point_l and point_h)
        //  - v = point on the feature
        if (edge_point == vertex->get_position()){
            return std::nullopt;
        }

        return dot(edge->get_direction(), edge_point - vertex->get_position());
    }

    std::optional<float> VClip::distance_derivative_sign(const glm::vec3 edge_point, const HalfEdgeP& edge, const FaceP& face)
    {
        // sign(D'_{F,X}(lambda) = { +sign(u * n) if distance of p to plane is >0 else -sign(u * n) }
        //  - u = direction of the edge
        //  - n = normal of the face
        //  - p = point on the edge (in clip_data we have point_l and point_h)
        const auto dot_result = dot(edge->get_direction(), face->get_plane().normal);
        const auto distance = face->get_plane().distance_to(edge_point);

        if (distance == 0) {
            return std::nullopt;
        }

        return distance > 0 ? dot_result : -dot_result;
    }

    /**
     * @return true, if the primary feature was updated
     */
    std::optional<FeatureP> VClip::post_clip_derivative_check(const ClipData& clip_data)
    {
        const auto is_feature_edge = static_cast<bool>(std::dynamic_pointer_cast<HalfEdge>(clip_data.clipping_feature));

        // If the primary feature is an edge we check the derivative against the appropriate neighbours
        // Page 10 of the original paper
        const std::optional<float> derivative_l = distance_derivative_sign(clip_data.point_l, clip_data.clipped_edge, is_feature_edge ? clip_data.neighbour_l : clip_data.clipping_feature);
        const std::optional<float> derivative_h = distance_derivative_sign(clip_data.point_h, clip_data.clipped_edge, is_feature_edge ? clip_data.neighbour_h : clip_data.clipping_feature);

        if (!derivative_l.has_value() && !derivative_h.has_value()){
            return std::nullopt; // Degenerate case
        }

        return post_clip_derivative_update(clip_data, derivative_l, derivative_h);
    }

    VClipState VClip::handle_local_minimum()
    {
        auto d_max = std::numeric_limits<float>::lowest();
        FaceP face_max = nullptr;

        const auto face = get_feature<Face>();
        const auto vertex = get_feature<Vertex>();

        // For all faces F' on F's polyhedron
        for (const auto& f : face->polyhedron->get_faces()){
            const auto d = f->get_plane().distance_to(vertex->get_position());
            if (d > d_max){
                d_max = d;
                face_max = f;
            }
        }

        if (d_max <= 0) {
            return PENETRATION;
        }

        set_feature<Face>(face_max);
        return CONTINUE;
    }

    FeatureP VClip::closest_face_vertex_to_edge(const HalfEdgeP& clipped_edge, const FaceP& face) {
        FeatureP closest_feature = nullptr;
        auto closest_distance = std::numeric_limits<float>::max();
        for (const auto& vertex : face->get_vertices()){
            const auto distance = feature_distance(vertex, clipped_edge);
            if (distance < closest_distance){
                closest_distance = distance;
                closest_feature = vertex;
            }
        }
        for (const auto& edge : face->get_edges()){
            const auto distance = feature_distance(edge, clipped_edge);
            if (distance < closest_distance){
                closest_distance = distance;
                closest_feature = edge;
            }
        }
        return closest_feature;
    }

    /**************
     *   STATES   *
     **************/
    VClipState VClip::execute_vertex_vertex() {
        // Test, whether the two vertices are in each other's voronoi region
        const auto vertex_1 = primary_feature<Vertex>();
        const auto vertex_2 = secondary_feature<Vertex>();

        const auto [in_region_1, plane_1] = Voronoi::in_voronoi_region(vertex_1, vertex_2->get_position());
        if (!in_region_1 && plane_1.has_value()){
            set_primary_feature(plane_1.value().edge);
            return CONTINUE;
        }

        const auto [in_region_2, plane_2] = Voronoi::in_voronoi_region(vertex_2, vertex_1->get_position());
        if (!in_region_2 && plane_2.has_value()){
            set_secondary_feature(plane_2.value().edge);
            return CONTINUE;
        }

        return DONE;
    }

    VClipState VClip::execute_vertex_edge() {
        const auto vertex = get_feature<Vertex>();
        const auto edge = get_feature<HalfEdge>();

        // Search for a voronoi plane VP(E, N) that V violates
        const auto [in_region, plane] = Voronoi::in_voronoi_region(edge, vertex->get_position());
        if (!in_region && plane.has_value()){
            // Violated plane exists
            set_feature<HalfEdge>(plane.value().feature);
            return CONTINUE;
        }

        // Clip E against V
        const auto clip_data = clip_edge(edge, vertex);
        if (clip_data.neighbour_l != nullptr && clip_data.neighbour_h == clip_data.neighbour_l){
            set_feature<Vertex>(clip_data.neighbour_l);
            return CONTINUE;
        }

        if (const auto feature_to_update = post_clip_derivative_check(clip_data); feature_to_update.has_value()){
            set_feature<Vertex>(feature_to_update.value());
            return CONTINUE;
        }

        return DONE;
    }

    VClipState VClip::execute_vertex_face() {
        const auto vertex = get_feature<Vertex>();
        const auto face = get_feature<Face>();

        // Find a voronoi plane VP(F, E) that V MAXIMALLY violates
        const auto plane = Voronoi::find_maximally_violating_voronoi_plane(face, vertex->get_position());
        if (plane.has_value()){
            set_feature<Face>(plane.value().edge);
            return CONTINUE;
        }

        // Look for an edge (incident to V) which endpoints satisfy: |D_p(V)| > |D_p(V')|
        // where V' is the other vertex of the incident edge and D_p is the distance to the face
        HalfEdgeP good_edge = nullptr;
        for (const auto& edge : vertex->get_edges()){
            const auto other_vertex = edge->end;
            const auto distance_vertex = face->get_plane().distance_to(vertex->get_position());
            const auto distance_other = face->get_plane().distance_to(other_vertex->get_position());

            // If the other vertex is closer to the face (from either direction)
            if (distance_vertex * distance_other < 0 || std::abs(distance_vertex) > std::abs(distance_other)){
                good_edge = edge;
                break;
            }
        }

        if (good_edge != nullptr){
            set_feature<Vertex>(good_edge);
            return CONTINUE;
        }

        if (face->get_plane().is_above(vertex->get_position())){
            return DONE;
        }

        // Local minimum situation
        return handle_local_minimum();
    }

    VClipState VClip::execute_edge_edge() {
        const auto edge_1 = primary_feature<HalfEdge>();
        const auto edge_2 = secondary_feature<HalfEdge>();

        const auto clip_against_edge = [this](const HalfEdgeP& active_edge, const HalfEdgeP& other_edge, bool set_primary) -> VClipState {
            // Clip the edge_2 against the edge_1's vertex-edge planes
            const auto clip_data_vertex = clip_edge(other_edge, active_edge, active_edge->get_neighbour_vertices());
            // If the edge_2 is completely beneath one of the vertex-edge planes
            if (clip_data_vertex.simply_excluded()){
                if (set_primary){
                    set_primary_feature(clip_data_vertex.neighbour_l);
                } else {
                    set_secondary_feature(clip_data_vertex.neighbour_l);
                }
                return CONTINUE;
            }

            // Check derivatives
            if (const auto feature_to_update = post_clip_derivative_check(clip_data_vertex); feature_to_update.has_value()){
                if (set_primary){
                    set_primary_feature(feature_to_update.value());
                } else {
                    set_secondary_feature(feature_to_update.value());
                }
                return CONTINUE;
            }

            // Clip the edge_2 against the edge_1's face-edge planes
            const auto clip_data_face = clip_edge(other_edge, active_edge, active_edge->get_neighbour_faces(), clip_data_vertex);
            // If the edge_2 is completely beneath one of the face-edge planes
            if (clip_data_face.simply_excluded()){
                if (set_primary){
                    set_primary_feature(clip_data_face.neighbour_l);
                } else {
                    set_secondary_feature(clip_data_face.neighbour_l);
                }
                return CONTINUE;
            }

            // Check derivatives
            if (const auto feature_to_update = post_clip_derivative_check(clip_data_face); feature_to_update.has_value()){
                if (set_primary){
                    set_primary_feature(feature_to_update.value());
                } else {
                    set_secondary_feature(feature_to_update.value());
                }
                return CONTINUE;
            }
            return DONE;
        };

        const auto result_1 = clip_against_edge(edge_1, edge_2, true);
        if (result_1 != DONE) {
            return result_1;
        }

        const auto result_2 = clip_against_edge(edge_2, edge_1, false);
        return result_2;
    }

    VClipState VClip::execute_edge_face() {
        const auto edge = get_feature<HalfEdge>();
        const auto face = get_feature<Face>();

        // Clip the edge against the face
        const auto clip_data = clip_edge(edge, face);

        // If the edge is excluded
        if (!clip_data.is_clipped) {
            const auto neighbour = clip_data.neighbour_l != nullptr ? clip_data.neighbour_l : clip_data.neighbour_h;
            if (neighbour == nullptr){
                std::cerr << "VClip: Degenerate case" << std::endl;
                return ERROR;
            }
            const auto closest_feature = closest_face_vertex_to_edge(edge, face);
            set_feature<Face>(closest_feature);
            return CONTINUE;
        }

        // Find the distances of clipped points to the face
        const auto distance_l = face->get_plane().distance_to(clip_data.point_l);
        const auto distance_h = face->get_plane().distance_to(clip_data.point_h);

        // If the sign of the distances is different we have a penetration
        if (distance_l * distance_h <= 0){
            return PENETRATION;
        }

        const auto derivative = distance_derivative_sign(clip_data.point_l, edge, face);
        if (!derivative.has_value()){
            std::cerr << "VClip: Degenerate case" << std::endl;
            return ERROR;
        }
        if (derivative.value() >= 0){
            if (clip_data.neighbour_l != nullptr){
                set_feature<Face>(clip_data.neighbour_l);
            } else {
                set_feature<HalfEdge>(edge->start);
            }
        } else {
            if (clip_data.neighbour_h != nullptr){
                set_feature<Face>(clip_data.neighbour_h);
            } else {
                set_feature<HalfEdge>(edge->end);
            }
        }

        return CONTINUE;
    }

    VClipState VClip::initialize() {
        // Check if the colliders are valid
        if (!collider_1->is_valid() && !collider_2->is_valid()){
            std::cerr << "VClip: Invalid colliders" << std::endl;
            return ERROR;
        }

        // Choose the two features randomly (first vertex)
        // TODO: Temporal coherence
        feature_1 = collider_1->get_shape()->get_vertex(0);
        feature_2 = collider_2->get_shape()->get_vertex(0);

        return CONTINUE;
    }

    VClipState VClip::step() {
        const auto primary_is_vertex = static_cast<bool>(primary_feature<Vertex>());
        const auto primary_is_edge = static_cast<bool>(primary_feature<HalfEdge>());
        const auto primary_is_face = static_cast<bool>(primary_feature<Face>());
        const auto secondary_is_vertex = static_cast<bool>(secondary_feature<Vertex>());
        const auto secondary_is_edge = static_cast<bool>(secondary_feature<HalfEdge>());
        const auto secondary_is_face = static_cast<bool>(secondary_feature<Face>());

        if (feature_1 == nullptr || feature_2 == nullptr){
            return initialize();
        }
        if (primary_is_vertex && secondary_is_vertex){
            return execute_vertex_vertex();
        }
        if (primary_is_vertex && secondary_is_edge || primary_is_edge && secondary_is_vertex){
            return execute_vertex_edge();
        }
        if (primary_is_vertex && secondary_is_face || primary_is_face && secondary_is_vertex){
            return execute_vertex_face();
        }
        if (primary_is_edge && secondary_is_edge){
            return execute_edge_edge();
        }
        if (primary_is_edge && secondary_is_face || primary_is_face && secondary_is_edge){
            return execute_edge_face();
        }
        std::cerr << "Invalid state" << std::endl;

        const auto transform1 = collider_1->get_transform();
        const auto transform2 = collider_2->get_transform();

        const auto position1 = glm::vec3(transform1[3]);
        const auto position2 = glm::vec3(transform2[3]);

        const auto rotation1 = glm::eulerAngles(glm::quat_cast(transform1));
        const auto rotation2 = glm::eulerAngles(glm::quat_cast(transform2));

        std::cerr << "Object 1: Position: " << position1.x << " " << position1.y << " " << position1.z << ", Rotation: " << rotation1.x << " " << rotation1.y << " " << rotation1.z << std::endl;
        std::cerr << "Object 2: Position: " << position2.x << " " << position2.y << " " << position2.z << ", Rotation: " << rotation2.x << " " << rotation2.y << " " << rotation2.z << std::endl;

        return ERROR;
    }

    CollisionData VClip::get_collision_data() {
        auto iteration = 0;
        std::pair<FeatureP, FeatureP> first_looping_features = { nullptr, nullptr };
        while (state == CONTINUE || state == INIT){
            state = step();
            if (iteration++ > 10000){
                if (first_looping_features.first == feature_1 && first_looping_features.second == feature_2){
                    std::cerr << "Infinite loop finished" << std::endl;
                    const auto transform1 = collider_1->get_transform();
                    const auto transform2 = collider_2->get_transform();

                    const auto position1 = glm::vec3(transform1[3]);
                    const auto position2 = glm::vec3(transform2[3]);

                    const auto rotation1 = glm::eulerAngles(glm::quat_cast(transform1));
                    const auto rotation2 = glm::eulerAngles(glm::quat_cast(transform2));

                    std::cerr << "Object 1: Position: " << position1.x << " " << position1.y << " " << position1.z << ", Rotation: " << rotation1.x << " " << rotation1.y << " " << rotation1.z << std::endl;
                    std::cerr << "Object 2: Position: " << position2.x << " " << position2.y << " " << position2.z << ", Rotation: " << rotation2.x << " " << rotation2.y << " " << rotation2.z << std::endl;
                    return { false };
                }
                std::cerr << "Infinite loop" << std::endl;
                std::cout << "Feature 1: " << feature_1->to_string() << std::endl;
                std::cout << "Feature 2: " << feature_2->to_string() << std::endl;
                std::cout << "Distance: " << feature_distance(feature_1, feature_2) << std::endl;
                std::cout << "Collider 1: " << collider_1->get_shape()->get_debug_data() << std::endl;
                std::cout << "Collider 2: " << collider_2->get_shape()->get_debug_data() << std::endl;
                if (first_looping_features.first == nullptr){
                    first_looping_features = { feature_1, feature_2 };
                }
            }
        }

        if (state == ERROR){
            return { false };
        }

        const bool is_colliding = state == PENETRATION;
        if (feature_1 == nullptr || feature_2 == nullptr){
            return { is_colliding };
        }

        const float distance = feature_distance(feature_1, feature_2) * (is_colliding ? -1.0f : 1.0f);
        const auto normal = glm::vec3(0.f);
        return { is_colliding, normal, distance, feature_1, feature_2 };
    }

    void VClip::reset() {
        state = INIT;
        feature_1 = nullptr;
        feature_2 = nullptr;
    }

    bool VClip::debug_brute_force(bool should_collide, const FeatureP& expected_feature_1, const FeatureP& expected_feature_2) const {
        auto bruteforce = NarrowBruteforce(collider_1, collider_2);
        const auto [is_colliding, normal, min_distance, min_feature_1, min_feature_2] = bruteforce.get_collision_data();

        if (is_colliding && should_collide){
            return true;
        }

        if (is_colliding != should_collide){
            std::cerr << "Brute force failed" << std::endl;
            std::cerr << "Expected: collision(" << should_collide << ")" << std::endl;
            std::cerr << "Got: collision(" << is_colliding << ")" << std::endl;
            return false;
        }

        auto check_if_features_equal = [](const FeatureP& f1, const FeatureP& f2) {
            if (f1 == nullptr || f2 == nullptr){
                return false;
            }
            if (f1 == f2) return true;

            // Check if the features are half_edges, if so check if they are potentially twins
            if (const auto edge_1 = std::dynamic_pointer_cast<HalfEdge>(f1); edge_1 != nullptr){
                if (const auto edge_2 = std::dynamic_pointer_cast<HalfEdge>(f2); edge_2 != nullptr){
                    return edge_1->start == edge_2->end && edge_1->end == edge_2->start;
                }
            }

            return false;
        };

        const auto expected_distance = feature_distance(expected_feature_1, expected_feature_2);
        const auto distance_difference = std::abs(expected_distance - min_distance);

        const auto result = check_if_features_equal(expected_feature_1, min_feature_1) && check_if_features_equal(expected_feature_2, min_feature_2) ||
               check_if_features_equal(expected_feature_1, min_feature_2) && check_if_features_equal(expected_feature_2, min_feature_1) ||
               distance_difference < 0.001f;

        if (!result) {
            std::cerr << "Brute force failed" << std::endl;
            std::cerr << "Expected: " << expected_feature_1->to_string() << " - " << expected_feature_2->to_string() << std::endl;
            std::cerr << "Got: " << min_feature_1->to_string() << " - " << min_feature_2->to_string() << std::endl;
            std::cerr << "Distance difference: " << distance_difference << std::endl;
            std::cerr << "Collider 1: " << collider_1->get_shape()->get_debug_data() << std::endl;
            std::cerr << "Collider 2: " << collider_2->get_shape()->get_debug_data() << std::endl;
            std::cerr << std::endl;
        }

        return result;
    }

    CollisionData VClipRaycast::get_collision_data()
    {
        return VClip::get_collision_data();
    }

    VClipState VClipRaycast::initialize()
    {
        // Check if the colliders are valid
        if (!collider_1->is_valid() && !collider_2->is_valid()){
            std::cerr << "VClip: Invalid colliders" << std::endl;
            return ERROR;
        }

        // Choose the two features randomly (first vertex)
        // TODO: Temporal coherence
        feature_1 = collider_1->get_shape()->get_vertex(0);
        feature_2 = collider_2->get_shape()->get_half_edge(0); // Choose the edge of the rayscast

        return CONTINUE;
    }

    VClipState VClipRaycast::execute_vertex_edge()
    {
        auto vertex = primary_feature<Vertex>();
        auto edge = secondary_feature<HalfEdge>();

        if (vertex == nullptr || edge == nullptr){
            // Spetial case, when endpoint of the ray is the vertex against an edge of the collider
            vertex = secondary_feature<Vertex>();
            edge = primary_feature<HalfEdge>();

            const auto [in_region, plane] = Voronoi::in_voronoi_region(edge, vertex->get_position());
            if (!in_region && plane.has_value()){
                // Violated plane exists
                set_feature<HalfEdge>(plane.value().feature);
                return CONTINUE;
            }
        }

        // Clip E against V
        const auto clip_data = clip_edge(edge, vertex);
        if (clip_data.neighbour_l != nullptr && clip_data.neighbour_h == clip_data.neighbour_l){
            set_feature<Vertex>(clip_data.neighbour_l);
            return CONTINUE;
        }

        if (const auto feature_to_update = post_clip_derivative_check(clip_data); feature_to_update.has_value()){
            set_feature<Vertex>(feature_to_update.value());
            return CONTINUE;
        }

        return DONE;
    }

    VClipState VClipRaycast::execute_edge_edge()
    {
        const auto collider_edge = primary_feature<HalfEdge>();
        const auto raycast_edge = secondary_feature<HalfEdge>();

        // Same as in the normal VClip but only for one edge
        const auto clip_data_vertex = clip_edge(raycast_edge, collider_edge, collider_edge->get_neighbour_vertices());
        // If the edge_2 is completely beneath one of the vertex-edge planes
        if (clip_data_vertex.simply_excluded()){
            set_primary_feature(clip_data_vertex.neighbour_l);
            return CONTINUE;
        }

        // Check derivatives
        if (const auto feature_to_update = post_clip_derivative_check(clip_data_vertex); feature_to_update.has_value()){
            set_primary_feature(feature_to_update.value());
            return CONTINUE;
        }

        // Clip the edge_2 against the edge_1's face-edge planes
        const auto clip_data_face = clip_edge(raycast_edge, collider_edge, collider_edge->get_neighbour_faces(), clip_data_vertex);
        // If the edge_2 is completely beneath one of the face-edge planes
        if (clip_data_face.simply_excluded()){
            set_primary_feature(clip_data_face.neighbour_l);
            return CONTINUE;
        }

        // Check derivatives
        if (const auto feature_to_update = post_clip_derivative_check(clip_data_face); feature_to_update.has_value()){
            set_primary_feature(feature_to_update.value());
            return CONTINUE;
        }

        return DONE;
    }

    VClipState VClipRaycast::execute_edge_face()
    {
        const auto face = primary_feature<Face>();
        const auto edge = secondary_feature<HalfEdge>();

        // Clip the edge against the face
        const auto clip_data = clip_edge(edge, face);

        // If the edge is excluded
        if (!clip_data.is_clipped) {
            const auto neighbour = clip_data.neighbour_l != nullptr ? clip_data.neighbour_l : clip_data.neighbour_h;
            if (neighbour == nullptr){
                std::cerr << "VClip: Degenerate case" << std::endl;
                return ERROR;
            }
            const auto closest_feature = closest_face_vertex_to_edge(edge, face);
            set_feature<Face>(closest_feature);
            return CONTINUE;
        }

        // Find the distances of clipped points to the face
        const auto distance_l = face->get_plane().distance_to(clip_data.point_l);
        const auto distance_h = face->get_plane().distance_to(clip_data.point_h);

        // If the sign of the distances is different we have a penetration
        if (distance_l * distance_h <= 0){
            return PENETRATION;
        }

        const auto derivative = distance_derivative_sign(clip_data.point_l, edge, face);
        if (!derivative.has_value()){
            std::cerr << "VClip: Degenerate case" << std::endl;
            return ERROR;
        }
        if (derivative.value() >= 0){
            if (clip_data.neighbour_l != nullptr){
                set_feature<Face>(clip_data.neighbour_l);
            } else {
                set_feature<HalfEdge>(edge->start);
            }
        } else {
            if (clip_data.neighbour_h != nullptr){
                set_feature<Face>(clip_data.neighbour_h);
            } else {
                set_feature<HalfEdge>(edge->end);
            }
        }

        return CONTINUE;
    }

    RayCastResult VClip::raycast(const Ray& ray, const ColliderP& collider) {
        const auto [is_colliding, normal, depth, feature_1, feature_2] = VClipRaycast(collider, ray).get_collision_data();
        if (is_colliding){
            // If we are coliding, the feature has to be a face -> calculate collision between the ray and the face
            const auto face = std::dynamic_pointer_cast<Face>(feature_1);
            glm::vec3 point;
            Ray::intersects_triangle(
                ray,
                face->get_vertices()[0]->get_position(),
                face->get_vertices()[1]->get_position(),
                face->get_vertices()[2]->get_position(),
                point
            );
            return { true, collider, point, normal, glm::distance(ray.origin, point) };
        }
        return {

        };
    }
}
