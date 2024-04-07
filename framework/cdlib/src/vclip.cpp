#include "vclip.hpp"

#include <functional>

#include "voronoi.hpp"

namespace cdlib
{
    ClipData clip_edge(const HalfEdgeP& clipped_edge, const FeatureP& feature) {
        return clip_edge(clipped_edge, feature, feature->get_neighbours());
    }

    bool VClip::post_clip_derivative_update(const ClipData& clip_data, float derivative_l, float derivative_h)
    {
        if (clip_data.neighbour_l != nullptr && derivative_l > 0){
            // Update secondary to neighbour_l
            set_primary_feature(clip_data.neighbour_l);
            return true;
        }
        if (clip_data.neighbour_h != nullptr && derivative_h < 0){
            // Update secondary to neighbour_h
            set_primary_feature(clip_data.neighbour_h);
            return true;
        }
        return false;
    }

    std::optional<float> VClip::distance_derivative_sign(const glm::vec3 edge_point, const HalfEdgeP& edge, const VertexP& vertex)
    {
        // sign(D'_{E,X}(lambda) = sign(u * (e(lambda) - v)))
        //  - u = direction of the edge
        //  - e(lambda) = point on the edge (in clip_data we have point_l and point_h)
        //  - v = point on the feature
        if (edge_point == vertex->position){
            return std::nullopt;
        }

        return dot(edge->get_direction(), edge_point - vertex->position);
    }

    std::optional<float> VClip::distance_derivative_sign(const glm::vec3 edge_point, const HalfEdgeP& edge, const FaceP& face)
    {
        // sign(D'_{F,X}(lambda) = { +sign(u * n) if distance of p to plane is >0 else -sign(u * n) }
        //  - u = direction of the edge
        //  - n = normal of the face
        //  - p = point on the edge (in clip_data we have point_l and point_h)
        const auto dot_result = dot(edge->get_direction(), face->plane.normal);
        const auto distance = face->plane.distance_to(edge_point);

        if (distance == 0) {
            return std::nullopt;
        }

        return distance > 0 ? dot_result : -dot_result;
    }

    /**
     *
     * @param clip_data
     * @return true, if the primary feature was updated
     */
    bool VClip::post_clip_derivative_check(const ClipData& clip_data)
    {
        const auto feature = primary_feature();
        const auto half_edge = secondary_feature<HalfEdge>();

        const auto is_feature_edge = static_cast<bool>(std::dynamic_pointer_cast<HalfEdge>(feature));

        // If the primary feature is an edge we check the derivative against the appropriate neighbours
        // Page 10 of the original paper
        const std::optional<float> derivative_l = distance_derivative_sign(clip_data.point_l, half_edge, is_feature_edge ? clip_data.neighbour_l : feature);
        const std::optional<float> derivative_h = distance_derivative_sign(clip_data.point_h, half_edge, is_feature_edge ? clip_data.neighbour_h : feature);

        if (!derivative_l.has_value() || !derivative_h.has_value()){
            return false; // Degenerate case
        }

        return post_clip_derivative_update(clip_data, derivative_l.value(), derivative_h.value());
    }

    VClipState VClip::handle_local_minimum()
    {
        auto d_max = std::numeric_limits<float>::min();
        FaceP face_max = nullptr;

        const auto face = get_feature<Face>();
        const auto vertex = get_feature<Vertex>();

        // For all faces F' on F's polyhedron
        for (const auto& f : face->polyhedron->faces){
            const auto d = f->plane.distance_to(vertex->position);
            if (d > d_max){
                d_max = d;
                face_max = f;
            }
        }

        if (d_max <= 0) {
            return PENETRATION;
        }

        set_primary_feature(face_max);
        return CONTINUE;
    }

    VertexP VClip::closest_face_vertex_to_edge(const HalfEdgeP& edge, const FaceP& face) {
        const auto face_edges = face->get_neighbours<HalfEdge>();
        const auto face_vertices = std::ranges::transform_view(face_edges, [](const HalfEdgeP& face_edge) {
            return face_edge->start;
        });

        // Find the closest vertex to the edge
        VertexP closest_vertex = nullptr;
        auto min_distance = std::numeric_limits<float>::max();

        for (const auto& vertex : face_vertices){
            const auto clip_data = clip_edge(edge, vertex);
            if (clip_data.is_clipped) {
                const auto distance_l = distance(clip_data.point_l, vertex->position);
                const auto distance_h = distance(clip_data.point_h, vertex->position);

                if (distance_l < min_distance){
                    min_distance = distance_l;
                    closest_vertex = vertex;
                }
                if (distance_h < min_distance){
                    min_distance = distance_h;
                    closest_vertex = vertex;
                }
            }
        }

        return closest_vertex;
    }

    /**************
     *   STATES   *
     **************/
    VClipState VClip::execute_vertex_vertex() {
        // Test, whether the two vertices are in each other's voronoi region
        const auto vertex_1 = primary_feature<Vertex>();
        const auto vertex_2 = secondary_feature<Vertex>();

        const auto [in_region_1, plane_1] = Voronoi::in_voronoi_region(vertex_1, vertex_2->position);
        if (!in_region_1 && plane_1.has_value()){
            set_primary_feature(plane_1.value().edge);
            return CONTINUE;
        }

        const auto [in_region_2, plane_2] = Voronoi::in_voronoi_region(vertex_2, vertex_1->position);
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
        const auto [in_region, plane] = Voronoi::in_voronoi_region(edge, vertex->position);
        if (!in_region && plane.has_value()){
            // Violated plane exists
            set_feature<HalfEdge>(plane.value().feature);
            return CONTINUE;
        }
        // Clip E against V
        auto updated = false;
        const auto clip_data = clip_edge(edge, vertex);
        if (clip_data.neighbour_l != nullptr && clip_data.neighbour_h != clip_data.neighbour_l){
            set_feature<Vertex>(clip_data.neighbour_l);
            updated = true;
        }
        else {
            // TODO: Investigate if the flip does something
            set_feature_as_primary<Vertex>();
            updated = post_clip_derivative_check(clip_data);
        }

        if (updated){
            return CONTINUE;
        }

        return DONE;
    }

    VClipState VClip::execute_vertex_face() {
        const auto vertex = get_feature<Vertex>();
        const auto face = get_feature<Face>();

        // Find a voronoi plane VP(F, E) that V MAXIMALLY violates
        const auto plane = Voronoi::find_maximally_violating_voronoi_plane(face, vertex->position);
        if (plane.has_value()){
            set_feature<Face>(plane.value().edge);
            return CONTINUE;
        }

        // Look for an edge (incident to V) which endpoints satisfy: |D_p(V)| > |D_p(V')|
        // where V' is the other vertex of the incident edge and D_p is the distance to the face
        HalfEdgeP good_edge = nullptr;
        for (const auto& edge : vertex->edges){
            // TODO: Check if the other vertex is actually the other vertex
            const auto other_vertex = edge->end;
            const auto distance_vertex = face->plane.distance_to(vertex->position);
            const auto distance_other = face->plane.distance_to(other_vertex->position);

            if (std::abs(distance_vertex) > std::abs(distance_other)){
                good_edge = edge;
                break;
            }
        }

        if (good_edge != nullptr){
            set_feature<Vertex>(good_edge);
            return CONTINUE;
        }

        if (face->plane.is_above(vertex->position)){
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
            if (clip_data_vertex.neighbour_l != nullptr && clip_data_vertex.neighbour_l == clip_data_vertex.neighbour_h){
                if (set_primary){
                    set_primary_feature(clip_data_vertex.neighbour_l);
                } else {
                    set_secondary_feature(clip_data_vertex.neighbour_l);
                }
                return CONTINUE;
            }

            // Check derivatives
            if (post_clip_derivative_check(clip_data_vertex)){
                return CONTINUE;
            }

            // Clip the edge_2 against the edge_1's face-edge planes
            const auto clip_data_face = clip_edge(other_edge, active_edge, active_edge->get_neighbour_faces(), clip_data_vertex);
            // If the edge_2 is completely beneath one of the face-edge planes
            if (clip_data_face.neighbour_l != nullptr && clip_data_face.neighbour_l == clip_data_face.neighbour_h){
                if (set_primary){
                    set_primary_feature(clip_data_vertex.neighbour_l);
                } else {
                    set_secondary_feature(clip_data_vertex.neighbour_l);
                }
                return CONTINUE;
            }

            // Check derivatives
            if (post_clip_derivative_check(clip_data_face)){
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
            const auto closest_vertex = closest_face_vertex_to_edge(edge, face);
            set_feature<Face>(closest_vertex);
            return CONTINUE;
        }

        // Find the distances of clipped points to the face
        const auto distance_l = face->plane.distance_to(clip_data.point_l);
        const auto distance_h = face->plane.distance_to(clip_data.point_h);

        // If the sign of the distances is different we have a penetration
        if (distance_l * distance_h <= 0){
            return PENETRATION;
        }

        const auto derivative = distance_derivative_sign(clip_data.point_l, edge, face);
        if (!derivative.has_value()){
            std::cerr << "Degenerate case" << std::endl;
            return ERROR;
        }
        if (derivative.value() > 0){
            if (clip_data.neighbour_l != nullptr){
                set_feature<Face>(clip_data.neighbour_l);
            } else {
                set_feature<HalfEdge>(edge->end);
            }
        } else {
            if (clip_data.neighbour_h != nullptr){
                set_feature<Face>(clip_data.neighbour_h);
            } else {
                set_feature<HalfEdge>(edge->start);
            }
        }

        return CONTINUE;
    }

    VClipState VClip::initialize() {
        // Check if the colliders are valid
        if (!collider_1->is_valid() && !collider_2->is_valid()){
            std::cerr << "Invalid colliders" << std::endl;
            return ERROR;
        }

        // Choose the two features randomly (first vertex)
        // TODO: Temporal coherence
        feature_1 = collider_1->get_shape()->vertices[0];
        feature_2 = collider_2->get_shape()->vertices[0];

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
        return ERROR;
    }

    std::optional<CollisionData> VClip::get_collision_data() {
        while (state == CONTINUE || state == INIT){
            state = step();
        }

        if (state == ERROR){
            return std::nullopt;
        }

        return CollisionData{};
    }

    void VClip::reset() {
        state = INIT;
        feature_1 = nullptr;
        feature_2 = nullptr;
    }
}
