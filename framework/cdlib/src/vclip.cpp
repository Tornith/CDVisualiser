#include "vclip.hpp"

#include "voronoi.hpp"

namespace cdlib
{
    ClipData clip_edge(const std::shared_ptr<HalfEdge>& clipped_edge, const std::shared_ptr<Feature>& feature) {
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

    /**
     *
     * @param clip_data
     * @return true, if the primary feature was updated
     */
    bool VClip::post_clip_derivative_check(const ClipData& clip_data)
    {
        // Page 10 of the original paper
        const auto half_edge = secondary_feature<HalfEdge>();

        // If the primary feature is a vertex
        // sign(D'_{E,X}(lambda) = sign(u * (e(lambda) - v)))
        //  - u = direction of the edge
        //  - e(lambda) = point on the edge (in clip_data we have point_l and point_h)
        //  - v = point on the feature
        const auto vertex_check = [](const glm::vec3 edge_direction, const glm::vec3 edge_point, const glm::vec3 feature_point) {
            return dot(edge_direction, edge_point - feature_point);
        };

        // If the primary feature is a face
        // sign(D'_{F,X}(lambda) = { +sign(u * n) if distance of p to plane is >0 else -sign(u * n) }
        //  - u = direction of the edge
        //  - n = normal of the face
        //  - p = point on the edge (in clip_data we have point_l and point_h)
        auto face_check = [](const glm::vec3 edge_direction, const Plane plane, const glm::vec3 point) {
            const auto dot_result = dot(edge_direction, plane.normal);
            const auto distance = plane.distance_to(point);
            return distance > 0 ? dot_result : -dot_result;
        };

        if (const auto vertex = primary_feature<Vertex>()){
            const auto direction = half_edge->get_direction();
            const auto feature_point = vertex->position;

            const auto derivative_l = vertex_check(direction, clip_data.point_l, feature_point);
            const auto derivative_h = vertex_check(direction, clip_data.point_h, feature_point);

            if (derivative_l == 0|| derivative_h == 0){
                return false; // Degenerate case
            }
            return post_clip_derivative_update(clip_data, derivative_l, derivative_h);
        }

        if (const auto face = primary_feature<Face>()){
            const auto direction = half_edge->get_direction();
            const auto normal = face->plane.normal;

            // TODO: Degenerate case
            const auto derivative_l = face_check(direction, face->plane, clip_data.point_l);
            const auto derivative_h = face_check(direction, face->plane, clip_data.point_h);

            return post_clip_derivative_update(clip_data, derivative_l, derivative_h);
        }

        if (const auto edge = primary_feature<HalfEdge>()){
            const auto direction = half_edge->get_direction();

            auto derivative_l = 0.0f;
            auto derivative_h = 0.0f;

            if (const auto vertex_l = std::dynamic_pointer_cast<Vertex>(clip_data.neighbour_l))
            {
                derivative_l = vertex_check(direction, clip_data.point_l, vertex_l->position);
            }

            if (const auto vertex_h = std::dynamic_pointer_cast<Vertex>(clip_data.neighbour_h))
            {
                derivative_h = vertex_check(direction, clip_data.point_h, vertex_h->position);
            }

            if (const auto face_l = std::dynamic_pointer_cast<Face>(clip_data.neighbour_l))
            {
                derivative_l = face_check(direction, face_l->plane, clip_data.point_l);
            }

            if (const auto face_h = std::dynamic_pointer_cast<Face>(clip_data.neighbour_h))
            {
                derivative_h = face_check(direction, face_h->plane, clip_data.point_h);
            }

            if (derivative_l == 0 || derivative_h == 0){
                return false; // Degenerate case
            }

            return post_clip_derivative_update(clip_data, derivative_l, derivative_h);
        }
        return false;
    }

    VClipState VClip::handle_local_minimum()
    {
        auto d_max = std::numeric_limits<float>::min();
        std::shared_ptr<Face> face_max = nullptr;

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

    std::shared_ptr<Feature> VClip::closest_face_vertex_to_edge(const std::shared_ptr<HalfEdge>& edge, const std::shared_ptr<Face>& face) {
        const auto face_edges = face->get_neighbours<HalfEdge>();

        std::shared_ptr<Feature> closest_feature = nullptr;
        auto min_distance = std::numeric_limits<float>::max();

        for (const auto& face_edge : face_edges){

        }
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
        std::shared_ptr<HalfEdge> good_edge = nullptr;
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

        const auto clip_against_edge = [this](const std::shared_ptr<HalfEdge>& active_edge, const std::shared_ptr<HalfEdge>& other_edge, const auto set_function) {
            // Clip the edge_2 against the edge_1's vertex-edge planes
            const auto clip_data_vertex = clip_edge(other_edge, active_edge, active_edge->get_neighbour_vertices());
            // If the edge_2 is completely beneath one of the vertex-edge planes
            if (clip_data_vertex.neighbour_l != nullptr && clip_data_vertex.neighbour_l == clip_data_vertex.neighbour_h){
                set_function(clip_data_vertex.neighbour_l);
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
                set_function(clip_data_face.neighbour_l);
                return CONTINUE;
            }

            // Check derivatives
            if (post_clip_derivative_check(clip_data_face)){
                return CONTINUE;
            }
            return DONE;
        };

        const auto result_1 = clip_against_edge(edge_1, edge_2, set_primary_feature);
        if (result_1 != DONE) {
            return result_1;
        }

        const auto result_2 = clip_against_edge(edge_2, edge_1, set_secondary_feature);
        return result_2;
    }

    VClipState VClip::execute_edge_face() {
        const auto edge = get_feature<HalfEdge>();
        const auto face = get_feature<Face>();

        // Clip the edge against the face
        const auto clip_data = clip_edge(edge, face);

        // If the edge is excluded
        if (!clip_data.is_clipped) {

        }
    }

    std::optional<CollisionData> VClip::get_collision_data() {
        return std::nullopt;
    }
}
