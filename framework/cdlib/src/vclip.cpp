#include "vclip.hpp"

#include "voronoi.hpp"

namespace cdlib
{
    ClipData clip_edge(const std::shared_ptr<HalfEdge>& edge, const std::shared_ptr<Feature>& feature) {
        auto is_clipped = true;
        float lambda_l = 0;
        float lambda_h = 1;
        std::shared_ptr<Feature> neighbour_l = nullptr;
        std::shared_ptr<Feature> neighbour_h = nullptr;

        // The original paper calculates the points in reverse order
        const auto head = edge->end->position;
        const auto tail = edge->start->position;

        // For all voronoi planes of the feature
        for (const auto& neighbour : feature->get_neighbours()){
            const auto voronoi_plane = Voronoi::get_voronoi_plane_safe(feature, neighbour);
            if (!voronoi_plane){
                continue;
            }

            // Get the distance of the edge points to the plane
            const float distance_head = voronoi_plane->distance_to(head);
            const float distance_tail = voronoi_plane->distance_to(tail);

            // Case 1: Both the points are below the plane => no intersection, the neighbouring feature is 'closer'
            if (distance_head < 0 && distance_tail < 0){
                is_clipped = false;
                neighbour_l = neighbour;
                neighbour_h = neighbour;
                break;
            }

            // Case 2: Tail is below the plane, head is above the plane
            if (distance_head >= 0 && distance_tail < 0){
                const float lambda = distance_tail / (distance_tail - distance_head);
                if (lambda > lambda_l){
                    lambda_l = lambda;
                    neighbour_l = neighbour;
                    // Check if the lower bound is greater than the upper bound
                    if (lambda_l > lambda_h){
                        is_clipped = false;
                        break;
                    }
                }
            }

            // Case 3: Head is below the plane, tail is above the plane
            else if (distance_head < 0 && distance_tail >= 0){
                const float lambda = distance_tail / (distance_tail - distance_head);
                if (lambda < lambda_h){
                    lambda_h = lambda;
                    neighbour_h = neighbour;
                    // Check if the lower bound is greater than the upper bound
                    if (lambda_l > lambda_h){
                        is_clipped = false;
                        break;
                    }
                }
            }
        }

        return {head, tail, is_clipped, lambda_l, lambda_h, neighbour_l, neighbour_h};
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

        const auto face = primary_feature<Face>();
        const auto vertex = secondary_feature<Vertex>();

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

        
    }

    VClipState VClip::execute_edge_face() {
        return VClipState::CONTINUE;
    }

    std::optional<CollisionData> VClip::get_collision_data() {
        return std::nullopt;
    }
}
