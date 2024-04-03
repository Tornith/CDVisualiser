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

        return {is_clipped, lambda_l, lambda_h, neighbour_l, neighbour_h};
    }

    void post_clip_derivative_check(const ClipData& clip_data, const std::shared_ptr<HalfEdge>& edge, const std::shared_ptr<Feature>& feature)
    {

    }
}