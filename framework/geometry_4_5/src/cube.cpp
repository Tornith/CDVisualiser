#include "cube.hpp"

/**
 * Updates the position of vertices of this cube.
 *
 * @param 	left_top_front	  	The left top front point.
 * @param 	right_top_front   	The right top front point.
 * @param 	left_bottom_front 	The left bottom front point.
 * @param 	right_bottom_front	The right bottom front point.
 * @param 	left_top_back	  	The left top back point.
 * @param 	right_top_back	  	The right top back point.
 * @param 	left_bottom_back  	The left bottom back point.
 * @param 	right_bottom_back 	The right bottom back point.
 */
void Cube::update(const glm::vec3& left_top_front, const glm::vec3& right_top_front, const glm::vec3& left_bottom_front,
            const glm::vec3& right_bottom_front, const glm::vec3& left_top_back, const glm::vec3& right_top_back,
            const glm::vec3& left_bottom_back, const glm::vec3& right_bottom_back) {
    std::array<float, cube_vertices_count* 14> vertices =
        generate_custom_cube_vertices(left_top_front, right_top_front, left_bottom_front, right_bottom_front, left_top_back, right_top_back,
                                      left_bottom_back, right_bottom_back);
    glNamedBufferSubData(vertex_buffer, 0, cube_vertices_count * sizeof(float) * 14, vertices.data());
}