#include "application.hpp"

#include <random>
#include <ranges>
#include <set>
#include <utility>

#include "utils.hpp"

#include "glm/gtx/hash.hpp"

Application::Application(int initial_width, int initial_height, std::vector<std::string> arguments) : PV227Application(initial_width, initial_height, std::move(arguments)) {
    Application::compile_shaders();
    prepare_cameras();
    prepare_textures();
    prepare_lights();
    prepare_scene();
}

Application::~Application() = default;


// ----------------------------------------------------------------------------
// Preparation methods
// ----------------------------------------------------------------------------

void Application::compile_shaders() {
    default_unlit_program = ShaderProgram(lecture_shaders_path / "object.vert", lecture_shaders_path / "unlit.frag");
    default_lit_program = ShaderProgram(lecture_shaders_path / "object.vert", lecture_shaders_path / "lit.frag");

    std::cout << "Shaders are reloaded." << std::endl;
}

void Application::prepare_cameras() {
    // Sets the default camera position.
    camera.set_eye_position(glm::radians(-45.f), glm::radians(20.f), 25.f);
    // Computes the projection matrix.
    auto aspect = static_cast<float>(this->width) / static_cast<float>(this->height);
    camera_ubo.set_projection(glm::perspective(glm::radians(45.f), aspect, 0.1f, 1000.0f));
    camera_ubo.update_opengl_data();
}

void Application::prepare_textures() {

}

void Application::prepare_lights() {
    // The rest is set in the update scene method.
    phong_lights_ubo.set_global_ambient(glm::vec3(0.0f));
}

void Application::prepare_scene() {
    prepare_convex_objects();

    world_axis_x = SceneObject{create_line_geometry(glm::vec3(-1000.0f, 0.0f, 0.0f), glm::vec3(1000.0f, 0.0f, 0.0f)), ModelUBO(glm::mat4(1.0f)), red_material_ubo};
    world_axis_y = SceneObject{create_line_geometry(glm::vec3(0.0f, -1000.0f, 0.0f), glm::vec3(0.0f, 1000.0f, 0.0f)), ModelUBO(glm::mat4(1.0f)), green_material_ubo};
    world_axis_z = SceneObject{create_line_geometry(glm::vec3(0.0f, 0.0f, -1000.0f), glm::vec3(0.0f, 0.0f, 1000.0f)), ModelUBO(glm::mat4(1.0f)), blue_material_ubo};
}

void Application::prepare_convex_objects() {
    auto [g1, v1] = generate_convex_hull_geometry(random_points(object_seed_1));
    auto [g2, v2] = generate_convex_hull_geometry(random_points(object_seed_2));

    convex_geometry_1 = std::move(g1);
    convex_geometry_2 = std::move(g2);

    scene_objects.clear();

    convex_object_1 = {convex_geometry_1, ModelUBO(glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.0f, -4.0f * object_distance))), red_material_ubo};
    convex_object_2 = {convex_geometry_2, ModelUBO(glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.0f, 4.0f * object_distance))), green_material_ubo};

    scene_objects.push_back(convex_object_1);
    scene_objects.push_back(convex_object_2);

    convex_object_vertices_1 = std::move(v1);
    convex_object_vertices_2 = std::move(v2);

    recalculate_positions();
    recalculate_minkowski_difference();

    scene_objects.push_back(minkowski_difference);

    gjk = cdlib::SteppableGJKEPA(convex_mesh_1.get(), convex_mesh_2.get());
}




Geometry Application::create_line_geometry(const glm::vec3&from, const glm::vec3&to) {
    const std::vector vertices = {
        from.x, from.y, from.z,  0.0f, 0.0f, 0.0f,  0.5f, 0.5f,  0.0f, 0.0f, 0.0f,  0.0f, 0.0f, 0.0f,
        to.x,   to.y,   to.z,    0.0f, 0.0f, 0.0f,  0.5f, 0.5f,  0.0f, 0.0f, 0.0f,  0.0f, 0.0f, 0.0f
    };
    const std::vector<uint32_t> indices = {0, 1};
    return Geometry{GL_LINES, 14, 2, vertices.data(), 2, indices.data()};
}

Geometry Application::create_line_geometry(const glm::vec3&origin, const glm::vec3&direction, float length) {
    return create_line_geometry(origin, origin + glm::normalize(direction) * length);
}


void Application::recalculate_positions() {
    // If the convex meshes are not initialized, initialize them
    if (!convex_mesh_1) {
        convex_mesh_1 = std::make_shared<ConvexCollider>();
    }
    if (!convex_mesh_2) {
        convex_mesh_2 = std::make_shared<ConvexCollider>();
    }

    // If the positions are not initialized, initialize them
    convex_object_positions_1.clear();
    convex_object_positions_2.clear();

    // Reserve space for the vertices
    convex_object_positions_1.reserve(convex_object_vertices_1.size() / 14);
    convex_object_positions_2.reserve(convex_object_vertices_2.size() / 14);

    auto pos_1 =  get_positions_from_buffer(convex_object_vertices_1);
    auto pos_2 = get_positions_from_buffer(convex_object_vertices_2);

    // Apply the model matrix to the vertices
    for (auto& pos : pos_1) {
        convex_object_positions_1.emplace_back(convex_object_1.get_model_ubo().get_matrix() * glm::vec4(pos, 1.0f));
    }
    for (auto& pos : pos_2) {
        convex_object_positions_2.emplace_back((convex_object_2.get_model_ubo().get_matrix() * glm::vec4(pos, 1.0f)));
    }

    // Set the new positions to the convex meshes
    convex_mesh_1->set_vertices(convex_object_positions_1);
    convex_mesh_2->set_vertices(convex_object_positions_2);
}

void Application::recalculate_minkowski_difference() {
    minkowski_difference_positions.clear();

    auto pos_minkowski = get_minkowski_difference_positions(convex_object_positions_1, convex_object_positions_2);
    minkowski_difference_positions.reserve(pos_minkowski.size());
    for (auto& pos : pos_minkowski) {
        minkowski_difference_positions.emplace_back(pos);
    }

    // Convert from glm::vec3 to std::array<float, 3>
    std::vector<std::array<float, 3>> minkowski_difference_positions_array;
    for (const auto& pos : minkowski_difference_positions) {
        minkowski_difference_positions_array.push_back({pos.x, pos.y, pos.z});
    }

    // Create the minkowski difference geometry
    auto [gm, vm] = generate_convex_hull_geometry(minkowski_difference_positions_array);
    minkowski_difference_geometry = std::move(gm);
    minkowski_difference_vertices = std::move(vm);

    minkowski_difference = {minkowski_difference_geometry, ModelUBO(glm::mat4(1.0f)), yellow_material_ubo};
}

std::vector<std::array<float, 3>> Application::random_points(int seed) {
    // Generate three lists of random X, Y and Z coordinates between [-1, 1]
    std::vector<std::array<float, 3>> points;

    std::mt19937 gen(seed);
    std::uniform_real_distribution dis(-1.0f, 1.0f);

    const int num_points = 15;
    for (int i = 0; i < num_points; i++) {
        points.push_back({dis(gen), dis(gen), dis(gen)});
    }

    return points;
}

std::pair<Geometry, std::vector<float>> Application::generate_convex_hull_geometry(const std::vector<std::array<float, 3>>& points) {
    quickhull::QuickHull<float> qh;

    // Convert points to std::vector<quickhull::Vector3<float>>
    std::vector<quickhull::Vector3<float>> points_vector;
    for (const auto& point : points) {
        points_vector.emplace_back(point[0], point[1], point[2]);
    }

    auto hull = qh.getConvexHull(points_vector, false, false);

    const auto& indexBuffer = hull.getIndexBuffer();
    const auto& vertexBuffer = hull.getVertexBuffer();

    // A map of indices to all normals for that index
    std::unordered_map<uint32_t, std::vector<glm::vec3>> normals;

    // Calculate normals for each triangle
    for (size_t i = 0; i < indexBuffer.size(); i += 3) {
        // Calculate the normal for the current triangle
        auto normal = glm::normalize(glm::cross(
            glm::vec3(vertexBuffer[indexBuffer[i + 1]]) - glm::vec3(vertexBuffer[indexBuffer[i]]),
            glm::vec3(vertexBuffer[indexBuffer[i + 2]]) - glm::vec3(vertexBuffer[indexBuffer[i]])
        ));

        // Add the normal to the map for each index
        for (size_t j = 0; j < 3; j++) {
            normals[indexBuffer[i + j]].push_back(normal);
        }
    }

    std::vector<float> vertices;
    for (size_t i = 0; i < vertexBuffer.size(); i++) {
        // Positions
        vertices.push_back(vertexBuffer[i].x);
        vertices.push_back(vertexBuffer[i].y);
        vertices.push_back(vertexBuffer[i].z);

        // Normals
        glm::vec3 normal(0.0f);
        for (const auto& n : normals[i]) {
            normal += n;
        }
        normal = glm::normalize(normal);
        vertices.push_back(normal.x);
        vertices.push_back(normal.y);
        vertices.push_back(normal.z);

        // Texture coordinates
        vertices.push_back(0.5f);
        vertices.push_back(0.5f);

        // Tangent
        vertices.push_back(0.0f);
        vertices.push_back(0.0f);
        vertices.push_back(0.0f);

        // Bitangent
        vertices.push_back(0.0f);
        vertices.push_back(0.0f);
        vertices.push_back(0.0f);
    }

    std::vector<uint32_t> indexBuffer32;
    for (const auto& index : indexBuffer) {
        indexBuffer32.push_back(static_cast<uint32_t>(index));
    }

    return {Geometry{GL_TRIANGLES, 14, static_cast<int>(vertices.size() / 14), vertices.data(), static_cast<int>(indexBuffer32.size()), indexBuffer32.data()}, vertices};
}

std::vector<glm::vec3> Application::get_positions_from_buffer(const std::vector<float>& buffer) {
    std::vector<glm::vec3> positions;
    for (size_t i = 0; i < buffer.size(); i += 14) {
        positions.emplace_back(buffer[i], buffer[i + 1], buffer[i + 2]);
    }
    return positions;
}

std::vector<glm::vec3> Application::get_minkowski_difference_positions(const std::vector<glm::vec3>&positions_1, const std::vector<glm::vec3>&positions_2) {
    std::vector<glm::vec3> minkowski_difference;
    for (const auto& pos_1 : positions_1) {
        for (const auto& pos_2 : positions_2) {
            auto difference = pos_1 - pos_2;

            // If another point, that is very close to the current one, is found, use the index of the current one
            if (auto result = std::ranges::find_if(minkowski_difference,
                [&difference](const auto& v) {
                    // If the distance to the current point is less than 0.0001, the point is considered the same
                        return glm::distance(v, difference) < 0.0001f;
                }); result == std::end(minkowski_difference)) {
                minkowski_difference.push_back(difference);
            }
        }
    }
    return minkowski_difference;
}



// ----------------------------------------------------------------------------
// Update
// ----------------------------------------------------------------------------
void Application::update(float delta) {
    PV227Application::update(delta);

    // Updates the main camera.
    const glm::vec3 eye_position = camera.get_eye_position();
    camera_ubo.set_view(glm::lookAt(eye_position, glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f)));
    camera_ubo.update_opengl_data();

    // Computes the light position.
    const glm::vec3 light_position =
        glm::vec3(15, 15, 15) * glm::vec3(cosf(light_position_rad / 6.0f) * sinf(light_position_rad), sinf(light_position_rad / 6.0f), cosf(light_position_rad / 6.0f) * cosf(light_position_rad));

    // Updates the OpenGL buffer storing the information about the light.
    phong_lights_ubo.clear();
    phong_lights_ubo.add(PhongLightData::CreateDirectionalLight(light_position, glm::vec3(0.75f), glm::vec3(0.9f), glm::vec3(1.0f)));
    phong_lights_ubo.update_opengl_data();

    if (abs(object_distance - last_object_distance) > 0.0001f) {
        last_object_distance = object_distance;
        recalculate_positions();
        recalculate_minkowski_difference();
    }
    update_object_positions(delta);
}

void Application::update_object_positions(float delta) {
    convex_object_1.get_model_ubo().set_matrix(glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.0f, -4.0f * object_distance)));
    convex_object_2.get_model_ubo().set_matrix(glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.0f, 4.0f * object_distance)));

    convex_object_1.get_model_ubo().update_opengl_data();
    convex_object_2.get_model_ubo().update_opengl_data();

    // Remove minkowski difference object and recalculate it
    scene_objects.pop_back();
    recalculate_minkowski_difference();
    scene_objects.push_back(minkowski_difference);
}

glm::vec3 get_position_from_buffer(std::vector<float> buffer, const size_t index) {
    return {buffer[index * 14], buffer[index * 14 + 1], buffer[index * 14 + 2]};
}

void Application::create_vertex_highlight_objects() {
    active_simplex_vertex_highlights.clear();
    simplex_vertex_highlights.clear();
    active_object_vertex_highlights.clear();
    object_vertex_highlights.clear();

    auto create_new_vertex_highlight_object = [this](const glm::vec3& position, const PhongMaterialUBO& material) {
        auto model_matrix = glm::translate(glm::mat4(1.0f), position);
        auto matrix = glm::scale(model_matrix, glm::vec3(0.05f));
        return std::move(SceneObject{sphere, ModelUBO(matrix), material});
    };

    auto current_point_a = gjk.get_current_point_a();
    auto current_point_b = gjk.get_current_point_b();
    auto current_new_point = gjk.get_current_new_point();

    auto simplex_1 = gjk.get_simplex_obj_1();
    auto simplex_2 = gjk.get_simplex_obj_2();
    auto simplex_minkowski = gjk.get_simplex();

    // Create highlights for new vertices
    active_object_vertex_highlights.push_back(create_new_vertex_highlight_object(current_point_a, blue_material_ubo));
    active_object_vertex_highlights.push_back(create_new_vertex_highlight_object(current_point_b, blue_material_ubo));
    active_simplex_vertex_highlights.push_back(create_new_vertex_highlight_object(current_new_point, magenta_material_ubo));

    // Create highlights for simplex vertices
    for (const auto& vertex : simplex_1) {
        object_vertex_highlights.push_back(create_new_vertex_highlight_object(vertex, yellow_material_ubo));
    }
    for (const auto& vertex : simplex_2) {
        object_vertex_highlights.push_back(create_new_vertex_highlight_object(vertex, yellow_material_ubo));
    }
    for (const auto& vertex : simplex_minkowski) {
        simplex_vertex_highlights.push_back(create_new_vertex_highlight_object(vertex, white_material_ubo));
    }
}

void Application::draw_direction_highlights() {
    direction_highlight_objects.clear();
    for (const auto& [direction, origin] : direction_highlights) {
        auto geometry = create_line_geometry(origin, direction, 1.0f);
        direction_highlight_objects.emplace_back(geometry, ModelUBO(glm::mat4(1.0f)), cyan_material_ubo);
    }
}

// ----------------------------------------------------------------------------
// Render
// ----------------------------------------------------------------------------
void Application::render() {
    // Starts measuring the elapsed time.
    glBeginQuery(GL_TIME_ELAPSED, render_time_query);

    // Binds the main window framebuffer.
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(0, 0, width, height);

    // Sets the clear values and clears the framebuffer.
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClearDepth(1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);

    // Render the axis
    default_unlit_program.use();
    // Set thickness of lines to 1
    glLineWidth(0.75f);
    render_object(world_axis_x, default_unlit_program);
    render_object(world_axis_y, default_unlit_program);
    render_object(world_axis_z, default_unlit_program);

    // Renders the scene.
    render_scene();

    // Resets the VAO and the program.
    glBindVertexArray(0);
    glUseProgram(0);

    // Stops measuring the elapsed time.
    glEndQuery(GL_TIME_ELAPSED);

    // Waits for OpenGL - don't forget OpenGL is asynchronous.
    glFinish();

    // Evaluates the query.
    GLuint64 render_time;
    glGetQueryObjectui64v(render_time_query, GL_QUERY_RESULT, &render_time);
    fps_gpu = 1000.f / (static_cast<float>(render_time) * 1e-6f);
}

void Application::render_scene() {
    camera_ubo.bind_buffer_base(CameraUBO::DEFAULT_CAMERA_BINDING);
    phong_lights_ubo.bind_buffer_base(PhongLightsUBO::DEFAULT_LIGHTS_BINDING);

    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);

    auto render_objects = [this]() {
        if (show_convex_objects) {
            render_object(convex_object_1, default_lit_program);
            render_object(convex_object_2, default_lit_program);
        }
        if (show_minkowski_difference) {
            render_object(minkowski_difference, default_lit_program);
        }
    };

    // If the wireframe mode is enabled, the wireframe is rendered.
    default_lit_program.use();
    if (show_wireframe) {
        // glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        // draw points
        glDisable(GL_CULL_FACE);
        glPointSize(5.0f);
        glPolygonMode(GL_FRONT_AND_BACK, GL_POINT);
        render_objects();
        glPointSize(1.0f);
        glLineWidth(1.f);
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        render_objects();
        glEnable(GL_CULL_FACE);
    } else {
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        render_objects();
    }

    // Render highlights
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    default_unlit_program.use();
    if (step_by_step) {
        for (SceneObject& object : active_simplex_vertex_highlights) {
            render_object(object, default_unlit_program);
        }
        for (SceneObject& object : simplex_vertex_highlights) {
            render_object(object, default_unlit_program);
        }
        if (show_convex_objects) {
            for (SceneObject& object : active_object_vertex_highlights) {
                render_object(object, default_unlit_program);
            }
            for (SceneObject& object : object_vertex_highlights) {
                render_object(object, default_unlit_program);
            }
        }
    }

    // Draw direction highlights
    glDisable(GL_CULL_FACE);
    glPointSize(2.5f);
    glPolygonMode(GL_FRONT_AND_BACK, GL_POINT);
    for (SceneObject& object : direction_highlight_objects) {
        render_object(object, default_unlit_program);
    }
    glPointSize(1.0f);
    glLineWidth(1.f);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    for (SceneObject& object : direction_highlight_objects) {
        render_object(object, default_unlit_program);
    }
    glEnable(GL_CULL_FACE);

    glDisable(GL_CULL_FACE);
}

void Application::render_object(SceneObject& object, ShaderProgram& program) {
    program.use();
    // Handles the textures.
    program.uniform("has_texture", object.has_texture());
    glBindTextureUnit(0, object.get_texture());
    // Binds the buffers with materials and model matrix.
    object.get_material().bind_buffer_base(PhongMaterialUBO::DEFAULT_MATERIAL_BINDING);
    object.get_model_ubo().bind_buffer_base(ModelUBO::DEFAULT_MODEL_BINDING);
    // Binds VAO and renders the object.
    object.get_geometry().bind_vao();
    object.get_geometry().draw();
}


// ----------------------------------------------------------------------------
// GUI
// ----------------------------------------------------------------------------
void Application::render_ui() {
    if (!show_gui) return;

    const float unit = ImGui::GetFontSize();

    ImGui::Begin("Settings", nullptr, ImGuiWindowFlags_NoDecoration);
    ImGui::SetWindowSize(ImVec2(25 * unit, 25 * unit));
    ImGui::SetWindowPos(ImVec2(2 * unit, 2 * unit));

    std::string fps_cpu_string = "FPS (CPU): ";
    ImGui::Text(fps_cpu_string.append(std::to_string(fps_cpu)).c_str());

    std::string fps_string = "FPS (GPU): ";
    ImGui::Text(fps_string.append(std::to_string(fps_gpu)).c_str());

    // Dropdown for selecting the collision method
    // const char* methods[2] = {"GJK+EPA", "V-Clip"};
    // Convert CollisionDetectionMethod enum to const char*
    const char* methods[4] = {"GJK+EPA", "V-Clip", "AABBTree", "Sweep-and-Prune"};
    // Treat &selected_method as int
    if (ImGui::Combo("Collision method", reinterpret_cast<int*>(&selected_method), methods, IM_ARRAYSIZE(methods))) {
        std::cout << "Selected method: " << selected_method << std::endl;
    }

    // Setters for object seeds
    ImGui::InputInt("Object 1 seed", &object_seed_1);
    ImGui::InputInt("Object 2 seed", &object_seed_2);

    // Regenerate objects
    if (object_seed_1 != last_object_seed_1 || object_seed_2 != last_object_seed_2) {
        last_object_seed_1 = object_seed_1;
        last_object_seed_2 = object_seed_2;
        prepare_convex_objects();
    }

    // Slider for moving the objects closer together
    ImGui::SliderFloat("Object distance", &object_distance, 1.0f, 0.0f);

    ImGui::Spacing();

    ImGui::Checkbox("Show minkowski difference", &show_minkowski_difference);
    ImGui::Checkbox("Show convex objects", &show_convex_objects);

    ImGui::Spacing();

    ImGui::Checkbox("Wireframe", &show_wireframe);

    ImGui::Spacing();

    ImGui::Checkbox("Auto-calculate collision", &auto_calculate_collision);

    if (!auto_calculate_collision) {
        // Button for manual collision calculation
        if (ImGui::Button("Calculate collision")) {
            // auto result = gjk.is_colliding();
            // std::cout << "Collision: " << result << std::endl;
            gjk.evaluate();
            auto result = gjk.get_collision_data();
            std::cout << "Collision: " << result.has_value() << std::endl;
            if (result.has_value()) {
                auto result_data = gjk.get_collision_data().value();
                std::cout << " - Normal: " << result_data.normal.x << " " << result_data.normal.y << " " << result_data.normal.z << std::endl;
                std::cout << " - Depth: " << result_data.depth << std::endl;

                // Draw the collision normal from origin
                direction_highlights.emplace_back(result_data.normal * result_data.depth, glm::vec3(0.0f));
            }
        }

        // Test iterate buttons that iterate over the vertices of the convex objects
        ImGui::Text("Iterate over vertices:");
        ImGui::Indent(10.0f);
        if (!step_by_step) {
            if (ImGui::Button("Start step-by-step")) {
                step_by_step = true;
                gjk.step_by_step_init();
            }
        } else {
            // ImGui::Text("Current vertex: %d / %d", current_vertex, max_vertex);
            if (ImGui::Button("Next step")) {
                gjk_step_visualize(gjk.step_by_step_next());
            }
            if (ImGui::Button("Cancel step-by-step")) {
                step_by_step = false;
                // Clear the highlights
                active_simplex_vertex_highlights.clear();
                simplex_vertex_highlights.clear();
                active_object_vertex_highlights.clear();
                object_vertex_highlights.clear();
                direction_highlights.clear();
            }
        }
    }
    else {
        direction_highlights.clear();
        direction_highlight_objects.clear();
        gjk.evaluate();
        auto result = gjk.get_collision_data();

        if (result.has_value()) {
            auto result_data = gjk.get_collision_data().value();

            ImGui::Text("The objects are: colliding");

            ImGui::Text("Collision normal: %f %f %f", result_data.normal.x, result_data.normal.y, result_data.normal.z);
            ImGui::Text("Collision depth: %f", result_data.depth);

            // Draw the collision normal from origin
            direction_highlights.emplace_back(result_data.normal * result_data.depth, glm::vec3(0.0f));
            draw_direction_highlights();
        } else {
            ImGui::Text("The objects are: not colliding");
        }
    }

    ImGui::End();
}

void Application::gjk_step_visualize(cdlib::SteppableGJKState state) {
    create_vertex_highlight_objects();
    if (state == cdlib::SteppableGJKState::UNDEFINED) {
        step_by_step = false;
        std::cerr << "Undefined state" << std::endl;
    }
    else if (state == cdlib::SteppableGJKState::FINISHED) {
        std::cout << "Finished with result: " << gjk.is_colliding() << std::endl;
        step_by_step = false;
    }
    else if (state == cdlib::SteppableGJKState::UNINITIALIZED) {
        std::cout << "Initializing simplex with direction" << std::endl;
        std::cout << " - Direction: " << gjk.get_direction().x << " " << gjk.get_direction().y << " " << gjk.get_direction().z << std::endl;
        // Create the line from the origin to the direction
        direction_highlights.emplace_back(gjk.get_direction(), glm::vec3(0.0f));
    }
    else if (state == cdlib::SteppableGJKState::ITERATION_1) {
        direction_highlights.clear();
        std::cout << "Iteration 1" << std::endl;
        std::cout << " - New points:" << std::endl;
        std::cout << "   - A:   " << gjk.get_current_point_a().x << " " << gjk.get_current_point_a().y << " " << gjk.get_current_point_a().z << std::endl;
        std::cout << "   - B:   " << gjk.get_current_point_b().x << " " << gjk.get_current_point_b().y << " " << gjk.get_current_point_b().z << std::endl;
        std::cout << "   - New: " << gjk.get_current_new_point().x << " " << gjk.get_current_new_point().y << " " << gjk.get_current_new_point().z << std::endl;
        if (gjk.get_current_state() == cdlib::SteppableGJKState::ITERATION_1) { // i.e. the next iteration is also iteration 1
            std::cout << " - Simplex too small to continue:" << std::endl;
            std::cout << " - New simplex:" << std::endl;
            for (int i = 0; i < gjk.get_simplex().size(); i++) {
                std::cout << "   - " << i << ": " << gjk.get_simplex()[i].x << " " << gjk.get_simplex()[i].y << " " << gjk.get_simplex()[i].z << std::endl;
            }
            std::cout << " - New Direction: " << gjk.get_direction().x << " " << gjk.get_direction().y << " " << gjk.get_direction().z << std::endl;

            // Average the points in the simplex
            auto average = glm::vec3(0.0f);
            for (const auto& point : gjk.get_simplex()) {
                average += point;
            }
            average /= static_cast<float>(gjk.get_simplex().size());
            // Create a new direction from the average to the origin
            direction_highlights.emplace_back(gjk.get_direction(), average);
        }
    }
    else if (state == cdlib::SteppableGJKState::ITERATION_2) {
        direction_highlights.clear();
        std::cout << "Iteration 2" << std::endl;
        std::cout << " - Sanity check: " << (gjk.get_gjk_finished() ? "Failed, stopped" : "Passed, continuing...") << std::endl;
    }
    else if (state == cdlib::SteppableGJKState::ITERATION_3) {
        std::cout << "Iteration 3" << std::endl;
        std::cout << " - New simplex:" << std::endl;
        for (int i = 0; i < gjk.get_simplex().size(); i++) {
            std::cout << "   - " << i << ": " << gjk.get_simplex()[i].x << " " << gjk.get_simplex()[i].y << " " << gjk.get_simplex()[i].z << std::endl;
        }
    }
    else if (state == cdlib::SteppableGJKState::ITERATION_4) {
        std::cout << "Iteration 4" << std::endl;
        std::cout << " - Origin in simplex check: " << (gjk.get_gjk_finished() ? "Passed, stopped" : "Failed, continuing...") << std::endl;
    }
    else if (state == cdlib::SteppableGJKState::ITERATION_5) {
        std::cout << "Iteration 5" << std::endl;
        std::cout << " - New direction: " << gjk.get_direction().x << " " << gjk.get_direction().y << " " << gjk.get_direction().z << std::endl;
        std::cout << " - New simplex:" << std::endl;
        for (int i = 0; i < gjk.get_simplex().size(); i++) {
            std::cout << "   - " << i << ": " << gjk.get_simplex()[i].x << " " << gjk.get_simplex()[i].y << " " << gjk.get_simplex()[i].z << std::endl;
        }
        // Average the points in the simplex
        auto average = glm::vec3(0.0f);
        for (const auto& point : gjk.get_simplex()) {
            average += point;
        }
        average /= static_cast<float>(gjk.get_simplex().size());
        // Create a new direction from the average to the origin
        direction_highlights.emplace_back(gjk.get_direction(), average);
    }
    else if (state == cdlib::SteppableGJKState::EPA) {
        std::cout << "EPA" << std::endl;
        const auto result = gjk.get_collision_data();
        if (result.has_value()) {
            std::cout << " - Collision normal: " << result.value().normal.x << " " << result.value().normal.y << " " << result.value().normal.z << std::endl;
            std::cout << " - Collision depth: " << result.value().depth << std::endl;

            // Draw the collision normal from origin
            direction_highlights.emplace_back(result.value().normal, glm::vec3(0.0f));
        }
        else {
            std::cout << " - No collision data" << std::endl;
        }
    }
    draw_direction_highlights();
}

void Application::on_key_pressed(int key, int scancode, int action, int mods) {
    // On F1, toggle GUI
    if (key == GLFW_KEY_F1 && action == GLFW_PRESS) {
        show_gui = !show_gui;
    }
}

void Application::on_resize(int width, int height) {
    PV227Application::on_resize(width, height);
    // Update aspect ratio
    auto aspect = static_cast<float>(width) / static_cast<float>(height);
    camera_ubo.set_projection(glm::perspective(glm::radians(45.f), aspect, 0.1f, 1000.0f));
    camera_ubo.update_opengl_data();
}
