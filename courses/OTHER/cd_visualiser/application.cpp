#include "application.hpp"

#include <map>
#include <random>
#include <ranges>
#include <set>
#include <utility>

#include "utils.hpp"
#include "vclip.hpp"
#include "voronoi.hpp"

#include "glm/gtx/hash.hpp"

Application::Application(int initial_width, int initial_height, std::vector<std::string> arguments) : PV227Application(initial_width, initial_height, std::move(arguments)) {
    run_tests();

    Application::compile_shaders();
    prepare_cameras();
    prepare_textures();
    prepare_lights();
    prepare_scene();
    prepare_collision_detectors();

    update_object_positions();
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

void Application::prepare_textures() {}

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

void Application::prepare_collision_detectors() {
    gjk = cdlib::SteppableGJKEPA(object_1->collider, object_2->collider);
    recalculate_minkowski_difference();

    vclip = cdlib::VClip(object_1->collider, object_2->collider);

    auto colliders = std::ranges::transform_view(convex_objects, [](std::shared_ptr<ConvexObject>& convex_object) { return convex_object->collider; });
    auto colliders_vector = std::vector<std::shared_ptr<cdlib::Collider>>(std::begin(colliders), std::end(colliders));
    sap = cdlib::SAP(colliders_vector);
}

void Application::prepare_convex_objects() {
    convex_objects.clear();

    // Create main objects (which can be moved)
    auto [g1, v1, i1] = generate_convex_hull_geometry(random_points(object_seed_1));
    auto [g2, v2, i2] = generate_convex_hull_geometry(random_points(object_seed_2));

    auto p1 = get_positions_from_buffer(v1);
    auto p2 = get_positions_from_buffer(v2);

    auto m1 = translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.0f, -4.0f * object_distance));
    auto m2 = translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.0f, 4.0f * object_distance));

    // Convert indicies to array of faces
    auto faces1 = get_faces_from_buffer(i1);
    auto faces2 = get_faces_from_buffer(i2);

    auto shape1 = cdlib::ConvexPolyhedron::build_dcel(p1, faces1);
    auto shape2 = cdlib::ConvexPolyhedron::build_dcel(p2, faces2);

    auto c1 = std::make_shared<cdlib::ConvexCollider>(shape1, m1);
    auto c2 = std::make_shared<cdlib::ConvexCollider>(shape2, m2);

    auto s1 = SceneObject{g1, ModelUBO(m1), red_material_ubo};
    auto s2 = SceneObject{g2, ModelUBO(m2), green_material_ubo};

    auto o1 = ConvexObject{ std::move(v1), m1, c1, s1 };
    auto o2 = ConvexObject{ std::move(v2), m2, c2, s2 };

    object_1 = std::make_shared<ConvexObject>(std::move(o1));
    object_2 = std::make_shared<ConvexObject>(std::move(o2));

    convex_objects.push_back(object_1);
    convex_objects.push_back(object_2);

    // Randomly generate extra objects
    for (int i = 0; i < extra_object_count; i++) {
        auto seed = extra_object_seed * extra_object_count + i;
        auto [g, v, index] = generate_convex_hull_geometry(random_points(seed));
        auto p = pseudorandom_point(seed);
        auto m = translate(glm::mat4(1.0f), p);
        auto faces = get_faces_from_buffer(index);
        auto shape = cdlib::ConvexPolyhedron::build_dcel(get_positions_from_buffer(v), faces);
        auto c = std::make_shared<cdlib::ConvexCollider>(shape, m);
        auto s = SceneObject{g, ModelUBO(m), white_material_ubo};
        auto o = ConvexObject{ std::move(v), m, c, s };
        convex_objects.push_back(std::make_shared<ConvexObject>(std::move(o)));
    }
}


std::shared_ptr<Geometry> Application::create_line_geometry(const glm::vec3&from, const glm::vec3&to) {
    const std::vector vertices = {
        from.x, from.y, from.z,  0.0f, 0.0f, 0.0f,  0.5f, 0.5f,  0.0f, 0.0f, 0.0f,  0.0f, 0.0f, 0.0f,
        to.x,   to.y,   to.z,    0.0f, 0.0f, 0.0f,  0.5f, 0.5f,  0.0f, 0.0f, 0.0f,  0.0f, 0.0f, 0.0f
    };
    const std::vector<uint32_t> indices = {0, 1};
    return std::make_shared<Geometry>(GL_LINES, 14, 2, vertices.data(), 2, indices.data());
}

std::shared_ptr<Geometry> Application::create_line_geometry(const glm::vec3&origin, const glm::vec3&direction, float length) {
    return create_line_geometry(origin, origin + direction * length);
}

glm::vec3 Application::pseudorandom_point(const int seed) {
    std::mt19937 gen(seed);
    std::uniform_real_distribution dis(-10.0f, 10.0f);
    return {dis(gen), dis(gen), dis(gen)};
}

void Application::recalculate_positions() {
    // Get a point on a circle around the origin (based on the object_rotation_pos)
    const auto angle = glm::radians(object_rotation_pos);
    const auto opposite_angle = angle + glm::radians(180.0f);

    const auto distance = object_distance * 4.0f;

    const auto x = glm::cos(angle) * distance;
    const auto z = glm::sin(angle) * distance;
    const auto x_opposite = glm::cos(opposite_angle) * distance;
    const auto z_opposite = glm::sin(opposite_angle) * distance;

    const auto m1 = translate(glm::mat4(1.0f), glm::vec3(x, 0.0f, z));
    const auto m2 = translate(glm::mat4(1.0f), glm::vec3(x_opposite, 0.0f, z_opposite));

    object_1->set_model_matrix(m1);
    object_2->set_model_matrix(m2);
}

void Application::recalculate_minkowski_difference() {
    // If the minkowski object pointer is not null, find it in the convex_objects array and remove it
    if (minkowski_object) {
        auto it = std::ranges::find_if(convex_objects, [this](const auto& object) {
            return object.get() == minkowski_object.get();
        });
        if (it != std::end(convex_objects)) {
            convex_objects.erase(it);
        }
    }

    auto pm = get_minkowski_difference_positions(object_1->collider->get_global_vertices(), raycasting ? raycast_collider->get_global_vertices() : object_2->collider->get_global_vertices());
    auto [gm, vm, _] = generate_convex_hull_geometry(pm);

    auto mm = glm::mat4(1.0f); // The vertices are already in world space

    auto sm = SceneObject{gm, ModelUBO(mm), yellow_material_ubo};

    auto om = ConvexObject{ std::move(vm), mm, nullptr, sm };

    minkowski_object = std::make_shared<ConvexObject>(std::move(om));
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

std::tuple<std::shared_ptr<Geometry>, std::vector<float>, std::vector<uint32_t>> Application::generate_convex_hull_geometry(const std::vector<std::array<float, 3>>& points) {
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
            normals[static_cast<uint32_t>(indexBuffer[i + j])].push_back(normal);
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
        for (const auto& n : normals[static_cast<uint32_t>(i)]) {
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

    return {std::make_shared<Geometry>(GL_TRIANGLES, 14, static_cast<int>(vertices.size() / 14), vertices.data(), static_cast<int>(indexBuffer32.size()), indexBuffer32.data()), vertices, indexBuffer32};
}

std::tuple<std::shared_ptr<Geometry>, std::vector<float>, std::vector<uint32_t>> Application::generate_convex_hull_geometry(const std::vector<glm::vec3>& points) {
    auto converted = std::vector<std::array<float, 3>>(points.size());
    for (size_t i = 0; i < points.size(); i++) {
        converted[i] = {points[i].x, points[i].y, points[i].z};
    }
    return generate_convex_hull_geometry(converted);
}

std::vector<glm::vec3> Application::get_positions_from_buffer(const std::vector<float>& buffer) {
    std::vector<glm::vec3> positions;
    for (size_t i = 0; i < buffer.size(); i += 14) {
        positions.emplace_back(buffer[i], buffer[i + 1], buffer[i + 2]);
    }
    return positions;
}

std::vector<std::vector<size_t>> Application::get_faces_from_buffer(const std::vector<uint32_t>& buffer) {
    std::vector<std::vector<size_t>> faces;
    for (size_t i = 0; i < buffer.size(); i += 3) {
        faces.push_back({buffer[i], buffer[i + 1], buffer[i + 2]});
    }
    return faces;
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

    if (abs(object_distance - last_object_distance) > std::numeric_limits<float>::epsilon() || abs(object_rotation_pos - last_object_rotation_pos) > std::numeric_limits<float>::epsilon()) {
        last_object_distance = object_distance;
        last_object_rotation_pos = object_rotation_pos;
        update_object_positions();
    }
}

void Application::update_object_positions() {
    recalculate_positions();
    recalculate_minkowski_difference();

    highlighted_feature_1 = nullptr;
    highlighted_feature_2 = nullptr;
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

void Application::clear_feature_highlights() {
    feature_highlights.clear();
}

void Application::create_feature_highlights(const cdlib::FeatureP& feature)
{
    auto create_new_vertex_highlight_object = [this](const glm::vec3& position, const PhongMaterialUBO& material) {
        auto model_matrix = translate(glm::mat4(1.0f), position);
        auto matrix = scale(model_matrix, glm::vec3(0.05f));
        return std::move(SceneObject{sphere, ModelUBO(matrix), material});
    };

    if (const auto vertex = std::dynamic_pointer_cast<cdlib::Vertex>(feature))
    {
        const auto position = vertex->get_position();
        feature_highlights.push_back(create_new_vertex_highlight_object(position, yellow_material_ubo));
    }
    else if (const auto edge = std::dynamic_pointer_cast<cdlib::HalfEdge>(feature))
    {
        const auto start = edge->start->get_position();
        const auto end = edge->end->get_position();
        const auto middle = (start + end) / 2.0f;
        feature_highlights.push_back(create_new_vertex_highlight_object(start, yellow_material_ubo));
        feature_highlights.push_back(create_new_vertex_highlight_object(end, yellow_material_ubo));
        feature_highlights.push_back(create_new_vertex_highlight_object(middle, yellow_material_ubo));
    }
    else if (const auto face = std::dynamic_pointer_cast<cdlib::Face>(feature))
    {
        const auto vertices = face->get_vertices();
        auto middle = glm::vec3(0.0f);
        for (const auto& v : vertices)
        {
            middle += v->get_position();
            feature_highlights.push_back(create_new_vertex_highlight_object(v->get_position(), yellow_material_ubo));
        }
        middle /= static_cast<float>(vertices.size());
        feature_highlights.push_back(create_new_vertex_highlight_object(middle, yellow_material_ubo));
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
        // Iterate over the convex objects and render them
        for (const auto& object : convex_objects) {
            // If the object is one of the main objects, check if the show_convex_objects flag is set
            if ((object == object_1 || object == object_2) && show_convex_objects)
                render_object(object->scene_object, default_lit_program);
            // If the object is anything else, but is not minkowski or main object, its an extra object and check if the show_extra_objects flag is set
            else if (object != object_1 && object != object_2 && show_extra_objects) {
                render_object(object->scene_object, default_lit_program);
            }
        }

        // If the current method is GJK+EPA and the show_minkowski_difference flag is set, render the minkowski object
        if (selected_method == GJK_EPA && show_minkowski_difference) {
            render_object(minkowski_object->scene_object, default_lit_program);
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

    if (selected_method == V_CLIP) {
        for (SceneObject& object : feature_highlights) {
            render_object(object, default_unlit_program);
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
// Raycasting
// ----------------------------------------------------------------------------

void Application::raycast()
{
    // Get the current camera position and direction
    const auto camera_position = camera.get_eye_position();
    const auto view_matrix = camera.get_view_matrix();
    auto camera_direction = glm::vec3(0.0f);
    camera_direction.x = view_matrix[0][2];
    camera_direction.y = view_matrix[1][2];
    camera_direction.z = view_matrix[2][2];

    // Create a ray from the camera position and direction
    ray_direction = -camera_direction * 1000.0f;
    ray_origin = camera_position;

    // Draw a line from the camera position in the direction of the ray
    direction_highlights.clear();
    direction_highlights.emplace_back(ray_direction, ray_origin);

    raycasting = true;

    // Perform raycasting
    std::vector<std::shared_ptr<cdlib::Collider>> hits;
    if (selected_method == GJK_EPA) {
        // Create raycasting collider
        auto ray_collider = std::make_shared<cdlib::RayCollider>(ray_origin, ray_direction);

        raycast_collider = ray_collider;

        for (const auto& convex_object : convex_objects)
        {
            auto gjk = cdlib::SteppableGJKEPA(convex_object->collider, ray_collider);
            gjk.evaluate();
            const auto [is_colliding, normal, depth, feature_1, feature_2] = gjk.get_collision_data();
            if (is_colliding)
            {
                std::cout << "Depth: " << depth << std::endl;
                hits.push_back(convex_object->collider);
            }
        }
        recalculate_minkowski_difference();
    }
    if (selected_method == V_CLIP) {
        for (const auto& convex_object : convex_objects)
        {
            auto vclip = cdlib::VClipRaycast(convex_object->collider, ray_origin, ray_direction);
            const auto [is_colliding, normal, depth, feature_1, feature_2] = vclip.get_collision_data();
            if (is_colliding)
            {
                std::cout << "Depth: " << depth << std::endl;
                hits.push_back(convex_object->collider);
            }
        }
    }

    std::cout << "Raycast hits: " << hits.size() << std::endl;
}


// ----------------------------------------------------------------------------
// GUI
// ----------------------------------------------------------------------------
void Application::render_ui() {
    if (!show_gui) return;

    const float unit = ImGui::GetFontSize();

    ImGui::Begin("Settings", nullptr, ImGuiWindowFlags_NoDecoration);
    ImGui::SetWindowSize(ImVec2(25 * unit, 40 * unit));
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
    if (object_seed_1 != last_object_seed_1 || object_seed_2 != last_object_seed_2 ||
        extra_object_seed != last_extra_object_seed || extra_object_count != last_extra_object_count) {
        last_object_seed_1 = object_seed_1;
        last_object_seed_2 = object_seed_2;
        last_extra_object_seed = extra_object_seed;
        last_extra_object_count = extra_object_count;

        prepare_convex_objects();
        prepare_collision_detectors();
        recalculate_minkowski_difference();
    }

    // Slider for moving the objects closer together
    ImGui::SliderFloat("Object distance", &object_distance, 1.0f, 0.0f);
    ImGui::SliderFloat("Object rotation", &object_rotation_pos, 0.0f, 360.0f);

    ImGui::Spacing();

    ImGui::Checkbox("Show convex objects", &show_convex_objects);

    ImGui::Spacing();

    ImGui::Checkbox("Show extra objects", &show_extra_objects);
    ImGui::InputInt("Extra object seed", &extra_object_seed);
    ImGui::SliderInt("Extra object count", &extra_object_count, 0, 100);

    ImGui::Spacing();

    ImGui::Checkbox("Wireframe", &show_wireframe);

    ImGui::Spacing();

    if (ImGui::Button("Raycast")) {
        raycast();
        draw_direction_highlights();
    }

    ImGui::Spacing();

    ImGui::Checkbox("Auto-calculate collision", &auto_calculate_collision);

    if (selected_method == GJK_EPA) {
        ImGui::Checkbox("Show minkowski difference", &show_minkowski_difference);

        if (!auto_calculate_collision) {
            // Button for manual collision calculation
            if (ImGui::Button("Calculate collision")) {
                // auto result = gjk.is_colliding();
                // std::cout << "Collision: " << result << std::endl;
                gjk.evaluate();
                auto [is_colliding, normal, depth, feature_1, feature_2] = gjk.get_collision_data();
                std::cout << "Collision: " << is_colliding << std::endl;
                if (is_colliding) {
                    std::cout << " - Normal: " << normal.x << " " << normal.y << " " << normal.z << std::endl;
                    std::cout << " - Depth: " << depth << std::endl;

                    // Draw the collision normal from origin
                    direction_highlights.emplace_back(normal * depth, glm::vec3(0.0f));
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
            auto [is_colliding, normal, depth, feature_1, feature_2] = gjk.get_collision_data();

            if (is_colliding) {
                ImGui::Text("The objects are: colliding");

                ImGui::Text("Collision normal: %f %f %f", normal.x, normal.y, normal.z);
                ImGui::Text("Collision depth: %f", depth);

                // Draw the collision normal from origin
                direction_highlights.emplace_back(normal * depth, glm::vec3(0.0f));
                draw_direction_highlights();
            } else {
                ImGui::Text("The objects are: not colliding");
            }
        }
    }
    else if (selected_method == V_CLIP) {
        ImGui::Checkbox("Bruteforce test", &brute_force_test);

        if (!auto_calculate_collision) {
            // Button for manual collision calculation
            if (ImGui::Button("Calculate collision")) {
                vclip.reset();
                const auto [is_colliding, normal, depth, feature_1, feature_2] = vclip.get_collision_data();

                std::cout << "Collision: " << is_colliding << std::endl;
                std::cout << " - Distance: " << depth << std::endl;
                std::cout << " - Normal: " << normal.x << " " << normal.y << " " << normal.z << std::endl;
                std::cout << " - Closest feature on object 1: " << feature_1 << std::endl;
                std::cout << " - Closest feature on object 2: " << feature_2 << std::endl;

                if (brute_force_test) {
                    auto bruteforce_result = vclip.debug_brute_force(feature_1, feature_2);
                    if (!bruteforce_result) {
                        std::cerr << "Bruteforce test failed for: dist=" << object_distance << " rot=" << object_rotation_pos << std::endl;
                    }
                }

                clear_feature_highlights();
                highlighted_feature_1 = feature_1;
                highlighted_feature_2 = feature_2;
                create_feature_highlights(feature_1);
                create_feature_highlights(feature_2);
            }

            ImGui::Text("Iterate over features:");
            ImGui::Indent(10.0f);
            if (!step_by_step) {
                if (ImGui::Button("Start step-by-step")) {
                    step_by_step = true;
                    vclip.reset();
                }
            } else {
                if (ImGui::Button("Next step")) {
                    const auto step_result = vclip.step();
                    if (step_result == cdlib::DONE || step_result == cdlib::PENETRATION) {
                        step_by_step = false;
                    }
                    const auto current_feature_1 = vclip.primary_feature();
                    const auto current_feature_2 = vclip.secondary_feature();
                    clear_feature_highlights();
                    highlighted_feature_1 = current_feature_1;
                    highlighted_feature_2 = current_feature_2;
                    create_feature_highlights(current_feature_1);
                    create_feature_highlights(current_feature_2);
                }
                if (ImGui::Button("Cancel step-by-step")) {
                    step_by_step = false;
                    clear_feature_highlights();
                }
            }
        }
        else {
            vclip.reset();
            auto [is_colliding, normal, depth, feature_1, feature_2] = vclip.get_collision_data();
            ImGui::Text("The objects are: %s", is_colliding ? "colliding" : "not colliding");
            ImGui::Text("Distance: %f", depth);

            if (brute_force_test) {
                auto bruteforce_result = vclip.debug_brute_force(feature_1, feature_2);
                if (!bruteforce_result) {
                    std::cerr << "Bruteforce test failed for: dist=" << object_distance << " rot=" << object_rotation_pos << std::endl;
                }
            }

            clear_feature_highlights();
            highlighted_feature_1 = feature_1;
            highlighted_feature_2 = feature_2;
            create_feature_highlights(feature_1);
            create_feature_highlights(feature_2);
        }
    }
    else if (selected_method == AABBTREE) {

    }
    else if (selected_method == SAP) {
        if (ImGui::Button("Calculate collision")) {
            auto collisions = sap.get_collisions();
            std::cout << "Collisions: " << collisions.size() << std::endl;
            for (const auto& pair : sap.get_collisions()) {
                std::cout << " - Colliding pair: " << pair.first << " " << pair.second << std::endl;
            }
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
        const auto [is_colliding, normal, depth, feature_1, feature_2] = gjk.get_collision_data();
        if (is_colliding) {
            std::cout << " - Collision normal: " << normal.x << " " << normal.y << " " << normal.z << std::endl;
            std::cout << " - Collision depth: " << depth << std::endl;

            // Draw the collision normal from origin
            direction_highlights.emplace_back(normal, glm::vec3(0.0f));
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
    if (width == 0 || height == 0) return;

    PV227Application::on_resize(width, height);
    // Update aspect ratio
    auto aspect = static_cast<float>(width) / static_cast<float>(height);
    camera_ubo.set_projection(glm::perspective(glm::radians(45.f), aspect, 0.1f, 1000.0f));
    camera_ubo.update_opengl_data();
}

/**************
 * Unit tests *
 **************/

void Application::run_tests() {
    std::cout << "Voronoi planes: " << test_voronoi_planes() << std::endl;
    std::cout << "Clip edges: " << test_clip_edge() << std::endl;
    std::cout << "Most violating plane: " << test_most_violating_plane() << std::endl;
}

std::shared_ptr<cdlib::ConvexPolyhedron> Application::create_test_cube_voronoi(glm::mat4 model_matrix) {
    // Vertices array
    std::vector vertices = {
        glm::vec3(-1, 1, 1),
        glm::vec3(1, 1, 1),
        glm::vec3(1, -1, 1),
        glm::vec3(-1, -1, 1),
        glm::vec3(-1, 1, -1),
        glm::vec3(1, 1, -1),
        glm::vec3(1, -1, -1),
        glm::vec3(-1, -1, -1)
    };

    // Transform the vertices
    for (auto& vertex : vertices) {
        vertex = glm::vec3(model_matrix * glm::vec4(vertex, 1.0f));
    }

    // Indices (face) array
    const std::vector<std::vector<size_t>> faces = {
        {0, 3, 2, 1}, // Front
        {5, 6, 7, 4}, // Back
        {1, 2, 6, 5}, // Right
        {4, 7, 3, 0}, // Left
        {4, 0, 1, 5}, // Top
        {3, 7, 6, 2} // Bottom
    };

    return cdlib::ConvexPolyhedron::build_dcel(vertices, faces);
}

std::shared_ptr<cdlib::ConvexPolyhedron> Application::create_test_polyhedron_voronoi(float alpha, float p_z, float h_vdz, float h_vdy) {
    // https://www.desmos.com/3d/b7bdf5b643
    auto alpha_rad = glm::radians(alpha);

    auto n1 = glm::vec3(0, 1, 0);
    auto n2 = glm::vec3(0, n1.y * cos(alpha_rad) - n1.z * sin(alpha_rad), n1.y * sin(alpha_rad) + n1.z * cos(alpha_rad));

    auto a = glm::vec3(2, 1, p_z);
    auto b = glm::vec3(-2, 1, p_z);

    auto d1 = glm::dot(n1, a);
    auto d2 = glm::dot(n2, a);

    auto v_dyz = (-n2.y * h_vdy + d2) / n2.z;

    // Create the vertices
    const std::vector vertices = {
        a,
        b,
        glm::vec3(2, 1, h_vdz),
        glm::vec3(-2, 1, h_vdz),
        glm::vec3(2, h_vdy, v_dyz),
        glm::vec3(-2, h_vdy, v_dyz)
    };

    const std::vector<std::vector<size_t>> faces = {
        {0, 2, 3, 1},
        {4, 0, 1, 5},
        {5, 3, 2, 4},
        {4, 2, 0},
        {1, 3, 5}
    };

    return cdlib::ConvexPolyhedron::build_dcel(vertices, faces);
}

bool Application::test_voronoi_planes() {
    bool result = true;
    auto inner_test = [](const std::shared_ptr<cdlib::ConvexPolyhedron>& object, const glm::vec3& point, const size_t type, const size_t idx) -> std::pair<bool, std::optional<cdlib::Voronoi::VoronoiPlane>> {
        if (type == 0) {
            const auto face = object->get_face(idx);
            return cdlib::Voronoi::in_voronoi_region(face, point);
        }
        else if (type == 1) {
            const auto hedge = object->get_half_edge(idx);
            return cdlib::Voronoi::in_voronoi_region(hedge, point);
        }
        else if (type == 2) {
            const auto vertex = object->get_vertex(idx);
            return cdlib::Voronoi::in_voronoi_region(vertex, point);
        }
        return std::make_pair(false, std::nullopt);
    };

    auto in_region_test = [&result, inner_test](const std::shared_ptr<cdlib::ConvexPolyhedron>& object, const glm::vec3& point, const size_t type, const size_t idx, bool expected) {
        result = result && inner_test(object, point, type, idx).first == expected;
    };

    auto unique_in_region_test = [&result, inner_test](const std::shared_ptr<cdlib::ConvexPolyhedron>& object, const glm::vec3& point, const size_t type, const size_t idx) {
        // Check if the point lies in the tested region and not any other
        for (size_t i_type = 0; i_type < 3; i_type++) {
            const auto max_idx = i_type == 0 ? object->get_faces().size() : (i_type == 1 ? object->get_half_edges().size() : object->get_vertices().size());
            for (size_t i_idx = 0; i_idx < max_idx; i_idx++) {
                if (i_type == type && i_idx == idx) {
                    result = result && inner_test(object, point, i_type, i_idx).first;
                }
                else {
                    // Check if the selected feature is not a twin edge of the checked edge
                    if (type == 1){
                        const auto hedge = object->get_half_edge(idx);
                        const auto twin = object->get_half_edge(i_idx)->twin;
                        if (i_type == 1 && twin == hedge)
                            continue;
                    }
                    const auto inner_result = inner_test(object, point, i_type, i_idx);
                    result = result && !inner_result.first;
                }
            }
        }
    };

    const auto cube = create_test_cube_voronoi();

    // Cube face tests
    //  - Test if the point (0, 2, 0) lies in the face 4
    unique_in_region_test(cube, glm::vec3(0, 2, 0), 0, 4);
    //  - Test if the point (2, 2, 2) does not lie in any face
    for (size_t i = 0; i < 6; i++) {
        in_region_test(cube, glm::vec3(2, 2, 2), 0, i, false);
    }
    //  - Test if the point (-1.5, 0, 0) lies in the face 3
    unique_in_region_test(cube, glm::vec3(-1.5, 0, 0), 0, 3);
    //  - Test if the point (0, 0, 0) lies in no face
    for (size_t i = 0; i < 6; i++) {
        in_region_test(cube, glm::vec3(0, 0, 0), 0, i, false);
    }

    // Cube edge tests
    // - Test if the point (1.5, 1.5, 0) lies in the edge 11
    unique_in_region_test(cube, glm::vec3(1.5, 1.5, 0), 1, 11);
    //  - Test if the point (0, -5, -2) lies in the edge 5
    unique_in_region_test(cube, glm::vec3(0, -5, -2), 1, 5);
    //  - Test if the point (0, 12.5, 0) does not lie in any edge
    for (size_t i = 0; i < 12; i++) {
        in_region_test(cube, glm::vec3(0, 12.5, 0), 1, i, false);
    }

    // Vertex tests
    //  - Test if the point (1.2, 1.1, 1.5) lies in the vertex 1
    unique_in_region_test(cube, glm::vec3(1.2, 1.1, 1.5), 2, 1);
    // - Test if the point (45, -33, 10) lies in the vertex 2
    unique_in_region_test(cube, glm::vec3(45, -33, 10), 2, 2);
    // - Test if the point (0, 0, 0) doesn't lie in any vertex
    for (size_t i = 0; i < 8; i++) {
        in_region_test(cube, glm::vec3(0, 0, 0), 2, i, false);
    }

    // Polyhedron tests
    const auto polyhedron = create_test_polyhedron_voronoi();

    // Face tests
    unique_in_region_test(polyhedron, glm::vec3(-1.1, 2.5, 0), 0, 0);
    unique_in_region_test(polyhedron, glm::vec3(1.1, 1.1, -0.5), 0, 0);

    unique_in_region_test(polyhedron, glm::vec3(-1.3, -0.2, 4.5), 0, 1);
    unique_in_region_test(polyhedron, glm::vec3(0.5, 1.8, 3.5), 0, 1);

    unique_in_region_test(polyhedron, glm::vec3(0, -1.8, 0), 0, 2);
    unique_in_region_test(polyhedron, glm::vec3(3.2, 0.5, 1.1), 0, 3);
    unique_in_region_test(polyhedron, glm::vec3(-2.2, 0.9, -0.1), 0, 4);

    // Edge tests
    unique_in_region_test(polyhedron, glm::vec3(-2.5, 1.9, 0.25), 1, 2);
    unique_in_region_test(polyhedron, glm::vec3(0.5, 2, 1.5), 1, 3);
    unique_in_region_test(polyhedron, glm::vec3(-1.5, 1.9, 3.0), 1, 3);
    unique_in_region_test(polyhedron, glm::vec3(-1.3, -0.9, 4.5), 1, 7);
    unique_in_region_test(polyhedron, glm::vec3(0, -1.75, 2.25), 1, 7);
    unique_in_region_test(polyhedron, glm::vec3(2.5, -1.75, 1.25), 1, 10);

    // Vertex tests
    unique_in_region_test(polyhedron, glm::vec3(2.5, 1.75, 1.25), 2, 0);
    unique_in_region_test(polyhedron, glm::vec3(-3, 1.75, 1.35), 2, 1);
    unique_in_region_test(polyhedron, glm::vec3(3.5, 1.75, -1.35), 2, 2);
    unique_in_region_test(polyhedron, glm::vec3(-2.5, -1.75, 2.75), 2, 5);

    return result;
}

bool Application::test_clip_edge() {
    bool result = true;

    auto inner_test = [&result](const glm::vec3 start, const glm::vec3 end, const std::shared_ptr<cdlib::Feature>& feature, const bool expected_is_clipped, const float expected_lambda_l, const float expected_lambda_h, const std::shared_ptr<cdlib::Feature> expected_neighbour_l, const std::shared_ptr<cdlib::Feature> expected_neighbour_h) {
        const auto edge = cdlib::HalfEdge::create(start, end);
        auto clip_data = clip_edge(edge, feature);
        const auto& [is_clipped, lambda_l, lambda_h, neighbour_l, neighbour_h, ce, cf, p1, p2] = clip_data;
        result = result && (is_clipped == expected_is_clipped);
        result = result && std::abs(lambda_l - expected_lambda_l) < 1e-6;
        result = result && std::abs(lambda_h - expected_lambda_h) < 1e-6;
        result = result && neighbour_l == expected_neighbour_l;
        result = result && neighbour_h == expected_neighbour_h;
    };

    const auto polyhedron = create_test_polyhedron_voronoi();

    inner_test(glm::vec3(-1.5, 3, -1), glm::vec3(0.5, 1.5, 3.5), polyhedron->get_half_edge(3), true, 0.444444f, 0.835436f, polyhedron->get_face(0), polyhedron->get_face(1));
    inner_test(glm::vec3(-0.33,2,0.25), glm::vec3(0.5,4.5,3.5), polyhedron->get_half_edge(3), true, 0.230769f, 1.0f, polyhedron->get_face(0), nullptr);
    inner_test(glm::vec3(-2.53,1,3.25), glm::vec3(0.5,4.5,3.5), polyhedron->get_face(1), true, 0.174917491749f, 0.281408880626f, polyhedron->get_half_edge(6), polyhedron->get_half_edge(5));
    inner_test(glm::vec3(0.5,4.5,3.5), glm::vec3(-3.53,1,3.25), polyhedron->get_face(1), false, 0.718591119374f, 0.620347394541f, polyhedron->get_half_edge(5), polyhedron->get_half_edge(6));
    inner_test(glm::vec3(0.5,4.5,3.5), glm::vec3(-1.53,2.5,3.25), polyhedron->get_face(1), false, 0.0f, 1.0f, polyhedron->get_half_edge(5), polyhedron->get_half_edge(5));
    inner_test(glm::vec3(-2.75,4.5,-0.5), glm::vec3(-2.53,-2.5,3.25), polyhedron->get_vertex(1), true, 0.4f, 0.481473f, polyhedron->get_half_edge(15), polyhedron->get_half_edge(6));
    inner_test(glm::vec3(-2.75,4.5,3.5), glm::vec3(-2.53,2.5,3.25), polyhedron->get_vertex(1), true, 0.0f, 1.0f, nullptr, nullptr);
    return result;
}

bool Application::test_most_violating_plane() {
    bool result = true;

    return result;
}