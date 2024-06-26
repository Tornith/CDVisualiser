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
    gjk = cdlib::SteppableGJK2EPA(object_1->collider, object_2->collider);
    recalculate_minkowski_difference();

    vclip = cdlib::VClip(object_1->collider, object_2->collider);

    auto colliders = std::ranges::transform_view(convex_objects, [](std::shared_ptr<ConvexObject>& convex_object) { return convex_object->collider; });
    const auto colliders_vector = std::set<cdlib::ColliderP>(std::begin(colliders), std::end(colliders));
    sap = cdlib::SAP(colliders_vector);
    aabb_tree = cdlib::AABBTree(colliders_vector);
}

void Application::prepare_convex_objects() {
    convex_objects.clear();

    // Create main objects (which can be moved)
    auto [g1, v1, i1] = generate_convex_hull_geometry(random_points(object_seed_1));
    auto [g2, v2, i2] = generate_convex_hull_geometry(random_points(object_seed_2));

    auto p1 = get_positions_from_buffer(v1);
    auto p2 = get_positions_from_buffer(v2);

    auto m1 = translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.0f, 0.0f));
    auto m2 = translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.0f, 0.0f));

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
        // Set their transform matrix of the oclliders
        convex_objects.back()->collider->set_transform(m);
    }

    update_object_positions();
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

    last_object_distance = object_distance;
    last_object_rotation_pos = object_rotation_pos;
}

void Application::recalculate_detailed_positioning() {
    // Create a rotation matrix from object_rotation_1 and object_rotation_2
    const auto rot1X = glm::rotate(glm::mat4(1.0f), object_rotation_1.x, glm::vec3(1.0f, 0.0f, 0.0f));
    const auto rot1Y = glm::rotate(glm::mat4(1.0f), object_rotation_1.y, glm::vec3(0.0f, 1.0f, 0.0f));
    const auto rot1Z = glm::rotate(glm::mat4(1.0f), object_rotation_1.z, glm::vec3(0.0f, 0.0f, 1.0f));
    const auto rot1 = rot1Z * rot1Y * rot1X;

    const auto rot2X = glm::rotate(glm::mat4(1.0f), object_rotation_2.x, glm::vec3(1.0f, 0.0f, 0.0f));
    const auto rot2Y = glm::rotate(glm::mat4(1.0f), object_rotation_2.y, glm::vec3(0.0f, 1.0f, 0.0f));
    const auto rot2Z = glm::rotate(glm::mat4(1.0f), object_rotation_2.z, glm::vec3(0.0f, 0.0f, 1.0f));
    const auto rot2 = rot2Z * rot2Y * rot2X;

    // Create a translation matrix from object_position_1 and object_position_2
    const auto trans1 = glm::translate(glm::mat4(1.0f), object_position_1);
    const auto trans2 = glm::translate(glm::mat4(1.0f), object_position_2);

    // Combine the rotation and translation matrices
    const auto m1 = trans1 * rot1;
    const auto m2 = trans2 * rot2;

    // Set the model matrices of the objects
    object_1->set_model_matrix(m1);
    object_2->set_model_matrix(m2);
}

void Application::random_move_objects() {
    // for each object, move it in a random direction
    for (size_t i = 0; i < convex_objects.size(); i++) {
        const auto offset = static_cast<float>(i) * 0.053421f;
        const auto random = perlin_noise(i, elapsed_time * random_move_speed * 0.001f + offset) * random_move_spread;
        const auto rotation = perlin_noise(i + 10000, elapsed_time * random_move_speed * 0.001f + offset) * 2.0f * glm::pi<float>();

        const auto rotX = rotate(glm::mat4(1.0f), rotation.x, glm::vec3(1.0f, 0.0f, 0.0f));
        const auto rotY = rotate(glm::mat4(1.0f), rotation.y, glm::vec3(0.0f, 1.0f, 0.0f));
        const auto rotZ = rotate(glm::mat4(1.0f), rotation.z, glm::vec3(0.0f, 0.0f, 1.0f));
        const auto rot = rotZ * rotY * rotX;

        const auto trans = translate(glm::mat4(1.0f), random);

        const auto m = trans * rot;

        convex_objects[i]->set_model_matrix(m);

        if (i == 0) {
            object_position_1 = random;
            object_rotation_1 = rotation;
        } else if (i == 1) {
            object_position_2 = random;
            object_rotation_2 = rotation;
        }
    }
}

glm::vec3 Application::perlin_noise(const size_t seed, const long double time) {
    const auto x = perlin(glm::vec2(seed, time));
    const auto y = perlin(glm::vec2(1354 * seed + 100, time));
    const auto z = perlin(glm::vec2(2708 * seed + 200, time));
    return {x, y, z};
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

    if (convex_objects.size() < 2 || object_1->collider == nullptr || object_2->collider == nullptr) {
        minkowski_object = nullptr;
        return;
    }

    auto pm = get_minkowski_difference_positions(object_1->collider->get_global_vertices(), object_2->collider->get_global_vertices());
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

    if (rotate_and_move_objects || detailed_positioning || (
        object_distance != last_object_distance ||
        object_rotation_pos != last_object_rotation_pos
    )) {
        update_object_positions();
    }

    // Perform collision detection
    if (auto_calculate_collision)
    {
        perform_collision_detection();
    }

    if (brute_force_test)
    {
        perform_bruteforce_test();
    }
}

void Application::update_object_positions() {
    if (rotate_and_move_objects) {
        random_move_objects();
    } else if (detailed_positioning) {
        recalculate_detailed_positioning();
    } else {
        recalculate_positions();
    }
    if (convex_objects.size() > 1) {
        recalculate_minkowski_difference();
    }

    if (selected_method == SAP || selected_method == AABBTREE) {
        // Capture time before the update
        const auto start = std::chrono::high_resolution_clock::now();

        for (const auto& object : convex_objects) {
            if (selected_method == SAP) sap.update_endpoints(object->collider);
            else aabb_tree.update(object->collider);
        }

        // Capture time after the update
        const auto end = std::chrono::high_resolution_clock::now();
        const auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

        // Show time taken
        if (show_time_taken) {
            const std::string method_name = (selected_method == SAP) ? "SAP" : "AABB Tree";
            std::cout << method_name << " update took: " << duration.count() << " microseconds" << std::endl;
        }
    }

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

    const auto mats = std::vector{
        red_material_ubo,
        green_material_ubo,
        blue_material_ubo,
        yellow_material_ubo,
        cyan_material_ubo,
        magenta_material_ubo,
        white_material_ubo
    };

    // Create highlights for simplex vertices
    for (const auto& vertex : simplex_1) {
        object_vertex_highlights.push_back(create_new_vertex_highlight_object(vertex, yellow_material_ubo));
    }
    for (const auto& vertex : simplex_2) {
        object_vertex_highlights.push_back(create_new_vertex_highlight_object(vertex, yellow_material_ubo));
    }
    // for (const auto& vertex : simplex_minkowski) {
    //     simplex_vertex_highlights.push_back(create_new_vertex_highlight_object(vertex, white_material_ubo));
    // }

    for (size_t i = 0; i < gjk.get_simplex().size(); i++) {
        simplex_vertex_highlights.push_back(create_new_vertex_highlight_object(gjk.get_simplex()[i], mats[i % mats.size()]));
    }
}

void Application::draw_direction_highlights() {
    direction_highlight_objects.clear();
    point_highlight_objects.clear();
    for (const auto& [direction, origin] : direction_highlights) {
        auto geometry = create_line_geometry(origin, direction, 1.0f);
        direction_highlight_objects.emplace_back(geometry, ModelUBO(glm::mat4(1.0f)), cyan_material_ubo);
    }
    for (const auto& point : point_highlights) {
        point_highlight_objects.emplace_back(sphere, ModelUBO(scale(translate(glm::mat4(1.0f), point), glm::vec3(0.05f))), cyan_material_ubo);
    }
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

void Application::update_aabb_objects() {
    aabb_objects.clear();
    // If the current method is AABB Tree, highlight the AABBs of the nodes
    if (selected_method == AABBTREE) {
        const auto nodes = aabb_tree.get_nodes();

        for (const auto& node : nodes) {
            auto cube = Cube();
            cube.update(node->aabb.min, node->aabb.max);
            bool is_leaf = node->is_leaf();
            bool is_colliding = std::ranges::any_of(broadphase_collisions, [node](const cdlib::CollisionPair& pair) {
                return pair.contains(node->collider);
            });
            const auto ubo = is_leaf ? (is_colliding ? red_material_ubo : yellow_material_ubo) : green_material_ubo;
            auto scene_object = SceneObject{cube, ModelUBO(glm::mat4(1.0f)), ubo};
            aabb_objects.push_back(scene_object);
        }
    } else {
        for (const auto& object : convex_objects) {
            // Create a new cube geometry
            auto cube = Cube();
            cube.update(object->collider->get_aabb().min, object->collider->get_aabb().max);

            // Create sceneobject
            constexpr auto model_matrix = glm::mat4(1.0f);

            // Find a CollisionPair, that contains the current collider
            const auto is_collliding = std::ranges::find_if(broadphase_collisions, [object](const auto& pair) {
                return pair.contains(object->collider);
            }) != std::end(broadphase_collisions);

            auto scene_object = SceneObject{cube, ModelUBO(model_matrix), is_collliding ? red_material_ubo : yellow_material_ubo};
            aabb_objects.push_back(scene_object);
        }
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
            if ((object == object_1 || object == object_2) && show_convex_objects) {
                render_object(object->scene_object, default_lit_program);
            }
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

    if (selected_method == SAP || selected_method == AABBTREE) {
        // Render AABB objects as wireframe
        glDisable(GL_CULL_FACE);
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        for (SceneObject& object : aabb_objects) {
            render_object(object, default_unlit_program);
        }
        glEnable(GL_CULL_FACE);
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
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    for (SceneObject& object : point_highlight_objects) {
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
    point_highlights.clear();
    direction_highlights.emplace_back(ray_direction, ray_origin);

    raycasting = true;

    // Perform raycasting
    cdlib::RayCastResultSet hits;
    if (selected_method == GJK_EPA) {
        for (const auto& convex_object : convex_objects)
        {
            auto result = cdlib::GJKEPA::raycast(cdlib::Ray(ray_origin, ray_direction), convex_object->collider);
            if (result.hit)
            {
                hits.insert(result);
            }
        }
    }
    if (selected_method == V_CLIP) {
        for (const auto& convex_object : convex_objects)
        {
            auto result = cdlib::VClip::raycast(cdlib::Ray(ray_origin, ray_direction), convex_object->collider);
            if (result.hit)
            {
                hits.insert(result);
            }
        }
    }

    if (selected_method == SAP) {
        hits = sap.raycast(cdlib::Ray(ray_origin, ray_direction));
    }

    if (selected_method == AABBTREE) {
        hits = aabb_tree.raycast(cdlib::Ray(ray_origin, ray_direction));
    }

    // Highlight the collision points and normals
    for (const auto& hit : hits)
    {
        point_highlights.emplace_back(hit.position);
        direction_highlights.emplace_back(hit.normal, hit.position);
    }

    std::cout << "Raycast hits: " << hits.size() << std::endl;
}

// ----------------------------------------------------------------------------
// Collision detection
// ----------------------------------------------------------------------------

void Application::perform_collision_detection() {
    if (selected_method == GJK_EPA) {
        if (!auto_calculate_collision) {
            direction_highlights.clear();
        }
        direction_highlight_objects.clear();
        gjk.reset();

        // Measure time for collision detection
        const auto start = std::chrono::high_resolution_clock::now();
        collision_data = gjk.get_collision_data();
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        if (show_time_taken) std::cout << "GJK+EPA took: " << duration.count() << " microseconds" << std::endl;

        auto [is_colliding, normal, depth, feature_1, feature_2] = collision_data;
        if (!auto_calculate_collision) std::cout << "Collision: " << is_colliding << std::endl;
        if (is_colliding) {
            if (!auto_calculate_collision) {
                std::cout << " - Normal: " << normal.x << " " << normal.y << " " << normal.z << std::endl;
                std::cout << " - Depth: " << depth << std::endl;
            }

            // Draw the collision normal from origin
            direction_highlights.emplace_back(normal * depth, glm::vec3(0.0f));
        }
    } else if (selected_method == V_CLIP) {
        vclip.reset();

        const auto start = std::chrono::high_resolution_clock::now();
        collision_data = vclip.get_collision_data();
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        if (show_time_taken) std::cout << "V-Clip took: " << duration.count() << " microseconds" << std::endl;

        const auto [is_colliding, normal, depth, feature_1, feature_2] = collision_data;

        if (!auto_calculate_collision) {
            std::cout << "Collision: " << is_colliding << std::endl;
            std::cout << " - Distance: " << depth << std::endl;
            std::cout << " - Normal: " << normal.x << " " << normal.y << " " << normal.z << std::endl;
            std::cout << " - Closest feature on object 1: " << feature_1 << std::endl;
            std::cout << " - Closest feature on object 2: " << feature_2 << std::endl;
        }

        if (brute_force_test && false) {
            auto bruteforce_result = vclip.debug_brute_force(is_colliding, feature_1, feature_2);
            if (!bruteforce_result) {
                std::cerr << "V-Clip bruteforce test failed for: dist=" << object_distance << " rot=" << object_rotation_pos << std::endl;
            }
        }

        feature_highlights.clear();
        highlighted_feature_1 = feature_1;
        highlighted_feature_2 = feature_2;
        create_feature_highlights(feature_1);
        create_feature_highlights(feature_2);
    } else if (selected_method == SAP || selected_method == AABBTREE) {
        // Perform sweep-and-prune collision detection
        const auto start = std::chrono::high_resolution_clock::now();
        broadphase_collisions = selected_method == SAP ? sap.get_collisions() : aabb_tree.get_collisions();
        const auto end = std::chrono::high_resolution_clock::now();
        const auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

        if (show_time_taken) {
            std::string method_name = (selected_method == SAP) ? "SAP" : "AABB Tree";
            std::cout << method_name << " took: " << duration.count() << " microseconds" << std::endl;
        }

        update_aabb_objects();

        if (!auto_calculate_collision) {
            std::cout << "Collisions: " << broadphase_collisions.size() << std::endl;
        }
    }
}

void Application::start_step_by_step_collision_detection() {
    step_by_step = true;
    gjk.reset();
    vclip.reset();
}

void Application::step_collision_detection() {
    if (!step_by_step) return;

    if (selected_method == GJK_EPA) {
        gjk_step_visualize(gjk.step());
    } else if (selected_method == V_CLIP) {
        const auto step_result = vclip.step();
        if (step_result == cdlib::DONE || step_result == cdlib::PENETRATION) {
            step_by_step = false;
        }
        const auto current_feature_1 = vclip.primary_feature();
        const auto current_feature_2 = vclip.secondary_feature();
        feature_highlights.clear();
        highlighted_feature_1 = current_feature_1;
        highlighted_feature_2 = current_feature_2;
        create_feature_highlights(current_feature_1);
        create_feature_highlights(current_feature_2);
    }
}

void Application::stop_step_by_step_collision_detection() {
    step_by_step = false;

    // Clear the highlights
    active_simplex_vertex_highlights.clear();
    simplex_vertex_highlights.clear();
    active_object_vertex_highlights.clear();
    object_vertex_highlights.clear();
    point_highlights.clear();
    direction_highlights.clear();
    feature_highlights.clear();
}

void Application::perform_bruteforce_test() {
    if (shoot_random_rays) {
        const auto colliders = std::ranges::transform_view(convex_objects, [](std::shared_ptr<ConvexObject>& convex_object) { return convex_object->collider; });
        const auto colliders_vector = std::set<cdlib::ColliderP>(std::begin(colliders), std::end(colliders));

        std::unordered_set<cdlib::ColliderP> bruteforce_hits;
        std::unordered_set<cdlib::ColliderP> hits;

        constexpr int ray_count = 10;

        for (int i = 0; i < ray_count; i++) {
            // Pick a random point in the scene
            const auto random_point = pseudorandom_point(0 + static_cast<int>(elapsed_time) + i * 1234);
            const auto random_direction = pseudorandom_point(1 + static_cast<int>(elapsed_time) + i * 1234);

            const auto ray = cdlib::Ray(random_point, random_direction);

            if (selected_method == GJK_EPA || selected_method == V_CLIP) {
                const auto current_bruteforce_hits = cdlib::NarrowBruteforce::raycast(ray, colliders_vector);
                for (const auto& hit : current_bruteforce_hits) {
                    bruteforce_hits.insert(hit.collider);
                }

                for (const auto& convex_object : convex_objects)
                {
                    cdlib::RayCastResult result;
                    result = selected_method == GJK_EPA ? cdlib::GJK::raycast(ray, convex_object->collider) : cdlib::VClip::raycast(ray, convex_object->collider);
                    if (result.hit)
                    {
                        hits.insert(convex_object->collider);
                    }
                }
            } else {
                const auto current_bruteforce_hits = cdlib::BroadBruteforce(colliders_vector).raycast(ray);
                const auto current_hits = selected_method == SAP ? sap.raycast(ray) : aabb_tree.raycast(ray);
                for (const auto& hit : current_bruteforce_hits) {
                    bruteforce_hits.insert(hit.collider);
                }
                for (const auto& hit : current_hits) {
                    hits.insert(hit.collider);
                }
            }
        }

        std::vector<cdlib::ColliderP> result;
        std::ranges::set_symmetric_difference(hits, bruteforce_hits, std::back_inserter(result));

        // If the hits are not the same, print out the debug message
        if (!result.empty()) {
            std::cerr << "Raycast test failed!" << std::endl;
            std::cerr << "Hits: " << hits.size() << " Bruteforce hits: " << bruteforce_hits.size() << std::endl;
            std::cout << "Missed/extra hits:" << std::endl;
            for (const auto& hit : result) {
                const auto id = static_cast<char>('A' + std::distance(colliders.begin(), std::ranges::find(colliders, hit)));
                std::cout << "   - " << id << (bruteforce_hits.contains(hit) ? "(missed)" : "(extra)") << std::endl;
            }
            invalid_ray_casts++;
        } else {
            std::cout << hits.size() << "==" << bruteforce_hits.size() << " (Invalid: " << invalid_ray_casts << ")" << std::endl;
        }
    }

    if (selected_method == SAP || selected_method == AABBTREE) {
        auto colliders = std::ranges::transform_view(convex_objects, [](std::shared_ptr<ConvexObject>& convex_object) { return convex_object->collider; });
        const auto colliders_vector = std::set<cdlib::ColliderP>(std::begin(colliders), std::end(colliders));
        auto bruteforce = cdlib::BroadBruteforce(colliders_vector);

        const auto bruteforce_collisions = bruteforce.get_collisions();
        if (bruteforce_collisions.size() == broadphase_collisions.size()) {
            // Perform a check, if both sets contain the same colliders
            if (std::ranges::all_of(broadphase_collisions, [&bruteforce_collisions](const auto& pair) {
                return std::ranges::any_of(bruteforce_collisions, [&pair](const auto& other) {
                    return pair.contains(other.collider_1) && pair.contains(other.collider_2);
                });
            })) {
                return;
            }
        }

        // Stop the object moving and rotating
        // rotate_and_move_objects = false;

        std::cerr << "Bruteforce test failed!" << std::endl;
        // Print what colliders are colliding in bruteforce

        std::vector<cdlib::CollisionPair> bruteforce_extras;
        std::vector<cdlib::CollisionPair> extras;

        for (const auto& pair : bruteforce_collisions) {
            if (!broadphase_collisions.contains(pair)) {
                bruteforce_extras.push_back(pair);
            }
        }

        for (const auto& pair : broadphase_collisions) {
            if (!bruteforce_collisions.contains(pair)) {
                extras.push_back(pair);
            }
        }

        std::cout << "Bruteforce extra collisions: ";
        for (const auto& pair : bruteforce_extras) {
            // Find the colliders in the colliders_vector array and print their indices as A, B, C, ...
            const auto id1 = static_cast<char>('A' + std::distance(colliders.begin(), std::ranges::find(colliders, pair.collider_1)));
            const auto id2 = static_cast<char>('A' + std::distance(colliders.begin(), std::ranges::find(colliders, pair.collider_2)));
            std::cout << "(" << id1 << id2 << "), ";
        }
        std::cout << std::endl;

        std::cout << "Extra collisions: ";
        for (const auto& pair : extras) {
            // Find the colliders in the colliders_vector array and print their indices as A, B, C, ...
            const auto id1 = static_cast<char>('A' + std::distance(colliders.begin(), std::ranges::find(colliders, pair.collider_1)));
            const auto id2 = static_cast<char>('A' + std::distance(colliders.begin(), std::ranges::find(colliders, pair.collider_2)));
            std::cout << "(" << id1 << id2 << "), ";
        }

        // Print what colliders are colliding in SAP
        if (selected_method == SAP) {
            sap.print_lists();
        }
    }

    if (selected_method == GJK_EPA || selected_method == V_CLIP) {
        auto bruteforce = cdlib::NarrowBruteforce(object_1->collider, object_2->collider);
        const auto expected_data = bruteforce.get_collision_data();
        if (expected_data.is_colliding == collision_data.is_colliding)
        {
            // If the selected method is V-Clip, check if the features are the same
            if (selected_method == V_CLIP)
            {
                if (collision_data.is_colliding) return; // Don't check the nearest features, since they are not always the same in this case

                auto check_if_features_equal = [](const cdlib::FeatureP& f1, const cdlib::FeatureP& f2) {
                    if (f1 == nullptr || f2 == nullptr){
                        return false;
                    }
                    if (f1 == f2) return true;

                    // Check if the features are half_edges, if so check if they are potentially twins
                    if (const auto edge_1 = std::dynamic_pointer_cast<cdlib::HalfEdge>(f1); edge_1 != nullptr){
                        if (const auto edge_2 = std::dynamic_pointer_cast<cdlib::HalfEdge>(f2); edge_2 != nullptr){
                            return edge_1->start == edge_2->end && edge_1->end == edge_2->start;
                        }
                    }

                    return false;
                };

                // Check distance difference
                const auto distance = feature_distance(collision_data.feature_1, collision_data.feature_2);
                const auto distance_difference = std::abs(distance - expected_data.depth);

                // Check if the features are the same (or swapped), considering half-edge twins
                if (check_if_features_equal(collision_data.feature_1, expected_data.feature_1) && check_if_features_equal(collision_data.feature_2, expected_data.feature_2) ||
                   check_if_features_equal(collision_data.feature_1, expected_data.feature_2) && check_if_features_equal(collision_data.feature_2, expected_data.feature_1) ||
                   distance_difference < 0.001f)
                {
                    return;
                }
                std::cerr << "Distance difference: " << distance_difference << std::endl;
                std::cerr << "Got: " << collision_data.feature_1->to_string() << " - " << collision_data.feature_2->to_string() << std::endl;
                std::cerr << "Expected: " << expected_data.feature_1->to_string() << " - " << expected_data.feature_2->to_string() << std::endl;
                std::cerr << "Collider 1: " << object_1->collider->get_shape()->get_debug_data() << std::endl;
                std::cerr << "Collider 2: " << object_2->collider->get_shape()->get_debug_data() << std::endl;
            }
            else
            {
                return;
            }
        }

        // Print-out debug message
        std::cerr << std::endl << "Bruteforce test failed!" << std::endl;
        std::cout << "--- Expected data ---" << std::endl;
        std::cout << "Is colliding: " << "Bruteforce: " << expected_data.is_colliding << "   ResultData: " << collision_data.is_colliding << std::endl;
        std::cout << std::endl;

        std::cout << "--- Positioning ---" << std::endl;
        std::cout << "Collider 1:" << std::endl;
        std::cout << "\tPosition: (" << object_position_1.x << ", " << object_position_1.y << ", " <<  object_position_1.z << ")" << std::endl;
        std::cout << "\tRotation: (" << object_rotation_1.x << ", " << object_rotation_1.y << ", " <<  object_rotation_1.z << ")" << std::endl;

        std::cout << "Collider 2:" << std::endl;
        std::cout << "\tPosition: (" << object_position_2.x << ", " << object_position_2.y << ", " <<  object_position_2.z << ")" << std::endl;
        std::cout << "\tRotation: (" << object_rotation_2.x << ", " << object_rotation_2.y << ", " <<  object_rotation_2.z << ")" << std::endl;
        std::cout << std::endl;
    }
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
    const char* methods[4] = {"GJK+EPA", "V-Clip", "AABBTree", "Sweep-and-Prune"};
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
    ImGui::Spacing();

    ImGui::Checkbox("Move object around", &rotate_and_move_objects);
    if (!rotate_and_move_objects) {
        ImGui::Checkbox("Detailed positioning", &detailed_positioning);
    } else {
        detailed_positioning = false;
    }

    if (!rotate_and_move_objects && !detailed_positioning)
    {
        ImGui::SliderFloat("Object distance", &object_distance, 1.0f, 0.0f);
        ImGui::SliderFloat("Object rotation", &object_rotation_pos, 0.0f, 360.0f);
    } else if (detailed_positioning) {
        ImGui::Text("Object 1:");
        ImGui::Columns(2, "objects", false);
        ImGui::InputFloat3("Position##object_1", &object_position_1.x);
        ImGui::NextColumn();
        ImGui::InputFloat3("Rotation##object_1", &object_rotation_1.x);

        ImGui::Columns();

        ImGui::Text("Object 2:");
        ImGui::Columns(2, "objects", false);
        ImGui::InputFloat3("Position##object_2", &object_position_2.x);
        ImGui::NextColumn();
        ImGui::InputFloat3("Rotation##object_2", &object_rotation_2.x);
    } else if (rotate_and_move_objects) {
        ImGui::SliderFloat("Random move speed", &random_move_speed, 0.0f, 10.0f);
        ImGui::SliderFloat("Random move spread", &random_move_spread, 0.0f, 100.0f);
    }
    ImGui::Columns();

    ImGui::Spacing();

    ImGui::Checkbox("Show convex objects", &show_convex_objects);

    ImGui::Spacing();

    ImGui::Checkbox("Show extra objects", &show_extra_objects);
    if (show_extra_objects) {
        ImGui::Indent(20.f);
        ImGui::InputInt("Extra object seed", &extra_object_seed);
        ImGui::SliderInt("Extra object count", &extra_object_count, 0, 100);
        ImGui::Unindent(20.f);
    }

    ImGui::Spacing();

    ImGui::Checkbox("Wireframe", &show_wireframe);
    // ImGui::Checkbox("Show AABBs", &show_aabbs);

    ImGui::Spacing();

    if (ImGui::Button("Raycast")) {
        raycast();
        draw_direction_highlights();
    }

    ImGui::Spacing();

    ImGui::Checkbox("Bruteforce test", &brute_force_test);
    if (brute_force_test) {
        auto_calculate_collision = true;
        ImGui::Indent(20.f);
        ImGui::Checkbox("Shoot random rays", &shoot_random_rays);
        ImGui::Unindent(20.f);
    }

    if (!brute_force_test)
    {
        ImGui::Checkbox("Auto-calculate collision", &auto_calculate_collision);
    }

    if (selected_method == GJK_EPA) {
        ImGui::Checkbox("Show minkowski difference", &show_minkowski_difference);
    }
    else if (selected_method == V_CLIP) {

    }
    else if (selected_method == AABBTREE) {

    }
    else if (selected_method == SAP) {

    }

    if (!auto_calculate_collision) {
        // Button for manual collision calculation
        if (ImGui::Button("Calculate collision")) {
            perform_collision_detection();
        }

        ImGui::Text("Step by step:");
        ImGui::Indent(20.f);
        if (!step_by_step) {
            if (ImGui::Button("Start step-by-step")) {
                start_step_by_step_collision_detection();
            }
        } else {
            if (ImGui::Button("Next step")) {
                step_collision_detection();
            }
            if (ImGui::Button("Cancel step-by-step")) {
                stop_step_by_step_collision_detection();
            }
        }
        ImGui::Unindent(20.f);
    }
    else {
        ImGui::Text("The objects are: %s", collision_data.is_colliding ? "colliding" : "not colliding");
        if (collision_data.is_colliding) {
            ImGui::Text("Collision normal: %f %f %f", collision_data.normal.x, collision_data.normal.y, collision_data.normal.z);
            ImGui::Text("Collision depth: %f", collision_data.depth);
        }

        if (collision_data.normal != glm::vec3(0.0f)) {
            direction_highlights.emplace_back(collision_data.normal * collision_data.depth, glm::vec3(0.0f));
            draw_direction_highlights();
        }
    }

    ImGui::End();
}

void Application::gjk_step_visualize(cdlib::GJKState state){
    create_vertex_highlight_objects();
    if (state == cdlib::GJKState::ERROR) {
        step_by_step = false;
        std::cerr << "Error state" << std::endl;
    }
    else if (state == cdlib::GJKState::EPA_FINISHED)
    {
        std::cout << "EPA" << std::endl;
        const auto [is_colliding, normal, depth, feature_1, feature_2] = gjk.get_result();
        if (is_colliding) {
            std::cout << " - Collision normal: " << normal.x << " " << normal.y << " " << normal.z << std::endl;
            std::cout << " - Collision depth: " << depth << std::endl;

            // Draw the collision normal from origin
            direction_highlights.emplace_back(normal, glm::vec3(0.0f));
        }
        else {
            std::cout << " - No collision data" << std::endl;
        }
        step_by_step = false;
    }
    else if (state == cdlib::GJKState::COLLISION)
    {
        std::cout << "Collision detected -> Moving onto EPA" << std::endl;
    }
    else if (state == cdlib::GJKState::NO_COLLISION)
    {
        std::cout << "No collision detected" << std::endl;
        step_by_step = false;
    }
    else if (state == cdlib::GJKState::CONTINUE || state == cdlib::GJKState::INIT)
    {
        direction_highlights.clear();
        std::cout << "Iteration step" << std::endl;
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
    else if (state == cdlib::GJKState::UPDATE_SIMPLEX) {
        std::cout << "Next support point: " << gjk.get_current_new_point().x << " " << gjk.get_current_new_point().y << " " << gjk.get_current_new_point().z << std::endl;
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