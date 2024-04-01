#pragma once
#include "camera_ubo.hpp"
#include "gjk.hpp"
#include "light_ubo.hpp"
#include "pv227_application.hpp"
#include "scene_object.hpp"

#include "QuickHull.hpp"
#include "sap.hpp"
#include "voronoi.hpp"

class ColliderAppWrapper {
    std::unique_ptr<cdlib::NarrowCollisionDetector> collision_detector;
};

enum CollisionDetectionMethod {
    GJK_EPA = 0,
    V_CLIP = 1,
    AABBTREE = 2,
    SAP = 3
};

struct ConvexObject {
    std::vector<float> vertices;
    glm::mat4 model_matrix;
    std::shared_ptr<cdlib::ConvexCollider> collider;
    SceneObject scene_object;

    void set_model_matrix(const glm::mat4& model_matrix) {
        this->model_matrix = model_matrix;
        scene_object.get_model_ubo().set_matrix(model_matrix);
        scene_object.get_model_ubo().update_opengl_data();

        if (collider) {
            collider->set_transform_matrix(model_matrix);
        }
    }
};

class Application : public PV227Application {
protected:
    std::vector<std::shared_ptr<ConvexObject>> convex_objects;

    std::vector<SceneObject> active_simplex_vertex_highlights;
    std::vector<SceneObject> simplex_vertex_highlights;
    std::vector<SceneObject> active_object_vertex_highlights;
    std::vector<SceneObject> object_vertex_highlights;

    // Vector of pairs of direction and their origin
    std::vector<std::pair<glm::vec3, glm::vec3>> direction_highlights;
    std::vector<SceneObject> direction_highlight_objects;

    // World Axis
    SceneObject world_axis_x;
    SceneObject world_axis_y;
    SceneObject world_axis_z;


    PhongLightsUBO phong_lights_ubo;
    float light_position_rad = glm::radians(60.f);


    CameraUBO camera_ubo;

    CollisionDetectionMethod selected_method = GJK_EPA;

    bool show_gui = true;
    float object_distance = 1.0f;
    float last_object_distance = 1.0f;

    bool show_wireframe = true;
    bool auto_calculate_collision = false;
    bool step_by_step = false;

    bool show_convex_objects = true;
    bool show_extra_objects = true;

    // Main objects
    int object_seed_1 = 7;
    int object_seed_2 = 5;

    int last_object_seed_1 = 7;
    int last_object_seed_2 = 5;

    std::shared_ptr<ConvexObject> object_1;
    std::shared_ptr<ConvexObject> object_2;

    // Extra objects
    int extra_object_seed = 150;
    int extra_object_count = 6;

    int last_extra_object_seed = 150;
    int last_extra_object_count = 6;

    // GJK Specific
    cdlib::SteppableGJKEPA gjk;

    std::shared_ptr<ConvexObject> minkowski_object;

    bool show_minkowski_difference = false;

    // V-Clip Specific

    // AABBTREE Specific

    // SAP Specific
    cdlib::SAP sap;

public:
    Application(int initial_width, int initial_height, std::vector<std::string> arguments = {});

    virtual ~Application();

    void compile_shaders() override;

public:
    void prepare_cameras();
    void prepare_textures();
    void prepare_lights();
    void prepare_scene();

    void prepare_convex_objects();
    void prepare_collision_detectors();

    void create_vertex_highlight_objects();
    void draw_direction_highlights();

    static std::shared_ptr<Geometry> create_line_geometry(const glm::vec3& from, const glm::vec3& to);
    static std::shared_ptr<Geometry> create_line_geometry(const glm::vec3& origin, const glm::vec3& direction, float length);

    static std::vector<std::array<float, 3>> random_points(int seed);
    // std::pair<Geometry, std::vector<float>> random_convex_geometry(const std::vector<std::array<float, 3>>& points);
    static std::pair<std::shared_ptr<Geometry>, std::vector<float>> generate_convex_hull_geometry(const std::vector<glm::vec3>& points);
    static std::pair<std::shared_ptr<Geometry>, std::vector<float>> generate_convex_hull_geometry(const std::vector<std::array<float, 3>>& points);

    static std::vector<glm::vec3> get_positions_from_buffer(const std::vector<float>& buffer);

    static std::vector<glm::vec3> get_minkowski_difference_positions(const std::vector<glm::vec3>& positions_1, const std::vector<glm::vec3>& positions_2);

    static glm::vec3 pseudorandom_point(int seed);

    void recalculate_positions();

    void recalculate_minkowski_difference();

    // ----------------------------------------------------------------------------
    // Update
    // ----------------------------------------------------------------------------
    /**
     * {@copydoc PV227Application::update}
     */
    void update(float delta) override;

    void update_object_positions();

    void gjk_step_visualize(cdlib::SteppableGJKState state);

    // ----------------------------------------------------------------------------
    // Render
    // ----------------------------------------------------------------------------
public:
    /** @copydoc PV227Application::render */
    void render() override;
    /** Renders the whole scene. */
    void render_scene();
    /**
     * Renders a single object using the specified program.
     *
     * @param 	object 	The object to render.
     * @param 	program	The program to be used.
     */
    static void render_object(SceneObject& object, ShaderProgram& program) ;

    // ----------------------------------------------------------------------------
    // GUI
    // ----------------------------------------------------------------------------
public:
    /** @copydoc PV227Application::render_ui */
    void render_ui() override;

    // ----------------------------------------------------------------------------
    // Events
    // ----------------------------------------------------------------------------
public:
    void on_key_pressed(int key, int scancode, int action, int mods) override;

    void on_resize(int width, int height) override;

    // ----------------------------------------------------------------------------
    // Testing
    // ----------------------------------------------------------------------------
public:
    void run_tests();

    static std::shared_ptr<cdlib::ConvexPolyhedron> create_test_cube_voronoi(glm::mat4 model_matrix = glm::mat4(1.0f));
    static std::shared_ptr<cdlib::ConvexPolyhedron> create_test_polyhedron_voronoi(float alpha = 67.0f, float p_z = 1.0f, float h_vdz = -1.0f, float h_vdy = -1.3f);

    bool test_voronoi_planes();
    bool test_clip_edge();
};