#pragma once
#include "camera_ubo.hpp"
#include "gjk.hpp"
#include "light_ubo.hpp"
#include "pv227_application.hpp"
#include "scene_object.hpp"

#include "QuickHull.hpp"

enum CollisionDetectionMethod {
    GJK_EPA = 0,
    V_CLIP = 1,
    AABBTREE = 2,
    SAP = 3
};

class Application : public PV227Application {
    // ----------------------------------------------------------------------------
    // Variables (Geometry)
    // ----------------------------------------------------------------------------
protected:
    /** The remaining scene objects. */
    std::vector<SceneObject> scene_objects;

    std::vector<float> convex_object_vertices_1;
    std::vector<float> convex_object_vertices_2;
    std::vector<float> minkowski_difference_vertices;

    std::vector<glm::vec3> convex_object_positions_1;
    std::vector<glm::vec3> convex_object_positions_2;
    std::vector<glm::vec3> minkowski_difference_positions;

    std::shared_ptr<ConvexCollider> convex_mesh_1;
    std::shared_ptr<ConvexCollider> convex_mesh_2;

    SceneObject convex_object_1;
    SceneObject convex_object_2;
    SceneObject minkowski_difference;

    Geometry convex_geometry_1;
    Geometry convex_geometry_2;
    Geometry minkowski_difference_geometry;

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

    // ----------------------------------------------------------------------------
    // Variables (Textures)
    // ----------------------------------------------------------------------------

    // ----------------------------------------------------------------------------
    // Variables (Light)
    // ----------------------------------------------------------------------------
protected:
    /** The UBO storing the data about lights - positions, colors, etc. */
    PhongLightsUBO phong_lights_ubo;

    float light_position_rad = glm::radians(60.f);

    // ----------------------------------------------------------------------------
    // Variables (Camera)
    // ----------------------------------------------------------------------------
protected:
    /** The UBO storing the information about camera. */
    CameraUBO camera_ubo;

    // ----------------------------------------------------------------------------
    // Variables (Shaders)
    // ----------------------------------------------------------------------------

    // ----------------------------------------------------------------------------
    // Variables (Systems)
    // ----------------------------------------------------------------------------

    cdlib::SteppableGJKEPA gjk;

    // ----------------------------------------------------------------------------
    // Variables (GUI)
    // ----------------------------------------------------------------------------
protected:
    bool show_gui = true;

    CollisionDetectionMethod selected_method = GJK_EPA;

    float object_distance = 1.0f;
    float last_object_distance = 1.0f;

    int object_seed_1 = 7;
    int object_seed_2 = 5;

    int last_object_seed_1 = 7;
    int last_object_seed_2 = 5;

    bool show_wireframe = true;
    bool auto_calculate_collision = false;
    bool step_by_step = false;

    bool show_minkowski_difference = false;
    bool show_convex_objects = true;

    // ----------------------------------------------------------------------------
    // Constructors
    // ----------------------------------------------------------------------------
public:
    Application(int initial_width, int initial_height, std::vector<std::string> arguments = {});

    /** Destroys the {@link Application} and releases the allocated resources. */
    virtual ~Application();

    // ----------------------------------------------------------------------------
    // Shaders
    // ----------------------------------------------------------------------------
    /**
     * {@copydoc PV227Application::compile_shaders}
     */
    void compile_shaders() override;

    // ----------------------------------------------------------------------------
    // Initialize Scene
    // ----------------------------------------------------------------------------
public:
    /** Prepares the required cameras. */
    void prepare_cameras();
    /** Prepares the required textures. */
    void prepare_textures();
    /** Prepares the lights. */
    void prepare_lights();
    /** Prepares the scene objects. */
    void prepare_scene();

    void prepare_convex_objects();

    void create_vertex_highlight_objects();
    void draw_direction_highlights();

    static Geometry create_line_geometry(const glm::vec3& from, const glm::vec3& to);
    static Geometry create_line_geometry(const glm::vec3& origin, const glm::vec3& direction, float length);

    static std::vector<std::array<float, 3>> random_points(int seed);
    // std::pair<Geometry, std::vector<float>> random_convex_geometry(const std::vector<std::array<float, 3>>& points);
    static std::pair<Geometry, std::vector<float>> generate_convex_hull_geometry(const std::vector<std::array<float, 3>>& points);

    static std::vector<glm::vec3> get_positions_from_buffer(const std::vector<float>& buffer);

    static std::vector<glm::vec3> get_minkowski_difference_positions(const std::vector<glm::vec3>& positions_1, const std::vector<glm::vec3>& positions_2);

    void recalculate_positions();

    void recalculate_minkowski_difference();

    // ----------------------------------------------------------------------------
    // Update
    // ----------------------------------------------------------------------------
    /**
     * {@copydoc PV227Application::update}
     */
    void update(float delta) override;

    void update_object_positions(float delta);

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
};