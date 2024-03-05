// ################################################################################
// Common Framework for Computer Graphics Courses at FI MUNI.
// 
// Copyright (c) 2021-2022 Visitlab (https://visitlab.fi.muni.cz)
// All rights reserved.
// ################################################################################

#pragma once

#include "opengl_utils.hpp"
#include "glm/glm.hpp"
#include "ubo.hpp"

/**
 * The structure holding the information about a single Phong light.
 *
 * @author	<a href="mailto:jan.byska@gmail.com">Jan Byška</a>
 * @author	<a href="mailto:cejka.honza@gmail.com ">Jan Čejka</a>
 */
struct PhongLightData {
    /** The position of the light. Note that position.w should be one for point lights and spot lights, and zero for directional lights. */
    glm::vec4 position{};
    /** The ambient part of the color of the light. */
    glm::vec3 ambient{};
    /** The padding to avoid problems with data alignment. */
    float padding1{};
    /** The diffuse part of the color of the light. */
    glm::vec3 diffuse{};
    /** The padding to avoid problems with data alignment. */
    float padding2{};
    /** The specular part of the color of the light. */
    glm::vec3 specular{};
    /** The padding to avoid problems with data alignment. */
    float padding3{};
    /** The direction of the spot light, irrelevant for point lights and directional lights. */
    glm::vec3 spot_direction{};
    /** The spot exponent of the spot light, irrelevant for point lights and directional lights. */
    float spot_exponent;
    /** The cosine of the spot light's cutoff angle, -1 point lights, irrelevant for directional lights*/
    float spot_cos_cutoff;
    /** The constant attenuation of spot lights and point lights, irrelevant for directional lights. For no attenuation, set this to 1. */
    float atten_constant;
    /** The linear attenuation of spot lights and point lights, irrelevant for directional lights.  For no attenuation, set this to 0. */
    float atten_linear;
    /** The quadratic attenuation of spot lights and point lights, irrelevant for directional lights. For no attenuation, set this to 0. */
    float atten_quadratic;

    // ----------------------------------------------------------------------------
    // Constructors
    // ----------------------------------------------------------------------------
public:
    /** Constructs a new @link CameraData with identity matrices. */
    PhongLightData()
        : position(glm::vec4(0.0f)), ambient(glm::vec3(0.0f)), padding1(0), diffuse(glm::vec3(0.0f)), padding2(0),
          specular(glm::vec3(0.0f)), padding3(0), spot_direction(glm::vec3(0.0f)),
          spot_exponent(0), spot_cos_cutoff(0), atten_constant(0), atten_linear(0), atten_quadratic(0) {
    }

    /**
     * Constructs a @link PhongLightData with specified values.
     */
    PhongLightData(const glm::vec4& position, const glm::vec3& ambient, const glm::vec3& diffuse, const glm::vec3& specular,
                   const glm::vec3& spot_direction, float spot_exponent,
                   float spot_cos_cutoff, float atten_constant, float atten_linear, float atten_quadratic)
        : position(position), ambient(ambient), diffuse(diffuse), specular(specular), spot_direction(spot_direction),
          spot_exponent(spot_exponent), spot_cos_cutoff(spot_cos_cutoff),
          atten_constant(atten_constant), atten_linear(atten_linear), atten_quadratic(atten_quadratic) {
    }

    // ----------------------------------------------------------------------------
    // Factory Methods
    // ----------------------------------------------------------------------------
public:
    /** Factory method for a point light, sets all attributes that are not specified to default values. */
    static PhongLightData CreatePointLight(const glm::vec3& position, const glm::vec3& ambient, const glm::vec3& diffuse,
                                           const glm::vec3& specular) {
        return CreatePointLight(position, ambient, diffuse, specular, 1.0f, 0.0f, 0.0f);
    }

    /** Factory method for a point light, sets all attributes that are not specified to default values. */
    static PhongLightData CreatePointLight(const glm::vec3& position, const glm::vec3& ambient, const glm::vec3& diffuse,
                                           const glm::vec3& specular, float atten_constant, float atten_linear,
                                           float atten_quadratic) {
        return PhongLightData(glm::vec4(position, 1.0f), ambient, diffuse, specular, glm::vec3(0.0f, 0.0f, 1.0f), 0.0f, -1.0f,
                              atten_constant, atten_linear, atten_quadratic);
    }

    /** Factory method for a directional light, sets all attributes that are not specified to default values. */
    static PhongLightData CreateDirectionalLight(const glm::vec3& position, const glm::vec3& ambient, const glm::vec3& diffuse,
                                                 const glm::vec3& specular) {
        return PhongLightData(glm::vec4(position, 0.0f), ambient, diffuse, specular, glm::vec3(0.0f, 0.0f, 1.0f), 0.0f, -1.0f, 1.0f, 0.0f,
                              0.0f);
    }

    /**  Factory method for a spot light, sets all attributes that are not specified to default values. */
    static PhongLightData CreateSpotLight(const glm::vec3& position, const glm::vec3& ambient, const glm::vec3& diffuse,
                                          const glm::vec3& specular, const glm::vec3& spot_direction,
                                          float spot_exponent, float spot_cos_cutoff) {
        return CreateSpotLight(position, ambient, diffuse, specular, spot_direction, spot_exponent, spot_cos_cutoff, 1.0f, 0.0f, 0.0f);
    }

    /**  Factory method for a spot light, sets all attributes that are not specified to default values. */
    static PhongLightData CreateSpotLight(const glm::vec3& position, const glm::vec3& ambient, const glm::vec3& diffuse,
                                          const glm::vec3& specular, const glm::vec3& spot_direction,
                                          float spot_exponent, float spot_cos_cutoff, float atten_constant, float atten_linear,
                                          float atten_quadratic) {
        return PhongLightData(glm::vec4(position, 1.0f), ambient, diffuse, specular, spot_direction, spot_exponent, spot_cos_cutoff,
                              atten_constant, atten_linear, atten_quadratic);
    }

    /** Factory method for a head light. Head light is a point light whose position is at the position of the camera. */
    static PhongLightData CreateHeadLight(const glm::mat4& view_matrix, const glm::vec3& ambient, const glm::vec3& diffuse,
                                          const glm::vec3& specular) {
        // The position of the camera (i.e., eye) is the translation part (the last column) of the inverse of the view matrix.
        return CreatePointLight(glm::vec3(inverse(view_matrix)[3]), ambient, diffuse, specular);
    }
};

/** The structure holding the metadata about the number of lights in the scene and global ambient color. */
struct PhongLightsMetaData {
    /** The global ambient color. */
    glm::vec3 global_ambient_color = {0, 0, 0};
    /** The number of lights stored in this buffer. */
    int lights_count = 0;
};

/**
 * This class contains the data of all Phong lights in the scene.
 *  It also contains information about the global scene ambient light, as do the old OpenGL.
 *
 *  The buffer is initialized for maximum of eight lights by default, but this number may be increased. To change
 *  the number of the lights, change the number of lights in GLSL, and construct the buffer with the requested
 *  number of lights.
 *
 *  Use this code in shaders:
 *  <code>
 *  struct PhongLight
 *  {
 * 	   vec4 position;                   // The position of the light. Note that position.w should be one for point lights and spot lights, and zero for directional lights.
 * 	   vec3 ambient;                    // The ambient part of the color of the light.
 * 	   vec3 diffuse;                    // The diffuse part of the color of the light.
 * 	   vec3 specular;                   // The specular part of the color of the light.
 * 	   vec3 spot_direction;             // The direction of the spot light, irrelevant for point lights and directional lights.
 * 	   float spot_exponent;             // The spot exponent of the spot light, irrelevant for point lights and directional lights.
 * 	   float spot_cos_cutoff;           // The cosine of the spot light's cutoff angle, -1 point lights, irrelevant for directional lights.
 * 	   float atten_constant;            // The constant attenuation of spot lights and point lights, irrelevant for directional lights. For no attenuation, set this to 1.
 * 	   float atten_linear;              // The linear attenuation of spot lights and point lights, irrelevant for directional lights.  For no attenuation, set this to 0.
 * 	   float atten_quadratic;           // The quadratic attenuation of spot lights and point lights, irrelevant for directional lights. For no attenuation, set this to 0.
 * 	};
 *
 * 	layout(std140, binding = 2) uniform PhongLightsBuffer
 *  {
 * 	   vec3 global_ambient_color;
 * 	   int lights_count;
 * 	   PhongLight lights[8];
 *  };
 *  </code>
 */
class PhongLightsUBO : public UBO<PhongLightData> {
    // ----------------------------------------------------------------------------
    // Layout Asserts
    // ----------------------------------------------------------------------------
    static_assert(offsetof(PhongLightData, position) == 0, "Incorrect PhongLightData layout.");
    static_assert(offsetof(PhongLightData, ambient) == 16, "Incorrect PhongLightData layout.");
    static_assert(offsetof(PhongLightData, diffuse) == 32, "Incorrect PhongLightData layout.");
    static_assert(offsetof(PhongLightData, specular) == 48, "Incorrect PhongLightData layout.");
    static_assert(offsetof(PhongLightData, spot_direction) == 64, "Incorrect PhongLightData layout.");
    static_assert(offsetof(PhongLightData, spot_exponent) == 76, "Incorrect PhongLightData layout.");
    static_assert(offsetof(PhongLightData, spot_cos_cutoff) == 80, "Incorrect PhongLightData layout.");
    static_assert(offsetof(PhongLightData, atten_constant) == 84, "Incorrect PhongLightData layout.");
    static_assert(offsetof(PhongLightData, atten_linear) == 88, "Incorrect PhongLightData layout.");
    static_assert(offsetof(PhongLightData, atten_quadratic) == 92, "Incorrect PhongLightData layout.");
    static_assert(sizeof(PhongLightData) == 96, "Incorrect PhongLightData layout.");

    // ----------------------------------------------------------------------------
    // Variables
    // ----------------------------------------------------------------------------
protected:
    /** The metadata about the number of lights in the scene and global ambient color. */
    PhongLightsMetaData header;
    /** The maximum number of lights the buffer can contain. */
    size_t max_lights_count;

private:
    // ----------------------------------------------------------------------------
    // Constructors
    // ----------------------------------------------------------------------------
public:
    /**
     * Constructs a new @link PhongLightsUBO.
     *
     * @param 	lights_count	TRhe maximum number of lights stored in this buffer.
     * @param 	target			The target to which this buffer can be bound.
     */
    PhongLightsUBO(int lights_count = 8, GLenum target = GL_UNIFORM_BUFFER)
        : UBO<PhongLightData>(OpenGLUtils::get_opengl_version() >= 4.5f ? GL_DYNAMIC_STORAGE_BIT : GL_DYNAMIC_DRAW, target, true /* we initialize the OpenGL buffer ourselves */) {
        // The UBO constructor makes a default instance.
        data.clear();

        // Stores the header data.
        max_lights_count = lights_count;

       create_ubo_on_gpu();
    }

    /**
     * Constructs a new @link @link PhongLightsUBO from another (copy constructor that performs a deep copy of the buffer).
     *
     * @param 	other	The other buffer that will be copied.
     */
    PhongLightsUBO(const PhongLightsUBO& other)
        : UBO<PhongLightData>(other), header(other.header), max_lights_count(other.max_lights_count) {
        create_ubo_on_gpu();
    }

    /**
     * Constructor that moves the buffer to the new object. Zeroes out the old object so that OpenGL's destructor does
     * nothing (the zeroing is done via initializing the object with empty constructor).
     *
     * @param 	other	The other buffer that will be moved.
     */
    PhongLightsUBO(PhongLightsUBO&& other)
        : PhongLightsUBO() { swap_fields(*this, other); }

private:
    /**
     * Creates a GPU representation of the UBO.
     * Note that the implementation can be found in respective OpenGL 3.3 or 4.5 modules.
     */
    void create_ubo_on_gpu();

    // ----------------------------------------------------------------------------
    // Operators
    // ----------------------------------------------------------------------------
public:
    /**
     * The copy assignment using copy-and-swap idiom.
     *
     * @param other The other buffer that will be copied (i.e., swapped into this).
     * @return A shallow copy of the buffer object that was moved into this object.
     */
    PhongLightsUBO& operator=(PhongLightsUBO other) {
        swap_fields(*this, other);
        return *this;
    }

    // ----------------------------------------------------------------------------
    // Methods
    // ----------------------------------------------------------------------------
public:
    /**
     * The custom swap method that exchanges the values of fields of two buffers.
     *
     * @param 	first 	The first buffer to swap.
     * @param 	second	The second buffer to swap.
     */
    void swap_fields(PhongLightsUBO& first, PhongLightsUBO& second) noexcept {
        UBO<PhongLightData>::swap_fields(first, second);
        std::swap(first.header, second.header);
        std::swap(first.max_lights_count, second.max_lights_count);
    }

    /**
     * Copies the data from CPU to GPU.
     * Note that the implementation can be found in respective OpenGL 3.3 or 4.5 modules.
     */
    void update_opengl_data() override;

    /** Removes the existing lights. */
    void clear() {
        data.clear();
        header.lights_count = std::min(static_cast<int>(max_lights_count), static_cast<int>(data.size()));
    }

    /** Adds a new light. */
    void add(PhongLightData light) {
        data.push_back(light);
        header.lights_count = std::min(static_cast<int>(max_lights_count), static_cast<int>(data.size()));
    }

    // ----------------------------------------------------------------------------
    // Getters & Setters
    // ----------------------------------------------------------------------------
public:
    /**
     * Set the intensity of the global ambient light.
     *
     * @param 	global_ambient_color	The global ambient light.
     */
    void set_global_ambient(const glm::vec3& global_ambient_color) { header.global_ambient_color = global_ambient_color; }

    /**
     * Returns the lights stored in the buffer.
     *
     * @return	The lights stored in the buffer.
     */
    std::vector<PhongLightData>& get_lights() { return this->data; }

    /**
     * Returns a particular light stored in the buffer on the specified position.
     *
     * @return	The light stored in the buffer on the specified position.
     */
    PhongLightData& get_light(int index) {
        // Fails if the light index is out of range.
        assert(index > 0 || index < this->data.size());
        return this->data[index];
    }
};
