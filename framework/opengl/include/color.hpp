#pragma once

#include "glm/glm.hpp"
#include <algorithm>

/**
 * This structure represents a color value in the framework. The class is based on the {@link glm::vec4} and it uses
 * the RGBA (Red-Green-Blue-Alpha) system with float values in the range between 0.0 - 1.0.
 *
 * @author	<a href="mailto:jan.byska@gmail.com">Jan Byška</a>
 */
class Color : public glm::vec4 {

    // ----------------------------------------------------------------------------
    // Variables
    // ----------------------------------------------------------------------------
  public:
    /** The constant for the red color: R=1.0, G=0.0, B=0.0, A=1.0. */
    static const Color RED;
    /** The constant for the green color: R=0.0, G=1.0, B=0.0, A=1.0. */
    static const Color GREEN;
    /** The constant for the blue color: R=0.0, G=0.0, B=1.0, A=1.0. */
    static const Color BLUE;
    /** The constant for the black color: R=0.0, G=0.0, B=0.0, A=1.0. */
    static const Color BLACK;
    /** The constant for the white color: R=1.0, G=1.0, B=1.0, A=1.0. */
    static const Color WHITE;
    /** The constant for the yellow color: R=1.0, G=1.0, B=0.0, A=1.0. */
    static const Color YELLOW;
    /** The constant for the magenta color: R=1.0, G=0.0, B=1.0, A=1.0. */
    static const Color MAGENTA;
    /** The constant for the cyan color: R=0.0, G=1.0, B=1.0, A=1.0. */
    static const Color CYAN;
    /** The constant for the orange color: R=1.0, G=0.43, B=0.2, A=1.0. */
    static const Color ORANGE;
    /** The constant for the default color (gray): R=0.5, G=0.5, B=0.5, A=1.0. */
    static const Color DEFAULT_COLOR;

    // ----------------------------------------------------------------------------
    // Constructors
    // ----------------------------------------------------------------------------
  public:
    /** Constructs a new (black) color. */
    Color() : Color(0, 0, 0, 1) {}

    /**
     * Constructs a new Color using the specified red, green, and blue values.
     *
     * @param red		The red component of the RGBA value.
     * @param green		The green component of the RGBA value.
     * @param blue		The blue component of the RGBA value.
     */
    Color(double red, double green, double blue) : Color(red, green, blue, 1.0) {}

    /**
     * Constructs a new Color using the specified red, green, and blue values.
     *
     * @param red		The red component of the RGBA value.
     * @param green		The green component of the RGBA value.
     * @param blue		The blue component of the RGBA value.
     * @param alpha		The alpha component of the RGBA value.
     */
    Color(double red, double green, double blue, double alpha)
        : glm::vec4(std::max(0.0, std::min(1.0, red)), std::max(0.0, std::min(1.0, green)), std::max(0.0, std::min(1.0, blue)),
                    std::max(0.0, std::min(1.0, alpha))) {}

    /**
     * Constructs a new Color using a hex value.
     *
     * @param colorHex   hex value (Red: 0xff0000, Green: 0x00ff00, Blue: 0x0000ff)
     */
    Color(int colorHex)
        : glm::vec4(((colorHex >> 16) & 0xFF) / 255.0, ((colorHex >> 8) & 0xFF) / 255.0, ((colorHex)&0xFF) / 255.0, 0) {}

    // ----------------------------------------------------------------------------
    // Static methods
    // ----------------------------------------------------------------------------
  public:
    /**
     * Returns a linear interpolation between two colors: (1-t)*c1 + t*c2.
     *
     * @param 	c1	The first Color.
     * @param 	c2	The second Color.
     * @param 	t 	The interpolation factor.
     * @return	The interpolated color.
     */
    static Color interpolate(Color c1, Color c2, double t) {
        return Color((1 - t) * c1.r + t * c2.r, (1 - t) * c1.g + t * c2.g, (1 - t) * c1.b + t * c2.b);
    }
};