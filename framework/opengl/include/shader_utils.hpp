#pragma once
#include <fstream>
#include <iostream>

/**
 * The class providing a collection of static utility methods for working with shaders.
 *
 * @author	<a href="mailto:jan.byska@gmail.com">Jan Byška</a>
 */
class ShaderUtils {

public:
    /**
     * The sole purpose of this class is to load a file and extract the text that is in it. In theory, this class could be used
     * for a variety of text- processing purposes, but it was initially designed to be used to load shader source code. Since the
     * entire class is a static class, you only have to add this single line of code to your project:
     * --------------------------------------------------------------------------------------
     * std::string shaderSource = Utils::load("./path/to/shader.extension");
     * --------------------------------------------------------------------------------------
     * This will (recursively) extract the source code from the first shader file. Now, you might be wondering, what is the point
     * of using your code for something so trivial as to loading a file and calling the "std::getline()" function on it? Well,
     * besides loading the shader source code from a single file, the loader also supports custom keywords that allow you to
     * include external files inside your shader source code!
     *
     * The source code was shared under MIT License.
     *
     * @author	Tahar Meijs (https://github.com/tntmeijs/GLSL-Shader-Includes)
     * @param 	path			The path to the "main" shader file.
     * @param 	include_indentifier	The include indentifier.
     * @return	The loaded string.
     */
    static std::string load_shader(const std::filesystem::path& input_path, std::string include_indentifier = "#pragma include") {
        std::filesystem::path path = input_path;
        path.make_preferred();
        std::filesystem::path directory = path;
        directory.remove_filename();

        include_indentifier += ' ';
        static bool is_recursive_call = false;

        std::string full_source_code = "";
        std::ifstream file(path);

        if (!file.is_open()) {
            std::cerr << "ERROR: could not open the shader at: " << path << "\n" << std::endl;
            return full_source_code;
        }

        std::string line_buffer;
        while (std::getline(file, line_buffer)) {
            // Looks for the new shader include identifier.
            if (line_buffer.find(include_indentifier) != line_buffer.npos) {
                // Removes the include identifier, this will cause the path to remain.
                line_buffer.erase(0, include_indentifier.size());

                // The include path is relative to the current shader file path.
                const std::filesystem::path include_path = directory / line_buffer;

                // By using recursion, the new include file can be extracted
                // and inserted at this location in the shader source code.
                is_recursive_call = true;
                full_source_code += load_shader(include_path);

                // Do not add this line to the shader source code, as the include
                // path would generate a compilation issue in the final source code.
                continue;
            }

            full_source_code += line_buffer + '\n';
        }

        // Only add the null terminator at the end of the complete file,
        // essentially skipping recursive function calls this way.
        if (!is_recursive_call)
            full_source_code += '\0';

        file.close();

        return full_source_code;
    }
};