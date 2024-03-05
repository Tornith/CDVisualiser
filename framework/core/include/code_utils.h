#pragma once

/**
 * The class providing a collection of static utility methods.
 *
 * @author	<a href="mailto:jan.byska@gmail.com">Jan Byška</a>
 */
class CodeUtils {
public:
    template <class T> static void hash_combine(std::size_t& seed, const T& v) {
        std::hash<T> hasher;
        seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
};