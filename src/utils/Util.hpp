#ifndef DRONE_UTILS_HPP
#define DRONE_UTILS_HPP

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <stdexcept>

namespace drone {

inline std::string readFile(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) throw std::runtime_error("cannot open file: " + path);
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

inline std::string readStdin() {
    std::ostringstream ss;
    ss << std::cin.rdbuf();
    return ss.str();
}

inline void writeFile(const std::string& path, const std::string& text) {
    std::ofstream out(path, std::ios::binary);
    if (!out) throw std::runtime_error("cannot write file: " + path);
    out << text;
}

namespace log {
inline void info(const std::string& m)  { std::cerr << "[INFO] "  << m << "\n"; }
inline void warn(const std::string& m)  { std::cerr << "[WARN] "  << m << "\n"; }
inline void error(const std::string& m) { std::cerr << "[ERROR] " << m << "\n"; }
}  // namespace log

}  // namespace drone

#endif  // DRONE_UTILS_HPP
