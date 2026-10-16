#include "a10_startup_config.hpp"
#include <cassert>
#include <chrono>
#include <fstream>
#include <iostream>

int main()
{
    namespace fs = std::filesystem;
    const auto directory = fs::temp_directory_path() / ("a10-startup-" +
        std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    fs::create_directories(directory / "bin");
    const auto old_cwd = fs::current_path();
    // A valid XML in the working directory must not hide a missing bin XML.
    std::ofstream(directory / "kaanh.xml") << "<source/>";
    fs::current_path(directory);
    const auto rejected = [&](const fs::path& explicit_path = {}) {
        try { a10_startup::xml_path(directory / "bin", explicit_path); }
        catch (const std::runtime_error&) { return true; }
        return false;
    };
    assert(rejected());
    std::ofstream(directory / "bin/kaanh.xml");
    assert(rejected());
    std::ofstream(directory / "bin/kaanh.xml") << "<runtime/>";
    assert(a10_startup::xml_path(directory / "bin") == directory / "bin/kaanh.xml");
    assert(a10_startup::xml_path(directory / "bin", "kaanh.xml") == directory / "kaanh.xml");
    assert(rejected("missing.xml"));
    fs::create_directory(directory / "not-a-file.xml");
    assert(rejected("not-a-file.xml"));
    assert(fs::is_directory(a10_startup::executable_directory("unused")));
    fs::current_path(old_cwd);
    fs::remove_all(directory);
    std::cout << "startup config tests passed\n";
}
