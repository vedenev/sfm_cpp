#include <filesystem>
namespace fs = std::filesystem;

void deleteDirectoryContents(const fs::path& dir_path);