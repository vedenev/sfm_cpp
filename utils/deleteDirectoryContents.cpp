#include "deleteDirectoryContents.hpp"
#include <filesystem>
namespace fs = std::filesystem;

//https://stackoverflow.com/questions/59077670/c-delete-all-files-and-subfolders-but-keep-the-directory-itself
void deleteDirectoryContents(const fs::path& dir_path) {
    for (auto& path: fs::directory_iterator(dir_path)) {
        fs::remove_all(path);
    }
}

