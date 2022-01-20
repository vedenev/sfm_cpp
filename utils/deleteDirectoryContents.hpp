#ifndef DELETEDIRECTORYCONTENTS_HPP_INCLUDED
#define DELETEDIRECTORYCONTENTS_HPP_INCLUDED

#include <filesystem>
namespace fs = std::filesystem;

void deleteDirectoryContents(const fs::path& dir_path);

#endif