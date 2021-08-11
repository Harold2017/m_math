//
// Created by Harold on 2021/5/6.
//

#ifndef M_MATH_IO_FILESYSTEM_H
#define M_MATH_IO_FILESYSTEM_H

#include <string>
#include <algorithm>
#include <vector>
#include <functional>
#include <cstdio>

#define MAX_PATH_LENGTH 255
#ifdef _WIN32
    #include <direct.h>
    #define getcwd _getcwd // MSVC "deprecation" warning
    #define rmdir _rmdir

    // windows requires define _CRT_INTERNAL_NONSTDC_NAMES first to define stat, S_ISDIR/S_ISREG macros in <sys/stat.h>
    #define _CRT_INTERNAL_NONSTDC_NAMES 1
    #include <sys/stat.h>
    #if !defined(S_ISDIR) && defined(S_IFMT) && defined(S_IFDIR)
        #define S_ISDIR(m) (((m) & S_IFMT) == S_IFDIR)
    #endif
    #if !defined(S_ISREG) && defined(S_IFMT) && defined(S_IFREG)
        #define S_ISREG(m) (((m) & S_IFMT) == S_IFREG)
    #endif

    // linux compatible dirent interface for windows (https://github.com/tronkko/dirent)
    // otherwise, need to use WIN32 API (FindFirstFile(), FindNextFile() and FindClose()) in windows.h
    #include "internal/dirent.h"

#else
    // chdir(), rmdir()
    #include <unistd.h>
    // closedir(), opendir(), readdir(), rewinddir(), seekdir(), telldir()
    #include <dirent.h> 
#endif

namespace M_FILESYSTEM {
    std::string GetFileNameWithoutDir(std::string const& filename) {
        auto slash_pos = filename.find_last_of("/\\");
        return slash_pos == std::string::npos ? filename : filename.substr(slash_pos + 1);
    }

    std::string GetFileParaentDir(std::string const& filename) {
        auto slash_pos = filename.find_last_of("/\\");
        return slash_pos == std::string::npos ? "" : filename.substr(0, slash_pos + 1);  // including ending slash
    }

    std::string GetFileExtensionLower(std::string const& filename) {
        auto dot_pos = filename.find_last_of('.');
        // no dot or find floder slash after dot
        if ((dot_pos >= filename.size()) || (filename.find_first_of("/\\", dot_pos) != std::string::npos)) return "";
        std::string ext = filename.substr(dot_pos + 1);
        std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c){ return std::tolower(c); });
        return ext;
    }

    std::string GetRegularizedDir(std::string const& dirname) {
        return dirname.back() != '/' && dirname.back() != '\\' ? dirname + "/" : dirname;
    }

    std::string GetWorkingDir() {
        char buff[MAX_PATH_LENGTH + 1];
        getcwd(buff, MAX_PATH_LENGTH + 1);
        return std::string(buff);
    }

    bool ChangeWorkingDir(std::string const& dirname) {
        return chdir(dirname.c_str()) == 0;
    }

    bool DirExists(std::string const& dirname) {
        struct stat info;
        if (stat(dirname.c_str(), &info) == -1) return false;
        return S_ISDIR(info.st_mode);
    }

    bool MakeDir(std::string const& dirname) {
#ifdef _WIN32
        return _mkdir(dirname.c_str()) == 0;
#else
        return mkdir(dirname.c_str(), S_IRWXU) == 0;
#endif
    }

    bool MakeDirFullPath(std::string const& dirname) {
        auto full_path = GetRegularizedDir(dirname);
        auto cur_pos = full_path.find_first_of("/\\", 1);
        while (cur_pos != std::string::npos) {
            auto sub_dir = full_path.substr(0, cur_pos + 1);
            if (!DirExists(sub_dir) && !MakeDir(sub_dir)) return false;
            cur_pos = full_path.find_first_of("/\\", cur_pos + 1);
        }
        return true;
    }

    bool RemoveDir(std::string const& dirname) {
#ifdef DEBUG
        if (rmdir(dirname.c_str()) == 0) return true;
        else { fprintf(stderr, "Can't remove %s: %s\n", dirname.c_str(), strerror(errno)); return false; }
#endif
        return rmdir(dirname.c_str()) == 0;
    }

    bool FileExists(std::string const& filename) {
        struct stat info;
        if (stat(filename.c_str(), &info) == -1) return false;
        return S_ISREG(info.st_mode);
    }

    bool RemoveFile(std::string const& filename) {
#ifdef DEBUG
        if (std::remove(filename.c_str()) == 0) return true;
        else { fprintf(stderr, "Can't remove %s: %s\n", filename.c_str(), strerror(errno)); return false; }
#endif
        return (std::remove(filename.c_str()) == 0);
    }

    bool ListContentsInDir(std::string const& dirname, std::vector<std::string>& sub_dirs, std::vector<std::string>& filenames) {
        if (dirname.empty()) return false;
        DIR *dir;
        struct dirent *ent;
        struct stat info;
        dir = opendir(dirname.c_str());
        if (!dir) return false;

        sub_dirs.clear();
        filenames.clear();
        while ((ent = readdir(dir)) != nullptr) {
            const std::string filename = ent->d_name;
            if (filename[0] == '.') continue;
            std::string full_filename = GetRegularizedDir(dirname) + filename;
            if (stat(full_filename.c_str(), &info) == -1) continue;
            if (S_ISDIR(info.st_mode))
                sub_dirs.push_back(full_filename);
            else if (S_ISREG(info.st_mode))
                filenames.push_back(full_filename);
        }
        closedir(dir);
        return true;
    }

    bool ListFilesInDir(std::string const& dirname, std::vector<std::string>& filenames) {
        std::vector<std::string> sub_dirs;
        return ListContentsInDir(dirname, sub_dirs, filenames);
    }

    bool ListFilesInDirWithPredicate(std::string const& dirname, std::function<bool(std::string const&)> predicate, std::vector<std::string>& filenames) {
        std::vector<std::string> all_files;
        if (!ListFilesInDir(dirname, all_files)) return false;
        for (auto &f : all_files)
            if (predicate(f)) filenames.push_back(f);
        return true;    
    }

    // for file operations, better to use <fsteam>
}

#endif //M_MATH_IO_FILESYSTEM_H