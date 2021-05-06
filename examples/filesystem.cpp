//
// Created by Harold on 2021/5/6.
//

#include <iostream>
#include <fstream>
#include <cassert>

#define DEBUG
#include "../utils/io/filesystem.h"

using namespace M_FILESYSTEM;

template<typename T>
void print_v(std::vector<T> const& v) {
    for (auto const& e : v)
        std::cout << e << ' ';
    std::cout << std::endl;
}

int main(int argc, char* argv[]) {
    std::string filename = argv[0];
    if (argc > 1) filename = argv[1];

    std::cout << GetFileNameWithoutDir(filename) << std::endl;
    std::cout << GetFileExtensionLower(filename) << std::endl;

    auto dir = GetFileParaentDir(filename);
    std::cout << dir << std::endl;
    std::cout << GetRegularizedDir(dir) << std::endl;
    assert(DirExists(dir));
    auto cwd = GetWorkingDir();
    std::cout << cwd << std::endl;
    assert(MakeDir("tmp"));
    assert(MakeDirFullPath("tmp/tmp1/tmp2"));
    assert(RemoveDir("tmp/tmp1/tmp2"));

    auto new_cwd = cwd + "/tmp";
    assert(ChangeWorkingDir(new_cwd));
    std::cout << GetWorkingDir() << std::endl;
    std::fstream tmp;
    tmp.open("tmp.txt", std::ios::out);
    tmp.close();
    assert(FileExists(filename));

    std::vector<std::string> subdirs, filenames;
    assert(ListContentsInDir(new_cwd, subdirs, filenames));
    print_v(subdirs);
    print_v(filenames);
    
    filenames.clear();
    assert(ListFilesInDirWithPredicate(new_cwd, [](std::string const& fn) { return GetFileNameWithoutDir(fn) == "tmp.txt"; }, filenames));
    print_v(filenames);

    assert(ChangeWorkingDir(cwd));
    // fail here, since rmdir requires dir is empty, need to rm all contents first
    assert(!RemoveDir("tmp"));

    assert(ChangeWorkingDir(new_cwd));
    assert(RemoveFile("tmp.txt"));
    assert(RemoveDir("tmp1"));
    assert(ChangeWorkingDir(cwd));
    std::cout << GetWorkingDir() << std::endl;
    assert(RemoveDir("tmp"));

    return 0;
}