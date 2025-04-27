#pragma once
#include <string>

extern "C" {
    #include "ff.h"
}

using namespace std;

struct FileEntry {
    string name;
    bool is_dir;
};

struct FileChooser {
    string rootdir;
    string curdir;
    string curfile;
    FATFS *fs;

    FileChooser();
    void set_fs(FATFS *fs);
    bool choose_file(string &res);      // return true if a file was chosen
    bool list_files(string dir, vector<FileEntry> &files, int start, int len, int *count);
};
