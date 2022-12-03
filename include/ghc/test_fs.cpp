//
// Created by waxz on 2020/5/9.
//

#if defined(__cplusplus) && __cplusplus >= 201703L && defined(__has_include)
#if __has_include(<filesystem>)
#define GHC_USE_STD_FS
#include <filesystem>
namespace fs = std::filesystem;
#endif
#endif
#ifndef GHC_USE_STD_FS
#include "filesystem.hpp"
//#include "ghc/filesystem.hpp"

namespace fs = ghc::filesystem;
#endif

#include <fstream>
#include <iostream>


int main(){
    fs::create_directories("sandbox/a/b");
    std::ofstream("sandbox/file1.txt");
    std::ofstream("sandbox/file2.txt");
    for(auto& p: fs::directory_iterator("sandbox")){
        std::cout <<"directory_iterator: " <<  p.path() << '\n';

    }

    for(auto& p: fs::recursive_directory_iterator("sandbox")){
        std::cout << "recursive_directory_iterator: " << p.path() << '\n';
        std::cout << "is_directory: " << p.is_directory() << "\n" ;
        std::cout << "is_regular_file: " << p.is_regular_file() << "\n" ;

        if(p.is_regular_file()){

            std::cout << "filename: " << p.path().filename() << "\n" ;
            std::cout << "root_path: " << p.path().root_path() << "\n" ;
            std::cout << "relative_path: " << p.path().relative_path() << "\n" ;
            std::cout << "root_name: " << p.path().root_name() << "\n" ;
            std::cout << "root_directory: " << p.path().root_directory() << "\n" ;
            std::cout << "extension: " << p.path().extension() << "\n" ;
            std::cout << "file_size: " << p.file_size() << "\n" ;
            std::cout << "Current path is " << fs::current_path() << '\n'
                      << "Absolute path for " << p << " is " << fs::absolute(p) << '\n';
        }
    }

    try {
        fs::copy("sandbox", "sandbox_copy", fs::copy_options::recursive);

    }catch (fs::filesystem_error e){
        std::cout << e.what() << std::endl;
     }

    std::uintmax_t n = fs::remove_all("sandbox");
    std::cout << "Deleted " << n << " files or directories\n";

}