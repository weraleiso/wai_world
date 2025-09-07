#include<dirent.h>
#include<string>
#include<string.h>
#include<fstream>

void GetSetArgTextfile(std::string s_filepath,
                       std::string s_arg_name,
                       double* d_arg_value,
                       std::string* s_arg_value_text,
                       std::string s_arg_new_value="");
