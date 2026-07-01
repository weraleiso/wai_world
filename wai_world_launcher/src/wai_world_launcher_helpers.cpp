#include<wai_world_launcher_helpers.h>


void GetSetArgTextfile(std::string s_filepath,
                       std::string s_arg_name,
                       double* d_arg_value,
                       std::string* s_arg_value_text,
                       std::string s_arg_new_value)
{
    // Prepare text file
    std::ifstream ifs_textfile(s_filepath);
    std::vector<std::string > vec_textfile_lines;
    std::string s_textfile_line="";
    if(!ifs_textfile)
    {
        ROS_WARN_STREAM("Launcher - Could not open wai_world_bringup launch file!");
        return;
    }

    std::string s_line_arg="<arg name=\""+s_arg_name; // Skip last quoation marks for now
    std::string s_line_arg_default="default=\"";
    std::string s_line_arg_default_end="\"/>"; // Todo: Make more robust, if written 'default="..." />' aso.

    while(std::getline(ifs_textfile,s_textfile_line))
    {
        // Find argument name in line of textfile
        size_t pos_s_par = s_textfile_line.find(s_line_arg);
        if(pos_s_par!=std::string::npos)
        {
            size_t siz_pos_arg_default_start = s_textfile_line.find(s_line_arg_default);
            size_t len_arg_default = s_line_arg_default.length();
            size_t siz_pos_arg_default_end = s_textfile_line.find(s_line_arg_default_end);
            size_t len_arg_default_end = s_line_arg_default_end.length();
            if(siz_pos_arg_default_start!=std::string::npos &&
                siz_pos_arg_default_end!=std::string::npos)
            {
                *s_arg_value_text=s_textfile_line.substr(
                            siz_pos_arg_default_start+len_arg_default,
                            siz_pos_arg_default_end-(siz_pos_arg_default_start+len_arg_default));
                try
                {
                    *d_arg_value=std::stod(*s_arg_value_text);
                }
                catch(std::exception e)
                {
                    *d_arg_value=0.0;
                }

                if(s_arg_new_value.compare("")!=0)
                {
                        s_textfile_line.replace(
                                    siz_pos_arg_default_start,
                                    siz_pos_arg_default_end-siz_pos_arg_default_start+len_arg_default_end,
                                    "default=\""+s_arg_new_value+"\"/>");
                }
            }
        }
        else
        {
            // Do nothing...
        }
        vec_textfile_lines.push_back(s_textfile_line);
    }
    ifs_textfile.close();

    // Write updated data to same textfile, if required
    if(s_arg_new_value.compare("")!=0)
    {
        std::ofstream ofs_textfile(s_filepath);
        for(int i=0;i<vec_textfile_lines.size();i++)
        {
            ofs_textfile << vec_textfile_lines[i] << std::endl;
        }
        ofs_textfile.close();
    }
}
