//
// Created by lcseu on 18-8-10.
//
#include "options.hpp"

using namespace boost::program_options;

options::options(): opts_desc_("  Options description")
{
    opts_desc_.add_options()
    ("help,h", "Print this message and exit.")
    ("player,p", value<int>()->default_value(0), "Player ID number.");
}

bool options::init(int argc, char *argv[])
{
    try
    {
        store(parse_command_line(argc, argv, opts_desc_), var_map_);
        id_ = arg<int>("player");
        if (var_map_.count("help"))
        {
            std::cout<<"\033[31m"<< opts_desc_ <<"\033[0m"<< std::endl;
            return false;
        }
    }
    catch (boost::program_options::unknown_option &e)
    {
        std::cerr<<"\033[31m"<< e.what() <<"\033[0m"<< std::endl;
    }
    catch (std::exception &e)
    {
        std::cerr<<"\033[31m"<< e.what() <<"\033[0m"<< std::endl;
        return false;
    }
    return true;
}
