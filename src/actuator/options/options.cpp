//
// Created by lcseu on 18-8-10.
//
#include <iostream>
#include "options.hpp"

using namespace std;
using namespace boost::program_options;

options::options(): opts_desc_("  Options description")
{
    opts_desc_.add_options()
    ("help,h", "Print this message and exit.")
    ("player,p", value<int>()->default_value(0), "Player ID number.")
    ("debug,d", value<bool>()->default_value(false), "If you want to start in debug mode.")
    ("gc,g", value<bool>()->default_value(false), "If you want to use gamecontroller.")
    ("camera,c", value<bool>()->default_value(true), "If you want to use camera.")
    ("robot,r", value<bool>()->default_value(true), "If you want to use robot.")
    ("mote,m", value<bool>()->default_value(false), "If you want to use remote.")
    ("say,s", value<bool>()->default_value(false), "If you want to use communication.");
}

bool options::init(int argc, char **argv)
{
    try
    {
        store(parse_command_line(argc, argv, opts_desc_), var_map_);
        id_ = arg<int>("player");
        use_debug_ = arg<bool>("debug");
        use_gc_ = arg<bool>("gc");
        use_camera_ = arg<bool>("camera");
        use_robot_ = arg<bool>("robot");
        use_comm_ = arg<bool>("say");
        use_remote_ = arg<bool>("mote");
        if (var_map_.count("help"))
        {
            cout<<"\033[31m"<< opts_desc_ <<"\033[0m"<< endl;
            return false;
        }
    }
    catch (boost::program_options::unknown_option &e)
    {
        cerr<<"\033[31m"<< e.what() <<"\033[0m"<< endl;
    }
    catch (std::exception &e)
    {
        cerr<<"\033[31m"<< e.what() <<"\033[0m"<< endl;
        return false;
    }
    return true;
}