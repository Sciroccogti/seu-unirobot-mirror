#include "options.hpp"
#include "logger.hpp"

using namespace std;
using namespace boost::program_options;

options::options(): opts_desc_("  Options description")
{
    opts_desc_.add_options()
    ("help,h", "Print this message and exit.")
    ("player,p", value<int>()->default_value(0),
            "Player ID number.")
    ("debug,d", value<bool>()->default_value(false),
            "If you want to start in debug mode.")
    ("gc,g", value<bool>()->default_value(false),
            "If you want to use gamecontroller.")
    ("camera,c", value<bool>()->default_value(true),
            "If you want to use camera.")
    ("robot,r", value<int>()->default_value(1),
            "0: Do not use robot.\n"
            "1: Use real robot.\n"
            "2: Use virtual robot")
    ("mote,m", value<bool>()->default_value(false),
            "If you want to use remote.")
    ("say,s", value<bool>()->default_value(false),
            "If you want to use communication.");
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
        robot_choice_ = static_cast<robot_choice >(arg<int>("robot"));
        use_comm_ = arg<bool>("say");
        use_remote_ = arg<bool>("mote");
        if (var_map_.count("help"))
        {
            LOG(LOG_INFO)<<opts_desc_<<"\n";
            return false;
        }
    }
    catch (boost::program_options::unknown_option &e)
    {
        LOG(LOG_WARN)<<e.what()<<"\n";
    }
    catch (std::exception &e)
    {
        LOG(LOG_ERROR)<<e.what()<<"\n";
        return false;
    }
    return true;
}