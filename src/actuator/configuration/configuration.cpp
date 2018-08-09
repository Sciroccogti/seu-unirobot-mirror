#include "configuration.hpp"

namespace config
{
    using namespace std;
    using namespace parser;

    configuration::configuration()
    {

    }

    bool configuration::init(int argc, char *argv[])
    {
        if(!opts_.parse_command(argc, argv)) return false;
        if(!config_parser::parse("data/actuator.conf", config_tree_)) return false;
        player_ = "players." + to_string(opts_.id());
        return true;
    }
}