#ifndef SEU_UNIROBOT_CONFIGURATION_HPP
#define SEU_UNIROBOT_CONFIGURATION_HPP

#include "options.hpp"
#include "singleton.hpp"
#include "parser/config_parser.hpp"

namespace config
{
    class configuration: public singleton<configuration>
    {
    public:
        configuration();
        bool init(int argc, char *argv[]);

        template<typename T>
        T get_config_value(const std::string &keyword) const
        {
            try
            {
                return config_tree_.get<T>(keyword);
            }
            catch (parser::bpt::ptree_error &e)
            {
                throw class_exception<configuration>("No such keyword: " + keyword);
            }
        }

        int id() const { return opts_.id(); }
        options opts() const { return opts_; }
        std::string robot_file() const { return config_tree_.get<std::string>(player_+".robot_file"); }
        std::string offset_file() const { return config_tree_.get<std::string>(player_+".offset_file"); }
        std::string action_file() const { return config_tree_.get<std::string>(player_+".action_file"); }

    private:
        parser::bpt::ptree config_tree_;
        options opts_;
        std::string player_;
    };

    #define CONF configuration::get_singleton()
}
#endif