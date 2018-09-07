#ifndef SEU_UNIROBOT_CONFIGURATION_HPP
#define SEU_UNIROBOT_CONFIGURATION_HPP

#include "parser/config_parser.hpp"
#include "singleton.hpp"

class configuration: public singleton<configuration>
{
public:
    bool init(const int &id=0)
    {
        id_ = id;
        if(!parser::config_parser::parse("data/config.conf", config_tree_)) return false;
        player_ = "players." + std::to_string(id_);
        return true;
    }

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

    int id() const { return id_; }
    std::string player() const { return player_; };
    std::string field_file() const { return config_tree_.get<std::string>("field_file"); }
    std::string robot_file() const { return config_tree_.get<std::string>(player_+".robot_file"); }
    std::string offset_file() const { return config_tree_.get<std::string>(player_+".offset_file"); }
    std::string action_file() const { return config_tree_.get<std::string>(player_+".action_file"); }

private:
    parser::bpt::ptree config_tree_;
    std::string player_;
    int id_;
};

#define CONF configuration::instance()
#endif 
