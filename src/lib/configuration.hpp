#pragma once

#include "parser/config_parser.hpp"
#include "singleton.hpp"

class configuration: public singleton<configuration>
{
public:
    bool init(const int &id = 0)
    {
        id_ = id;

        if (!parser::config_parser::parse("data/config.conf", config_tree_))
        {
            return false;
        }

        player_ = "players." + std::to_string(id_);
        team_number_ = config_tree_.get<int>("team_number");
        return true;
    }

    template<typename T>
    inline T get_config_value(const std::string &keyword) const
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

    template<typename T>
    inline std::vector<T> get_config_vector(const std::string &keyword) const
    {
        std::vector<T> v;
        try
        {
            parser::bpt::ptree tpt = config_tree_.get_child(keyword);
            for(auto &t:tpt)
            {
                v.push_back(t.second.get_value<T>());
            }
            return v;
        }
        catch (parser::bpt::ptree_error &e)
        {
            throw class_exception<configuration>("No such keyword: " + keyword);
        }
    }

    std::string get_my_role() const
    {
        parser::bpt::ptree tpt = config_tree_.get_child("strategy");
        for(auto &t:tpt)
        {
            std::string role = t.first;
            parser::bpt::ptree cpt = t.second;
            if(cpt.get<int>("id") == id_)
                return role;
        }
        LOG(LOG_ERROR)<<"can't find my role"<<endll;
        return "";
    }

    inline int id() const
    {
        return id_;
    }
    inline int keeper_id() const
    {
        return config_tree_.get<int>("keeper_id");
    }
    inline int team_number() const
    {
        return team_number_;
    }
    inline std::string player() const
    {
        return player_;
    };
    inline std::string field_file() const
    {
        return config_tree_.get<std::string>("field_file");
    }
    inline std::string robot_file() const
    {
        return config_tree_.get<std::string>(player_ + ".robot_file");
    }
    inline std::string offset_file() const
    {
        return config_tree_.get<std::string>(player_ + ".offset_file");
    }
    inline std::string action_file() const
    {
        return config_tree_.get<std::string>(player_ + ".action_file");
    }

private:
    parser::bpt::ptree config_tree_;
    std::string player_;
    int id_, team_number_;;
};

#define CONF configuration::instance()

