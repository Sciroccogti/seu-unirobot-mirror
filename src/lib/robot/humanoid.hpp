#ifndef SEU_UNIROBOT_HUMANOID_HPP
#define SEU_UNIROBOT_HUMANOID_HPP

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include "robot_define.hpp"
#include "singleton.hpp"
#include "parser/robot_parser.hpp"
#include "parser/action_parser.hpp"
#include "parser/offset_parser.hpp"

namespace robot
{

    class humanoid: public singleton<humanoid>
    {
    public:
        void init(const std::string &robot_file, const std::string &action_file, const std::string &offset_file)
        {
            main_bone_ = parser::robot_parser::parse(robot_file, bone_map_, joint_map_);
            parser::action_parser::parse(action_file, acts_, poses_);
            parser::offset_parser::parse(offset_file, offsets_);
            for(auto j:joint_map_)
                if(offsets_.find(j.first) == offsets_.end())
                    offsets_[j.first] = 0.0;
        }

        joint_map joint_map_;
        bone_map bone_map_;
        bone_ptr main_bone_;
        std::map<std::string, float> offsets_;
        std::map<std::string, robot::robot_act> acts_;
        std::map<std::string, robot::robot_pos> poses_;
    };

    #define ROBOT humanoid::get_singleton()
}
#endif