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
            parser::action_parser::parse(action_file, act_map_, pos_map_);
            parser::offset_parser::parse(offset_file, joint_map_);
        }

        joint_ptr get_joint(const int &id)
        {
            for(auto j:joint_map_)
                if(j.second->jid_ == id) return j.second;
            return nullptr;
        }

        joint_map &get_joint_map() { return joint_map_; }
        joint_map get_joint_map() const { return joint_map_; }
        bone_map &get_bone_map() { return bone_map_; }
        bone_map get_bone_map() const { return bone_map_; }
        act_map &get_act_map() { return act_map_; }
        act_map get_act_map() const { return act_map_; }
        pos_map &get_pos_map() { return pos_map_; }
        pos_map get_pos_map() const { return pos_map_; }
        bone_ptr get_main_bone() { return main_bone_; }

    private:
        bone_ptr main_bone_;
        joint_map joint_map_;
        bone_map bone_map_;
        act_map act_map_;
        pos_map pos_map_;
    };

    #define ROBOT humanoid::get_singleton()
}
#endif