#pragma once

#include <vector>
#include <string>
#include "basic_parser.hpp"
#include "model.hpp"

namespace parser
{
    class camera_parser: public basic_parser
    {
    public:
        static void parse(const std::string &filename, std::vector<camera_ctrl_info> &ctrl_infos)
        {
            bpt::ptree pt;

            ctrl_infos.clear();

            if (!get_tree_from_file(filename, pt))
            {
                return;
            }

            for (auto &ctrl : pt)
            {
                camera_ctrl_info info;
                info.qctrl.id = static_cast<unsigned int>(std::atoi(ctrl.first.c_str()));
                info.ctrl.id = info.qctrl.id;
                info.ctrl.value = ctrl.second.get<int>("value");
                memcpy(info.qctrl.name, ctrl.second.get<std::string>("name").c_str(), ctrl.second.get<std::string>("name").size());
                info.qctrl.name[ctrl.second.get<std::string>("name").size()] = 0;
                info.qctrl.minimum = ctrl.second.get<int>("minimum");
                info.qctrl.maximum = ctrl.second.get<int>("maximum");
                info.qctrl.default_value = ctrl.second.get<int>("default_value");
                info.menu = ctrl.second.get<std::string>("menu");
                ctrl_infos.push_back(info);
            }
        }

        static void save(const std::string &filename, const std::vector<camera_ctrl_info> &ctrl_infos)
        {
            bpt::ptree pt;

            for (camera_ctrl_info item : ctrl_infos)
            {
                bpt::ptree item_pt;
                item_pt.add("name", item.qctrl.name);
                item_pt.add("value", item.ctrl.value);
                item_pt.add("minimum", item.qctrl.minimum);
                item_pt.add("maximum", item.qctrl.maximum);
                item_pt.add("default_value", item.qctrl.default_value);
                item_pt.add("menu", item.menu);
                pt.add_child(std::to_string(item.qctrl.id), item_pt);
            }

            write_tree_to_file(filename, pt);
        }
    };
}
