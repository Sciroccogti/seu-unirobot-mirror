#ifndef SEU_UNIROBOT_CAMERA_PARSER_HPP
#define SEU_UNIROBOT_CAMERA_PARSER_HPP

#include <vector>
#include "basic_parser.hpp"
#include "model.hpp"

namespace parser
{
    class camera_parser: public basic_parser
    {
    public:
        static void parse(const std::string &filename, std::vector<ctrl_item_info> &ctrl_items)
        {
            bpt::ptree pt;
            ctrl_item_info info;
            ctrl_items.clear();
            if(!get_tree_from_file(filename, pt)) return;
            for(auto &ctrl : pt)
            {
                info.id = static_cast<unsigned int>(std::atoi(ctrl.first.c_str()));
                info.value = ctrl.second.get<int>("value");
                info.name = ctrl.second.get<std::string>("name");
                ctrl_items.push_back(info);
            }
        }

        static void save(const std::string &filename, const std::vector<ctrl_item_info> &ctrl_items)
        {
            bpt::ptree pt;
            for(auto &item : ctrl_items)
            {
                bpt::ptree item_pt;
                item_pt.add("value", item.value);
                item_pt.add("name", item.name);
                pt.add_child(std::to_string(item.id), item_pt);
            }
            write_tree_to_file(filename, pt);
        }
    };
}
#endif