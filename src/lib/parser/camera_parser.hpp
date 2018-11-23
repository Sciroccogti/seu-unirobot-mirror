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
        static void parse(const std::string &filename, std::map<std::string, camera_para> &ctrl_infos)
        {
            bpt::ptree pt;

            ctrl_infos.clear();

            if (!get_tree_from_file(filename, pt))
            {
                return;
            }

            for (auto &para : pt)
            {
                camera_para info;
                info.name = para.first;
                info.id = para.second.get<int>("id");
                info.value = para.second.get<float>("value");
                info.default_value = para.second.get<float>("default");
                info.min_value = para.second.get<float>("min");
                info.max_value = para.second.get<float>("max");
                ctrl_infos[info.name] = info;
            }
        }

        static void save(const std::string &filename, const std::map<std::string, camera_para> &ctrl_infos)
        {
            bpt::ptree pt;

            for (auto item : ctrl_infos)
            {
                bpt::ptree item_pt;
                item_pt.add("id", item.second.id);
                item_pt.add("value", item.second.value);
                item_pt.add("default", item.second.default_value);
                item_pt.add("min", item.second.min_value);
                item_pt.add("max", item.second.max_value);
                pt.add_child(item.first, item_pt);
            }

            write_tree_to_file(filename, pt);
        }
    };
}
