#pragma once

#include <map>
#include "basic_parser.hpp"
#include "image/image_define.hpp"

namespace parser
{
    class color_parser: public basic_parser
    {
    public:
        static void parse(const std::string &filename, std::vector<imageproc::Color> &colors)
        {
            bpt::ptree pt;

            colors.clear();

            if (!get_tree_from_file(filename, pt))
            {
                return;
            }

            for (auto &info : pt)
            {
                imageproc::Color color;
                std::string name = info.first;
                color.H.minimum = info.second.get<float>("HL");
                color.H.maximum = info.second.get<float>("HH");
                color.S.minimum = info.second.get<float>("SL");
                color.S.maximum = info.second.get<float>("SH");
                color.I.minimum = info.second.get<float>("IL");
                color.I.maximum = info.second.get<float>("IH");
                color.c = imageproc::get_color_by_name(name);
                colors.push_back(color);
            }
        }

        static void save(const std::string &filename, const std::vector<imageproc::Color> &colors)
        {
            bpt::ptree pt;
            for(auto item : colors)
            {
                bpt::ptree item_pt;
                item_pt.add("HL", item.H.minimum);
                item_pt.add("HH", item.H.maximum);
                item_pt.add("SL", item.S.minimum);
                item_pt.add("SH", item.S.maximum);
                item_pt.add("IL", item.I.minimum);
                item_pt.add("IH", item.I.maximum);
                pt.add_child(imageproc::get_name_by_color(item.c), item_pt);
            }

            write_tree_to_file(filename, pt);
        }
    };
}