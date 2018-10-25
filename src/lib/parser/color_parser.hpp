#pragma once

#include <map>
#include "basic_parser.hpp"
#include "image/image_define.hpp"

namespace parser
{
    class color_parser: public basic_parser
    {
    public:
        static void parse(const std::string &filename, std::map<imageproc::Color, imageproc::ColorHSI> &colors)
        {
            bpt::ptree pt;

            colors.clear();

            if (!get_tree_from_file(filename, pt))
            {
                return;
            }

            for (auto &info : pt)
            {
                imageproc::ColorHSI color;
                std::string name = info.first;
                color.H.minimum = info.second.get<float>("HL");
                color.H.maximum = info.second.get<float>("HH");
                color.S.minimum = info.second.get<float>("SL");
                color.S.maximum = info.second.get<float>("SH");
                color.I.minimum = info.second.get<float>("IL");
                color.I.maximum = info.second.get<float>("IH");
                colors[imageproc::get_color_by_name(name)] = color;
            }
        }

        static void save(const std::string &filename, const std::map<imageproc::Color, imageproc::ColorHSI> &colors)
        {
            bpt::ptree pt;
            for(auto item : colors)
            {
                bpt::ptree item_pt;
                item_pt.add("HL", item.second.H.minimum);
                item_pt.add("HH", item.second.H.maximum);
                item_pt.add("SL", item.second.S.minimum);
                item_pt.add("SH", item.second.S.maximum);
                item_pt.add("IL", item.second.I.minimum);
                item_pt.add("IH", item.second.I.maximum);
                pt.add_child(imageproc::get_name_by_color(item.first), item_pt);
            }

            write_tree_to_file(filename, pt);
        }
    };
}