#ifndef SEU_UNIROBOT_OFFSET_PARSER_HPP
#define SEU_UNIROBOT_OFFSET_PARSER_HPP

#include <map>
#include "basic_parser.hpp"

namespace parser
{
    class offset_parser: public basic_parser
    {
    public:
        static void parse(const std::string &filename, std::map<std::string, float> &offsets)
        {
            bpt::ptree pt;
            offsets.clear();
            if(!get_tree_from_file(filename, pt)) return;
            for(auto offset : pt)
                offsets[offset.first] = offset.second.get_value<float>();
        }

        static void save(const std::string &filename, const std::map<std::string, float> &offsets)
        {
            bpt::ptree pt;
            for(auto offset : offsets)
                pt.add(offset.first, offset.second);
            write_tree_to_file(filename, pt);
        }
    };
}
#endif