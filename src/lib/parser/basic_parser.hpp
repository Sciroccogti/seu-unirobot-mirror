#ifndef SEU_UNIROBOT_BASIC_PARSER_HPP
#define SEU_UNIROBOT_BASIC_PARSER_HPP

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

namespace parser
{
    namespace bpt = boost::property_tree;

    class basic_parser
    {
    protected:
        static bool get_tree_from_file(const std::string &filename, bpt::ptree &pt)
        {
            std::ifstream ifs(filename.c_str(), std::ios::in);
            std::string line;
            int count_of_quotatuion = 0;
            std::stringstream json_data;

            if (!ifs) return false;

            while(std::getline(ifs, line))
            {
                count_of_quotatuion = 0;
                for(auto c: line)
                {
                    if(c == '\'' || c == '\"') count_of_quotatuion++;
                    if(c == '#'&& count_of_quotatuion%2 == 0) break;
                    json_data<<c;
                }
            }
            bpt::read_json(json_data, pt);
            return true;
        }

        static void write_tree_to_file(const std::string &filename, const bpt::ptree &pt)
        {
            std::ostringstream os;
            bpt::write_json(os, pt);
            std::ofstream tree(filename.c_str());
            tree << os.str();
            tree.close();
        }
    };
}
#endif