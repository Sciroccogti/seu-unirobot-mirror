#ifndef SEU_UNIROBOT_OPTIONS_HPP
#define SEU_UNIROBOT_OPTIONS_HPP

#include <memory>
#include <boost/program_options.hpp>
#include <iostream>

namespace config
{
    using namespace boost::program_options;

    class options
    {
    public:
        options(): opts_desc_("   Options description")
        {
            opts_desc_.add_options()
            ("help,h", "Print this message and exit.")
            ("player,p", value<int>()->default_value(0), "Player ID number.")
            ("debug,d", value<bool>()->default_value(false), "If you want to start in debug mode.")
            ("gc,g", value<bool>()->default_value(false), "If you want to use gamecontroller.")
            ("camera,c", value<bool>()->default_value(true), "If you want to use camera.")
            ("robot,r", value<bool>()->default_value(true), "If you want to use robot.")
            ("say,s", value<bool>()->default_value(false), "If you want to use communication.");
        }

        bool parse_command(int argc, char *argv[])
        {
            try
            {
                store(parse_command_line(argc, argv, opts_desc_), var_map_);
                id_ = arg<int>("player");
                use_debug_ = arg<bool>("debug");
                use_gc_ = arg<bool>("gc");
                use_camera_ = arg<bool>("camera");
                use_robot_ = arg<bool>("robot");
                use_comm_ = arg<bool>("say");
                if (var_map_.count("help"))
                {
                    std::cout<<"\033[31m"<< opts_desc_ <<"\033[0m"<< std::endl;
                    return false;
                }
            }
            catch (boost::program_options::unknown_option &e)
            {
                std::cerr<<"\033[31m"<< e.what() <<"\033[0m"<< std::endl;
            }
            catch (std::exception &e)
            {
                std::cerr<<"\033[31m"<< e.what() <<"\033[0m"<< std::endl;
                return false;
            }
            return true;
        }

        int id() const { return id_; }
        bool use_debug() const { return use_debug_; }
        bool use_gc() const { return use_gc_; }
        bool use_camera() const { return use_camera_; }
        bool use_robot() const { return use_robot_; }
        bool use_comm() const { return use_comm_; }

    private:
        template<typename T>
        T arg(const std::string &opt) const
        {
            return var_map_[opt].as<T>();
        }

        options_description opts_desc_;
        variables_map var_map_;
        int id_;
        bool use_debug_;
        bool use_gc_;
        bool use_camera_;
        bool use_robot_;
        bool use_comm_;
    };
}

#endif
