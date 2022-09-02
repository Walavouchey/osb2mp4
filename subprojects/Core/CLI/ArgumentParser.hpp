#pragma once
#include <functional>
#include <iostream>
#include <string_view>

namespace CLI {

class ArgumentParser {
public:
    void add_flag(std::string_view long_name,
                  std::string_view short_name,
                  std::string_view explanation,
                  std::function<void()>&& callback)
    {
        flags.push_back({ long_name, short_name, explanation });
        long_flag_callbacks[long_name] = callback;
        short_flag_callbacks[short_name] = callback;
    }

    void add_option(std::string_view long_name,
                    std::string_view short_name,
                    std::string_view placeholder,
                    std::string_view explanation,
                    std::function<void(std::string_view)>&& callback)
    {
        options.push_back({ long_name, short_name, explanation, placeholder });
        long_option_callbacks[long_name] = callback;
        short_option_callbacks[short_name] = callback;
    }

    void add_positional_argument(std::string_view placeholder, std::function<void(std::string_view)>&& callback)
    {
        positional_argument_placeholders.push_back(placeholder);
        positional_argument_callbacks.push_back(callback);
    }

    void run(int argc, char const* argv[]) const;

    void print_usage_and_exit(std::string_view program_name, int exit_code = 0) const;

private:
    struct Flag {
        std::string_view long_name;
        std::string_view short_name;
        std::string_view explanation;
    };
    std::vector<Flag> flags {};

    struct Option {
        std::string_view long_name;
        std::string_view short_name;
        std::string_view explanation;
        std::string_view placeholder;
    };
    std::vector<Option> options {};

    std::unordered_map<std::string_view, std::function<void()>> short_flag_callbacks {};
    std::unordered_map<std::string_view, std::function<void()>> long_flag_callbacks {};
    std::unordered_map<std::string_view, std::function<void(std::string_view)>> short_option_callbacks {};
    std::unordered_map<std::string_view, std::function<void(std::string_view)>> long_option_callbacks {};

    std::vector<std::string_view> positional_argument_placeholders {};
    std::vector<std::function<void(std::string_view)>> positional_argument_callbacks {};
};

}
