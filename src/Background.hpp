#pragma once
#include <string>

namespace sb {

    class Background
    {
    public:
        Background() = default;

        Background(const std::string& filepath, std::pair<double, double> offset)
            :
            filepath(filepath),
            offset(offset)
        {
            exists = true;
        }

        Background& operator=(const Background&) = default;
        std::string filepath = "";
        std::pair<double, double> offset = { 0,0 };
        bool exists = false;
    };

}
