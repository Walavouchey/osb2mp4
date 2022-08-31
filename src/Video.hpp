#pragma once
#include <string>

namespace sb {

    class Video
    {
    public:
        Video() = default;

        Video(double starttime, const std::string& filepath, std::pair<double, double> offset)
            :
            starttime(starttime),
            filepath(filepath),
            offset(offset)
        {
            exists = true;
        }

        Video& operator=(const Video&) = default;
        double starttime = 0;
        std::string filepath = "";
        std::pair<double, double> offset = { 0, 0 };
        bool exists = false;
    };

}
