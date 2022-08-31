#include <Animation.hpp>

namespace sb {

    int Animation::frameIndexAt(double time) const
    {
        if (time - activetime.first < framecount * framedelay || looptype == LoopType::LoopForever)
        {
            return (int)std::fmod(((time - activetime.first) / framedelay), (double)framecount);
        }
        else return framecount - 1;
    }

    const std::string Animation::GetFilePath(double time) const
    {
        std::size_t pos = filepath.rfind(".");
        std::string base = filepath.substr(0, pos);
        std::string ext = filepath.substr(pos);
        return base + std::to_string(frameIndexAt(time)) + ext;
    }

    std::vector<std::string> Animation::GetFilePaths() const
    {
        std::size_t pos = filepath.rfind(".");
        std::string base = filepath.substr(0, pos);
        std::string ext = filepath.substr(pos);
        std::vector<std::string> paths = std::vector<std::string>();
        for (int i = 0; i < framecount; i++)
        {
            paths.emplace_back(base + std::to_string(i) + ext);
        }
        return paths;
    }

}
