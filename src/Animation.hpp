#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <limits>
#include <vector>
#include <utility>
#include <memory>
#include <algorithm>
#include <optional>

#include <Sprite.hpp>

namespace sb {

    class Animation : public Sprite
    {
    public:
        Animation(Layer layer, Origin origin, const std::string& filepath, const std::pair<double, double>& coordinates, int framecount, double framedelay, LoopType looptype)
            :
            Sprite(layer, origin, filepath, coordinates),
            framecount(framecount),
            framedelay(framedelay),
            looptype(looptype)
        {}

        virtual ~Animation() = default;

        int frameIndexAt(double time) const;

        const std::string GetFilePath(double time) const;

        std::vector<std::string> GetFilePaths() const;

    private:
        const int framecount;
        const double framedelay;
        const LoopType looptype;
    };

}
