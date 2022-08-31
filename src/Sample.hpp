#pragma once
#include <Keyframes.hpp>

namespace sb {

    class Sample
    {
    public:
        Sample(double starttime, Layer layer, const std::string& filepath, float volume)
            :
            starttime(starttime),
            layer(layer),
            filepath(filepath),
            volume(volume)
        {}

        const double starttime;
        const Layer layer;
        const std::string filepath;
        const float volume;
    };

}
