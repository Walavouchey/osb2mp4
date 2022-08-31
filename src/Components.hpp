#pragma once

#include <Keyframes.hpp>

#include <opencv2/opencv.hpp>
#include <string>
#include <limits>
#include <vector>
#include <utility>
#include <memory>
#include <algorithm>
#include <optional>

namespace sb
{
    class Loop
    {
    public:
        Loop(double starttime, int loopcount)
            :
            starttime(starttime),
            loopcount(loopcount)
        {
            endtime = 0;
            looplength = 0;
        }
        template <typename T>
        void AddEvent(std::unique_ptr<Event<T>> event)
        {
            events.push_back(std::move(event));
        }

        void Initialise();

        std::vector<std::unique_ptr<IEvent>>& GetEvents()
        {
            return events;
        }

        double GetStartTime() const
        {
            return starttime;
        }

        double GetEndTime() const
        {
            return endtime;
        }

        double GetLoopLength() const
        {
            return looplength;
        }

        int GetLoopCount() const
        {
            return loopcount;
        }

    private:
        std::vector<std::unique_ptr<IEvent>> events;
        double starttime;
        double endtime;
        double looplength;
        int loopcount;
    };

    class Trigger
    {
    public:
        Trigger(const std::string& triggerName, double starttime, double endtime, int groupNumber)
            :
            triggerName(triggerName),
            starttime(starttime),
            endtime(endtime),
            groupNumber(groupNumber)
        {}

        template <typename T>
        void AddEvent(std::unique_ptr<Event<T>> event)
        {
            events.push_back(std::move(event));
        }

        void Initialise(std::vector<std::pair<double, HitSound>>& hitSounds, std::vector<std::tuple<double, double, int>>& activations, int& id);

        std::vector<std::unique_ptr<IEvent>>& GetEvents()
        {
            return events;
        }

        const std::string& GetTriggerName() const
        {
            return triggerName;
        }

        double GetStartTime() const
        {
            return starttime;
        }

        double GetEndTime() const
        {
            return endtime;
        }

        int GetGroupNumber() const
        {
            return groupNumber;
        }

        bool IsActivated() const
        {
            return activated;
        }

    private:
        std::vector<std::unique_ptr<IEvent>> events;
        std::string triggerName;
        double starttime;
        double endtime;
        double looplength = 0;
        int groupNumber;
        bool activated = false;
    };

    class Sprite
    {
    public:
        Sprite(Layer layer, Origin origin, const std::string& filepath, const std::pair<double, double>& coordinates)
            :
            filepath(filepath),
            layer(layer),
            origin(origin),
            coordinates(coordinates)
        {}

        virtual ~Sprite() = default;

        template <typename T>
        void AddEvent(std::unique_ptr<Event<T>> event)
        {
            events.push_back(std::move(event));
        }

        template <typename T>
        void AddEventInLoop(std::unique_ptr<Event<T>> event)
        {
            (loops.end() - 1)->AddEvent(std::move(event));
        }

        template <typename T>
        void AddEventInTrigger(std::unique_ptr<Event<T>> event)
        {
            (triggers.end() - 1)->AddEvent(std::move(event));
        }

        void AddLoop(Loop loop)
        {
            loops.push_back(std::move(loop));
        }

        void AddTrigger(Trigger trigger)
        {
            triggers.push_back(std::move(trigger));
        }

        void Initialise(std::vector<std::pair<double, HitSound>>& hitSounds);

        std::pair<double, double> PositionAt(double time) const
        {
            return keyframeValueAt<double>(positionKeyframes, time);
        }

        double RotationAt(double time) const
        {
            return keyframeValueAt<double>(rotationKeyframes, time);
        }

        std::pair<double, double> ScaleAt(double time) const
        {
            return keyframeValueAt<double>(scaleKeyframes, time);
        }

        Colour ColourAt(double time) const
        {
            return keyframeValueAt<Colour>(colourKeyframes, time);
        }

        double OpacityAt(double time) const
        {
            return keyframeValueAt<double>(opacityKeyframes, time);
        }

        bool EffectAt(double time, ParameterType effect) const
        {
            return keyframeValueAt<bool>(effect == ParameterType::FlipV ? flipVKeyframes : effect == ParameterType::FlipH ? flipHKeyframes : additiveKeyframes, time);
        }

        Layer GetLayer() const
        {
            return layer;
        }

        Origin GetOrigin() const
        {
            return origin;
        }

        virtual const std::string GetFilePath([[maybe_unused]] double time) const
        {
            return filepath;
        }

        virtual std::vector<std::string> GetFilePaths() const
        {
            return std::vector<std::string>({ filepath });
        }

        const std::pair<double, double>& GetCoordinates() const
        {
            return coordinates;
        }

        const std::pair<double, double>& GetActiveTime() const
        {
            return activetime;
        }

        const std::pair<double, double>& GetVisibleTime() const
        {
            return visibletime;
        }

    protected:
        std::vector<Loop> loops;
        std::vector<Trigger> triggers;
        std::pair<double, double> activetime;
        std::pair<double, double> visibletime;
        const std::string filepath;
    private:
        std::vector<std::unique_ptr<IEvent>> events;
        bool initialised = false;
        const Layer layer;
        const Origin origin;
        const std::pair<double, double> coordinates;
        std::pair<std::vector<Keyframe<double>>, std::vector<Keyframe<double>>> positionKeyframes;
        std::vector<Keyframe<double>> rotationKeyframes;
        std::pair<std::vector<Keyframe<double>>, std::vector<Keyframe<double>>> scaleKeyframes;
        std::vector<Keyframe<Colour>> colourKeyframes;
        std::vector<Keyframe<double>> opacityKeyframes;
        std::vector<Keyframe<bool>> flipVKeyframes;
        std::vector<Keyframe<bool>> flipHKeyframes;
        std::vector<Keyframe<bool>> additiveKeyframes;
    };

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

        int frameIndexAt(double time) const
        {
            if (time - activetime.first < framecount * framedelay || looptype == LoopType::LoopForever)
            {
                return (int)std::fmod(((time - activetime.first) / framedelay), (double)framecount);
            }
            else return framecount - 1;
        }

        const std::string GetFilePath(double time) const
        {
            std::size_t pos = filepath.rfind(".");
            std::string base = filepath.substr(0, pos);
            std::string ext = filepath.substr(pos);
            return base + std::to_string(frameIndexAt(time)) + ext;
        }

        std::vector<std::string> GetFilePaths() const
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

    private:
        const int framecount;
        const double framedelay;
        const LoopType looptype;
    };

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
