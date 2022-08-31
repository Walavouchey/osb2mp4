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

#include <Loop.hpp>
#include <Trigger.hpp>

namespace sb {

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

}
