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
        void Initialise()
        {
            if (loopcount < 1) loopcount = 1; // today i learned that loops behave like this
            looplength = (*(events.end() - 1))->GetEndTime();
            std::vector<std::unique_ptr<IEvent>> expandedEvents;
            std::vector<std::pair<double, double>> durations;
            for (int i = 0; i < loopcount; i++)
            {
                int j = 0;
                for (std::unique_ptr<IEvent>& event : events)
                {
                    if (i == 0) durations.emplace_back(std::pair<double, double>{event->GetStartTime(), event->GetEndTime()});
                    event->SetStartTime(starttime + durations[j].first + looplength * i);
                    event->SetEndTime(starttime + durations[j].second + looplength * i);
                    expandedEvents.push_back(event->copy());
                    j++;
                }
            }
            events = std::move(expandedEvents);
            endtime = starttime + looplength * loopcount;
        }
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
        void Initialise(std::vector<std::pair<double, HitSound>>& hitSounds, std::vector<std::tuple<double, double, int>>& activations, int& id)
        {
            if (!HitSound::IsHitSound(triggerName)) return; // TODO: ignoring failing and passing state triggers for now
            looplength = (*(events.end() - 1))->GetEndTime();
            std::vector<double> activationTimes;
            for (const std::pair<double, HitSound>& hitSound : hitSounds)
                if (hitSound.first >= starttime && hitSound.first < endtime
                    && hitSound.second == HitSound(triggerName))
                {
                    activations.push_back(std::tuple<double, double, int>(hitSound.first, hitSound.first + looplength, groupNumber));
                    activationTimes.push_back(hitSound.first);
                    activated = true;
                }
            std::vector<std::unique_ptr<IEvent>> expandedEvents;
            if (activated)
            {
                for (double activationTime : activationTimes)
                {
                    for (std::unique_ptr<IEvent>& event : events)
                    {
                        std::unique_ptr<IEvent> copy = event->copy();
                        copy->SetTriggerID(id, activationTime, groupNumber);
                        copy->SetStartTime(activationTime + copy->GetStartTime());
                        copy->SetEndTime(activationTime + copy->GetEndTime());
                        expandedEvents.push_back(std::move(copy));
                    }
                    id++;
                }
                events = std::move(expandedEvents);
            }
        }
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
        void Initialise(std::vector<std::pair<double, HitSound>>& hitSounds)
        {
            initialised = true;
            for (Loop& loop : loops) loop.Initialise();
            std::vector<std::tuple<double, double, int>> activations;
            int id = 1;
            for (Trigger& trigger : triggers) trigger.Initialise(hitSounds, activations, id);
            for (Loop& loop : loops)
                for (std::unique_ptr<IEvent>& event : loop.GetEvents())
                    events.emplace_back(std::move(event));
            for (Trigger& trigger : triggers)
                if (trigger.IsActivated())
                    for (std::unique_ptr<IEvent>& event : trigger.GetEvents())
                        events.emplace_back(std::move(event));
            std::stable_sort(events.begin(), events.end(), [](const std::unique_ptr<IEvent>& a, const std::unique_ptr<IEvent>& b) {
                int aT = a->GetStartTime();
                int bT = b->GetStartTime();
                int aID = a->GetTriggerID();
                int bID = b->GetTriggerID();
                return aID != 0 && bID != 0 && aID != bID ? aID < bID : aT < bT;
                });

            double endTime = std::numeric_limits<double>::min();
            double startTime = std::numeric_limits<double>::max();
            for (const std::unique_ptr<IEvent>& event : events)
            {
                endTime = std::max(endTime, event->GetEndTime());
                startTime = std::min(startTime, event->GetStartTime());
            }
            activetime = std::pair<double, double>({ startTime, endTime });

            std::optional<double> visibleEndTime;
            std::optional<double> visibleStartTime;
            for (auto it = events.begin(); it != events.end(); it++)
                if ((*it)->GetType() == EventType::F)
                {
                    if (dynamic_cast<Event<double>*>((*it).get())->GetStartValue() == 0)
                        visibleStartTime = (*it)->GetStartTime();
                    break;
                }
            for (auto it = events.rbegin(); it != events.rend(); it++)
                if ((*it)->GetType() == EventType::F)
                {
                    if (dynamic_cast<Event<double>*>((*it).get())->GetEndValue() == 0)
                        visibleEndTime = (*it)->GetEndTime();
                    break;
                }
            visibletime = std::pair<double, double>({
                visibleStartTime.value_or(startTime),
                visibleEndTime.value_or(endTime)
                });

            positionKeyframes = generateKeyframesForEvent<EventType::M, std::pair<std::vector<Keyframe<double>>, std::vector<Keyframe<double>>>>(events, coordinates, activations);
            rotationKeyframes = generateKeyframesForEvent<EventType::R, std::vector<Keyframe<double>>>(events, coordinates, activations);
            scaleKeyframes = generateKeyframesForEvent<EventType::S, std::pair<std::vector<Keyframe<double>>, std::vector<Keyframe<double>>>>(events, coordinates, activations);
            colourKeyframes = generateKeyframesForEvent<EventType::C, std::vector<Keyframe<Colour>>>(events, coordinates, activations);
            opacityKeyframes = generateKeyframesForEvent<EventType::F, std::vector<Keyframe<double>>>(events, coordinates, activations);
            flipHKeyframes = generateKeyframesForEvent<EventType::P, std::vector<Keyframe<bool>>, ParameterType::FlipH>(events, coordinates, activations);
            flipVKeyframes = generateKeyframesForEvent<EventType::P, std::vector<Keyframe<bool>>, ParameterType::FlipV>(events, coordinates, activations);
            additiveKeyframes = generateKeyframesForEvent<EventType::P, std::vector<Keyframe<bool>>>(events, coordinates, activations);
        }
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
