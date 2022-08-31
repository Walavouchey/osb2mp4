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

namespace sb {

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

}
