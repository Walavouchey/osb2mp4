#pragma once

#include <Keyframes.hpp>

namespace sb {

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

}
