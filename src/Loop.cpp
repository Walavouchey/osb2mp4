#include <Loop.hpp>

namespace sb {

    void Loop::Initialise()
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

}
