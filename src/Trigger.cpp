#include <Trigger.hpp>

namespace sb {

    void Trigger::Initialise(std::vector<std::pair<double, HitSound>>& hitSounds, std::vector<std::tuple<double, double, int>>& activations, int& id)
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


}
