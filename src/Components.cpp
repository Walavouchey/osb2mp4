#include <Components.hpp>

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

    void Sprite::Initialise(std::vector<std::pair<double, HitSound>> &hitSounds)
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

}
