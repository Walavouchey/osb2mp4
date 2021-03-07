#pragma once

#include <Interpolation.hpp>
#include <Types.hpp>

#include <limits>
#include <vector>
#include <memory>

namespace sb
{
    template<typename T>
    class Keyframe
    {
    public:
        Keyframe()
        {}
        Keyframe(double time, T value, Easing easing, double interpolationOffset = std::numeric_limits<double>::infinity())
            :
            time(time),
            value(value),
            easing(easing),
            interpolationOffset(interpolationOffset == std::numeric_limits<double>::infinity() ? time : interpolationOffset)
        {}
        double time;
        T value;
        Easing easing;
        double interpolationOffset;
    };

    template <typename T>
    T keyframeValueAt(const std::vector<Keyframe<T>>& keyframes, double time)
    {
        Keyframe<T> keyframe = Keyframe<T>();
        Keyframe<T> endKeyframe = Keyframe<T>();
        bool found = false;
        for (int i = 0; i < keyframes.size(); i++)
        {
            if (keyframes[i].time > time)
            {
                keyframe = keyframes[i - 1];
                endKeyframe = keyframes[i];
                found = true;
                break;
            }
            else continue;
        }
        if (!found) keyframe = *(keyframes.end() - 1);
        if (keyframe.easing == Easing::Step)
            return keyframe.value;
        double t = (time - keyframe.interpolationOffset) / (std::max(endKeyframe.time, endKeyframe.interpolationOffset) - keyframe.interpolationOffset);
        t = applyEasing(keyframe.easing, t);
        return InterpolateLinear(keyframe.value, endKeyframe.value, t);
    }

    template <class T>
    std::pair<T, T> keyframeValueAt(const std::pair<std::vector<Keyframe<T>>, std::vector<Keyframe<T>>>& keyframes, double time)
    {
        T first = keyframeValueAt<T>(keyframes.first, time);
        T second = keyframeValueAt<T>(keyframes.second, time);
        return std::pair<T, T>(first, second);
    }
    template <typename T, typename V, typename Selector>
    void addKeyframe(Selector W, std::vector<Keyframe<T>>& keyframes, double time, V value, bool alt, Easing easing, double interpolationOffset = std::numeric_limits<double>::infinity())
    {
        keyframes.push_back(Keyframe<T>(time, W(value), easing, interpolationOffset));
    }

    template <typename T = bool, typename V = ParameterType, typename Selector>
    void addKeyframe(Selector W, std::vector<Keyframe<T>>& keyframes, double time, ParameterType value, bool alt, Easing easing, double interpolationOffset = std::numeric_limits<double>::infinity())
    {
        keyframes.push_back(Keyframe<T>(time, alt, Easing::Step, interpolationOffset));
    }

    template <typename T, typename V, typename Selector>
    void generateKeyframes(std::vector<Keyframe<T>>& keyframes, const std::vector<std::unique_ptr<Event<V>>>& events, const std::vector<std::tuple<double, double, int>>& activations, Selector W)
    {
        for (typename std::vector<std::unique_ptr<Event<V>>>::const_iterator it = events.begin(); it < events.end(); it++)
        {
            const std::unique_ptr<Event<V>>& event = *it;
            bool appendEndtime = event->GetEndTime() > event->GetStartTime();
            if (it == events.begin())
            {
                // the starting event overrides the sprite's initial position
                addKeyframe(W, keyframes, -std::numeric_limits<double>::infinity(), event->GetStartValue(), true, Easing::Step);
                addKeyframe(W, keyframes, event->GetStartTime(), appendEndtime ? event->GetStartValue() : event->GetEndValue(), true, appendEndtime ? event->GetEasing() : Easing::Step);
                if (appendEndtime)
                    addKeyframe(W, keyframes, event->GetEndTime(), event->GetEndValue(), false, Easing::Step);
                continue;
            }
            std::tuple<double, double, int> currActivation;
            std::tuple<double, double, int> nextActivation;
            bool triggersOverlap = false;
            bool eventsOverlap = keyframes[keyframes.size() - 1].time > event->GetStartTime();
            if (activations.size() > 0)
                for (std::vector<std::tuple<double, double, int>>::const_iterator activationIt = activations.begin(); activationIt + 1 < activations.end(); activationIt++)
                    if (std::get<0>(*activationIt) == event->GetTriggerST())
                    {
                        currActivation = *activationIt;
                        nextActivation = *(activationIt + 1);
                        break;
                    }
            if (event->GetTriggerID() != 0 && std::get<1>(currActivation) > std::get<0>(nextActivation))
                triggersOverlap = true;
            double starttime = triggersOverlap && event->GetEndTime() > std::get<0>(nextActivation) ?
                std::get<0>(nextActivation)
                : (eventsOverlap ?
                    keyframes[keyframes.size() - 1].time
                    : event->GetStartTime());
            double endtime = triggersOverlap && event->GetEndTime() > std::get<0>(nextActivation) ?
                std::get<0>(nextActivation)
                : event->GetEndTime();
            // the first event overrides subsequent overlapping events, but their interpolation still starts from their respective times
            // if two trigger activations overlap, the latter overrides the former. events within triggers still override like before
            // this also means non-trigger events before a trigger override the trigger
            // TODO: implement trigger group numbers correctly
            // TODO: check other miscellaneous edge cases
            addKeyframe(W, keyframes,
                starttime,
                appendEndtime ? event->GetStartValue() : event->GetEndValue(), true,
                appendEndtime ? event->GetEasing() : Easing::Step,
                event->GetStartTime()
            );
            if (eventsOverlap) keyframes[keyframes.size() - 2].time = event->GetStartTime();
            if (appendEndtime)
                addKeyframe(W, keyframes, endtime, event->GetEndValue(), false, Easing::Step, event->GetEndTime());
        }
    }

    // a little bit of dumb bullshit down below
    template <typename T>
    struct nop
    {
        T operator()(T in) { return in; }
    };
    template <class ... t>
    constexpr bool alwaysFalse = false;
    template <EventType T, typename R, ParameterType P = ParameterType::Additive>
    R generateKeyframesForEvent(const std::vector<std::unique_ptr<IEvent>>& events, std::pair<double, double> coordinates, const std::vector<std::tuple<double, double, int>>& activations)
    {
        static_assert(T != EventType::P && P == ParameterType::Additive || T == EventType::P, "Invalid template arguments");
        auto XKeyframes = std::vector<Keyframe<double>>();
        auto YKeyframes = std::vector<Keyframe<double>>();
        R keyframes = R();
        auto applicableEvents = std::vector<std::unique_ptr<IEvent>>();
        struct isApplicable
        {
            bool operator()(const std::unique_ptr<IEvent>& event)
            {
                EventType type = event->GetType();
                if constexpr (T == EventType::M) return type == EventType::M || type == EventType::MX || type == EventType::MY;
                else if constexpr (T == EventType::R) return type == EventType::R;
                else if constexpr (T == EventType::S) return type == EventType::S || type == EventType::V;
                else if constexpr (T == EventType::C) return type == EventType::C;
                else if constexpr (T == EventType::F) return type == EventType::F;
                else if constexpr (T == EventType::P && P == ParameterType::Additive) return type == EventType::P && dynamic_cast<Event<ParameterType>*>(event.get())->GetStartValue() == ParameterType::Additive;
                else if constexpr (T == EventType::P && P == ParameterType::FlipH) return type == EventType::P && dynamic_cast<Event<ParameterType>*>(event.get())->GetStartValue() == ParameterType::FlipH;
                else if constexpr (T == EventType::P && P == ParameterType::FlipV) return type == EventType::P && dynamic_cast<Event<ParameterType>*>(event.get())->GetStartValue() == ParameterType::FlipV;
                else static_assert(alwaysFalse<T>, "Template argument T invalid");
            }
        };
        isApplicable isApplicable;
        for (const std::unique_ptr<IEvent>& event : events)
            if (isApplicable(event))
                applicableEvents.push_back(event->copy());
        if (applicableEvents.size() == 0)
        {
            if constexpr (T == EventType::M)
            {
                XKeyframes.push_back(Keyframe<double>(-std::numeric_limits<double>::infinity(), coordinates.first, Easing::Step, -std::numeric_limits<double>::infinity()));
                YKeyframes.push_back(Keyframe<double>(-std::numeric_limits<double>::infinity(), coordinates.second, Easing::Step, -std::numeric_limits<double>::infinity()));
                return std::pair<std::vector<Keyframe<double>>, std::vector<Keyframe<double>>>(XKeyframes, YKeyframes);
            }
            else if constexpr (T == EventType::R)
            {
                keyframes.push_back(Keyframe<double>(-std::numeric_limits<double>::infinity(), 0, Easing::Step, -std::numeric_limits<double>::infinity()));
                return keyframes;
            }
            else if constexpr (T == EventType::S)
            {
                XKeyframes.push_back(Keyframe<double>(-std::numeric_limits<double>::infinity(), 1, Easing::Step, -std::numeric_limits<double>::infinity()));
                YKeyframes.push_back(Keyframe<double>(-std::numeric_limits<double>::infinity(), 1, Easing::Step, -std::numeric_limits<double>::infinity()));
                return std::pair<std::vector<Keyframe<double>>, std::vector<Keyframe<double>>>(XKeyframes, YKeyframes);
            }
            else if constexpr (T == EventType::C)
            {
                keyframes.push_back(Keyframe<Colour>(-std::numeric_limits<double>::infinity(), Colour(1, 1, 1), Easing::Step, -std::numeric_limits<double>::infinity()));
                return keyframes;
            }
            else if constexpr (T == EventType::F)
            {
                keyframes.push_back(Keyframe<double>(-std::numeric_limits<double>::infinity(), 1, Easing::Step, -std::numeric_limits<double>::infinity()));
                return keyframes;
            }
            else if constexpr (T == EventType::P)
            {
                keyframes.push_back(Keyframe<bool>(-std::numeric_limits<double>::infinity(), false, Easing::Step, -std::numeric_limits<double>::infinity()));
                return keyframes;
            }
        }
        if constexpr (T == EventType::M || T == EventType::S)
        {
            bool compatibilityMode = applicableEvents[0]->GetType() == EventType::M || applicableEvents[0]->GetType() == EventType::V;
            auto applicableEventsXY = std::vector<std::unique_ptr<Event<std::pair<double, double>>>>();
            auto applicableEventsX = std::vector<std::unique_ptr<Event<double>>>();
            auto applicableEventsY = std::vector<std::unique_ptr<Event<double>>>();
            if (compatibilityMode)
            {
                for (const std::unique_ptr<IEvent>& event : applicableEvents)
                {
                    EventType type = event->GetType();
                    if (type == EventType::M)
                        applicableEventsXY.push_back(std::make_unique<Event<std::pair<double, double>>>(*dynamic_cast<Event<std::pair<double, double>>*>(event->copy().get())));
                    if (type == EventType::V)
                        applicableEventsXY.push_back(std::make_unique<Event<std::pair<double, double>>>(*dynamic_cast<Event<std::pair<double, double>>*>(event->copy().get())));
                }
            }
            else
            {
                for (const std::unique_ptr<IEvent>& event : applicableEvents)
                {
                    EventType type = event->GetType();
                    if (type == EventType::MX)
                        applicableEventsX.push_back(std::make_unique<Event<double>>(*dynamic_cast<Event<double>*>(event->copy().get())));
                    if (type == EventType::MY)
                        applicableEventsY.push_back(std::make_unique<Event<double>>(*dynamic_cast<Event<double>*>(event->copy().get())));
                    if (type == EventType::S)
                    {
                        applicableEventsX.push_back(std::make_unique<Event<double>>(*dynamic_cast<Event<double>*>(event->copy().get())));
                        applicableEventsY.push_back(std::make_unique<Event<double>>(*dynamic_cast<Event<double>*>(event->copy().get())));
                    }
                }
            }
            struct first
            {
                double operator()(std::pair<double, double> in) { return in.first; }
            };
            struct second
            {
                double operator()(std::pair<double, double> in) { return in.second; }
            };
            if (compatibilityMode)
            {
                generateKeyframes<double, std::pair<double, double>>(XKeyframes, applicableEventsXY, activations, first());
                generateKeyframes<double, std::pair<double, double>>(YKeyframes, applicableEventsXY, activations, second());
            }
            else
            {
                if (applicableEventsX.size() == 0)
                    if constexpr (T == EventType::M)
                        XKeyframes.push_back(Keyframe<double>(-std::numeric_limits<double>::infinity(), coordinates.first, Easing::Step, -std::numeric_limits<double>::infinity()));
                    else
                        XKeyframes.push_back(Keyframe<double>(-std::numeric_limits<double>::infinity(), 1, Easing::Step, -std::numeric_limits<double>::infinity()));
                else
                    generateKeyframes<double, double>(XKeyframes, applicableEventsX, activations, nop<double>());
                if (applicableEventsY.size() == 0)
                    if constexpr (T == EventType::M)
                        YKeyframes.push_back(Keyframe<double>(-std::numeric_limits<double>::infinity(), coordinates.second, Easing::Step, -std::numeric_limits<double>::infinity()));
                    else
                        YKeyframes.push_back(Keyframe<double>(-std::numeric_limits<double>::infinity(), 1, Easing::Step, -std::numeric_limits<double>::infinity()));
                else
                    generateKeyframes<double, double>(YKeyframes, applicableEventsY, activations, nop<double>());
            }
            return std::pair<std::vector<Keyframe<double>>, std::vector<Keyframe<double>>>(XKeyframes, YKeyframes);
        }
        else if constexpr (T == EventType::C)
        {
            auto ev = std::vector<std::unique_ptr<Event<Colour>>>();
            for (const auto& e : applicableEvents)
                ev.push_back(std::make_unique<Event<Colour>>(*dynamic_cast<Event<Colour>*>(e->copy().get())));
            generateKeyframes<Colour, Colour>(keyframes, ev, activations, nop<Colour>());
            return keyframes;
        }
        else if constexpr (T == EventType::P)
        {
            auto ev = std::vector<std::unique_ptr<Event<ParameterType>>>();
            for (const auto& e : applicableEvents)
                ev.push_back(std::make_unique<Event<ParameterType>>(*dynamic_cast<Event<ParameterType>*>(e->copy().get())));
            generateKeyframes<bool, ParameterType>(keyframes, ev, activations, nop<ParameterType>());
            return keyframes;
        }
        else
        {
            auto ev = std::vector<std::unique_ptr<Event<double>>>();
            for (const auto& e : applicableEvents)
                ev.push_back(std::make_unique<Event<double>>(*dynamic_cast<Event<double>*>(e->copy().get())));
            generateKeyframes<double, double>(keyframes, ev, activations, nop<double>());
            return keyframes;
        }
    }
}
