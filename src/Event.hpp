#pragma once
#include <Interpolation.hpp>
#include <memory>
#include <Forward.hpp>

namespace sb {

    class IEvent
    {
    public:
        virtual std::unique_ptr<IEvent> copy() const = 0;
        virtual EventType GetType() const = 0;
        virtual Easing GetEasing() const = 0;
        virtual double GetStartTime() const = 0;
        virtual double GetEndTime() const = 0;
        virtual void SetStartTime(double) = 0;
        virtual void SetEndTime(double) = 0;
        virtual int GetTriggerID() const = 0;
        virtual double GetTriggerST() const = 0;
        virtual int GetTriggerGP() const = 0;
        virtual void SetTriggerID(int, double, int) = 0;
        virtual ~IEvent() {}
    };
    template <typename T>
    class Event : public IEvent
    {
    public:
        Event() = default;
        Event(EventType type, Easing easing, double starttime, double endtime, T startvalue, T endvalue, int triggerID = 0, double triggerST = 0, double triggerGP = 0)
            :
            type(type),
            easing(easing),
            starttime(starttime),
            endtime(endtime),
            startvalue(startvalue),
            endvalue(endvalue),
            triggerID(triggerID),
            triggerST(triggerST),
            triggerGP(triggerGP)
        {}
        std::unique_ptr<IEvent> copy() const
        {
            return std::make_unique<Event<T>>(type, easing, starttime, endtime, startvalue, endvalue, triggerID, triggerST, triggerGP);
        }
        EventType GetType() const
        {
            return type;
        }
        Easing GetEasing() const
        {
            return easing;
        }
        double GetStartTime() const
        {
            return starttime;
        }
        double GetEndTime() const
        {
            return endtime;
        }
        void SetStartTime(double time)
        {
            starttime = time;
        }
        void SetEndTime(double time)
        {
            endtime = time;
        }
        T GetStartValue() const
        {
            return startvalue;
        }
        T GetEndValue() const
        {
            return endvalue;
        }
        int GetTriggerID() const
        {
            return triggerID;
        }
        double GetTriggerST() const
        {
            return triggerST;
        }
        int GetTriggerGP() const
        {
            return triggerGP;
        }
        void SetTriggerID(int id, double st, int gp)
        {
            triggerID = id;
            triggerST = st;
            triggerGP = gp;
        }
    private:
        EventType type;
        Easing easing;
        double starttime;
        double endtime;
        T startvalue;
        T endvalue;
        int triggerID;
        double triggerST;
        int triggerGP;
    };

}
