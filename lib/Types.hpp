#pragma once

#include <memory>
#include <string>

namespace sb
{
	class Colour
	{
	public:
		Colour()
		{}
		Colour(double R, double G, double B)
			:
			R(R),
			G(G),
			B(B)
		{}
		double operator[](const int index) const
		{
			return index == 0 ? R : index == 1 ? G : index == 2 ? B : -1;
		};
		Colour operator+(const Colour& other) const
		{
			return Colour(R + other.R, G + other.G, B + other.B);
		};
		Colour operator-(const Colour& other) const
		{
			return Colour(R - other.R, G - other.G, B - other.B);
		};
		Colour operator*(const double other) const
		{
			return Colour(R * other, G * other, B * other);
		};
		double R;
		double G;
		double B;
	};

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

	// https://osu.ppy.sh/wiki/en/osu%21_File_Formats/Osu_%28file_format%29#hitsounds
	// https://osu.ppy.sh/wiki/en/Storyboard_Scripting/Compound_Commands
	class HitSound
	{
	public:
		HitSound() {}
		HitSound(int normalSet, int additionSet, int additionFlag, int index)
			:
			normalSet(normalSet),
			additionSet(additionSet),
			additionFlag(additionFlag),
			index(index)
		{}
		HitSound(const std::string& triggerType)
		{
			int p = 8;
			if (triggerType.find("All", p) == p) { p += 3; normalSet = -1; }
			else if (triggerType.find("Normal", p) == p) { p += 6; normalSet = 1; }
			else if (triggerType.find("Soft", p) == p) { p += 4; normalSet = 2; }
			else if (triggerType.find("Drum", p) == p) { p += 4; normalSet = 3; }
			else normalSet = -1;

			if (triggerType.find("All", p) == p) { p += 3; additionSet = -1; }
			else if (triggerType.find("Normal", p) == p) { p += 6; additionSet = 1; }
			else if (triggerType.find("Soft", p) == p) { p += 4; additionSet = 2; }
			else if (triggerType.find("Drum", p) == p) { p += 4; additionSet = 3; }
			else additionSet = -1;

			if (triggerType.find("Whistle", p) == p) { p += 7; additionFlag = 2; }
			else if (triggerType.find("Finish", p) == p) { p += 6; additionFlag = 4; }
			else if (triggerType.find("Clap", p) == p) { p += 4; additionFlag = 8; }
			else additionFlag = -1;

			index = triggerType.size() > p ? std::stoi(triggerType.substr(p)) : -1;
		}
		bool operator==(const HitSound& other) const
		{
			return (other.normalSet == -1 ? true : normalSet == other.normalSet)
				&& (other.additionSet == -1 ? true : additionSet == other.additionSet)
				&& (other.additionFlag == -1 ? true :
					(additionFlag & 2) >> 1 && (other.additionFlag & 2) >> 1
					|| (additionFlag & 4) >> 2 && (other.additionFlag & 4) >> 2
					|| (additionFlag & 8) >> 3 && (other.additionFlag & 8) >> 3)
				&& (other.index == -1 ? true : index == other.index);
		}
		static bool IsHitSound(const std::string& triggerType)
		{
			return triggerType.find("HitSound") == 0;
		}

	private:
		char normalSet = 0; // 0 - no sample set, 1 - normal, 2 - soft, 3 - drum
		char additionSet = 0; // 0 - no sample set, 1 - normal, 2 - soft, 3 - drum
		char additionFlag = 0; // bitflag: 0 - normal, 1 - whistle, 2 - finish, 3 - clap
		char index = 0;
	};
}