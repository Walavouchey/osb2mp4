#include <iostream>
#include <fstream>
#include <filesystem>
#include <string>
#include <cstddef>
#include <limits>
#include <unordered_map>
#include <vector>
#include <utility>
#include <memory>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <cstdlib>
#include <exception>

namespace sb
{
	void stringReplace(std::string &s, const std::string &search, const std::string &replace) {
		std::size_t pos = 0;
		while ((pos = s.find(search, pos)) != std::string::npos) {
			 s.replace(pos, search.length(), replace);
			 pos += replace.length();
		}
	}

	std::vector<std::string> stringSplit(std::string s, const std::string &delimiter)
	{
		std::size_t pos = 0;
		std::vector<std::string> split;
		for (; (pos = s.find(delimiter)) != std::string::npos; s.erase(0, pos + delimiter.length()))
			split.emplace_back(s.substr(0, pos));
		split.emplace_back(s);
		return split;
	}

	void applyVariables(std::string &line, const std::unordered_map<std::string, std::string> &variables)
	{
		for (const std::pair<std::string, std::string> &e : variables)
			stringReplace(line, e.first, e.second);
	}

	std::string removePathQuotes(const std::string& s)
	{
		return s[0] == '"' && s[s.length() - 1] == '"' ? s.substr(1, s.length() - 2) : s;
	}

    enum class Layer
    {
        Background,
        Fail,
        Pass,
        Foreground,
        Overlay
    };
    static const std::unordered_map<std::string, Layer> LayerStrings =
    {
        {"Background", Layer::Background},
        {"Fail", Layer::Fail},
        {"Pass", Layer::Pass},
        {"Foreground", Layer::Foreground},
        {"Overlay", Layer::Overlay}
    };

    enum class Origin
    {
        TopLeft,
        TopCentre,
        TopRight,
        CentreLeft,
        Centre,
        CentreRight,
        BottomLeft,
        BottomCentre,
        BottomRight
    };
    static const std::unordered_map<std::string, Origin> OriginStrings =
    {
        {"TopLeft", Origin::TopLeft},
        {"TopCentre", Origin::TopCentre},
        {"TopRight", Origin::TopRight},
        {"TopRight", Origin::TopRight},
        {"CentreLeft", Origin::CentreLeft},
        {"Centre", Origin::Centre},
        {"CentreRight", Origin::CentreRight},
        {"BottomLeft", Origin::BottomLeft},
        {"BottomCentre", Origin::BottomCentre},
        {"BottomRight", Origin::BottomRight}
    };

    enum class LoopType
    {
        LoopForever,
        LoopOnce,
        Custom
    };
    static const std::unordered_map<std::string, LoopType> LoopTypeStrings =
    {
        {"LoopForever", LoopType::LoopForever},
        {"LoopOnce", LoopType::LoopOnce}
    };

    enum class Easing
    {
        None,
        Out,
        In,
        InQuad,
        OutQuad,
        InOutQuad,
        InCubic,
        OutCubic,
        InOutCubic,
        InQuart,
        OutQuart,
        InOutQuart,
        InQuint,
        OutQuint,
        InOutQuint,
        InSine,
        OutSine,
        InOutSine,
        InExpo,
        OutExpo,
        InOutExpo,
        InCirc,
        OutCirc,
        InOutCirc,
        InElastic,
        OutElastic,
        OutElasticHalf,
        OutElasticQuarter,
        InOutElastic,
        InBack,
        OutBack,
        InOutBack,
        InBounce,
        OutBounce,
        InOutBounce,
        Step
    };

    constexpr double PI = 3.14159265358979323846;
	double Reverse(double(*f)(double), double t) { return 1 - f(1 - t); }
	double ToInOut(double(*f)(double), double t) { return 0.5 * (t < 0.5 ? f(2 * t) : (2 - f(2 - 2 * t))); }
	double Step(double t) { return t >= 1 ? 1 : 0; }
	double Linear(double t) { return t; }
	double InQuad(double t){ return t * t; }
	double OutQuad(double t){ return Reverse(InQuad, t); }
	double InOutQuad(double t){ return ToInOut(InQuad, t); }
	double InCubic(double t){ return t * t * t; }
	double OutCubic(double t){ return Reverse(InCubic, t); }
	double InOutCubic(double t){ return ToInOut(InCubic, t); }
	double InQuart(double t){ return t * t * t * t; }
	double OutQuart(double t){ return Reverse(InQuart, t); }
	double InOutQuart(double t){ return ToInOut(InQuart, t); }
	double InQuint(double t){ return t * t * t * t * t; }
	double OutQuint(double t){ return Reverse(InQuint, t); }
	double InOutQuint(double t){ return ToInOut(InQuint, t); }
	double InSine(double t){ return 1 - std::cos(t * PI / 2); }
	double OutSine(double t){ return Reverse(InSine, t); }
	double InOutSine(double t){ return ToInOut(InSine, t); }
	double InExpo(double t){ return std::pow(2, 10 * (t - 1)); }
	double OutExpo(double t){ return Reverse(InExpo, t); }
	double InOutExpo(double t){ return ToInOut(InExpo, t); }
	double InCirc(double t){ return 1 - std::sqrt(1 - t * t); }
	double OutCirc(double t){ return Reverse(InCirc, t); }
	double InOutCirc(double t){ return ToInOut(InCirc, t); }
	double InBack(double t){ return t * t * ((1.70158 + 1) * t - 1.70158); }
	double OutBack(double t){ return Reverse(InBack, t); }
	double InOutBack(double t) { return ToInOut([](double y) { return y * y * ((1.70158 * 1.525 + 1) * y - 1.70158 * 1.525); }, t); }
	double OutBounce(double t){ return t < 1 / 2.75 ? 7.5625 * t * t : t < 2 / 2.75 ? 7.5625 * (t -= (1.5 / 2.75)) * t + .75 : t < 2.5 / 2.75 ? 7.5625 * (t -= (2.25 / 2.75)) * t + .9375 : 7.5625 * (t -= (2.625 / 2.75)) * t + .984375; }
	double InBounce(double t){ return Reverse(OutBounce, t); }
	double InOutBounce(double t){ return ToInOut(InBounce, t); }
	double OutElastic(double t){ return std::pow(2, -10 * t) * std::sin((t - 0.075) * (2 * t) / .3) + 1; }
	double InElastic(double t){ return Reverse(OutElastic, t); }
	double OutElasticHalf(double t){ return std::pow(2, -10 * t) * std::sin((0.5 * t - 0.075) * (2 * PI) / .3) + 1; }
	double OutElasticQuarter(double t){ return std::pow(2, -10 * t) * std::sin((0.25 * t - 0.075) * (2 * PI) / .3) + 1; }
	double InOutElastic(double t){ return ToInOut(InElastic, t); }

    double applyEasing(Easing easing, double t)
    {
        switch (easing)
        {
        case sb::Easing::Step: return Step(t); break;
        case sb::Easing::None: return Linear(t); break;
        case sb::Easing::Out: return OutQuad(t); break;
        case sb::Easing::In: return InQuad(t); break;
        case sb::Easing::InQuad: return InQuad(t); break;
        case sb::Easing::OutQuad: return OutQuad(t); break;
        case sb::Easing::InOutQuad: return InOutQuad(t); break;
        case sb::Easing::InCubic: return InCubic(t); break;
        case sb::Easing::OutCubic: return OutCubic(t); break;
        case sb::Easing::InOutCubic: return InOutCubic(t); break;
        case sb::Easing::InQuart: return InQuart(t); break;
        case sb::Easing::OutQuart: return OutQuart(t); break;
        case sb::Easing::InOutQuart: return InOutQuart(t); break;
        case sb::Easing::InQuint: return InQuint(t); break;
        case sb::Easing::OutQuint: return OutQuint(t); break;
        case sb::Easing::InOutQuint: return InOutQuint(t); break;
        case sb::Easing::InSine: return InSine(t); break;
        case sb::Easing::OutSine: return OutSine(t); break;
        case sb::Easing::InOutSine: return InOutSine(t); break;
        case sb::Easing::InExpo: return InExpo(t); break;
        case sb::Easing::OutExpo: return OutExpo(t); break;
        case sb::Easing::InOutExpo: return InOutExpo(t); break;
        case sb::Easing::InCirc: return InCirc(t); break;
        case sb::Easing::OutCirc: return OutCirc(t); break;
        case sb::Easing::InOutCirc: return InOutCirc(t); break;
        case sb::Easing::InElastic: return InElastic(t); break;
        case sb::Easing::OutElastic: return OutElastic(t); break;
        case sb::Easing::OutElasticHalf: return OutElasticHalf(t); break;
        case sb::Easing::OutElasticQuarter: return OutElasticQuarter(t); break;
        case sb::Easing::InOutElastic: return InOutElastic(t); break;
        case sb::Easing::InBack: return InBack(t); break;
        case sb::Easing::OutBack: return OutBack(t); break;
        case sb::Easing::InOutBack: return InOutBack(t); break;
        case sb::Easing::InBounce: return InBounce(t); break;
        case sb::Easing::OutBounce: return OutBounce(t); break;
        case sb::Easing::InOutBounce: return InOutBounce(t); break;
        default: return Linear(t); break;
        }
    }

    enum class ParameterType
    {
        FlipH,
        FlipV,
        Additive
    };
    static const std::unordered_map<std::string, ParameterType> ParameterTypeStrings =
    {
        {"H", ParameterType::FlipH},
        {"V", ParameterType::FlipV},
        {"A", ParameterType::Additive}
    };

    enum class EventType
    {
        None,
        F,
        S,
        V,
        R,
        M,
        MX,
        MY,
        C,
        P
    };
    static const std::unordered_map<std::string, EventType> EventTypeStrings
    {
        {"F", EventType::F},
        {"S", EventType::S},
        {"V", EventType::V},
        {"R", EventType::R},
        {"M", EventType::M},
        {"MX", EventType::MX},
        {"MY", EventType::MY},
        {"C", EventType::C},
        {"P", EventType::P}
    };

    enum class Keyword
    {
        None,
        Sprite,
        Animation,
        Sample,
        T,
        L
    };
    static const std::unordered_map<std::string, Keyword> KeywordStrings =
    {
        {"Sprite", Keyword::Sprite},
        {"Animation", Keyword::Animation},
        {"Sample", Keyword::Sample},
        {"T", Keyword::T},
        {"L", Keyword::L},
    };

    template<typename T>
    class Keyframe
    {
        public:
            Keyframe()
            {}
			Keyframe(double time, T value, Easing easing, double actualStarttime)
				:
				time(time),
				value(value),
				easing(easing),
				actualStarttime(actualStarttime)
			{}
			double time;
			T value;
			Easing easing;
			double actualStarttime;
    };

    class Color
    {
		public:
            Color()
            {}
            Color(double R, double G, double B)
                :
                R(R),
                G(G),
                B(B)
            {}
            double operator[](const int index) const
            {
                return index == 0 ? R : index == 1 ? G : index == 2 ? B : -1;
            };

			double R;
			double G;
			double B;
    };

    double Interpolate(double start, double end, double t)
    {
        return start + (end - start) * t;
    }
    Color Interpolate(Color start, Color end, double t)
    {
        return Color(Interpolate(start.R, end.R, t), Interpolate(start.G, end.G, t), Interpolate(start.B, end.B, t));
    }
    std::pair<double, double> Interpolate(std::pair<double, double> start, std::pair<double, double> end, double t)
    {
        return std::pair<double, double>(Interpolate(start.first, end.first, t), Interpolate(start.second, end.second, t));
    }

    template <typename T>
    T keyframeValueAt(const std::vector<Keyframe<T>> &keyframes, double time)
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
        double t = (time - keyframe.actualStarttime) / (endKeyframe.time - keyframe.actualStarttime);
        t = applyEasing(keyframe.easing, t);
        return Interpolate(keyframe.value, endKeyframe.value, t);
    }

    template <class T>
    std::pair<T, T> keyframeValueAt(const std::pair<std::vector<Keyframe<T>>, std::vector<Keyframe<T>>> &keyframes, double time)
    {
        T first = keyframeValueAt<T>(keyframes.first, time);
        T second = keyframeValueAt<T>(keyframes.second, time);
        return std::pair<T, T>(first, second);
    }

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
            virtual ~IEvent() {}
    };
    template <typename T>
    class Event : public IEvent
    {
        public:
            Event() = default;
            Event(EventType type, Easing easing, double starttime, double endtime, T startvalue, T endvalue)
                :
                type(type),
                easing(easing),
                starttime(starttime),
                endtime(endtime),
                startvalue(startvalue),
                endvalue(endvalue)
            {}
            std::unique_ptr<IEvent> copy() const
            {
                return std::make_unique<Event<T>>(type, easing, starttime, endtime, startvalue, endvalue);
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
        private:
            EventType type;
            Easing easing;
            double starttime;
            double endtime;
            T startvalue;
            T endvalue;
    };

    template <typename T, typename V, typename Selector>
    void generateKeyframes(std::vector<Keyframe<T>>& keyframes, const std::vector<std::unique_ptr<Event<V>>>& events, Selector W)
    {
		int i = -2;
		for (const std::unique_ptr<Event<V>>& event : events)
		{
			i += 2;
			bool appendEndtime = event->GetEndTime() > event->GetStartTime();
			if (i == 0)
			{
				keyframes.push_back(Keyframe<T>(-std::numeric_limits<double>::infinity(), W(event->GetStartValue()), Easing::Step, -std::numeric_limits<double>::infinity()));
				keyframes.push_back(Keyframe<T>(event->GetStartTime(), appendEndtime ? W(event->GetStartValue()) : W(event->GetEndValue()), appendEndtime ? event->GetEasing() : Easing::Step, event->GetStartTime()));
                if (appendEndtime)
                {
                    keyframes.push_back(Keyframe<T>(event->GetEndTime(), W(event->GetEndValue()), Easing::Step, event->GetEndTime()));
                }
                else i--;
				continue;
			}
			if (keyframes[i - 1].time >= event->GetStartTime())
			{
				keyframes.push_back(Keyframe<T>(keyframes[i - 1].time, appendEndtime ? W(event->GetStartValue()) : W(event->GetEndValue()), appendEndtime ? event->GetEasing() : Easing::Step, event->GetStartTime()));
				i--;
			}
			else
			{
				keyframes.push_back(Keyframe<T>(event->GetStartTime(), appendEndtime ? W(event->GetStartValue()) : W(event->GetEndValue()), appendEndtime ? event->GetEasing() : Easing::Step, event->GetStartTime()));
			}
			if (appendEndtime)
			{
				keyframes.push_back(Keyframe<T>(event->GetEndTime(), W(event->GetEndValue()), Easing::Step, event->GetEndTime()));
			}
			else i--;
		}
    }
    // TODO: will want to merge these but i managed to dig myself a little hole at the beginning
    void generateParameterKeyframes(std::vector<Keyframe<bool>>& keyframes, const std::vector<std::unique_ptr<Event<ParameterType>>>& events)
    {
		int i = -2;
		for (const std::unique_ptr<Event<ParameterType>>& event : events)
		{
			i += 2;
			bool appendEndtime = event->GetEndTime() > event->GetStartTime();
			if (i == 0)
			{
				keyframes.push_back(Keyframe<bool>(-std::numeric_limits<double>::infinity(), true, Easing::Step, -std::numeric_limits<double>::infinity()));
                if (appendEndtime)
                {
                    keyframes.push_back(Keyframe<bool>(event->GetEndTime(), false, Easing::Step, event->GetEndTime()));
                }
                else i--;
				continue;
			}
			if (keyframes[i - 1].time >= event->GetStartTime())
			{
				keyframes.push_back(Keyframe<bool>(keyframes[i - 1].time, true, appendEndtime ? event->GetEasing() : Easing::Step, event->GetStartTime()));
				i--;
			}
			else
			{
				keyframes.push_back(Keyframe<bool>(event->GetStartTime(), true, appendEndtime ? event->GetEasing() : Easing::Step, event->GetStartTime()));
			}
			if (appendEndtime)
			{
				keyframes.push_back(Keyframe<bool>(event->GetEndTime(), false, Easing::Step, event->GetEndTime()));
			}
			else i--;
		}
    }

    // a little bit of dumb bullshit down below
	template <typename T>
	struct nop
	{
		T operator()(T in)
		{
			return in;
		}
	};
    template <class ... t>
    constexpr bool alwaysFalse = false;
    template <EventType T, typename R, ParameterType P = ParameterType::Additive>
    R calculateKeyframes(const std::vector<std::unique_ptr<IEvent>> &events, std::pair<double, double> coordinates)
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
        for (const std::unique_ptr<IEvent> &event : events)
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
                keyframes.push_back(Keyframe<Color>(-std::numeric_limits<double>::infinity(), Color(1, 1, 1), Easing::Step, -std::numeric_limits<double>::infinity()));
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
				double operator()(std::pair<double, double> in)
				{
					return in.first;
				}
			};
			struct second
			{
				double operator()(std::pair<double, double> in)
				{
					return in.second;
				}
			};
			if (compatibilityMode)
			{
				generateKeyframes<double, std::pair<double, double>>(XKeyframes, applicableEventsXY, first());
				generateKeyframes<double, std::pair<double, double>>(YKeyframes, applicableEventsXY, second());
			}
			else
			{
				if (applicableEventsX.size() == 0)
                    if constexpr(T == EventType::M)
					    XKeyframes.push_back(Keyframe<double>(-std::numeric_limits<double>::infinity(), coordinates.first, Easing::Step, -std::numeric_limits<double>::infinity()));
                    else
					    XKeyframes.push_back(Keyframe<double>(-std::numeric_limits<double>::infinity(), 1, Easing::Step, -std::numeric_limits<double>::infinity()));

				else
					generateKeyframes<double, double>(XKeyframes, applicableEventsX, nop<double>());
				if (applicableEventsY.size() == 0)
                    if constexpr(T == EventType::M)
					    YKeyframes.push_back(Keyframe<double>(-std::numeric_limits<double>::infinity(), coordinates.second, Easing::Step, -std::numeric_limits<double>::infinity()));
                    else
					    YKeyframes.push_back(Keyframe<double>(-std::numeric_limits<double>::infinity(), 1, Easing::Step, -std::numeric_limits<double>::infinity()));
				else
					generateKeyframes<double, double>(YKeyframes, applicableEventsY, nop<double>());
			}
			return std::pair<std::vector<Keyframe<double>>, std::vector<Keyframe<double>>>(XKeyframes, YKeyframes);
        }
        else if constexpr (T == EventType::C)
        {
            auto ev = std::vector<std::unique_ptr<Event<Color>>>();
            for (const auto &e : applicableEvents)
            {
                ev.push_back(std::make_unique<Event<Color>>(*dynamic_cast<Event<Color>*>(e->copy().get())));
            }
            generateKeyframes<Color, Color>(keyframes, ev, nop<Color>());
            return keyframes;
        }
        else if constexpr (T == EventType::P)
        {
            auto ev = std::vector<std::unique_ptr<Event<ParameterType>>>();
            for (const auto &e : applicableEvents)
            {
                ev.push_back(std::make_unique<Event<ParameterType>>(*dynamic_cast<Event<ParameterType>*>(e->copy().get())));
            }
            generateParameterKeyframes(keyframes, ev);
            return keyframes;
        }
        else
        {
            auto ev = std::vector<std::unique_ptr<Event<double>>>();
            for (const auto &e : applicableEvents)
            {
                ev.push_back(std::make_unique<Event<double>>(*dynamic_cast<Event<double>*>(e->copy().get())));
            }
            generateKeyframes<double, double>(keyframes, ev, nop<double>());
            return keyframes;
        }
    }

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
                    for (std::unique_ptr<IEvent> &event : events)
                    {
                        if (i == 0) durations.emplace_back(std::pair<double, double>{event->GetStartTime(), event->GetEndTime()});
                        event->SetStartTime(starttime + durations[j].first + looplength * i);
                        event->SetEndTime(starttime + durations[j].second + looplength * i);
                        expandedEvents.push_back(event->copy());
                        j++;
                    }
                }
                events = std::move(expandedEvents);
                endtime = starttime + (looplength) * loopcount;
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
            Trigger(const std::string &triggerName, double starttime, double endtime, int groupNumber)
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
            void Initialise()
            {
                // TODO
            }
            const std::vector<std::unique_ptr<IEvent>>& GetEvents() const
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
        private:
        std::vector<std::unique_ptr<IEvent>> events;
        std::string triggerName;
        double starttime;
        double endtime;
        int groupNumber;
    };

    class Sprite
    {
        public:
            Sprite(Layer layer, Origin origin, const std::string &filepath, const std::pair<double, double> &coordinates)
                :
                layer(layer),
                origin(origin),
                filepath(filepath),
                coordinates(coordinates)
            {}
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
            void Initialise()
            {
                initialised = true;
                for (Loop &loop : loops) loop.Initialise();
                for (Trigger &trigger : triggers) trigger.Initialise();
                for (Loop &loop : loops)
                    for (std::unique_ptr<IEvent> &event : loop.GetEvents())
                        events.emplace_back(std::move(event));
                std::sort(events.begin(), events.end(), [](const std::unique_ptr<IEvent>& a, const std::unique_ptr<IEvent>& b) { return a->GetStartTime() < b->GetStartTime(); });
                double endTime = std::numeric_limits<double>::min();
                for (const std::unique_ptr<IEvent> &event : events)
                    endTime = std::max(endTime, event->GetEndTime());
                activetime = std::pair<double, double>({events.size() > 0 ? (*events.begin())->GetStartTime() : std::numeric_limits<double>::max(), endTime});

                positionKeyframes = calculateKeyframes<EventType::M, std::pair<std::vector<Keyframe<double>>, std::vector<Keyframe<double>>>>(events, coordinates);
                rotationKeyframes = calculateKeyframes<EventType::R, std::vector<Keyframe<double>>>(events, coordinates);
                scaleKeyframes = calculateKeyframes<EventType::S, std::pair<std::vector<Keyframe<double>>, std::vector<Keyframe<double>>>>(events, coordinates);
                colorKeyframes = calculateKeyframes<EventType::C, std::vector<Keyframe<Color>>>(events, coordinates);
                opacityKeyframes = calculateKeyframes<EventType::F, std::vector<Keyframe<double>>>(events, coordinates);
                flipHKeyframes = calculateKeyframes<EventType::P, std::vector<Keyframe<bool>>, ParameterType::FlipH>(events, coordinates);
                flipVKeyframes = calculateKeyframes<EventType::P, std::vector<Keyframe<bool>>, ParameterType::FlipV>(events, coordinates);
                additiveKeyframes = calculateKeyframes<EventType::P, std::vector<Keyframe<bool>>>(events, coordinates);
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
            Color ColorAt(double time) const
            {
                return keyframeValueAt<Color>(colorKeyframes, time);
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
            virtual const std::string GetFilePath(double time) const
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
        protected:
            std::vector<Loop> loops;
            std::vector<Trigger> triggers;
            std::pair<double, double> activetime;
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
            std::vector<Keyframe<Color>> colorKeyframes;
            std::vector<Keyframe<double>> opacityKeyframes;
            std::vector<Keyframe<bool>> flipVKeyframes;
            std::vector<Keyframe<bool>> flipHKeyframes;
            std::vector<Keyframe<bool>> additiveKeyframes;
    };

    class Animation : public Sprite
    {
        public:
            Animation(Layer layer, Origin origin, const std::string &filepath, const std::pair<double, double> &coordinates, int framecount, double framedelay, LoopType looptype)
                :
                Sprite(layer, origin, filepath, coordinates),
                framecount(framecount),
                framedelay(framedelay),
                looptype(looptype)
            {}
            int frameIndexAt(double time) const
            {
                if (time - activetime.first < framecount * framedelay || looptype == LoopType::LoopForever)
                {
                    return (int) std::fmod(((time - activetime.first) / framedelay), (double) framecount); 
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
            Sample(double starttime, Layer layer, const std::string &filepath, float volume)
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

	std::string exec(const std::string& cmd)
	{
		char buffer[128];
		std::string result = "";
		FILE* pipe = _popen(cmd.c_str(), "r");
		if (!pipe) throw std::runtime_error("popen() failed!");
		try {
			while (fgets(buffer, sizeof buffer, pipe) != NULL) {
				result += buffer;
			}
		} catch (...) {
			_pclose(pipe);
			throw;
		}
		_pclose(pipe);
		return result;
	}

    class Storyboard
    {
        public:
            Storyboard(const std::filesystem::path& directory, const std::filesystem::path& osb, std::vector<std::unique_ptr<Sprite>>& sprites, std::vector<Sample>& samples, const std::unordered_map<std::string, std::string>& info, /* Background background, Video video, */ std::pair<std::size_t, std::size_t> resolution, double musicVolume, double effectVolume)
                :
                directory(directory),
                osb(osb),
                samples(samples),
                info(info),
                resolution(resolution),
                musicVolume(musicVolume),
                effectVolume(effectVolume),
                frameScale(resolution.first / 854.0),
                xOffset((int) (resolution.first - resolution.second * 4 / 3.0) * 0.5)
            {
                this->sprites = std::move(sprites);
                std::cout << "Initialising storyboard (" << this->sprites.size() << " sprites, " << samples.size() << " samples)" << "\n";
                for (std::unique_ptr<Sprite>& sprite : this->sprites)
                    sprite->Initialise();
                std::cout << "Initialised " << this->sprites.size() << " sprites/animations\n";

                for (const std::unique_ptr<Sprite>& sprite : this->sprites)
                {
                    std::vector<std::string> filePaths = sprite->GetFilePaths();
                    for (std::string filePath : filePaths)
                    {
                        if (spriteImages.find(filePath) != spriteImages.end()) continue;
                        cv::Mat image = cv::imread((directory / filePath).generic_string(), cv::IMREAD_UNCHANGED);
                        cv::Mat converted;
                        cv::cvtColor(image, converted, cv::COLOR_BGR2BGRA);
                        converted.convertTo(image, CV_32F);
                        spriteImages.emplace(filePath, image);
                    }
                }
            }

            void generateAudio(const std::string& outFile)
            {
                std::cout << "Generating audio...\n";
                std::string command = "ffmpeg -y -hide_banner -v error -stats";
                auto k = info.find("AudioFilename");
                if (k != info.end()) command += " -i \"" + (directory / k->second).generic_string() + "\"";
                std::vector<int> delays;
                std::vector<double> volumes;
                volumes.push_back((samples.size() + 1) * musicVolume);
                auto l = info.find("AudioLeadIn");
                if (k != info.end() && l != info.end()) delays.push_back(std::stoi(l->second));
                else if (k != info.end()) delays.push_back(1);
                int maxDelay = samples.size() == 0 ? 0 : std::numeric_limits<int>::lowest();
                double maxDuration = samples.size() == 0 ? 0 : std::numeric_limits<double>::lowest();
                for (const Sample& sample: samples)
                {
                    command += " -i \"" + (directory / sample.filepath).generic_string() + "\"";
                    int delay = (int)sample.starttime;
                    delays.push_back(delay);
                    volumes.push_back(sample.volume / 100.0 * (samples.size() + 1) * effectVolume);
                    maxDelay = std::max(maxDelay, delay);
                    maxDuration = std::max(maxDuration, std::stod(exec("ffprobe -v quiet -show_entries format=duration -of default=noprint_wrappers=1:nokey=1 \"" + (directory / sample.filepath).generic_string() + "\"")));
                }
                command += " -filter_complex ";
                std::string filters;
                for (int i = 0; i < delays.size(); i++)
                    filters += "[" + std::to_string(i) + ":a]volume=" + std::to_string(volumes[i]) + ",adelay=delays=" + std::to_string(delays[i]) + ":all=1[d" + std::to_string(i) + "];";
                for (int i = 0; i < delays.size(); i++)
                    filters += "[d" + std::to_string(i) + "]";
                filters += "amix=inputs=" + std::to_string(delays.size()) + ":dropout_transition=" + std::to_string(maxDelay + maxDuration) + "[a]";
                command += "\"" + filters + "\"";
                command += " -map [a] ";
                command += outFile;
                //std::cout << command << "\n";
                system(command.c_str());
            }
            cv::Mat DrawFrame(double time)
            {
                cv::Mat frame = cv::Mat::zeros(resolution.second, resolution.first, CV_8UC3);
                for (const std::unique_ptr<Sprite>& sprite : sprites)
                {

                    if (!(sprite->GetActiveTime().first <= time && sprite->GetActiveTime().second > time))
                        continue;
                    double alpha = sprite->OpacityAt(time);
                    if (alpha == 0) continue;
                    std::pair<double, double> scale = sprite->ScaleAt(time);
                    if (scale.first == 0 || scale.second == 0) continue;

                    std::unordered_map<std::string, cv::Mat>::const_iterator k = spriteImages.find(sprite->GetFilePath(time));
                    if (k == spriteImages.end()) continue;
                    cv::Mat image = k->second.clone();

                    scale = std::pair<double, double>(scale.first * frameScale, scale.second * frameScale);
                    std::pair<double, double> newSize = std::pair<double, double>(image.cols * std::abs(scale.first), image.rows * std::abs(scale.second));

                    if (newSize.first < 1 || newSize.second < 1) continue;

                    double rotation = sprite->RotationAt(time);
                    cv::Mat rotMatrix;
                    bool shouldRotate = rotation != 0;
                    double width1 = newSize.first;
                    double height1 = newSize.second;
                    double width2 = newSize.first;
                    double height2 = newSize.second;
                    if (shouldRotate)
                    {
                        double w = width1;
                        double h = height1;
                        double cx = width1 / 2;
                        double cy = height1 / 2;
                        rotMatrix = cv::getRotationMatrix2D(cv::Point2f(cx, cy), -rotation * 180 / PI, 1.0);
                        double cos = std::abs(rotMatrix.at<double>(0, 0));
                        double sin = std::abs(rotMatrix.at<double>(0, 1));
                        width2 = h * sin + w * cos;
                        height2 = h * cos + w * sin;
                        cv::MatIterator_<double> it = rotMatrix.begin<double>() + 2;
                        *it += width2 / 2 - cx;
                        it += 3;
                        *it += height2 / 2 - cy;
                    }
                    
                    double dx = 0.5 * (width2 - width1);
                    double dy = 0.5 * (height2 - height1);
                    double midx = width2 * 0.5;
                    double midy = height2 * 0.5;
                    std::pair<double, double> origin;
                    switch (sprite->GetOrigin())
                    {
						case Origin::TopLeft: origin = { dx, dy }; break;
						case Origin::TopCentre: origin = { midx, dy }; break;
						case Origin::TopRight: origin = { width2 - dx, dy }; break;
						case Origin::CentreLeft: origin = { dx, midy }; break;
						case Origin::Centre: origin = { midx, midy }; break;
						case Origin::CentreRight: origin = { width2 - dx, midy }; break;
						case Origin::BottomLeft: origin = { dx, height2 - dy }; break;
						case Origin::BottomCentre: origin = { midx, height2 - dy }; break;
						case Origin::BottomRight: origin = { width2 - dx, height2 - dy }; break;
						default: origin = { midx, midy }; break;
                    }
                    if (shouldRotate)
                    {
                        origin = { origin.first - midx, origin.second - midy };
                        origin =
                        {
                            origin.first * std::cos(rotation) - origin.second * std::sin(rotation),
                            origin.second * std::cos(rotation) + origin.first * std::sin(rotation)
                        };
                        origin = { origin.first + midx, origin.second + midy };
                    }

                    std::pair<double, double> position = sprite->PositionAt(time);
                    std::pair<double, double> framePosition =
                    {
                        (int)(position.first * frameScale - origin.first) + xOffset,
                        (int)(position.second * frameScale - origin.second)
                    };
                    int x1 = framePosition.first;
                    int x2 = framePosition.first + width2;
                    int y1 = framePosition.second;
                    int y2 = framePosition.second + height2;
                    int sx1 = x1 >= 0 ? 0 : -x1;
                    int sx2 = x2 <= resolution.first ? width2 : width2 - (x2 - resolution.first);
                    int sy1 = y1 >= 0 ? 0 : -y1;
                    int sy2 = y2 <= resolution.second ? height2 : height2 - (y2 - resolution.second);
                    y1 = std::clamp(y1, 0, (int) resolution.second);
                    y2 = std::clamp(y2, 0, (int) resolution.second);
                    x1 = std::clamp(x1, 0, (int) resolution.first);
                    x2 = std::clamp(x2, 0, (int) resolution.first);

                    if (!(y2 - y1 > 0 && x2 - x1 > 0)) continue;

                    bool flipH = sprite->EffectAt(time, ParameterType::FlipH) || scale.first < 0;
                    bool flipV = sprite->EffectAt(time, ParameterType::FlipV) || scale.second < 0;
                    int flip = (flipH && flipV) ? -1 : flipV ? 0 : flipH ? 1 : 999;
                    if (flip != 999) cv::flip(image, image, flip);

                    Color color = sprite->ColorAt(time);

                    cv::Mat resized;
                    cv::resize(image, resized, cv::Size(newSize.first, newSize.second));
                    image = std::move(resized);

                    if (shouldRotate)
                    {
                        cv::Mat rotated;
                        cv::warpAffine(image, rotated, rotMatrix, cv::Size(width2, height2));
                        image = std::move(rotated);
                    }

                    cv::Mat frameRegion = cv::Mat(frame, cv::Rect(x1, y1, x2 - x1, y2 - y1));
                    cv::Mat imageSource = cv::Mat(image, cv::Rect(sx1, sy1, sx2 - sx1, sy2 - sy1));
                    int nPixels = frameRegion.rows * frameRegion.cols * frameRegion.channels();
                    cv::MatConstIterator_<cv::Vec<float, 4>> imageIt = imageSource.begin<cv::Vec<float, 4>>();
                    cv::MatIterator_<cv::Vec<uint8_t, 3>> frameIt = frameRegion.begin<cv::Vec<uint8_t, 3>>();
                    cv::MatConstIterator_<cv::Vec<uint8_t, 3>> frameItEnd = frameRegion.end<cv::Vec<uint8_t, 3>>();
                    if (sprite->EffectAt(time, ParameterType::Additive))
                    {
                        for (; frameIt != frameItEnd; imageIt++, frameIt++)
                        {
                            float newAlpha = (*imageIt)[3] * alpha / 255.0f;
                            *frameIt = cv::Vec<uint8_t, 3>(
                                cv::saturate_cast<uint8_t>((*frameIt)[0] + newAlpha * (*imageIt)[0] * color[2]),
                                cv::saturate_cast<uint8_t>((*frameIt)[1] + newAlpha * (*imageIt)[1] * color[1]),
                                cv::saturate_cast<uint8_t>((*frameIt)[2] + newAlpha * (*imageIt)[2] * color[0])
                            );
                        }
                    }
                    else
                    {
                        for (; frameIt != frameItEnd; imageIt++, frameIt++)
                        {
                            float newAlpha = (*imageIt)[3] * alpha / 255.0f;
                            *frameIt = cv::Vec<uint8_t, 3>(
                                cv::saturate_cast<uint8_t>((1 - newAlpha) * (*frameIt)[0] + newAlpha * (*imageIt)[0] * color[2]),
                                cv::saturate_cast<uint8_t>((1 - newAlpha) * (*frameIt)[1] + newAlpha * (*imageIt)[1] * color[1]),
                                cv::saturate_cast<uint8_t>((1 - newAlpha) * (*frameIt)[2] + newAlpha * (*imageIt)[2] * color[0])
                            );
                        }
                    }
                }
                return frame;
            }
        private:
            std::filesystem::path directory;
            std::filesystem::path osb;
            std::vector<std::unique_ptr<Sprite>> sprites;
            std::vector<Sample> samples;
            std::unordered_map<std::string, std::string> info;
            // const Background background;
            // const Video video;
            const std::pair<std::size_t, std::size_t> resolution;
            double musicVolume;
            double effectVolume;
            std::unordered_map<std::string, cv::Mat> spriteImages;
            double frameScale;
            double xOffset;
    };
    
    void parseFile(std::ifstream& file, std::vector<std::unique_ptr<Sprite>>& sprites, std::vector<Sample>& samples, std::unordered_map<std::string, std::string>& variables, std::unordered_map<std::string, std::string>& info, size_t& lineNumber)
    {
        std::string line;
        bool inLoop = false;
        bool inTrigger = false;
        enum class Section
        {
            None,
            Events,
            Variables,
            Info
        };
        Section section = Section::None;

        while (file.good())
        {
            lineNumber++;
            constexpr int lineMaxReadLength = 10000;
            char lineTemp[lineMaxReadLength];
            file.getline(lineTemp, lineMaxReadLength);
            line = std::string(lineTemp);

            if (line.length() == 0) continue;
            if (line.rfind("//", 0) == 0) continue;

            // Determine start of a new section.
            if (line.find("[Events]") != std::string::npos)
            {
                section = Section::Events;
                continue;
            }
            else if (line.find("[Variables]") != std::string::npos)
            {
                section = Section::Variables;
                continue;
            }
            else if (line.find("[General]") != std::string::npos || line.find("[Metadata]") != std::string::npos)
            {
                section = Section::Info;
                continue;
            }
            else if (line.find("[") != std::string::npos)
            {
                section = Section::None;
                continue;
            }

            switch (section)
            {
                case Section::None:
                    continue;

                case Section::Events:
					{
						std::size_t depth = 0;
						while (line[depth] == ' ' || line[depth] == '_') depth++;
                        line.erase(0, depth);

						applyVariables(line, variables);
						std::vector<std::string> split = stringSplit(line, ",");

						if (inTrigger && depth < 2) inTrigger = false;
						if (inLoop && depth < 2) inLoop = false;

                        std::unordered_map<std::string, Keyword>::const_iterator k = KeywordStrings.find(split[0]);
                        Keyword keyword = k == KeywordStrings.end() ? Keyword::None : k->second;
						switch (keyword)
						{
							case Keyword::Sprite:
							{
								// TODO: Error handling
								Layer layer = LayerStrings.find(split[1])->second;
								Origin origin = OriginStrings.find(split[2])->second;
								std::string path = removePathQuotes(split[3]);
								float x = std::stof(split[4]);
								float y = std::stof(split[5]);
								sprites.push_back(std::make_unique<Sprite>(layer, origin, path, std::pair<double, double>(x, y)));
							}
							break;
							case Keyword::Animation:
							{
								Layer layer = LayerStrings.find(split[1])->second;
								Origin origin = OriginStrings.find(split[2])->second;
								std::string path = removePathQuotes(split[3]);
								float x = std::stof(split[4]);
								float y = std::stof(split[5]);
								int frameCount = std::stoi(split[6]);
								double frameDelay = std::stod(split[7]);
                                auto l = LoopTypeStrings.find(split[8]); // TODO: parse enums either by integer or name
                                LoopType loopType = split.size() > 8 && l != LoopTypeStrings.end() ? l->second : LoopType::LoopForever;
								sprites.push_back(std::make_unique<class Animation>(layer, origin, path, std::pair<double, double>(x, y), frameCount, frameDelay, loopType));
							}
							break;
							case Keyword::Sample:
							{
								double time = std::stod(split[1]);
                                Layer layer = static_cast<Layer>(std::stoi(split[2]));
								std::string path = removePathQuotes(split[3]);
								float volume = std::stof(split[4]);
								samples.emplace_back(time, layer, path, volume);
							}
							break;
							case Keyword::T:
							{
								if (inTrigger || inLoop)
								{
									// TODO: Error
								}
								std::string triggerName = split[1];
								double startTime = std::stod(split[2]);
								double endTime = std::stod(split[3]);
								int groupNumber = split.size() > 4 ? std::stoi(split[4]) : 0;
								(*(sprites.end() - 1))->AddTrigger({ triggerName, startTime, endTime, groupNumber });
								inTrigger = true;
							}
							break;
							case Keyword::L:
							{
								if (inLoop || inTrigger)
								{
									// TODO: Error
								}
								double startTime = std::stod(split[1]);
								int loopCount = std::stoi(split[2]);
								(*(sprites.end() - 1))->AddLoop({ startTime, loopCount });
								inLoop = true;
							}
							break;
                            case Keyword::None:
							default:
							{
                                if (split[0] == "0" && split[1] == "0" && depth == 0)
                                {
                                    std::string path = split[2];
                                    std::pair<double, double> offset = split.size() < 3 ? std::pair<double, double>( std::stoi(split[3]), std::stoi(split[4]) ) : std::pair<double, double>( 0, 0 );
                                    // TODO: background
                                    break;
                                }
                                if ((split[0] == "Video" || split[0] == "1") && depth == 0)
                                {
                                    // TODO: video
                                    break;
                                }
                                if (depth == 0) break;
								if (split[3].length() == 0)
									split[3] = split[2];

								Easing easing = static_cast<Easing>(std::stoi(split[1]));
								double startTime = std::stod(split[2]);
								double endTime = std::stod(split[3]);

								std::unordered_map<std::string, EventType>::const_iterator k = EventTypeStrings.find(split[0]);
								EventType eventType = k == EventTypeStrings.end() ? EventType::None : k->second;

								switch (eventType)
								{
									case EventType::F:
									{
										double startValue = std::stod(split[4]);
										double endValue = split.size() > 5 ? std::stod(split[5]) : startValue;
										std::unique_ptr<Event<double>> event = std::make_unique<Event<double>>(EventType::F, easing, startTime, endTime, startValue, endValue);
										if (inTrigger) (*(sprites.end() - 1))->AddEventInTrigger(std::move(event));
										else if (inLoop) (*(sprites.end() - 1))->AddEventInLoop(std::move(event));
                                        else (*(sprites.end() - 1))->AddEvent(std::move(event));
									}
									break;
									case EventType::S:
									{
										double startValue = std::stod(split[4]);
										double endValue = split.size() > 5 ? std::stod(split[5]) : startValue;
										std::unique_ptr<Event<double>> event = std::make_unique<Event<double>>(EventType::S, easing, startTime, endTime, startValue, endValue);
										if (inTrigger) (*(sprites.end() - 1))->AddEventInTrigger(std::move(event));
										else if (inLoop) (*(sprites.end() - 1))->AddEventInLoop(std::move(event));
                                        else (*(sprites.end() - 1))->AddEvent(std::move(event));
									}
									break;
									case EventType::V:
									{
										double startX = std::stod(split[4]);
										double startY = std::stod(split[5]);
										double endX = split.size() > 6 ? std::stod(split[6]) : startX;
										double endY = split.size() > 7 ? std::stod(split[7]) : startY;
										std::unique_ptr<Event<std::pair<double, double>>> event = std::make_unique<Event<std::pair<double, double>>>(EventType::V, easing, startTime, endTime, std::pair<double, double>{ startX, startY }, std::pair<double, double>{ endX, endY });
										if (inTrigger) (*(sprites.end() - 1))->AddEventInTrigger(std::move(event));
										else if (inLoop) (*(sprites.end() - 1))->AddEventInLoop(std::move(event));
										else (*(sprites.end() - 1))->AddEvent(std::move(event));
									}
									break;
									case EventType::R:
									{
										double startValue = std::stod(split[4]);
										double endValue = split.size() > 5 ? std::stod(split[5]) : startValue;
										std::unique_ptr<Event<double>> event = std::make_unique<Event<double>>(EventType::R, easing, startTime, endTime, startValue, endValue);
										if (inTrigger) (*(sprites.end() - 1))->AddEventInTrigger(std::move(event));
										else if (inLoop) (*(sprites.end() - 1))->AddEventInLoop(std::move(event));
										else (*(sprites.end() - 1))->AddEvent(std::move(event));
									}
									break;
									case EventType::M:
									{
										double startX = std::stod(split[4]);
										double startY = std::stod(split[5]);
										double endX = split.size() > 6 ? std::stod(split[6]) : startX;
										double endY = split.size() > 7 ? std::stod(split[7]) : startY;
										std::unique_ptr<Event<std::pair<double, double>>> event = std::make_unique<Event<std::pair<double, double>>>(EventType::M, easing, startTime, endTime, std::pair<double, double>{ startX, startY }, std::pair<double, double>{ endX, endY });
										if (inTrigger) (*(sprites.end() - 1))->AddEventInTrigger(std::move(event));
										else if (inLoop) (*(sprites.end() - 1))->AddEventInLoop(std::move(event));
										else (*(sprites.end() - 1))->AddEvent(std::move(event));
									}
									break;
									case EventType::MX:
									{
										double startValue = std::stod(split[4]);
										double endValue = split.size() > 5 ? std::stod(split[5]) : startValue;
										std::unique_ptr<Event<double>> event = std::make_unique<Event<double>>(EventType::MX, easing, startTime, endTime, startValue, endValue);
										if (inTrigger) (*(sprites.end() - 1))->AddEventInTrigger(std::move(event));
										else if (inLoop) (*(sprites.end() - 1))->AddEventInLoop(std::move(event));
										else (*(sprites.end() - 1))->AddEvent(std::move(event));
									}
									break;
									case EventType::MY:
									{
										double startValue = std::stod(split[4]);
										double endValue = split.size() > 5 ? std::stod(split[5]) : startValue;
										std::unique_ptr<Event<double>> event = std::make_unique<Event<double>>(EventType::MY, easing, startTime, endTime, startValue, endValue);
										if (inTrigger) (*(sprites.end() - 1))->AddEventInTrigger(std::move(event));
										else if (inLoop) (*(sprites.end() - 1))->AddEventInLoop(std::move(event));
										else (*(sprites.end() - 1))->AddEvent(std::move(event));
									}
									break;
									case EventType::C:
									{
										int startR = std::stoi(split[4]);
										int startG = std::stoi(split[5]);
										int startB = std::stoi(split[6]);
										int endR = split.size() > 7 ? std::stoi(split[7]) : startR;
										int endG = split.size() > 8 ? std::stoi(split[8]) : startG;
										int endB = split.size() > 9 ? std::stoi(split[9]) : startB;
										std::unique_ptr<Event<Color>> event = std::make_unique<Event<Color>>(EventType::C, easing, startTime, endTime, Color{ startR / 255.0f, startG / 255.0f, startB / 255.0f }, Color{ endR / 255.0f, endG / 255.0f, endB / 255.0f });
										if (inTrigger) (*(sprites.end() - 1))->AddEventInTrigger(std::move(event));
										else if (inLoop) (*(sprites.end() - 1))->AddEventInLoop(std::move(event));
										else (*(sprites.end() - 1))->AddEvent(std::move(event));
									}
									break;
									case EventType::P:
									{
										ParameterType parameterType = ParameterTypeStrings.find(split[4])->second;
										std::unique_ptr<Event<ParameterType>> event;
										switch (parameterType)
										{
										case ParameterType::Additive:
											event = std::make_unique<Event<ParameterType>>(EventType::P, easing, startTime, endTime, ParameterType::Additive, ParameterType::Additive);
											break;
										case ParameterType::FlipH:
											event = std::make_unique<Event<ParameterType>>(EventType::P, easing, startTime, endTime, ParameterType::FlipH, ParameterType::FlipH);
											break;
										case ParameterType::FlipV:
											event = std::make_unique<Event<ParameterType>>(EventType::P, easing, startTime, endTime, ParameterType::FlipV, ParameterType::FlipV);
											break;
										}
										if (inTrigger) (*(sprites.end() - 1))->AddEventInTrigger(std::move(event));
										else if (inLoop) (*(sprites.end() - 1))->AddEventInLoop(std::move(event));
                                        else (*(sprites.end() - 1))->AddEvent(std::move(event));
									}
									break;
                                    case EventType::None:
                                        break;
								}
						    }
						    break;
						}
					}
                    break;

                case Section::Variables:
					{
						std::size_t splitPos = line.find('=');
						// Ignore invalid variables
						if (splitPos == std::string::npos || splitPos == line.length() - 1) continue;
						std::string key = line.substr(0, splitPos);
						std::string value = line.substr(splitPos + 1, line.length() - splitPos - 1);
						variables.emplace(key, value); // doesn't overwrite previous values TODO: Check if this is correct behaviour
					}
                    break;

                case Section::Info:
					{
						std::size_t splitPos = line.find(':');
						// Ignore invalid variables
						if (splitPos == std::string::npos || splitPos == line.length() - 1) continue;
						std::string key = line.substr(0, splitPos);
						std::string value = line.substr(splitPos + 1, line.length() - splitPos - 1);
						while (std::isspace(value[0])) value.erase(0, 1);
						info.emplace(key, value); // TODO: Check if this is correct behaviour
					}
                    break;
            }
        }
    }

    std::unique_ptr<Storyboard> ParseStoryboard(const std::string &directory, const std::string &diff, std::pair<size_t, size_t> resolution, double musicVolume, double effectVolume)
    {
		std::string osb = std::string();
		for (const std::filesystem::directory_entry& entry : std::filesystem::directory_iterator(directory))
		{
			if (entry.path().extension() == ".osb")
			{
				osb = entry.path().string();
				break;
			}
		}
		if (osb.empty())
		{
            throw std::exception("No .osb file found.\n");
		}
        std::ifstream osbFile(osb);
        if (!osbFile.is_open()) throw std::exception(("Failed to open \"" + osb + "\".\n").c_str());
        std::ifstream diffFile(std::filesystem::path(directory) / diff);
        if (!diffFile.is_open()) throw std::exception(("Failed to open \"" + diff + "\".\n").c_str());
        std::vector<std::unique_ptr<Sprite>> sprites;
        std::vector<Sample> samples;
        std::unordered_map<std::string, std::string> variables;
        std::unordered_map<std::string, std::string> info;
        bool inLoop = false;
        bool inTrigger = false;
        std::size_t lineNumber = 0;

        std::cout << "Parsing " << osb << "...\n";
        parseFile(osbFile, sprites, samples, variables, info, lineNumber);
        osbFile.close();
        std::cout << "Parsed " << lineNumber << " lines\n";

        lineNumber = 0;
        std::cout << "Parsing " << diff << "...\n";
        parseFile(diffFile, sprites, samples, variables, info, lineNumber);
        diffFile.close();
        std::cout << "Parsed " << lineNumber << " lines\n";

        std::unique_ptr<Storyboard> sb = std::make_unique<Storyboard>(directory, osb, sprites, samples, info, resolution, musicVolume, effectVolume);
        return sb;
    }
}

int main(int argc, char *argv[]) {
    if (argc <= 4) std::cout << "Specify directory, diff name, start time and duration in milliseconds\n";
    std::string directory = argv[1];
    std::string diff = argv[2];
    int starttime = std::stoi(argv[3]);
    int duration = std::stoi(argv[4]);
    int frameWidth = 1920;
    int frameHeight = 1080;
    int fps = 30;
    std::string outputFile = "video.mp4";

	std::unique_ptr<sb::Storyboard> sb = sb::ParseStoryboard(directory, diff, { 1920, 1080 }, 0.2, 0.2);
    int frameCount = (int) std::round(fps / (1000.0 / (float)duration));
    std::cout << "Rendering video...\n";
    cv::VideoWriter writer = cv::VideoWriter(
        "export.mp4",
        cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
        fps,
        cv::Size(frameWidth, frameHeight)
    );
    for (int i = 0; i < frameCount; i++)
    {
        std::cout << i + 1 << "/" << frameCount << '\r';
        cv::Mat frame = sb->DrawFrame(starttime + i * 1000.0 / fps);
        writer.write(frame);
    }
    writer.release();
    std::cout << "\ndone\n";
    sb->generateAudio("audio.mp3");
    std::cout << "Merging audio and video...\n";
    std::stringstream command;
    command << "ffmpeg -y -loglevel error -stats -i export.mp4 -ss " << starttime << "ms -to "
        << starttime + duration << "ms -accurate_seek -i audio.mp3 " << outputFile;
    system(command.str().c_str());
    std::cout << "Done\n";
    /*
    try
    {
        std::unique_ptr<sb::Storyboard> sb = sb::ParseStoryboard(directory, diff, { 1920, 1080 });
        sb->generateAudio("audio.mp3");
    }
    catch (std::exception e)
    {
        std::cout << e.what();
        return 1;
    }
    */
    return 0;
}
