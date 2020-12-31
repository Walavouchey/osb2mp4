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
	void stringReplace(std::string& s, const std::string& search, const std::string& replace) {
		std::size_t pos = 0;
		while ((pos = s.find(search, pos)) != std::string::npos) {
			s.replace(pos, search.length(), replace);
			pos += replace.length();
		}
	}

	std::vector<std::string> stringSplit(std::string s, const std::string& delimiter)
	{
		std::size_t pos = 0;
		std::vector<std::string> split;
		for (; (pos = s.find(delimiter)) != std::string::npos; s.erase(0, pos + delimiter.length()))
			split.emplace_back(s.substr(0, pos));
		split.emplace_back(s);
		return split;
	}

	void applyVariables(std::string& line, const std::unordered_map<std::string, std::string>& variables)
	{
		for (const std::pair<std::string, std::string>& e : variables)
			stringReplace(line, e.first, e.second);
	}

	std::string removePathQuotes(const std::string& s)
	{
		return s[0] == '"' && s[s.length() - 1] == '"' ? s.substr(1, s.length() - 2) : s;
	}

	// taken from https://stackoverflow.com/questions/478898#478960
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
		}
		catch (...) {
			_pclose(pipe);
			throw;
		}
		_pclose(pipe);
		return result;
	}

	double getAudioDuration(const std::string& filepath)
	{
		return std::stod(exec("ffprobe -v quiet -show_entries format=duration -of default=noprint_wrappers=1:nokey=1 \"" + filepath + "\""));
	}

	cv::Mat convertImage(cv::Mat image)
	{
		int depth = image.depth();
		cv::Mat converted;
		cv::cvtColor(image, converted, cv::COLOR_BGR2BGRA);
		double scale = depth == CV_16U ? 1.0 / 257 : 1;
		converted.convertTo(image, CV_32F, scale);
		return image;
	}

	cv::Mat readImageFile(const std::string& filepath)
	{
		return convertImage(cv::imread(filepath, cv::IMREAD_UNCHANGED));
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
	double InQuad(double t) { return t * t; }
	double OutQuad(double t) { return Reverse(InQuad, t); }
	double InOutQuad(double t) { return ToInOut(InQuad, t); }
	double InCubic(double t) { return t * t * t; }
	double OutCubic(double t) { return Reverse(InCubic, t); }
	double InOutCubic(double t) { return ToInOut(InCubic, t); }
	double InQuart(double t) { return t * t * t * t; }
	double OutQuart(double t) { return Reverse(InQuart, t); }
	double InOutQuart(double t) { return ToInOut(InQuart, t); }
	double InQuint(double t) { return t * t * t * t * t; }
	double OutQuint(double t) { return Reverse(InQuint, t); }
	double InOutQuint(double t) { return ToInOut(InQuint, t); }
	double InSine(double t) { return 1 - std::cos(t * PI / 2); }
	double OutSine(double t) { return Reverse(InSine, t); }
	double InOutSine(double t) { return ToInOut(InSine, t); }
	double InExpo(double t) { return std::pow(2, 10 * (t - 1)); }
	double OutExpo(double t) { return Reverse(InExpo, t); }
	double InOutExpo(double t) { return ToInOut(InExpo, t); }
	double InCirc(double t) { return 1 - std::sqrt(1 - t * t); }
	double OutCirc(double t) { return Reverse(InCirc, t); }
	double InOutCirc(double t) { return ToInOut(InCirc, t); }
	double InBack(double t) { return t * t * ((1.70158 + 1) * t - 1.70158); }
	double OutBack(double t) { return Reverse(InBack, t); }
	double InOutBack(double t) { return ToInOut([](double y) { return y * y * ((1.70158 * 1.525 + 1) * y - 1.70158 * 1.525); }, t); }
	double OutBounce(double t) { return t < 1 / 2.75 ? 7.5625 * t * t : t < 2 / 2.75 ? 7.5625 * (t -= (1.5 / 2.75)) * t + .75 : t < 2.5 / 2.75 ? 7.5625 * (t -= (2.25 / 2.75)) * t + .9375 : 7.5625 * (t -= (2.625 / 2.75)) * t + .984375; }
	double InBounce(double t) { return Reverse(OutBounce, t); }
	double InOutBounce(double t) { return ToInOut(InBounce, t); }
	double OutElastic(double t) { return std::pow(2, -10 * t) * std::sin((t - 0.075) * (2 * t) / .3) + 1; }
	double InElastic(double t) { return Reverse(OutElastic, t); }
	double OutElasticHalf(double t) { return std::pow(2, -10 * t) * std::sin((0.5 * t - 0.075) * (2 * PI) / .3) + 1; }
	double OutElasticQuarter(double t) { return std::pow(2, -10 * t) * std::sin((0.25 * t - 0.075) * (2 * PI) / .3) + 1; }
	double InOutElastic(double t) { return ToInOut(InElastic, t); }

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
		double R;
		double G;
		double B;
	};

	template <typename T>
	T InterpolateLinear(T start, T end, double t)
	{
		return start + (end - start) * t;
	}
	Colour InterpolateLinear(Colour start, Colour end, double t)
	{
		return Colour(InterpolateLinear(start.R, end.R, t), InterpolateLinear(start.G, end.G, t), InterpolateLinear(start.B, end.B, t));
	}
	std::pair<double, double> InterpolateLinear(std::pair<double, double> start, std::pair<double, double> end, double t)
	{
		return std::pair<double, double>(InterpolateLinear(start.first, end.first, t), InterpolateLinear(start.second, end.second, t));
	}
	template <typename T>
	T InterpolateBilinear(T topLeft, T topRight, T bottomLeft, T bottomRight, double tx, double ty)
	{
		return InterpolateLinear(
			InterpolateLinear(topLeft, topRight, tx),
			InterpolateLinear(bottomLeft, bottomRight, tx),
			ty
		);
	}
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
		double t = (time - keyframe.actualStarttime) / (endKeyframe.time - keyframe.actualStarttime);
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
	R generateKeyframesForEvent(const std::vector<std::unique_ptr<IEvent>>& events, std::pair<double, double> coordinates)
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
					if constexpr (T == EventType::M)
						XKeyframes.push_back(Keyframe<double>(-std::numeric_limits<double>::infinity(), coordinates.first, Easing::Step, -std::numeric_limits<double>::infinity()));
					else
						XKeyframes.push_back(Keyframe<double>(-std::numeric_limits<double>::infinity(), 1, Easing::Step, -std::numeric_limits<double>::infinity()));

				else
					generateKeyframes<double, double>(XKeyframes, applicableEventsX, nop<double>());
				if (applicableEventsY.size() == 0)
					if constexpr (T == EventType::M)
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
			auto ev = std::vector<std::unique_ptr<Event<Colour>>>();
			for (const auto& e : applicableEvents)
			{
				ev.push_back(std::make_unique<Event<Colour>>(*dynamic_cast<Event<Colour>*>(e->copy().get())));
			}
			generateKeyframes<Colour, Colour>(keyframes, ev, nop<Colour>());
			return keyframes;
		}
		else if constexpr (T == EventType::P)
		{
			auto ev = std::vector<std::unique_ptr<Event<ParameterType>>>();
			for (const auto& e : applicableEvents)
			{
				ev.push_back(std::make_unique<Event<ParameterType>>(*dynamic_cast<Event<ParameterType>*>(e->copy().get())));
			}
			generateParameterKeyframes(keyframes, ev);
			return keyframes;
		}
		else
		{
			auto ev = std::vector<std::unique_ptr<Event<double>>>();
			for (const auto& e : applicableEvents)
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
			endtime = starttime + (looplength)*loopcount;
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
		Sprite(Layer layer, Origin origin, const std::string& filepath, const std::pair<double, double>& coordinates)
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
			for (Loop& loop : loops) loop.Initialise();
			for (Trigger& trigger : triggers) trigger.Initialise();
			for (Loop& loop : loops)
				for (std::unique_ptr<IEvent>& event : loop.GetEvents())
					events.emplace_back(std::move(event));
			std::sort(events.begin(), events.end(), [](const std::unique_ptr<IEvent>& a, const std::unique_ptr<IEvent>& b) { return a->GetStartTime() < b->GetStartTime(); });
			double endTime = std::numeric_limits<double>::min();
			for (const std::unique_ptr<IEvent>& event : events)
				endTime = std::max(endTime, event->GetEndTime());
			activetime = std::pair<double, double>({ events.size() > 0 ? (*events.begin())->GetStartTime() : std::numeric_limits<double>::max(), endTime });

			positionKeyframes = generateKeyframesForEvent<EventType::M, std::pair<std::vector<Keyframe<double>>, std::vector<Keyframe<double>>>>(events, coordinates);
			rotationKeyframes = generateKeyframesForEvent<EventType::R, std::vector<Keyframe<double>>>(events, coordinates);
			scaleKeyframes = generateKeyframesForEvent<EventType::S, std::pair<std::vector<Keyframe<double>>, std::vector<Keyframe<double>>>>(events, coordinates);
			colourKeyframes = generateKeyframesForEvent<EventType::C, std::vector<Keyframe<Colour>>>(events, coordinates);
			opacityKeyframes = generateKeyframesForEvent<EventType::F, std::vector<Keyframe<double>>>(events, coordinates);
			flipHKeyframes = generateKeyframesForEvent<EventType::P, std::vector<Keyframe<bool>>, ParameterType::FlipH>(events, coordinates);
			flipVKeyframes = generateKeyframesForEvent<EventType::P, std::vector<Keyframe<bool>>, ParameterType::FlipV>(events, coordinates);
			additiveKeyframes = generateKeyframesForEvent<EventType::P, std::vector<Keyframe<bool>>>(events, coordinates);
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

	class Storyboard
	{
	public:
		Storyboard(const std::filesystem::path& directory, const std::filesystem::path& osb, std::vector<std::unique_ptr<Sprite>>& sprites, std::vector<Sample>& samples, Background background, Video video, const std::unordered_map<std::string, std::string>& info, std::pair<std::size_t, std::size_t> resolution, double musicVolume, double effectVolume)
			:
			directory(directory),
			osb(osb),
			samples(samples),
			video(video),
			info(info),
			resolution(resolution),
			musicVolume(musicVolume),
			effectVolume(effectVolume),
			frameScale(resolution.second / 480.0),
			xOffset((resolution.first - resolution.second * 4 / 3.0) * 0.5)
		{
			this->sprites = std::move(sprites);
			std::cout << "Initialising storyboard (" << this->sprites.size() << " sprites, " << samples.size() << " samples)" << "\n";
			for (std::unique_ptr<Sprite>& sprite : this->sprites)
				sprite->Initialise();
			std::pair<double, double> activetime = { std::numeric_limits<int>::max(), std::numeric_limits<int>::min() };

			bool backgroundIsASprite = false;
			for (const std::unique_ptr<Sprite>& sprite : this->sprites)
			{
				std::pair<double, double> at = sprite->GetActiveTime();
				activetime.first = std::min(activetime.first, at.first);
				activetime.second = std::max(activetime.second, at.second);
				if (background.exists)
					backgroundIsASprite = std::max(background.filepath == sprite->GetFilePath(0), backgroundIsASprite);
			}
			this->activetime = activetime;
			auto k = info.find("AudioFilename");
			if (k != info.end()) this->audioDuration = 1000 * getAudioDuration((directory / k->second).generic_string());
			else this->audioDuration = 0;
			auto l = info.find("AudioLeadIn");
			if (k != info.end() && l != info.end()) this->audioLeadIn = std::stoi(l->second);
			else this->audioLeadIn = 0;
			std::cout << "Initialised " << this->sprites.size() << " sprites/animations\n";

			blankImage = cv::Mat::zeros(resolution.second, resolution.first, CV_8UC3);
			backgroundImage = cv::Mat::zeros(resolution.second, resolution.first, CV_8UC3);
			if (background.exists && !backgroundIsASprite)
			{
				cv::Mat image = readImageFile((directory / background.filepath).generic_string());
				cv::RotatedRect quadRect = cv::RotatedRect(
					cv::Point2f(
						resolution.first / 2.0f + background.offset.first * frameScale,
						resolution.second / 2.0f + background.offset.second * frameScale
					),
					cv::Size2f(resolution.second * image.cols / (double)image.rows, resolution.second),
					0
				);
				cv::Point2f quad[4];
				quadRect.points(quad);
				RasteriseQuad(backgroundImage.begin<cv::Vec<uint8_t, 3>>(), image.begin<cv::Vec<float, 4>>(), image.cols, image.rows, quad, Colour(1, 1, 1), false, 1);
			}

			if (video.exists && !(videoOpen = videoCap.open((directory / video.filepath).generic_string()))) videoCap.release();

			for (const std::unique_ptr<Sprite>& sprite : this->sprites)
			{
				std::vector<std::string> filePaths = sprite->GetFilePaths();
				for (std::string filePath : filePaths)
				{
					if (spriteImages.find(filePath) != spriteImages.end()) continue;
					cv::Mat image = readImageFile((directory / filePath).generic_string());
					spriteImages.emplace(filePath, image);
				}
			}
		}
		std::pair<double, double> GetActiveTime() const
		{
			return activetime;
		}
		double GetAudioDuration() const
		{
			return audioDuration;
		}
		double GetAudioLeadIn() const
		{
			return audioLeadIn;
		}
		void generateAudio(const std::string& outFile) const
		{
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
			for (const Sample& sample : samples)
			{
				command += " -i \"" + (directory / sample.filepath).generic_string() + "\"";
				int delay = (int)sample.starttime;
				delays.push_back(delay);
				volumes.push_back(sample.volume / 100.0 * (samples.size() + 1) * effectVolume);
				maxDelay = std::max(maxDelay, delay);
				maxDuration = std::max(maxDuration, getAudioDuration((directory / sample.filepath).generic_string()));
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
			cv::Mat frame = video.exists ? GetVideoImage(time) : backgroundImage.clone();
			cv::MatIterator_<cv::Vec<uint8_t, 3>> frameStart = frame.begin<cv::Vec<cv::uint8_t, 3>>();
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
				const cv::Mat image = k->second;
				const cv::MatConstIterator_<cv::Vec<float, 4>> imageStart = image.begin<cv::Vec<float, 4>>();

				scale = std::pair<double, double>(scale.first * frameScale, scale.second * frameScale);
				std::pair<double, double> newSize = std::pair<double, double>(image.cols * std::abs(scale.first), image.rows * std::abs(scale.second));

				if (newSize.first < 1 || newSize.second < 1) continue;

				// create frame-space quad
				float width = newSize.first;
				float height = newSize.second;
				std::pair<double, double> position = sprite->PositionAt(time);
				cv::Vec2f positionVec = { (float)position.first, (float)position.second };
				cv::Vec2f origin = GetOriginVector(sprite->GetOrigin(), width, height);
				cv::Vec2f centre = GetOriginVector(Origin::Centre, width, height);
				float rotation = sprite->RotationAt(time);
				if (rotation != 0)
				{
					centre -= origin;
					centre =
					{
						centre[0] * std::cos(rotation) - centre[1] * std::sin(rotation),
						centre[1] * std::cos(rotation) + centre[0] * std::sin(rotation)
					};
					centre += origin;
				}
				cv::Vec2f frameQuadCentrePoint = positionVec * frameScale
					+ centre - origin
					+ cv::Vec2f(xOffset, 0);
				cv::RotatedRect quadRect = cv::RotatedRect(frameQuadCentrePoint, cv::Size2f(width, height), rotation != 0 ? rotation * 180.0f / PI : 0);
				cv::Point2f quad[4]; // bottomLeft, topLeft, topRight, bottomRight
				quadRect.points(quad);

				// flip quad if needed
				bool flipH = sprite->EffectAt(time, ParameterType::FlipH) || scale.first < 0;
				bool flipV = sprite->EffectAt(time, ParameterType::FlipV) || scale.second < 0;
				if (flipH && flipV)
				{
					std::swap(quad[0], quad[2]);
					std::swap(quad[1], quad[3]);
				}
				else if (flipH)
				{
					std::swap(quad[0], quad[3]);
					std::swap(quad[1], quad[2]);
				}
				else if (flipV)
				{
					std::swap(quad[0], quad[1]);
					std::swap(quad[2], quad[3]);
				}

				Colour colour = sprite->ColourAt(time);
				bool additive = sprite->EffectAt(time, ParameterType::Additive);

				RasteriseQuad(frameStart, imageStart, image.cols, image.rows, quad, colour, additive, alpha);
			}
			return frame;
		}
	private:
		cv::Mat GetVideoImage(double time)
		{
			int offset = 500;
			int crossfadeDuration = 1000;
			cv::Mat frame = time + offset >= video.starttime && time < video.starttime ? backgroundImage.clone() : blankImage.clone();
			cv::Mat image;
#pragma omp critical
			if (videoOpen)
			{
				// TODO: fix frames rarely coming in incorrect order
				videoCap.set(cv::VideoCaptureProperties::CAP_PROP_POS_MSEC, time - video.starttime);
				videoCap.read(image);
			}
			if (image.empty()) return frame;
			else if (time + offset < video.starttime) return backgroundImage.clone();
			image = convertImage(image);
			cv::RotatedRect quadRect = cv::RotatedRect(
				cv::Point2f(
					resolution.first / 2.0f + video.offset.first * frameScale,
					resolution.second / 2.0f + video.offset.second * frameScale
				),
				cv::Size2f(resolution.second * image.cols / (double)image.rows, resolution.second),
				0
			);
			cv::Point2f quad[4];
			quadRect.points(quad);
			double alpha = 1;
			if (time + offset + 100 >= video.starttime && time < video.starttime)
			{
				alpha = std::clamp(InterpolateLinear(1.0, 0.0, (video.starttime - time) / crossfadeDuration), 0.0, 1.0);
				cv::Mat p = cv::Mat::zeros(1, 1, CV_32FC3);
				cv::Mat pixel;
				cv::cvtColor(p, pixel, cv::COLOR_BGR2BGRA);
				RasteriseQuad(frame.begin<cv::Vec<uint8_t, 3>>(), pixel.begin<cv::Vec<float, 4>>(), 1, 1, quad, Colour(1, 1, 1), false, 1 - alpha);
			}
			RasteriseQuad(frame.begin<cv::Vec<uint8_t, 3>>(), image.begin<cv::Vec<float, 4>>(), image.cols, image.rows, quad, Colour(1, 1, 1), false, alpha);
			return frame;
		}
		void RasteriseQuad(cv::MatIterator_<cv::Vec<uint8_t, 3>> frameStart, cv::MatConstIterator_<cv::Vec<float, 4>> imageStart, int imageWidth, int imageHeight, cv::Point2f quad[4], Colour colour, bool additive, double alpha) const
		{
			alpha /= 255.0;
			std::vector<std::pair<int, int>> ContourX; int y;
			ContourX.reserve(resolution.second);

			for (y = 0; y < resolution.second; y++)
			{
				ContourX.push_back({ std::numeric_limits<int>::max(), std::numeric_limits<int>::min() });
			}

			ScanLine(quad[0].x, quad[0].y, quad[1].x, quad[1].y, ContourX);
			ScanLine(quad[1].x, quad[1].y, quad[2].x, quad[2].y, ContourX);
			ScanLine(quad[2].x, quad[2].y, quad[3].x, quad[3].y, ContourX);
			ScanLine(quad[3].x, quad[3].y, quad[0].x, quad[0].y, ContourX);

			float imageX = 0;
			float imageY = 0;

			float minY = std::numeric_limits<float>::max();
			float maxY = std::numeric_limits<float>::min();
			for (int i = 0; i < 4; i++)
			{
				minY = std::min(quad[i].y, minY);
				maxY = std::max(quad[i].y, maxY);
			}
			minY = std::max(minY, 0.0f);
			maxY = std::min(maxY, (float)resolution.second - 1);

			for (y = (int)minY; y <= (int)maxY; y++)
			{
				if (ContourX[y].second >= ContourX[y].first)
				{
					int x = ContourX[y].first;
					int len = 1 + ContourX[y].second - ContourX[y].first;

					while (len--)
					{
						// image-space coords
						float u = (-(x - quad[0].x) * (quad[1].y - quad[0].y) + (y - quad[0].y) * (quad[1].x - quad[0].x))
							/ (-(quad[2].x - quad[0].x) * (quad[1].y - quad[0].y) + (quad[2].y - quad[0].y) * (quad[1].x - quad[0].x));
						float v = (-(x - quad[1].x) * (quad[2].y - quad[1].y) + (y - quad[1].y) * (quad[2].x - quad[1].x))
							/ (-(quad[0].x - quad[1].x) * (quad[2].y - quad[1].y) + (quad[0].y - quad[1].y) * (quad[2].x - quad[1].x));

						// sample colour with bilinear interpolation
						Colour imageColour;
						float imageAlpha;
						SampleColourAndAlpha(imageStart, imageWidth, imageHeight, u, v, imageColour, imageAlpha);

						cv::Vec<uint8_t, 3> framePixel = GetPixel(frameStart, resolution.first, x, y);

						float newAlpha = imageAlpha * alpha;

						// additive (linear dodge) or normal blend mode
						cv::Vec<uint8_t, 3> newPixel;
						if (additive)
						{
							newPixel = cv::Vec<uint8_t, 3>(
								cv::saturate_cast<uint8_t>(framePixel[0] + newAlpha * imageColour[2] * colour[2]),
								cv::saturate_cast<uint8_t>(framePixel[1] + newAlpha * imageColour[1] * colour[1]),
								cv::saturate_cast<uint8_t>(framePixel[2] + newAlpha * imageColour[0] * colour[0])
								);
						}
						else
						{
							newPixel = cv::Vec<uint8_t, 3>(
								cv::saturate_cast<uint8_t>((1 - newAlpha) * framePixel[0] + newAlpha * imageColour[2] * colour[2]),
								cv::saturate_cast<uint8_t>((1 - newAlpha) * framePixel[1] + newAlpha * imageColour[1] * colour[1]),
								cv::saturate_cast<uint8_t>((1 - newAlpha) * framePixel[2] + newAlpha * imageColour[0] * colour[0])
								);
						}

						SetPixel(frameStart, resolution.first, x++, y, newPixel);
					}
				}
			}
		}
		void SetPixel(cv::MatIterator_<cv::Vec<uint8_t, 3>> imageStart, int width, const int x, int y, cv::Vec<uint8_t, 3> pixel) const
		{
			*(imageStart + y * width + x) = pixel;
		}
		cv::Vec<uint8_t, 3> GetPixel(const cv::MatIterator_<cv::Vec<uint8_t, 3>> imageStart, int width, int x, int y) const
		{
			return *(imageStart + y * width + x);
		}
		void SampleColourAndAlpha(cv::MatConstIterator_<cv::Vec<float, 4>> imageStart, int width, int height, float u, float v, Colour& outputColour, float& outputAlpha) const
		{
			// i sure hope i'm doing this correctly
			float x = u * width;
			float y = v * height;
			if (y < 0 || x < 0 || x >= width || y >= height)
			{
				outputColour = Colour(0, 0, 0);
				outputAlpha = 0;
				return;
			}
			auto it = imageStart + (int)y * width + (int)x;

			float dx = x - (int)x;
			float dy = y - (int)y;
			bool onRightEdge = (int)x == width - 1;
			bool onLastRow = (int)y == height - 1;
			auto right = onRightEdge ? it : it + 1;
			auto down = onLastRow ? it : it + width;
			auto downright = onRightEdge ? onLastRow ? it : it + width : onLastRow ? it + 1 : it + width + 1;

			// interpolate nearest neighbour
			// cv::Vec<float, 4> sample = dx < 0.5f ? dy < 0.5f ? *it : *down : dy < 0.5f ? *right : *downright;

			cv::Vec<float, 4> sample = InterpolateBilinear(*it, *right, *down, *downright, dx, dy);

			outputColour = Colour(sample[2], sample[1], sample[0]);
			outputAlpha = sample[3];
		}
		cv::Vec2f GetOriginVector(const Origin origin, const int width, const int height) const
		{
			switch (origin)
			{
			case Origin::TopLeft: return cv::Vec2f(0, 0);
			case Origin::TopCentre: return cv::Vec2f(width * 0.5f, 0);
			case Origin::TopRight: return cv::Vec2f(width, 0);
			case Origin::CentreLeft: return cv::Vec2f(0, height * 0.5f);
			case Origin::Centre: return cv::Vec2f(width * 0.5f, height * 0.5f);
			case Origin::CentreRight: return cv::Vec2f(width, height * 0.5f);
			case Origin::BottomLeft: return cv::Vec2f(0, height);
			case Origin::BottomCentre: return cv::Vec2f(width * 0.5f, height);
			case Origin::BottomRight: return cv::Vec2f(width, height);
			default: return cv::Vec2f(width * 0.5f, height * 0.5f);
			}
		}
		// i couldn't be bothered to write my own rasterizer so this next function is adapted from
		// https://stackoverflow.com/questions/7870533/c-triangle-rasterization#7870925
		// this algorithm can be used for any convex polygon apparently. i'm using quads for the sprites
		void ScanLine(int x1, int y1, int x2, int y2, std::vector<std::pair<int, int>>& ContourX) const
		{
			int sx, sy, dx1, dy1, dx2, dy2, x, y, m, n, k, cnt;

			sx = x2 - x1;
			sy = y2 - y1;

			if (sx > 0) dx1 = 1;
			else if (sx < 0) dx1 = -1;
			else dx1 = 0;

			if (sy > 0) dy1 = 1;
			else if (sy < 0) dy1 = -1;
			else dy1 = 0;

			m = std::abs(sx);
			n = std::abs(sy);
			dx2 = dx1;
			dy2 = 0;

			if (m < n)
			{
				m = std::abs(sy);
				n = std::abs(sx);
				dx2 = 0;
				dy2 = dy1;
			}

			x = x1; y = y1;
			cnt = m + 1;
			k = n / 2;

			while (cnt--)
			{
				if ((y >= 0) && (y < resolution.second))
				{
					if (x < ContourX[y].first) ContourX[y].first = std::max(x, 0);
					if (x > ContourX[y].second) ContourX[y].second = std::min(x + 1, (int)resolution.first - 1);
				}

				k += n;
				if (k < m)
				{
					x += dx2;
					y += dy2;
				}
				else
				{
					k -= m;
					x += dx1;
					y += dy1;
				}
			}
		}
	private:
		std::filesystem::path directory;
		std::filesystem::path osb;
		std::vector<std::unique_ptr<Sprite>> sprites;
		std::vector<Sample> samples;
		std::unordered_map<std::string, std::string> info;
		std::pair<double, double> activetime;
		const std::pair<std::size_t, std::size_t> resolution;
		double musicVolume;
		double effectVolume;
		double audioDuration;
		double audioLeadIn;
		std::unordered_map<std::string, cv::Mat> spriteImages;
		cv::Mat blankImage;
		cv::Mat backgroundImage;
		Video video;
		cv::VideoCapture videoCap;
		cv::Mat videoImage;
		bool videoOpen = false;
		double lastFrame;
		double frameScale;
		double xOffset;
	};

	// written mostly in reference to the parser used in osu!lazer (https://github.com/ppy/osu/blob/master/osu.Game/Beatmaps/Formats/LegacyStoryboardDecoder.cs)
	void parseFile(std::ifstream& file, std::vector<std::unique_ptr<Sprite>>& sprites, std::vector<Sample>& samples, Background& background, Video& video, std::unordered_map<std::string, std::string>& variables, std::unordered_map<std::string, std::string>& info, size_t& lineNumber)
	{
		std::string line;
		bool inLoop = false;
		bool inTrigger = false;
		bool hasBackground = false;
		bool hasVideo = false;
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

			// Determine start of a new section
			if (line.find("[Events]") == 0)
			{
				section = Section::Events;
				continue;
			}
			else if (line.find("[Variables]") == 0)
			{
				section = Section::Variables;
				continue;
			}
			else if (line.find("[General]") == 0 || line.find("[Metadata]") == 0)
			{
				section = Section::Info;
				continue;
			}
			else if (line[0] == '[')
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
					double starttime = std::stod(split[2]);
					double endTime = std::stod(split[3]);
					int groupNumber = split.size() > 4 ? std::stoi(split[4]) : 0;
					(*(sprites.end() - 1))->AddTrigger({ triggerName, starttime, endTime, groupNumber });
					inTrigger = true;
				}
				break;
				case Keyword::L:
				{
					if (inLoop || inTrigger)
					{
						// TODO: Error
					}
					double starttime = std::stod(split[1]);
					int loopCount = std::stoi(split[2]);
					(*(sprites.end() - 1))->AddLoop({ starttime, loopCount });
					inLoop = true;
				}
				break;
				case Keyword::None:
				default:
				{
					if (split[0] == "0" && split[1] == "0" && depth == 0 && !hasBackground)
					{
						std::string path = removePathQuotes(split[2]);
						std::pair<double, double> offset = split.size() < 3 ? std::pair<double, double>(std::stoi(split[3]), std::stoi(split[4])) : std::pair<double, double>(0, 0);
						background = Background(path, offset);
						hasBackground = true;
						break;
					}
					if (((split[0] == "Video" || split[0] == "1") && !hasVideo) && depth == 0)
					{
						double starttime = std::stod(split[1]);
						std::string path = removePathQuotes(split[2]);
						std::pair<double, double> offset = split.size() < 3 ? std::pair<double, double>(std::stoi(split[3]), std::stoi(split[4])) : std::pair<double, double>(0, 0);
						video = Video(starttime, path, offset);
						hasVideo = true;
						break;
					}
					if (depth == 0) break;
					if (split[3].length() == 0)
						split[3] = split[2];

					Easing easing = static_cast<Easing>(std::stoi(split[1]));
					double starttime = std::stod(split[2]);
					double endTime = std::stod(split[3]);

					std::unordered_map<std::string, EventType>::const_iterator k = EventTypeStrings.find(split[0]);
					EventType eventType = k == EventTypeStrings.end() ? EventType::None : k->second;

					switch (eventType)
					{
					case EventType::F:
					{
						double startValue = std::stod(split[4]);
						double endValue = split.size() > 5 ? std::stod(split[5]) : startValue;
						std::unique_ptr<Event<double>> event = std::make_unique<Event<double>>(EventType::F, easing, starttime, endTime, startValue, endValue);
						if (inTrigger) (*(sprites.end() - 1))->AddEventInTrigger(std::move(event));
						else if (inLoop) (*(sprites.end() - 1))->AddEventInLoop(std::move(event));
						else (*(sprites.end() - 1))->AddEvent(std::move(event));
					}
					break;
					case EventType::S:
					{
						double startValue = std::stod(split[4]);
						double endValue = split.size() > 5 ? std::stod(split[5]) : startValue;
						std::unique_ptr<Event<double>> event = std::make_unique<Event<double>>(EventType::S, easing, starttime, endTime, startValue, endValue);
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
						std::unique_ptr<Event<std::pair<double, double>>> event = std::make_unique<Event<std::pair<double, double>>>(EventType::V, easing, starttime, endTime, std::pair<double, double> { startX, startY }, std::pair<double, double>{ endX, endY });
						if (inTrigger) (*(sprites.end() - 1))->AddEventInTrigger(std::move(event));
						else if (inLoop) (*(sprites.end() - 1))->AddEventInLoop(std::move(event));
						else (*(sprites.end() - 1))->AddEvent(std::move(event));
					}
					break;
					case EventType::R:
					{
						double startValue = std::stod(split[4]);
						double endValue = split.size() > 5 ? std::stod(split[5]) : startValue;
						std::unique_ptr<Event<double>> event = std::make_unique<Event<double>>(EventType::R, easing, starttime, endTime, startValue, endValue);
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
						std::unique_ptr<Event<std::pair<double, double>>> event = std::make_unique<Event<std::pair<double, double>>>(EventType::M, easing, starttime, endTime, std::pair<double, double> { startX, startY }, std::pair<double, double>{ endX, endY });
						if (inTrigger) (*(sprites.end() - 1))->AddEventInTrigger(std::move(event));
						else if (inLoop) (*(sprites.end() - 1))->AddEventInLoop(std::move(event));
						else (*(sprites.end() - 1))->AddEvent(std::move(event));
					}
					break;
					case EventType::MX:
					{
						double startValue = std::stod(split[4]);
						double endValue = split.size() > 5 ? std::stod(split[5]) : startValue;
						std::unique_ptr<Event<double>> event = std::make_unique<Event<double>>(EventType::MX, easing, starttime, endTime, startValue, endValue);
						if (inTrigger) (*(sprites.end() - 1))->AddEventInTrigger(std::move(event));
						else if (inLoop) (*(sprites.end() - 1))->AddEventInLoop(std::move(event));
						else (*(sprites.end() - 1))->AddEvent(std::move(event));
					}
					break;
					case EventType::MY:
					{
						double startValue = std::stod(split[4]);
						double endValue = split.size() > 5 ? std::stod(split[5]) : startValue;
						std::unique_ptr<Event<double>> event = std::make_unique<Event<double>>(EventType::MY, easing, starttime, endTime, startValue, endValue);
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
						std::unique_ptr<Event<Colour>> event = std::make_unique<Event<Colour>>(EventType::C, easing, starttime, endTime, Colour { startR / 255.0f, startG / 255.0f, startB / 255.0f }, Colour{ endR / 255.0f, endG / 255.0f, endB / 255.0f });
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
							event = std::make_unique<Event<ParameterType>>(EventType::P, easing, starttime, endTime, ParameterType::Additive, ParameterType::Additive);
							break;
						case ParameterType::FlipH:
							event = std::make_unique<Event<ParameterType>>(EventType::P, easing, starttime, endTime, ParameterType::FlipH, ParameterType::FlipH);
							break;
						case ParameterType::FlipV:
							event = std::make_unique<Event<ParameterType>>(EventType::P, easing, starttime, endTime, ParameterType::FlipV, ParameterType::FlipV);
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
				if (splitPos == std::string::npos || splitPos == line.length() - 1) continue;
				std::string key = line.substr(0, splitPos);
				std::string value = line.substr(splitPos + 1, line.length() - splitPos - 1);
				variables.emplace(key, value); // doesn't overwrite previous values TODO: Check if this is correct behaviour
			}
			break;

			case Section::Info:
			{
				std::size_t splitPos = line.find(':');
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

	std::unique_ptr<Storyboard> ParseStoryboard(const std::string& directory, const std::string& diff, std::pair<size_t, size_t> resolution, double musicVolume, double effectVolume)
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
		Background background;
		Video video;
		std::unordered_map<std::string, std::string> variables;
		std::unordered_map<std::string, std::string> info;
		bool inLoop = false;
		bool inTrigger = false;
		std::size_t lineNumber = 0;

		std::cout << "Parsing " << osb << "...\n";
		parseFile(osbFile, sprites, samples, background, video, variables, info, lineNumber);
		osbFile.close();
		std::cout << "Parsed " << lineNumber << " lines\n";

		lineNumber = 0;
		std::cout << "Parsing " << diff << "...\n";
		parseFile(diffFile, sprites, samples, background, video, variables, info, lineNumber);
		diffFile.close();
		std::cout << "Parsed " << lineNumber << " lines\n";

		std::unique_ptr<Storyboard> sb = std::make_unique<Storyboard>(directory, osb, sprites, samples, background, video, info, resolution, musicVolume, effectVolume);
		return sb;
	}
}

int main(int argc, char* argv[]) {
	if (argc <= 2) std::cout << "Specify directory, diff name and optionally start time and duration in milliseconds\n";
	std::string directory = argv[1];
	std::string diff = argv[2];
	int frameWidth = 1920;
	int frameHeight = 1080;
	int fps = 30;

	std::unique_ptr<sb::Storyboard> sb = sb::ParseStoryboard(directory, diff, { frameWidth, frameHeight }, 0.2, 0.2);
	std::pair<double, double> activetime = sb->GetActiveTime();
	double audioLeadIn = sb->GetAudioLeadIn();
	double audioDuration = sb->GetAudioDuration();
	double starttime = argc <= 3 ? std::min(activetime.first, audioLeadIn) : std::stod(argv[3]);
	double duration = argc <= 4 ? activetime.second < audioDuration + 60000 ? std::max(activetime.second, audioDuration) - starttime : audioDuration : std::stod(argv[4]);
	std::string outputFile = "video.mp4";
	int frameCount = (int)std::round(fps / (1000.0 / (float)duration));
	std::cout << "Rendering video...\n";
	cv::VideoWriter writer = cv::VideoWriter(
		"export.avi",
		cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
		fps,
		cv::Size(frameWidth, frameHeight)
	);
#pragma omp parallel for ordered schedule(dynamic)
	for (int i = 0; i < frameCount; i++)
	{
		std::cout << i + 1 << "/" << frameCount << '\r';
		cv::Mat frame = sb->DrawFrame(starttime + i * 1000.0 / fps);
#pragma omp ordered
		writer.write(frame);
	}
	writer.release();
	std::cout << "\nGenerating audio...\n";
	sb->generateAudio("audio.mp3");
	std::cout << "Merging audio and video...\n";
	std::stringstream command;
	command << "ffmpeg -y -v error -stats -i export.avi -ss " << starttime + sb->GetAudioLeadIn() << "ms -to "
		<< starttime + duration + sb->GetAudioLeadIn() << "ms -accurate_seek -i audio.mp3 -c:v copy " << outputFile;
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