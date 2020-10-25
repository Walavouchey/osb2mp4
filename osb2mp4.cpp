#include <iostream>
#include <fstream>
#include <string>
#include <cstddef>
#include <limits>
#include <unordered_map>
#include <vector>
#include <utility>
#include <memory>
#include <algorithm>
#include <cmath>

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
    for (; (pos = s.find(delimiter, pos)) != std::string::npos; s.erase(0, delimiter.length()))
        split.emplace_back(s.substr(0, pos));
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

namespace sb
{
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
        InOutBounce
    };

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
    struct Keyframe
    {
        public:
            double time;
            T value;
            Easing easing;
            double actualStarttime;
    };

    template <class T>
    T keyframeValueAt(const std::vector<Keyframe<T>> &keyframes, double time)
    {
        // TODO
        return keyframes[0].value;
    }

    class Color
    {
		public:
            Color(float R, float G, float B)
                :
                R(R),
                G(G),
                B(B)
            {}

			float R;
			float G;
			float B;
    };

    class IEvent
    {
        public:
            virtual EventType GetType() const = 0;
            virtual double GetStartTime() const = 0;
            virtual double GetEndTime() const = 0;
            virtual void SetStartTime(double) = 0;
            virtual void SetEndTime(double) = 0;
            //virtual typename T GetStartValue() const = 0;
            //virtual typename T GetEndValue() const = 0; // TODO: how do you do this shit
    };
    template <typename T>
    class Event : public IEvent
    {
        public:
            Event() = default;
            Event(EventType type, Easing easing, double starttime, double endttime, T startvalue, T endvalue)
                :
                type(type),
                easing(easing),
                starttime(starttime),
                endtime(endtime),
                startvalue(startvalue),
                endvalue(endvalue)
            {}
            /*
            T ValueAt(double time)
            {
                // TODO
            }
            */
            EventType GetType() const
            {
                return type;
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

    class Loop
    {
        public:
            Loop(double starttime, int loopcount)
                :
                starttime(starttime),
                loopcount(loopcount)
            {}
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
                        expandedEvents.push_back(std::move(event));
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
            Sprite(Layer layer, Origin origin, const std::string &filepath, const std::pair<float, float> &coordinates)
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
                (loops.end() - 1)->AddEvent(std::move(event));
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
                std::vector<std::unique_ptr<IEvent>> expandedEvents;
                for (Loop &loop : loops)
                    for (std::unique_ptr<IEvent> &event : loop.GetEvents())
                        events.push_back(std::move(event));
                std::sort(events.begin(), events.end(), [](const std::unique_ptr<IEvent> &a, const std::unique_ptr<IEvent> &b) { return a->GetStartTime() < b->GetStartTime(); });
                double endTime = std::numeric_limits<double>::min();
                for (const std::unique_ptr<IEvent> &event : events)
                    endTime = std::max(endTime, event->GetEndTime());
                activetime = std::pair<double, double>({events.size() > 0 ? (*events.begin())->GetStartTime() : std::numeric_limits<double>::max(), endTime});
            }
            std::pair<float, float> PositionAt(double time) const
            {
                return keyframeValueAt<std::pair<float, float>>(positionKeyframes, time);
            }
            float RotationAt(double time) const
            {
                return keyframeValueAt<float>(rotationKeyframes, time);
            }
            float ScaleAt(double time) const
            {
                return keyframeValueAt<float>(scaleKeyframes, time);
            }
            Color ColorAt(double time) const
            {
                return keyframeValueAt<Color>(colorKeyframes, time);
            }
            float OpacityAt(double time) const
            {
                return keyframeValueAt<float>(opacityKeyframes, time);
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
            const std::string& GetFilePath() const
            {
                return filepath;
            }
            const std::pair<float, float>& GetCoordinates() const
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
        private:
            std::vector<std::unique_ptr<IEvent>> events;
            std::vector<std::unique_ptr<IEvent>> expandedEvents;
            bool initialised = false;
            const Layer layer;
            const Origin origin;
            const std::string filepath;
            const std::pair<float, float> coordinates;
            std::vector<Keyframe<std::pair<float, float>>> positionKeyframes;
            std::vector<Keyframe<float>> rotationKeyframes;
            std::vector<Keyframe<float>> scaleKeyframes;
            std::vector<Keyframe<Color>> colorKeyframes;
            std::vector<Keyframe<float>> opacityKeyframes;
            std::vector<Keyframe<bool>> flipVKeyframes;
            std::vector<Keyframe<bool>> flipHKeyframes;
            std::vector<Keyframe<bool>> additiveKeyframes;
    };

    class Animation : public Sprite
    {
        public:
            Animation(Layer layer, Origin origin, const std::string &filepath, const std::pair<float, float> &coordinates, int framecount, double framedelay, LoopType looptype)
                :
                Sprite(layer, origin, filepath, coordinates),
                framecount(framecount),
                framedelay(framedelay),
                looptype(looptype)
            {}
            int frameIndexAt(double time)
            {
                if (time - activetime.first < framecount * framedelay || looptype == LoopType::LoopForever)
                {
                    return static_cast<int>(std::fmod(((time - activetime.first) / framedelay), (double) framecount)); 
                }
                else return framecount - 1;
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
        private:
            const double starttime;
            const Layer layer;
            const std::string filepath;
            const float volume;
    };

    class Storyboard
    {
        public:
            Storyboard(std::vector<std::unique_ptr<class Sprite>> &sprites, std::vector<class Sample> &samples,/* Background background, Video video, */ std::pair<std::size_t, std::size_t> resolution)
                :
                sprites(sprites),
                samples(samples),
                resolution(resolution)
            {
                for (std::unique_ptr<class Sprite>& sprite : sprites)
                    sprite->Initialise();
                std::cout << "Initialised " << sprites.size() << " sprites/animations\n";
            }
        private:
            std::vector<std::unique_ptr<class Sprite>> &sprites;
            std::vector<class Sample> samples;
            // const Background background;
            // const Video video;
            const std::pair<std::size_t, std::size_t> resolution;
    };

    std::unique_ptr<Storyboard> ParseStoryboard(const std::string &filePath, std::pair<size_t, size_t> resolution)
    {
        std::ifstream file(filePath);
        if (!file.is_open()) std::cout << "Failed to open " << filePath << '\n';
        std::vector<std::unique_ptr<class Sprite>> sprites;
        std::vector<class Sample> samples;
        std::unordered_map<std::string, std::string> variables;
        std::unordered_map<std::string, std::string> info;
        bool inLoop = false;
        bool inTrigger = false;
        std::size_t lineNumber = 0;
        std::string line;
        enum class Section
        {
            None,
            Events,
            Variables,
            Info
        };
        Section section = Section::None;

        std::cout << "Parsing " << filePath << '\n';
        while (file.good())
        {
            lineNumber++;
            std::cout << lineNumber << '\n';
            constexpr int lineMaxReadLength = 10000;
            char lineTemp[lineMaxReadLength];
            file.getline(lineTemp, lineMaxReadLength);
            line = std::string(lineTemp);

            std::cout << lineNumber << ' ' << line << '\n';
            if (line.length() == 0) continue;
            if (line.rfind("//", 0) == 0) continue;

            // Determine start of a new section.
            if (line.find("[Events]") != std::string::npos)
            {
                section = Section::Events;
            }
            else if (line.find("[Variables]") != std::string::npos)
            {
                section = Section::Variables;
            }
            else if (line.find("[General]") != std::string::npos || line.find("[Metadata]") != std::string::npos)
            {
                section = Section::Info;
            }
            else if (line.find("[") != std::string::npos)
            {
                section = Section::None;
            }

            std::cout << (int) section << '\n';
            switch (section)
            {
                case Section::None:
                    // All parts that won't be parsed are skipped here
                    continue;

                case Section::Events:
					{
                        std::cout << (int) section << " parsing \n";
						std::size_t depth = 0;
						while (std::isspace(line[depth])) depth++;

						applyVariables(line, variables);
						std::vector<std::string> split = stringSplit(line, ",");

						if (inTrigger && depth < 2) inTrigger = false;
						if (inLoop && depth < 2) inLoop = false;

						Keyword keyword = KeywordStrings.find(split[0])->second;
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
                            std::cout << line << '\n';
							sprites.push_back(std::make_unique<class Sprite>(layer, origin, path, std::pair<float, float>(x, y)));
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
							LoopType loopType = LoopTypeStrings.find(split[8])->second;
						    sprites.push_back(std::make_unique<class Animation>(layer, origin, path, std::pair<float, float>(x, y), frameCount, frameDelay, loopType));
						}
						break;
						case Keyword::Sample:
						{
							double time = std::stod(split[1]);
							Layer layer = LayerStrings.find(split[2])->second;
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
						default:
						{
							if (split[3].length() == 0)
								split[3] = split[2];

							Easing easing = static_cast<Easing>(std::stoi(split[1]));
							double startTime = std::stod(split[2]);
							double endTime = std::stod(split[3]);

						    EventType eventType = EventTypeStrings.find(split[0])->second;
							switch (eventType)
							{
								case EventType::F:
								{
									double startValue = std::stod(split[4]);
									double endValue = split.size() > 5 ? std::stod(split[5]) : startValue;
									std::unique_ptr<Event<double>> event = std::make_unique<Event<double>>(EventType::F, easing, startTime, endTime, startValue, endValue);
									if (inTrigger) (*(sprites.end() - 1))->AddEventInTrigger(std::move(event));
									if (inLoop) (*(sprites.end() - 1))->AddEventInLoop(std::move(event));
									(*(sprites.end() - 1))->AddEvent(std::move(event));
								}
								break;
								case EventType::S:
								{
									double startValue = std::stod(split[4]);
									double endValue = split.size() > 5 ? std::stod(split[5]) : startValue;
									std::unique_ptr<Event<double>> event = std::make_unique<Event<double>>(EventType::S, easing, startTime, endTime, startValue, endValue);
									if (inTrigger) (*(sprites.end() - 1))->AddEventInTrigger(std::move(event));
									if (inLoop) (*(sprites.end() - 1))->AddEventInLoop(std::move(event));
									(*(sprites.end() - 1))->AddEvent(std::move(event));
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
									if (inLoop) (*(sprites.end() - 1))->AddEventInLoop(std::move(event));
									(*(sprites.end() - 1))->AddEvent(std::move(event));
								}
								break;
								case EventType::R:
								{
									double startValue = std::stod(split[4]);
									double endValue = split.size() > 5 ? std::stod(split[5]) : startValue;
									std::unique_ptr<Event<double>> event = std::make_unique<Event<double>>(EventType::R, easing, startTime, endTime, startValue, endValue);
									if (inTrigger) (*(sprites.end() - 1))->AddEventInTrigger(std::move(event));
									if (inLoop) (*(sprites.end() - 1))->AddEventInLoop(std::move(event));
									(*(sprites.end() - 1))->AddEvent(std::move(event));
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
									if (inLoop) (*(sprites.end() - 1))->AddEventInLoop(std::move(event));
									(*(sprites.end() - 1))->AddEvent(std::move(event));
								}
								break;
								case EventType::MX:
								{
									double startValue = std::stod(split[4]);
									double endValue = split.size() > 5 ? std::stod(split[5]) : startValue;
									std::unique_ptr<Event<double>> event = std::make_unique<Event<double>>(EventType::MX, easing, startTime, endTime, startValue, endValue);
									if (inTrigger) (*(sprites.end() - 1))->AddEventInTrigger(std::move(event));
									if (inLoop) (*(sprites.end() - 1))->AddEventInLoop(std::move(event));
									(*(sprites.end() - 1))->AddEvent(std::move(event));
								}
								break;
								case EventType::MY:
								{
									double startValue = std::stod(split[4]);
									double endValue = split.size() > 5 ? std::stod(split[5]) : startValue;
									std::unique_ptr<Event<double>> event = std::make_unique<Event<double>>(EventType::MY, easing, startTime, endTime, startValue, endValue);
									if (inTrigger) (*(sprites.end() - 1))->AddEventInTrigger(std::move(event));
									if (inLoop) (*(sprites.end() - 1))->AddEventInLoop(std::move(event));
									(*(sprites.end() - 1))->AddEvent(std::move(event));
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
									if (inLoop) (*(sprites.end() - 1))->AddEventInLoop(std::move(event));
									(*(sprites.end() - 1))->AddEvent(std::move(event));
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
									if (inLoop) (*(sprites.end() - 1))->AddEventInLoop(std::move(event));
									(*(sprites.end() - 1))->AddEvent(std::move(event));
								}
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
						if (splitPos == std::string::npos || splitPos == line.length() - 2) continue;
						std::string key = line.substr(0, splitPos);
						std::string value = line.substr(splitPos + 1, line.length() - splitPos - 2);
						variables.emplace(key, value); // doesn't overwrite previous values TODO: Check if this is correct behaviour
					}
                    break;

                case Section::Info:
					{
						std::size_t splitPos = line.find(':');
						// Ignore invalid variables
						if (splitPos == std::string::npos || splitPos == line.length() - 2) continue;
						std::string key = line.substr(0, splitPos);
						std::string value = line.substr(splitPos + 1, line.length() - splitPos - 2);
						while (std::isspace(value[0])) value.erase(0); // Trim left
						info.emplace(key, value); // TODO: Check if this is correct behaviour
					}
                    break;
            }
        }
        file.close();
        std::cout << "Parsed " << lineNumber << " lines\n";
        std::unique_ptr<Storyboard> sb = std::make_unique<Storyboard>(sprites, samples, resolution);
        return sb;
    }
}

int main(int argc, char *argv[]) {
    if (argc == 1) std::cout << "Specify a directory.\n";
    std::string filePath = argv[1];
    std::unique_ptr<sb::Storyboard> sb = sb::ParseStoryboard(filePath, { 1920, 1080 });
    return 0;
}
