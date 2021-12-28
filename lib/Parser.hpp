#pragma once

#include <Components.hpp>
#include <Helpers.hpp>

#include <iostream>
#include <fstream>
#include <filesystem>
#include <string>
#include <unordered_map>
#include <vector>
#include <utility>
#include <memory>
#include <exception>

namespace sb
{
    // written mostly in reference to the parser used in osu!lazer (https://github.com/ppy/osu/blob/master/osu.Game/Beatmaps/Formats/LegacyStoryboardDecoder.cs)
    void parseFile(std::ifstream& file, size_t& lineNumber, std::unordered_map<std::string, std::string>& variables, std::vector<std::unique_ptr<Sprite>>& sprites, std::vector<Sample>& samples, std::vector<std::pair<double, HitSound>>& hitSounds, Background& background, Video& video, std::unordered_map<std::string, std::string>& info)
    {
        // check for utf-8 bom, which is present when exported through storybrew
        char buf[4];
        for (int i = 0; i < 3; i++)
            buf[i] = (file.seekg(i), file.peek());
        buf[3] = '\0';
        if (std::string(buf) == "\xEF\xBB\xBF") file.seekg(3);
        else file.seekg(0);

        std::string line;
        bool inLoop = false;
        bool inTrigger = false;
        bool hasBackground = false;
        bool hasVideo = false;
        struct ControlPoint
        {
            ControlPoint() = default;
            ControlPoint(double time, double beatLength)
                :
                time(time),
                beatLength(beatLength)
            {}
            double time = 0;
            double beatLength = 0;
            double sliderMultiplier = beatLength >= 0 ? 1.0 : 100.0 / -beatLength;
            int meter = 4;
            int sampleSet = 0;
            int sampleIndex = 0;
            int volume = 100;
            bool uninherited = 1;
            int effects = 0;
        };
        std::vector<ControlPoint> controlPoints;
        enum class Section
        {
            None,
            Events,
            Variables,
            Info,
            TimingPoints,
            HitObjects
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
            else if (line.find("[General]") == 0 || line.find("[Metadata]") == 0 || line.find("[Difficulty]") == 0)
            {
                section = Section::Info;
                continue;
            }
            else if (line.find("[TimingPoints]") == 0)
            {
                section = Section::TimingPoints;
                continue;
            }
            else if (line.find("[HitObjects]") == 0)
            {
                section = Section::HitObjects;
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

                Keyword keyword = parseEnum(KeywordStrings, split[0]).value_or(Keyword::None);
                switch (keyword)
                {
                case Keyword::Background:
                {
                    std::string path = removePathQuotes(split[2]);
                    std::pair<double, double> offset = split.size() < 3 ? std::pair<double, double>(std::stoi(split[3]), std::stoi(split[4])) : std::pair<double, double>(0, 0);
                    background = Background(path, offset);
                    hasBackground = true;
                }
                break;
                case Keyword::Video:
                {
                    double starttime = std::stod(split[1]);
                    std::string path = removePathQuotes(split[2]);
                    std::pair<double, double> offset = split.size() < 3 ? std::pair<double, double>(std::stoi(split[3]), std::stoi(split[4])) : std::pair<double, double>(0, 0);
                    video = Video(starttime, path, offset);
                    hasVideo = true;
                }
                break;
                case Keyword::Sprite:
                {
                    // TODO: Error handling
                    Layer layer = parseEnum(LayerStrings, split[1]).value_or(Layer::Background);
                    Origin origin = parseEnum(OriginStrings, split[2]).value_or(Origin::Centre);
                    std::string path = removePathQuotes(split[3]);
                    float x = std::stof(split[4]);
                    float y = std::stof(split[5]);
                    sprites.push_back(std::make_unique<Sprite>(layer, origin, path, std::pair<double, double>(x, y)));
                }
                break;
                case Keyword::Animation:
                {
                    Layer layer = parseEnum(LayerStrings, split[1]).value_or(Layer::Background);
                    Origin origin = parseEnum(OriginStrings, split[2]).value_or(Origin::Centre);
                    std::string path = removePathQuotes(split[3]);
                    float x = std::stof(split[4]);
                    float y = std::stof(split[5]);
                    int frameCount = std::stoi(split[6]);
                    double frameDelay = std::stod(split[7]);
                    LoopType loopType = parseEnum(LoopTypeStrings, split[8]).value_or(LoopType::LoopForever);
                    sprites.push_back(std::make_unique<class Animation>(layer, origin, path, std::pair<double, double>(x, y), frameCount, frameDelay, loopType));
                }
                break;
                case Keyword::Sample:
                {
                    double time = std::stod(split[1]);
                    Layer layer = parseEnum(LayerStrings, split[2]).value_or(Layer::Background);
                    std::string path = removePathQuotes(split[3]);
                    float volume = std::stof(split[4]);
                    samples.emplace_back(time, layer, path, volume);
                }
                break;
                default:
                {
                    if (split[0] == "T")
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
                        break;
                    }
                    if (split[0] == "L")
                    {
                        if (inLoop || inTrigger)
                        {
                            // TODO: Error
                        }
                        double starttime = std::stod(split[1]);
                        int loopCount = std::stoi(split[2]);
                        (*(sprites.end() - 1))->AddLoop({ starttime, loopCount });
                        inLoop = true;
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
                variables.emplace(key, value); // TODO: Error if already exists
            }
            break;

            case Section::Info:
            {
                std::size_t splitPos = line.find(':');
                if (splitPos == std::string::npos || splitPos == line.length() - 1) continue;
                std::string key = line.substr(0, splitPos);
                std::string value = line.substr(splitPos + 1, line.length() - splitPos - 1);
                while (std::isspace(static_cast<unsigned char>(value[0]))) value.erase(0, 1);
                info.insert_or_assign(key, value);
            }
            break;
            case Section::TimingPoints:
            {
                std::vector<std::string> split = stringSplit(line, ",");
                ControlPoint controlPoint = ControlPoint(std::stod(split[0]), std::stod(split[1]));
                if (split.size() > 2) controlPoint.meter = std::stoi(split[2]);
                if (split.size() > 3) controlPoint.sampleIndex = std::stoi(split[3]);
                if (split.size() > 4) controlPoint.sampleSet = std::stoi(split[4]);
                if (split.size() > 5) controlPoint.volume = std::stoi(split[5]);
                if (split.size() > 6) controlPoint.uninherited = std::stoi(split[6]) == 1;
                if (split.size() > 7) controlPoint.effects = std::stoi(split[7]);
                controlPoints.push_back(std::move(controlPoint));
            }
            break;
            case Section::HitObjects:
            {
                // all we care to obtain are pairs of timestamps and hitsound identifiers for resolving triggers
                std::vector<std::string> split = stringSplit(line, ",");
                if (split.size() > 3)
                {
                    int type = std::stoi(split[3]);
                    if (type & 1 && split.size() > 5) // hit circle
                    {
                        int time = std::stoi(split[2]);
                        std::vector<std::string> hitSample = stringSplit(split[5], ":");
                        int normalSet = hitSample.size() > 2 ? std::stoi(hitSample[0]) : 0;
                        int additionSet = hitSample.size() > 2 ? std::stoi(hitSample[1]) : 0;
                        int additionFlag = std::stoi(split[4]);
                        int index = hitSample.size() > 2 ? std::stoi(hitSample[2]) : 0;
                        hitSounds.emplace_back(std::pair<double, HitSound>{ time, HitSound(normalSet, additionSet, additionFlag, index) });
                    }
                    if (type & 2 && split.size() > 10) // slider
                    {
                        double time = std::stod(split[2]);
                        ControlPoint currentControlPoint;
                        ControlPoint currentTimingPoint;
                        for (const ControlPoint& controlPoint : controlPoints)
                        {
                            if (controlPoint.time >= time && !controlPoint.uninherited) break;
                            if (!controlPoint.uninherited) currentControlPoint = controlPoint;
                        }
                        for (const ControlPoint& controlPoint : controlPoints)
                        {
                            if (controlPoint.time >= time && controlPoint.uninherited) break;
                            if (controlPoint.uninherited) currentTimingPoint = controlPoint;
                        }
                        int slides = std::stoi(split[6]);
                        double length = std::stod(split[7]);
                        double beatmapSliderMultiplier = std::stod(info.find("SliderMultiplier")->second);
                        double travelDuration = currentTimingPoint.beatLength * length / beatmapSliderMultiplier / 100.0 / currentControlPoint.sliderMultiplier;
                        std::vector<std::string> hitSample = stringSplit(split[10], ":");
                        std::vector<std::string> edgeSounds = stringSplit(split[8], "|");
                        std::vector<std::string> edgeSets = stringSplit(split[9], "|");
                        for (int i = 0; i < slides + 1; i++)
                        {
                            int normalSet = edgeSets.size() > i ? std::stoi(stringSplit(edgeSets[i], ":")[0]) : 0;
                            int additionSet = edgeSets.size() > i ? std::stoi(stringSplit(edgeSets[i], ":")[1]) : 0;
                            int additionFlag = edgeSounds.size() > i ? stoi(edgeSounds[i]) : 0;
                            int index = 0;
                            hitSounds.emplace_back(std::pair<double, HitSound>{ time + travelDuration * i, HitSound(normalSet, additionSet, additionFlag, index) });
                        }
                    }
                }
            }
            break;
            }
        }
    }

    void ParseStoryboard(const std::filesystem::path& directory, const std::string& osb, const std::string& diff,
        std::vector<std::unique_ptr<Sprite>>& sprites,
        std::vector<Sample>& samples,
        std::vector<std::pair<double, HitSound>>& hitSounds,
        Background& background,
        Video& video,
        std::unordered_map<std::string, std::string>& info
    )
    {
        std::ifstream osbFile(osb);
        if (!osbFile.is_open()) throw std::exception(("Failed to open .osb file \"" + osb + "\"").c_str());
        std::ifstream diffFile(std::filesystem::path(directory) / diff);
        if (!diffFile.is_open()) throw std::exception(("Failed to open .osu file \"" + diff + "\"").c_str());

        std::size_t lineNumber = 0;
        std::unordered_map<std::string, std::string> variables;

        std::cout << "Parsing " << diff << "...\n";
        parseFile(diffFile, lineNumber, variables, sprites, samples, hitSounds, background, video, info);
        diffFile.close();
        std::cout << "Parsed " << lineNumber << " lines\n";

        lineNumber = 0;
        std::cout << "Parsing " << osb << "...\n";
        parseFile(osbFile, lineNumber, variables, sprites, samples, hitSounds, background, video, info);
        osbFile.close();
        std::cout << "Parsed " << lineNumber << " lines\n";
    }
}
