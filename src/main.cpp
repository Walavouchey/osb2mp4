#include <progressbar.hpp>
#include <Storyboard.hpp>

#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <memory>
#include <vector>
#include <string>
#include <functional>
#include <optional>
#include <string_view>
#include <Sprite.hpp>

struct Option {
    std::string_view option;
    std::string_view alias;
    std::string_view explanation;
    std::string_view placeholder;
    std::function<void(std::string_view arg)> onUse;
    bool requiresArgument;
};

static void printUsageAndExit(const std::vector<Option>& options, std::string_view filename);

int main(int argc, char* argv[]) {
    std::string filename = argv[0];
    std::string directory;
    std::string diff;
    std::optional<double> _starttime;
    std::optional<double> _endtime;
    std::optional<double> _duration;
    int frameWidth = 1920;
    int frameHeight = 1080;
    float fps = 30;
    float volume = 1.0f;
    float musicVolume = 1.0f;
    float effectVolume = 1.0f;
    float dim = 1.0f;
    bool useStoryboardAspectRatio = false;
    bool showFailLayer = false;
    std::string outputFile = "video.mp4";
    bool keepTemporaryFiles = false;
    float zoom = 1;

    std::vector<std::string> arguments;
    for (int i = 0; i < argc; i++)
        arguments.push_back(std::string(argv[i]));

    auto set_optional_double = [](std::optional<double>& value) {
        return [&](std::string_view arg) { value = std::stod(std::string(arg)); };
    };
    auto set_int = [](int& value) {
        return [&](std::string_view arg) { value = std::stoi(std::string(arg)); };
    };
    auto set_float_with_scale = [](float& value, float scale) {
        return [&, scale](std::string_view arg) { value = std::stof(std::string(arg)) * scale; };
    };
    std::vector<Option> options
    {
        {
            "-s",    "--start-time",   "start time in ms (default: automatic)",       "time",
            set_optional_double(_starttime), true
        },
        { 
            "-e",    "--end-time",     "end time in ms (default: automatic)",         "time",
            set_optional_double(_endtime), true
        },
        { 
            "-d",    "--duration",     "duration in ms (default: automatic)",         "time",
            set_optional_double(_duration), true
        },
        { 
            "-o",    "--output",       "output video name (default: video.mp4)",      "time",
            [&](auto arg){ outputFile = arg; }, true
        },
        { 
            "-diff", "--difficulty",   "difficulty file name (default: first found)", "filename",
            [&](auto arg){ diff = arg; }, true
        },
        { 
            "-w",    "--width",        "video width (default: 1920)",                 "pixels",
            set_int(frameWidth), true
        },
        {
            "-h",    "--height",       "video height (default: 1080)",                "pixels",
            set_int(frameHeight), true
        },
        { 
            "-f",    "--frame-rate",   "video frame rate (default: 30)",              "fps",
            set_float_with_scale(fps, 1), true
        },
        {
            "-v",    "--volume",       "overall volume from 0 to 100 (default: 100)", "volume",
            set_float_with_scale(volume, 0.01), true
        },
        {
            "-mv",   "--music-volume", "music volume from 0 to 100 (default: 100)",   "volume", 
            set_float_with_scale(musicVolume, 0.01), true
        },
        {
            "-ev",   "--effect-volume", "effect volume from 0 to 100, i.e. samples (default: 100)", "volume",
            set_float_with_scale(effectVolume, 0.01), true
        },
        {
            "-dim", "--background-dim", "background dim value from 0 to 100 (default: 0)", "dim",
            [&](auto arg) { dim = 1 - std::stof(std::string(arg)) / 100.0f; }, true
        },
        {
            "-ar", "--respect-aspect-ratio", "change to 4:3 aspect ratio if WidescreenStoryboard is disabled in the difficulty file", "",
            [&](auto) { useStoryboardAspectRatio = true; }, false
        },
        {
            "-fail", "--show-fail-layer", "show the fail layer instead of the pass layer", "",
            [&](auto) { showFailLayer = true; }, false
        },
        {
            "-keep", "--keep-temp-files", "don't delete temporary files (temp.mp3 & temp.avi)", "",
            [&](auto) { keepTemporaryFiles = true; }, false
        },
        {
            "-z", "--zoom", "zoom factor to use when rendering, useful for checking out-of-bounds sprites (default: 1)", "factor", 
            [&](auto arg) { zoom = std::stof(std::string(arg)); }, true
        }
    };

    for (std::vector<std::string>::iterator arg = arguments.begin() + 1; arg < arguments.end(); arg++)
    {
        bool optionFound = false;
        for (auto& option : options)
        {
            if (*arg == option.option || *arg == option.alias)
            {
                if (arg + 1 != arguments.end() || !option.requiresArgument)
                {
                    try { option.onUse(*(arg += (arg + 1 != arguments.end() && option.requiresArgument) ? 1 : 0)); }
                    catch (...)
                    {
                        std::cerr << *(--arg) << ": invalid argument \"" << *(++arg) << "\"\n";
                        printUsageAndExit(options, filename);
                    }
                    optionFound = true;
                    break;
                }
                else
                {
                    std::cerr << "Option \"" << *arg << "\" requires an argument!\n";
                    printUsageAndExit(options, filename);
                };
            }
        }
        if (!optionFound && !directory.empty())
        {
            std::cerr << "Unrecognised option: " << *arg << std::endl;
            printUsageAndExit(options, filename);
        }
        if (!optionFound && directory.empty()) directory = *arg;
    }
    if (directory.empty())
    {
        std::cerr << "No song folder specified!\n";
        printUsageAndExit(options, filename);
    }

    std::unique_ptr<sb::Storyboard> sb;

    try
    {
        sb = std::make_unique<sb::Storyboard>(
            directory, diff, std::pair<unsigned, unsigned>(frameWidth, frameHeight),
            musicVolume * volume, effectVolume * volume, dim, useStoryboardAspectRatio, showFailLayer, zoom);
    }
    catch (std::exception e)
    {
        std::cerr << e.what() << std::endl;
        exit(1);
    }

    std::pair<double, double> activetime = sb->GetActiveTime();
    std::pair<unsigned, unsigned> resolution = sb->GetResolution();
    double audioLeadIn = sb->GetAudioLeadIn();
    double audioDuration = sb->GetAudioDuration();
    double starttime = _starttime.value_or(std::min(activetime.first, audioLeadIn));
    double duration = _duration.has_value() ?
        _duration.value()
        : (_endtime.has_value() ?
            _endtime.value() - starttime
            : std::max(activetime.second, audioDuration) - starttime);
    
    std::cout << "Generating audio...";
    sb->generateAudio("temp.mp3");

    int frameCount = (int)std::ceil(fps * duration / 1000.0);
    cv::VideoWriter writer = cv::VideoWriter(
        "temp.avi",
        cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
        fps,
        cv::Size(resolution.first, resolution.second)
    );

    ProgressBar progress("Rendering video: ", frameCount, 0, 0.5f);
#pragma omp parallel for ordered schedule(dynamic)
    for (int i = 0; i < frameCount; i++)
    {
        cv::Mat frame = sb->DrawFrame(starttime + i * 1000.0 / fps);
#pragma omp ordered
        {
            writer.write(frame);
            progress.update();
        }
    }
    writer.release();
    progress.finish();

    std::cout << "Merging audio and video...\n";
    std::stringstream command;
    command << std::fixed << "ffmpeg -y -v error -stats -i temp.avi -ss " << starttime + sb->GetAudioLeadIn() << "ms -to "
        << starttime + duration + sb->GetAudioLeadIn() << "ms -accurate_seek -i temp.mp3 -c:v copy " << outputFile;
    system(command.str().c_str());
    if (!keepTemporaryFiles)
    {
        std::cout << "Deleting temporary files...\n";
        sb::removeFile("temp.mp3");
        sb::removeFile("temp.avi");
    }

    std::cout << "Done\n";
    return 0;
}


static void printUsageAndExit(const std::vector<Option>& options, std::string_view filename)
{
    constexpr unsigned wrapLimit = 80;
    constexpr unsigned tabLimit = 20;
    std::cerr << "\nUsage: " << std::filesystem::path(filename).filename().string() << " song_folder [options]\n\noptions:\n";
    for (auto const& o : options)
    {
        std::string optionsString = " " + std::string(o.option) + ", " + std::string(o.alias) + " ";
        std::string optionString = optionsString
            + (o.requiresArgument ? std::string(o.placeholder) + "\t" : "\t")
            + (optionsString.size() >= tabLimit ? "" : "\t")
            + std::string(o.explanation) + "\n";
        int size = optionString.size();
        int end = wrapLimit;
        while (size > wrapLimit)
        {
            int pos = optionString.rfind(" ", end) + 1;
            size -= pos - 33;
            end += 33;
            optionString.insert(pos, "\n\t\t\t\t");
        }
        std::cerr << optionString;
    }
    exit(1);
}
