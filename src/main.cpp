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
#include <Sprite.hpp>

void printUsageAndExit(std::vector<std::tuple<bool, std::string, std::string, std::function<void(std::string&)>, std::string, std::string>> options, std::string filename)
{
    constexpr unsigned wrapLimit = 80;
    constexpr unsigned tabLimit = 20;
    std::cerr << "\nUsage: " << std::filesystem::path(filename).filename().string() << " song_folder [options]\n\noptions:\n";
    for (auto& o : options)
    {
        std::string optionsString = " " + std::get<1>(o) + ", " + std::get<2>(o) + " ";
        std::string optionString = optionsString
            + (std::get<0>(o) ? std::get<5>(o) + "\t" : "\t")
            + (optionsString.size() >= tabLimit ? "" : "\t")
            + std::get<4>(o) + "\n";
        size_t size = optionString.size();
        size_t end = wrapLimit;
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

    std::vector<std::tuple<bool, std::string, std::string, std::function<void(std::string&)>, std::string, std::string>> options
    {
#define opt(requiresArgument, option, alias, var, assign, explanation, placeholder) \
            std::tuple<bool, std::string, std::string, std::function<void(std::string&)>, std::string, std::string>( \
                requiresArgument, \
                option, \
                alias, \
                std::function([&var]([[maybe_unused]] std::string& arg) { var = assign; }), \
                explanation, \
                placeholder \
            )
        opt(true, "-s", "--start-time", _starttime, std::stod(arg), "start time in ms (default: automatic)", "time"),
        opt(true, "-e", "--end-time", _endtime, std::stod(arg), "end time in ms (default: automatic)", "time"),
        opt(true, "-d", "--duration", _duration, std::stod(arg), "duration in ms (default: automatic)", "time"),
        opt(true, "-o", "--output", outputFile, arg, "output video name (default: video.mp4)", "time"),
        opt(true, "-diff", "--difficulty", diff, arg, "difficulty file name (default: first found)", "filename"),
        opt(true, "-w", "--width", frameWidth, std::stoi(arg), "video width (default: 1920)", "pixels"),
        opt(true, "-h", "--height", frameHeight, std::stoi(arg), "video height (default: 1080)", "pixels"),
        opt(true, "-f", "--frame-rate", fps, std::stof(arg), "video frame rate (default: 30)", "fps"),
        opt(true, "-v", "--volume", volume, std::stof(arg) / 100.0f, "overall volume from 0 to 100 (default: 100)", "volume"),
        opt(true, "-mv", "--music-volume", musicVolume, std::stof(arg) / 100.0f, "music volume from 0 to 100 (default: 100)", "volume"),
        opt(true, "-ev", "--effect-volume", effectVolume, std::stof(arg) / 100.0f, "effect volume from 0 to 100, i.e. samples (default: 100)", "volume"),
        opt(true, "-dim", "--background-dim", dim, 1 - std::stof(arg) / 100.0f, "background dim value from 0 to 100 (default: 0)", "dim"),
        opt(false, "-ar", "--respect-aspect-ratio", useStoryboardAspectRatio, true, "change to 4:3 aspect ratio if WidescreenStoryboard is disabled in the difficulty file", ""),
        opt(false, "-fail", "--show-fail-layer", showFailLayer, true, "show the fail layer instead of the pass layer", ""),
        opt(false, "-keep", "--keep-temp-files", keepTemporaryFiles, true, "don't delete temporary files (temp.mp3 & temp.avi)", ""),
        opt(true, "-z", "--zoom", zoom, std::stof(arg), "zoom factor to use when rendering, useful for checking out-of-bounds sprites (default: 1)", "factor")
#undef opt
    };

    for (std::vector<std::string>::iterator arg = arguments.begin() + 1; arg < arguments.end(); arg++)
    {
        bool optionFound = false;
        for (auto& option : options)
        {
            if (*arg == std::get<1>(option) || *arg == std::get<2>(option))
            {
                if (arg + 1 != arguments.end() || !std::get<0>(option))
                {
                    try { std::get<3>(option)(*(arg += (arg + 1 != arguments.end() && std::get<0>(option) ? 1 : 0))); }
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
