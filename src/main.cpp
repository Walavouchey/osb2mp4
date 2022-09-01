#include <ArgumentParser.hpp>
#include <Helpers.hpp>
#include <Sprite.hpp>
#include <Storyboard.hpp>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <opencv2/videoio.hpp>
#include <optional>
#include <progressbar.hpp>

int main(int argc, char const* argv[])
{
    auto argument_parser = sb::ArgumentParser();

    auto program_name = argv[0];
    argument_parser.add_flag("--help", "-h", "show help message", [&] {
        argument_parser.print_usage_and_exit(program_name, 0);
    });

    std::optional<double> maybe_start_time = {};
    argument_parser.add_option("--start-time", "-s", "time",
        "start time in ms (default: automatic)",
        [&](auto arg) {
            maybe_start_time = std::stod(std::string(arg));
        });

    std::optional<double> maybe_end_time = {};
    argument_parser.add_option("--end-time", "-e", "time",
        "end time in ms (default: automatic)",
        [&](auto arg) {
            maybe_end_time = std::stod(std::string(arg));
        });

    std::optional<double> maybe_duration = {};
    argument_parser.add_option("--duration", "-d", "time",
        "duration in ms (default: automatic)",
        [&](auto arg) {
            maybe_duration = std::stod(std::string(arg));
        });

    std::string_view output_file_name = "video.mp4";
    argument_parser.add_option("--output", "-o", "file",
        "output video name (default: video.mp4)",
        [&](auto arg) {
            output_file_name = arg;
        });

    std::string_view difficulty_file_name = {};
    argument_parser.add_option("--difficulty", "-diff", "file",
        "difficulty file name (default: first found)",
        [&](auto arg) {
            difficulty_file_name = arg;
        });

    uint64_t width = 1920;
    argument_parser.add_option("--width", "-w", "pixels",
        "video width (default: 1920)",
        [&](auto arg) {
            width = std::stoul(std::string(arg));
        });

    uint64_t height = 1080;
    argument_parser.add_option("--height", "-h", "pixels",
        "video height (default: 1080)",
        [&](auto arg) {
            height = std::stoul(std::string(arg));
        });

    uint64_t frame_rate = 30;
    argument_parser.add_option("--frame-rate", "-f", "fps",
        "video frame rate (default: 30)",
        [&](auto arg) {
            frame_rate = std::stoul(std::string(arg));
        });

    float volume = 1.0;
    argument_parser.add_option("--volume", "-v", "volume",
        "overall volume from 0 to 100 (default: 100)",
        [&](auto arg) {
                volume = std::stoul(std::string(arg)) / 100.0;
        });

    float music_volume = 1.0;
    argument_parser.add_option("--volume", "-v", "volume",
        "overall volume from 0 to 100 (default: 100)",
        [&](auto arg) {
            music_volume = std::stoul(std::string(arg)) / 100.0;
        });

    float effect_volume = 1.0;
    argument_parser.add_option("--volume", "-v", "volume",
        "overall volume from 0 to 100 (default: 100)",
        [&](auto arg) {
            effect_volume = std::stoul(std::string(arg)) / 100.0;
        });

    float background_dim = 1.0;
    argument_parser.add_option("--background-dim", "-dim", "dim",
        "background dim value from 0 to 100 (default: 0)",
        [&](auto arg) {
            background_dim = 1 - std::stoul(std::string(arg)) / 100.0;
        });

    bool respect_aspect_ratio = false;
    argument_parser.add_flag("--respect-aspect-ratio", "-ar",
        "change to 4:3 aspect ratio if WidescreenStoryboard is disabled in the difficulty file",
        [&] {
            respect_aspect_ratio = true;
        });

    bool show_fail_layer = false;
    argument_parser.add_flag("--show-fail-layer", "-fail",
        "show the fail layer instead of the pass layer",
        [&] {
            show_fail_layer = true;
        });

    bool keep_temporary_files = false;
    argument_parser.add_flag("--keep-temporary-files", "-keep",
        "don't delete temporary files",
        [&] {
            keep_temporary_files = true;
        });

    float zoom_factor = 1.0;
    argument_parser.add_option("--zoom", "-z", "factor",
        "zoom factor to use when rendering, useful for checking out-of-bounds sprites (default: 1)",
        [&](auto arg) {
            zoom_factor = std::stof(std::string(arg));
        });

    std::string_view osu_folder = {};
    argument_parser.add_positional_argument("osu-folder",
        [&](auto arg) {
            osu_folder = arg;
        });

    argument_parser.run(argc, argv);


    auto value_or = [](std::optional<double> maybe, std::string fallback) {
        return maybe ? std::to_string(*maybe) : fallback;
    };
    std::cout << "CONFIG:" << std::endl;
    std::cout << "\tstart time: " << value_or(maybe_start_time, "automatic") << std::endl;
    std::cout << "\tend time: " << value_or(maybe_end_time, "automatic") << std::endl;
    std::cout << "\tduration: " << value_or(maybe_duration, "automatic") << std::endl;
    std::cout << "\toutput: " << output_file_name << std::endl;
    std::cout << "\tdifficulty file: " << (difficulty_file_name.empty() ? "first found" : difficulty_file_name) << std::endl;
    std::cout << "\twidth: " << width << std::endl;
    std::cout << "\theight: " << height << std::endl;
    std::cout << "\tframe rate: " << frame_rate << std::endl;
    std::cout << "\tvolume: " << volume << std::endl;
    std::cout << "\tmusic volume: " << music_volume << std::endl;
    std::cout << "\teffect volume: " << effect_volume << std::endl;
    std::cout << "\tbackground dim: " << background_dim << std::endl;
    std::cout << "\trespect aspect ratio: " << respect_aspect_ratio << std::endl;
    std::cout << "\tshow fail layer: " << show_fail_layer << std::endl;
    std::cout << "\tkeep temporary files: " << keep_temporary_files << std::endl;
    std::cout << "\tzoom factor: " << zoom_factor << std::endl;
    std::cout << "\tosu-folder: " << osu_folder << std::endl;

    auto storyboard = sb::Storyboard(osu_folder, difficulty_file_name,
        { width, height }, music_volume * volume, effect_volume * volume,
        background_dim, respect_aspect_ratio, show_fail_layer, zoom_factor);

    auto activetime = storyboard.GetActiveTime();
    auto resolution = storyboard.GetResolution();
    auto audioLeadIn = storyboard.GetAudioLeadIn();
    auto audioDuration = storyboard.GetAudioDuration();
    auto starttime = maybe_start_time.value_or(std::min(activetime.first, audioLeadIn));
    double duration = maybe_duration.has_value() ?
        maybe_duration.value()
        : (maybe_end_time.has_value() ?
            maybe_end_time.value() - starttime
            : std::max(activetime.second, audioDuration) - starttime);
    
    char temporary_audio_file_path[] = "XXXXXX.mp3";
    if (mkstemps(temporary_audio_file_path, 4) < 0) {
        perror("mkstemps");
        exit(1);
    }
    std::cout << "Temporary audio path: "
              << temporary_audio_file_path << std::endl;
    std::cout << "Generating audio...";
    storyboard.generateAudio(temporary_audio_file_path);

    char temporary_video_file_path[] = "XXXXXX.avi";
    if (mkstemps(temporary_video_file_path, 4) < 0) {
        sb::removeFile(temporary_audio_file_path);
        perror("mkstemps");
        exit(1);
    }
    std::cout << "Temporary video path: "
              << temporary_video_file_path << std::endl;
    int frameCount = (int)std::ceil(frame_rate * duration / 1000.0);
    cv::VideoWriter writer = cv::VideoWriter(
        temporary_video_file_path,
        cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
        frame_rate,
        cv::Size(resolution.first, resolution.second)
    );

    ProgressBar progress("Rendering video: ", frameCount, 0, 0.5f);
#pragma omp parallel for ordered schedule(dynamic)
    for (int i = 0; i < frameCount; i++)
    {
        cv::Mat frame = storyboard.DrawFrame(starttime + i * 1000.0 / frame_rate);
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
    command << std::fixed << "ffmpeg -y -v error -stats "
            << "-i " << temporary_video_file_path
            << " -ss " << starttime + storyboard.GetAudioLeadIn()
            << "ms -to " << starttime + duration + storyboard.GetAudioLeadIn()
            << "ms -accurate_seek "
            << "-i " << temporary_audio_file_path << " -c:v copy "
            << output_file_name;
    system(command.str().c_str());
    if (!keep_temporary_files)
    {
        std::cout << "Deleting temporary files...\n";
        sb::removeFile(temporary_audio_file_path);
        sb::removeFile(temporary_video_file_path);
    }

    std::cout << "Done\n";
    return 0;
}
