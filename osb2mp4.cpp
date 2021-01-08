#include <Storyboard.hpp>

#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <memory>

int main(int argc, char* argv[]) {
	if (argc <= 2) std::cout << "Specify directory, diff name and optionally start time and duration in milliseconds\n";
	std::string directory = argv[1];
	std::string diff = argv[2];
	int frameWidth = 1920;
	int frameHeight = 1080;
	int fps = 30;
	float musicVolume = 0.2f;
	float effectVolume = 0.2f;
	float dim = 1.0f;
	bool useStoryboardAspectRatio = true;
	bool showFailLayer = false;

	std::unique_ptr<sb::Storyboard> sb = std::make_unique<sb::Storyboard>(
		directory, diff, std::pair<unsigned, unsigned>(frameWidth, frameHeight),
		musicVolume, effectVolume, dim, useStoryboardAspectRatio, showFailLayer);

	std::pair<double, double> activetime = sb->GetActiveTime();
	std::pair<unsigned, unsigned> resolution = sb->GetResolution();
	double audioLeadIn = sb->GetAudioLeadIn();
	double audioDuration = sb->GetAudioDuration();
	double starttime = argc <= 3 ? std::min(activetime.first, audioLeadIn) : std::stod(argv[3]);
	double duration = argc <= 4 ?
		(activetime.second < audioDuration + 60000 ?
			std::max(activetime.second, audioDuration) - starttime
			: audioDuration)
		: std::stod(argv[4]);

	std::cout << "\nGenerating audio...\n";
	sb->generateAudio("audio.mp3");

	std::string outputFile = "video.mp4";
	int frameCount = (int)std::ceil(fps * duration / 1000.0);
	std::cout << "Rendering video...\n";
	cv::VideoWriter writer = cv::VideoWriter(
		"export.avi",
		cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
		fps,
		cv::Size(resolution.first, resolution.second)
	);

#pragma omp parallel for ordered schedule(dynamic)
	for (int i = 0; i < frameCount; i++)
	{
		cv::Mat frame = sb->DrawFrame(starttime + i * 1000.0 / fps);
#pragma omp ordered
		{
			std::cout << i + 1 << "/" << frameCount << '\r';
			writer.write(frame);
		}
	}
	writer.release();

	std::cout << "Merging audio and video...\n";
	std::stringstream command;
	command << "ffmpeg -y -v error -stats -i export.avi -ss " << starttime + sb->GetAudioLeadIn() << "ms -to "
		<< starttime + duration + sb->GetAudioLeadIn() << "ms -accurate_seek -i audio.mp3 -c:v copy " << outputFile;
	system(command.str().c_str());

	std::cout << "Done\n";
	return 0;
}