#pragma once

#include <Components.hpp>
#include <Parser.hpp>

#include <opencv2/opencv.hpp>
#include <iostream>
#include <filesystem>
#include <string>
#include <limits>
#include <unordered_map>
#include <vector>
#include <memory>
#include <exception>

namespace sb
{
	class Storyboard
	{
	public:
		Storyboard(const std::filesystem::path& directory, const std::string& diff, std::pair<unsigned, unsigned> resolution, float musicVolume, float effectVolume, float dim, bool useStoryboardAspectRatio, bool showFailLayer)
			:
			directory(directory),
			diff(diff),
			resolution(resolution),
			musicVolume(musicVolume),
			effectVolume(effectVolume),
			dim(dim),
			showFailLayer(showFailLayer),
			frameScale(resolution.second / 480.0),
			xOffset((resolution.first - resolution.second / 3.0 * 4) * 0.5)
		{
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
			ParseStoryboard(directory, osb, diff, sprites, samples, hitSounds, background, video, info);

			auto wdsb = info.find("WidescreenStoryboard");
			bool widescreenStoryboard = wdsb != info.end() && std::stoi(wdsb->second) != 0;
			if (useStoryboardAspectRatio && !widescreenStoryboard)
				this->resolution = std::pair<unsigned, unsigned>(resolution.second / 3.0f * 4, resolution.second);

			this->sprites = std::move(sprites);
			std::cout << "Initialising storyboard (" << this->sprites.size() << " sprites, " << samples.size() << " samples)" << "\n";
			for (std::unique_ptr<Sprite>& sprite : this->sprites)
				sprite->Initialise(hitSounds);
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

			blankImage = cv::Mat::zeros(this->resolution.second, this->resolution.first, CV_8UC3);
			backgroundImage = cv::Mat::zeros(this->resolution.second, this->resolution.first, CV_8UC3);
			if (background.exists && !backgroundIsASprite)
			{
				cv::Mat image = readImageFile((directory / background.filepath).generic_string());
				cv::RotatedRect quadRect = cv::RotatedRect(
					cv::Point2f(
						this->resolution.first / 2.0f + background.offset.first * frameScale,
						this->resolution.second / 2.0f + background.offset.second * frameScale
					),
					cv::Size2f(this->resolution.second * image.cols / (double)image.rows, this->resolution.second),
					0
				);
				cv::Point2f quad[4];
				quadRect.points(quad);
				RasteriseQuad(backgroundImage.begin<cv::Vec<uint8_t, 3>>(), image.begin<cv::Vec<float, 4>>(), image.cols, image.rows, quad, Colour(1, 1, 1), false, 1);
			}

			if (video.exists && !(videoOpen = videoCap.open((directory / video.filepath).generic_string()))) videoCap.release();

			std::cout << "Loading images..." << std::endl;
			for (const std::unique_ptr<Sprite>& sprite : this->sprites)
			{
				std::vector<std::string> filePaths = sprite->GetFilePaths();
				for (std::string filePath : filePaths)
				{
					if (spriteImages.find(filePath) != spriteImages.end()) continue;
					cv::Mat image = readImageFile((directory / filePath).generic_string());
					auto ret = spriteImages.emplace(filePath, image);
				}
			}
		}
		std::pair<unsigned, unsigned> GetResolution() const
		{
			return resolution;
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
			std::unordered_map<std::string, int> sampleIndices;
			std::vector<int> indices;
			std::string command = "ffmpeg -y -hide_banner -v error -stats";
			auto k = info.find("AudioFilename");
			if (k != info.end()) command += " -i \"" + (directory / k->second).generic_string() + "\"";
			std::vector<int> delays;
			std::vector<double> volumes;
			volumes.push_back((samples.size() + 1) * musicVolume);
			indices.push_back(0);
			auto l = info.find("AudioLeadIn");
			if (k != info.end() && l != info.end()) delays.push_back(std::stoi(l->second));
			else if (k != info.end()) delays.push_back(1);
			int maxDelay = samples.size() == 0 ? 0 : std::numeric_limits<int>::lowest();
			double maxDuration = samples.size() == 0 ? 0 : std::numeric_limits<double>::lowest();
			int count = 1;
			int unique = 1;
			int totalSamples = samples.size();
			for (const Sample& sample : samples)
			{
				std::string filepath = (directory / sample.filepath).generic_string();
				auto ret = sampleIndices.emplace(filepath, unique);
				indices.push_back(ret.first->second);
				int delay = (int)sample.starttime;
				delays.push_back(delay);
				volumes.push_back(sample.volume / 100.0 * (samples.size() + 1) * effectVolume);
				maxDelay = std::max(maxDelay, delay);
				count++;
				if (ret.second)
				{
					command += " -i \"" + filepath + "\"";
					maxDuration = std::max(maxDuration, getAudioDuration((directory / sample.filepath).generic_string()));
					unique++;
					count = 0;
				}
				else totalSamples--;
				std::cout << unique << "/" << totalSamples + 1 << "  \r";
			}
			std::cout << std::endl;
			command += " -filter_complex ";
			std::string filters;
			for (int i = 0; i < delays.size(); i++)
				filters += "[" + std::to_string(indices[i]) + ":a]volume=" + std::to_string(volumes[i]) + ",adelay=delays=" + std::to_string(delays[i]) + ":all=1[d" + std::to_string(i) + "];";
			for (int i = 0; i < delays.size(); i++)
				filters += "[d" + std::to_string(i) + "]";
			filters += "amix=inputs=" + std::to_string(delays.size()) + ":dropout_transition=" + std::to_string(maxDelay + (int)maxDuration) + "[a]";
			command += "\"" + filters + "\"";
			command += " -map [a] ";
			command += outFile;
			//std::cout << command << "\n";
			// this can fail if the command length becomes very long, i.e. there are a lot of samples
			// TODO: replace this with a better idea
			int ret = system(command.c_str());
			if (ret != 0) std::cout << "Audio generation failed!" << std::endl;
		}
		cv::Mat DrawFrame(double time)
		{
			cv::Mat frame = video.exists ? GetVideoImage(time) : backgroundImage.clone();
			cv::MatIterator_<cv::Vec<uint8_t, 3>> frameStart = frame.begin<cv::Vec<cv::uint8_t, 3>>();
			for (const std::unique_ptr<Sprite>& sprite : sprites)
			{
				if (!(sprite->GetActiveTime().first <= time && sprite->GetActiveTime().second > time))
					continue;
				if (sprite->GetLayer() == (showFailLayer ? Layer::Pass : Layer::Fail)) continue;
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
				// TODO: check if it's OR or XOR
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
								cv::saturate_cast<uint8_t>(framePixel[0] + newAlpha * imageColour[2] * colour[2] * dim),
								cv::saturate_cast<uint8_t>(framePixel[1] + newAlpha * imageColour[1] * colour[1] * dim),
								cv::saturate_cast<uint8_t>(framePixel[2] + newAlpha * imageColour[0] * colour[0] * dim)
								);
						}
						else
						{
							newPixel = cv::Vec<uint8_t, 3>(
								cv::saturate_cast<uint8_t>((1 - newAlpha) * framePixel[0] + newAlpha * imageColour[2] * colour[2] * dim),
								cv::saturate_cast<uint8_t>((1 - newAlpha) * framePixel[1] + newAlpha * imageColour[1] * colour[1] * dim),
								cv::saturate_cast<uint8_t>((1 - newAlpha) * framePixel[2] + newAlpha * imageColour[0] * colour[0] * dim)
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

			float dx = x - (int)x;
			float dy = y - (int)y;

			cv::MatConstIterator_<cv::Vec<float, 4>> it, down, right, downright;
			it = down = right = downright = imageStart + (int)y * width + (int)x;

			if (!((int)x == width - 1)) // if not on right edge
			{
				right++;
				downright++;
			}
			if (!((int)y == height - 1)) // if not on last row
			{
				down += width;
				downright += width;
			}

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
		std::string osb;
		std::string diff;
		std::vector<std::unique_ptr<Sprite>> sprites;
		Background background;
		std::vector<Sample> samples;
		std::vector<std::pair<double, HitSound>> hitSounds;
		std::unordered_map<std::string, std::string> info;
		std::pair<double, double> activetime;
		std::pair<unsigned, unsigned> resolution;
		float musicVolume;
		float effectVolume;
		float dim;
		bool showFailLayer;
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
}