#pragma once

#include <Parser.hpp>

#include <opencv2/opencv.hpp>
#include <iostream>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <limits>
#include <unordered_map>
#include <vector>
#include <memory>
#include <exception>

#include <Video.hpp>
#include <Background.hpp>
#include <Sample.hpp>
#include <HitSound.hpp>

namespace sb
{
    class Storyboard
    {
    public:
        Storyboard(const std::filesystem::path& directory, const std::string& diff, std::pair<unsigned, unsigned> resolution, float musicVolume, float effectVolume, float dim, bool useStoryboardAspectRatio, bool showFailLayer, float zoom = 1);

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

        void generateAudio(const std::string& outFile) const;

        cv::Mat DrawFrame(double time);

    private:
        cv::Mat GetVideoImage(double time);

        void RasteriseQuad(cv::MatIterator_<cv::Vec<uint8_t, 3>> frameStart, cv::MatConstIterator_<cv::Vec<float, 4>> imageStart, int imageWidth, int imageHeight, cv::Point2f quad[4], Colour colour, bool additive, double alpha) const;

        void SetPixel(cv::MatIterator_<cv::Vec<uint8_t, 3>> imageStart, int width, const int x, int y, cv::Vec<uint8_t, 3> pixel) const
        {
            *(imageStart + y * width + x) = pixel;
        }

        cv::Vec<uint8_t, 3> GetPixel(const cv::MatIterator_<cv::Vec<uint8_t, 3>> imageStart, int width, int x, int y) const
        {
            return *(imageStart + y * width + x);
        }

        void SampleColourAndAlpha(cv::MatConstIterator_<cv::Vec<float, 4>> imageStart, int width, int height, float u, float v, Colour& outputColour, float& outputAlpha) const;

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

        void ScanLine(int x1, int y1, int x2, int y2, std::vector<std::pair<int, int>>& ContourX) const;

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
        [[maybe_unused]] double lastFrame;
        double frameScale;
        double xOffset;
        float zoom;
    };
}
