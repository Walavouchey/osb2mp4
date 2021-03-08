#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <unordered_map>
#include <vector>
#include <utility>
#include <exception>

namespace sb
{
    void stringReplace(std::string& s, const std::string& search, const std::string& replace)
    {
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
        if (image.empty()) image = cv::Mat::zeros(1, 1, CV_8UC4);
        else cv::cvtColor(image, converted, cv::COLOR_BGR2BGRA);
        double scale = depth == CV_16U ? 1.0 / 257 : 1;
        converted.convertTo(image, CV_32F, scale);
        return image;
    }

    cv::Mat readImageFile(const std::string& filepath)
    {
        return convertImage(cv::imread(filepath, cv::IMREAD_UNCHANGED));
    }
}