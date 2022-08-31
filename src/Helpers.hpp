#pragma once

#include <string>
#include <unordered_map>
#include <opencv2/core.hpp>
#include <filesystem>

namespace sb
{
    void stringReplace(std::string& s, const std::string& search, const std::string& replace);

    std::vector<std::string> stringSplit(std::string s, const std::string& delimiter);

    void applyVariables(std::string& line, const std::unordered_map<std::string, std::string>& variables);

    std::string removePathQuotes(const std::string& s);

    std::string exec(const std::string& cmd);

    double getAudioDuration(const std::string& filepath);

    cv::Mat convertImage(cv::Mat image);

    cv::Mat readImageFile(const std::string& filepath);

    void removeFile(const std::filesystem::path& filepath);
}
