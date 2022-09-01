#pragma once

#include <Keyframes.hpp>
#include <Helpers.hpp>

#include <iostream>
#include <fstream>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>
#include <utility>
#include <memory>
#include <exception>

#include <Forward.hpp>

namespace sb
{
    void parseFile(std::ifstream& file, size_t& lineNumber, std::unordered_map<std::string, std::string>& variables, std::vector<std::unique_ptr<Sprite>>& sprites, std::vector<Sample>& samples, std::vector<std::pair<double, HitSound>>& hitSounds, Background& background, Video& video, std::unordered_map<std::string, std::string>& info);

    void ParseStoryboard(const std::filesystem::path& directory, const std::string& osb, std::string_view diff,
        std::vector<std::unique_ptr<Sprite>>& sprites,
        std::vector<Sample>& samples,
        std::vector<std::pair<double, HitSound>>& hitSounds,
        Background& background,
        Video& video,
        std::unordered_map<std::string, std::string>& info
    );
}
