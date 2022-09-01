#include <Helpers.hpp>

#include <ImageReader.hpp>
#include <utility>
#include <exception>
#include <fstream>
#include <sndfile.h>

#if _WIN32
#define popen _popen
#define pclose _pclose
#endif

namespace sb {

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
        for (auto e : variables)
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
        FILE* pipe = popen(cmd.c_str(), "r");
        if (!pipe) throw std::runtime_error("popen() failed!");
        try {
            while (fgets(buffer, sizeof buffer, pipe) != NULL) {
                result += buffer;
            }
        }
        catch (...) {
            pclose(pipe);
            throw;
        }
        pclose(pipe);
        return result;
    }

    double getAudioDuration(const std::string& filepath)
    {
        SF_INFO info;
        auto file = sf_open(filepath.c_str(), SFM_READ, &info);
        if (!file) {
            throw std::runtime_error(sf_strerror(nullptr));
        }
        sf_close(file);
        return (double)info.samplerate / (double)info.frames / (double)info.channels;
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
        return convertImage(readImage(filepath));
    }

    void removeFile(const std::filesystem::path& filepath)
    {
        try
        {
            std::filesystem::remove(filepath);
        }
        catch (std::filesystem::filesystem_error e)
        {
            std::cerr << "Could not delete \"" << filepath << "\": " << e.what() << std::endl;
        }
    }

}
