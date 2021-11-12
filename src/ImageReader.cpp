#include <ImageReader.hpp>

#include <gifdec.h>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

namespace sb
{
    cv::Mat readImage(const std::string& filepath)
    {
        cv::Mat fallback = cv::Mat::zeros(1, 1, CV_8UC4);
        if (!std::filesystem::exists(filepath))
            return fallback;

        // try opencv
        cv::Mat image = cv::imread(filepath, cv::IMREAD_UNCHANGED);
        if (!image.empty()) return image;

        // try gifdec
        gd_GIF* gif = gd_open_gif(filepath.c_str());
        if (gif)
        {
            image = cv::Mat(gif->height, gif->width, CV_8UC4);
            int ret = gd_get_frame(gif);
            if (ret == 0) return fallback;
            cv::MatIterator_<cv::Vec<uint8_t, 4>> imageStart = image.begin<cv::Vec<uint8_t, 4>>();
            std::vector<uint8_t> buffer = std::vector<uint8_t>();
            buffer.resize(gif->width * gif->height * 3);
            uint8_t* ptr = buffer.data();
            gd_render_frame(gif, ptr);
            for (int y = 0; y < gif->height; y++)
            {
                for (int x = 0; x < gif->width; x++)
                {
                    cv::Vec<uint8_t, 4> pixel;
                    if (gd_is_bgcolor(gif, ptr))
                    {
                        pixel = cv::Vec<uint8_t, 4>(0, 0, 0, 0);
                    }
                    else
                    {
                        pixel = cv::Vec<uint8_t, 4>(
                            ptr[2],
                            ptr[1],
                            ptr[0],
                            255
                            );
                    }
                    *(imageStart + y * gif->width + x) = pixel;
                    ptr += 3;
                }
            }
            return image;
        }

        std::cout << "Image " + std::filesystem::path(filepath).filename().string()
            + " either is invalid or otherwise could not be read.";
        return fallback;
    }
}