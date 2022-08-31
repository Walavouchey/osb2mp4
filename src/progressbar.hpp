#pragma once

#include <atomic>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <string>
#include <sstream>
#include <thread>

class ProgressBar
{
public:
    ProgressBar(std::string prefix, int denominator, int precision = 0, float reportInterval = -1)
        :
        prefix(prefix),
        denominator(denominator),
        precision(precision),
        reportInterval(reportInterval)
    {
        print();
        start = std::chrono::steady_clock::now();
        if (this->reportInterval.count() >= 0) thread = std::thread(&ProgressBar::timer, this);
    }
    void print()
    {
        now = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = now - start;
        std::cout << '\r' << prefix << numerator << '/' << denominator << ", "
            << std::setprecision(precision) << std::fixed
            << 100 * numerator / (float) denominator << "%, elapsed: "
            << formatTime(elapsed.count()) << ", remaining: "
            << (numerator == 0 ? "" : formatTime(elapsed.count() * (denominator / (float) numerator - 1)));
    }
    void update()
    {
        numerator++;
        if (reportInterval.count() < 0) print();
    }
    void finish()
    {
        done = true;
        if (thread.joinable()) thread.join();
        std::cout << std::endl;
    }
    ~ProgressBar()
    {
        if (!done) finish();
    }
private:
    std::thread thread;
    std::string prefix;
    std::atomic<int> numerator = 0;
    int denominator;
    int precision;
    std::chrono::duration<float> reportInterval;
    std::chrono::time_point<std::chrono::steady_clock> start;
    std::chrono::time_point<std::chrono::steady_clock> now;
    std::atomic<bool> done = false;
    void timer()
    {
        while (!done)
        {
            print();
            std::this_thread::sleep_for(reportInterval);
        }
        print();
    }
    std::string formatTime(const double t)
    {
        int ms = t * 1000;
        int h = ms / (1000 * 60 * 60);
        ms -= h * (1000 * 60 * 60);
        int m = ms / (1000 * 60);
        ms -= m * (1000 * 60);
        int s = ms / 1000;
        ms -= s * 1000;
        return (std::ostringstream() << std::setfill('0') << std::setw(2) << h << ':' << std::setw(2) << m
            << ':' << std::setw(2) << s << '.' << std::setw(3) << ms).str();
    }
};
