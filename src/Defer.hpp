#pragma once

template <typename F>
class Defer {
public:
    constexpr Defer(F callback)
        : callback(callback)
    {
    }

    ~Defer()
    {
        callback();
    }
private:
    F callback;
};
