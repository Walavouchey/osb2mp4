#pragma once
#include "Core/ErrorOr.hpp"
#include <string_view>

namespace Core {

struct MappedFile {

    MappedFile(MappedFile&& other)
        : m_data(other.m_data)
        , m_size(other.m_size)
        , m_fd(other.m_fd)
    {
        other.invalidate();
    }

    static ErrorOr<MappedFile> open(std::string_view path);
    ~MappedFile();

    std::string_view view() const
    {
        return std::string_view(m_data, m_size);
    }

private:
    constexpr MappedFile() = default;

#if __linux__
    constexpr MappedFile(c_string data, u32 size, int fd)
        : m_data(data)
        , m_size(size)
        , m_fd(fd)
    {
    }
#else
#error "unimplemented"
#endif

    constexpr bool is_valid() const { return m_data; }
    void invalidate() { m_data = nullptr; }

    char const* m_data { nullptr };
    u32 m_size { 0 };
#if __linux__
    int m_fd { 0 };
#endif
};

}
