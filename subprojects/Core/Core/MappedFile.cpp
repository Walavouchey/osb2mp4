#include <Core/Defer.hpp>
#include <Core/MappedFile.hpp>
#include <cstring>
#include <iostream>
#include <string>

#if __linux__
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

namespace Core {

#if __linux__
ErrorOr<MappedFile> MappedFile::open(std::string_view path)
{
    auto path_string = std::string(path);
    auto fd = ::open(path_string.c_str(), O_RDONLY);
    if (fd < 0)
        return Error::from_string_literal(strerror(errno));
    auto should_close_file = true;
    Defer close_file = [=] {
        if (should_close_file)
            ::close(fd);
    };
    struct stat st;
    if (fstat(fd, &st) < 0)
        return Error::from_string_literal(strerror(errno));
    if (!S_ISREG(st.st_mode))
        return Error::from_string_literal("file is not a regular file");
    u32 size = st.st_size;
    auto data = mmap(0, size, PROT_READ, MAP_PRIVATE, fd, 0);
    if (data == MAP_FAILED)
        return Error::from_string_literal(strerror(errno));
    should_close_file = false;
    return MappedFile((c_string)data, size, fd);
}
#else
#error "unimplemented"
#endif

#if __linux__
MappedFile::~MappedFile()
{
    if (is_valid()) {
        munmap((void*)m_data, m_size);
        close(m_fd);
        invalidate();
    }
}
#else
#error "unimplemented"
#endif

}
