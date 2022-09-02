#include <Lexer/util.hpp>
#include <string_view>

#include <iostream>

namespace Lexer {

std::optional<LineAndColumn> line_and_column_for(
    std::string_view source,
    u32 index)
{
    if (index >= source.size())
        return {};

    u32 line = 0;
    u32 column = 0;
    for (u32 i = 0; i < index; i++) {
        if (source[i] == '\n') {
            line++;
            column = 0;
        } else {
            column++;
        }
    }

    return LineAndColumn { line, column };
}

std::string_view fetch_line(std::string_view source, u32 line_to_find)
{
    u32 line = 0;
    u32 start = 0;
    while (start < source.size() && line != line_to_find)
        if (source[start++] == '\n')
            line++;
    u32 end = start;
    while (end < source.size())
        if (source[end++] == '\n')
            break;
    return source.substr(start, end - 1 - start);
}

}
