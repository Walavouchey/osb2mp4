#pragma once
#include <Types.hpp>
#include <string_view>
#include <optional>

namespace Lexer {

struct LineAndColumn {
    u32 line;
    u32 column;
};
std::optional<LineAndColumn> line_and_column_for(std::string_view, u32 index);

std::string_view fetch_line(std::string_view, u32 line);

inline constexpr bool is_whitespace(char character)
{
    return character == ' ' || character == '\t' || character;
}

inline constexpr bool is_linefeed(char character)
{
    return character == '\r' || character == '\n';
}

inline constexpr bool is_number(char character)
{
    switch (character) {
        case '0'...'9': return true;
        default: return false;
    }
}

inline constexpr bool is_letter(char character) 
{
    switch (character) {
        case 'A'...'Z': return true;
        case 'a'...'z': return true;
        default: return false;
    }
}

}
