#pragma once
#include <Types.hpp>
#include <string_view>
#include <vector>

namespace OSB {

enum class TokenType {
    LeftBracket,
    RightBracket,

    Comma,
    EqualSign,
    NewLine,
    Number,
    Space,

    Identifier,
    QuotedString,
    String,

    Variables,
    Events,

    Invalid
};

struct Token {
    constexpr Token(TokenType type, u32 start, u32 end)
        : type(type)
        , start_index(start)
        , end_index(end)
    {
    }

    constexpr Token() = default;

    void dump(std::string_view source) const;
    std::string_view text(std::string_view source) const
    {
        return source.substr(start_index, end_index - start_index);
    }

    TokenType type { TokenType::Invalid };
    u32 start_index { 0 };
    u32 end_index { 0 };
};
using Tokens = std::vector<Token>;

}
