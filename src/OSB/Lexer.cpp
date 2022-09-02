#include "OSB/Token.hpp"
#include <Core/Error.hpp>
#include <OSB/Lexer.hpp>
#include <Lexer/util.hpp>
#include <iostream>

namespace OSB {

Core::ErrorOr<void> LexError::show(Lexer::SourceFile source) const
{
    auto maybe_line_and_column = Lexer::line_and_column_for(source.text, source_index);
    if (!maybe_line_and_column)
        return Core::Error::from_string_literal("could not fetch line");

    auto line_number = maybe_line_and_column->line;
    auto column_number = maybe_line_and_column->column;
    auto line = Lexer::fetch_line(source.text, line_number);

    std::cerr << "Lex error: " << message << ' '
              << '['  << source.file_name
              << ':'  << line_number + 1
              << ':'  << column_number + 1
              << ']'  << '\n'
              ;
    std::cerr << line << '\n';
    for (u32 column = 0; column < column_number; column++)
        std::cerr << ' ';
    std::cerr << '^' << '\n';

    return {};
}

static bool is_delimiter(char character);
static bool is_identifier_character(char character);

static Token lex_string(std::string_view source, u32 start);
static Token lex_quoted(std::string_view source, u32 start);
static Token lex_identifier(std::string_view source, u32 start);
static Token lex_number(std::string_view source, u32 start);

using LexItemResult = Core::ErrorOr<Token, LexError>;
static LexItemResult lex_single_item(std::string_view source, u32 start);

LexResult lex_osb(std::string_view source)
{
    Tokens tokens;

    for (u32 start = 0; start < source.size();) {
        auto character = source[start];
        if (character == '\r')
            continue;
        auto result = lex_single_item(source, start);
        if (result.is_error())
            return result.release_error();
        auto token = result.release_value();
        tokens.push_back(token);
        start = token.end_index;
    }

    return tokens;
}

static LexItemResult lex_single_item(std::string_view source, u32 start)
{
    auto character = source[start];

    if (character == '[')
        return Token { TokenType::LeftBracket, start, start + 1 };

    if (character == ']')
        return Token { TokenType::RightBracket, start, start + 1 };

    if (character == ',')
        return Token { TokenType::Comma, start, start + 1 };

    if (character == '=')
        return Token { TokenType::EqualSign, start, start + 1 };

    if (character == '\n')
        return Token { TokenType::NewLine, start, start + 1 };

    if (Lexer::is_number(character))
        return lex_number(source, start);

    if (character == '\"')
        return lex_quoted(source, start);

    if (Lexer::is_letter(character)) {
        auto token = lex_string(source, start);
        auto value = token.text(source);
        if (value == "Variables")
            token.type = TokenType::Variables;
        if (value == "Events")
            token.type = TokenType::Events;
        return token;
    }

    if (is_identifier_character(character))
        return lex_identifier(source, start);

    return LexError { "unknown token", start };
}

static Token lex_string(std::string_view source, u32 start)
{
    u32 end = start;
    for (;end < source.size(); end++) {
        if (Lexer::is_letter(source[end]))
            continue;
        break;
    }
    return { TokenType::String, start, end };
}

static Token lex_quoted(std::string_view source, u32 start)
{
    u32 end = start + 1;
    for (;end < source.size(); end++) {
        char character = source[end];
        if (character != '"')
            continue;
        break;
    }
    return { TokenType::QuotedString, start, end + 1 };
}

static Token lex_identifier(std::string_view source, u32 start)
{
    u32 end = start + 1;
    for (;end < source.size(); end++) {
        char character = source[end];
        if (Lexer::is_letter(character))
            continue;
        if (Lexer::is_number(character))
            continue;
        break;
    }
    return { TokenType::Identifier, start, end };
}

static Token lex_number(std::string_view source, u32 start)
{
    u32 end = start;
    for (;end < source.size(); end++) {
        char character = source[end];
        if (character == '.')
            continue;
        if (!Lexer::is_number(source[end]))
            break;
    }
    return { TokenType::Number, start, end };
}

static bool is_delimiter(char character)
{
    switch (character) {
        case ',': return true;
        case '\r': return true;
        case '\n': return true;
        case '=': return true;
        default: return false;
    }
}

static bool is_identifier_character(char character)
{
    return Lexer::is_letter(character) || character == '$';
}

}
