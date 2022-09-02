#include "Core/ErrorOr.hpp"
#include "Lexer/util.hpp"
#include "OSB/Token.hpp"
#include <OSB/Parser.hpp>
#include <iostream>

namespace OSB {

std::string_view parsed_item_type_string(ParsedItemType type)
{
    switch (type) {
#define CASE_RETURN(variant) case ParsedItemType::variant: return #variant
        CASE_RETURN(Variable);
        CASE_RETURN(VariableSection);
        CASE_RETURN(Invalid);
#undef CASE_RETURN
    }
}

void Variable::dump(std::string_view source, u32) const
{
    std::cout << "Variable("
              << '\''
              << name.text(source)
              << '\''
              << ' '
              << '\''
              << source.substr(start_index, end_index - start_index)
              << '\''
              << ')'
              ;
}

void VariableSection::dump(std::string_view source, u32 indent) const
{
    std::cout.width(indent);
    std::cout << "VariableSection {\n";
    for (auto variable : variables) {
        std::cout << ' ';
        variable.dump(source, indent+1);
        std::cout << '\n';
        for (u32 i = 0; i < indent; i++)
            std::cout << ' ';
    }
    std::cout << "}";
}

void ParsedItem::dump(std::string_view source, u32 indent) const
{
    switch (type()) {
    case ParsedItemType::Variable:
        as_variable().dump(source, indent);
        break;
    case ParsedItemType::VariableSection:
        as_variable_section().dump(source, indent);
        break;
    case ParsedItemType::Invalid:
        __builtin_unreachable();
    }
}

Core::ErrorOr<void> ParseError::show(Lexer::SourceFile source) const
{
    auto start = *Lexer::line_and_column_for(source.text, offending_token.start_index);
    auto end = *Lexer::line_and_column_for(source.text, offending_token.end_index);
    auto line = Lexer::fetch_line(source.text, start.line);

    std::cerr << "Parse error: " << message << " "
              << source.file_name
              << ':'
              << start.line
              << ':'
              << start.column
              << '\n';
    std::cerr << line << '\n';
    
    for (u32 i = 0; i < start.column; i++)
        std::cerr << ' ';
    for (u32 i = start.column; i < end.column; i++)
        std::cerr << '^';
    std::cerr << '\n';

    if (!hint.empty()) {
        std::cerr << "Hint: " << hint << '\n';
    }

    return {};
}

using ParseSingleItemResult = Core::ErrorOr<ParsedItem, ParseError>;
static ParseSingleItemResult parse_single_item(Tokens const& tokens,
    u32 start);

ParseResult parse_osb(Tokens const& tokens)
{
    ParsedItems items;
    for (u32 start = 0; start < tokens.size(); start++) {
        if (tokens[start].type == TokenType::NewLine)
            continue; // Ignore leading and trailing new lines.
        auto parse_result = parse_single_item(tokens, start);
        if (parse_result.is_error())
            return parse_result.release_error();
        auto parsed_item = parse_result.release_value();
        items.push_back(std::move(parsed_item));
        start = parsed_item.end_token_index;
    }
    return items;
}

static ParseSingleItemResult parse_section(Tokens const& tokens,
    u32 start);

static ParseSingleItemResult parse_variable_section(
    Tokens const& tokens,
    u32 start);

static ParseSingleItemResult parse_single_item(Tokens const& tokens,
    u32 start)
{
    auto token = tokens[start];
    if (token.type == TokenType::LeftBracket)
        return parse_section(tokens, start);

    return ParseError { "unexpected token", "", token };
}

static ParseSingleItemResult parse_section(Tokens const& tokens,
    u32 start)
{
    if (tokens[start + 2].type != TokenType::RightBracket) {
        return ParseError {
            "unexpected token",
            "expected ']'",
            tokens[start + 2]
        };
    }

    if (tokens[start + 3].type != TokenType::NewLine) {
        return ParseError {
            "unexpected token",
            "expected new line",
            tokens[start + 3]
        };
    }

    auto section_token = tokens[start + 1];
    auto section_type = section_token.type;

    if (section_type == TokenType::Variables)
        return parse_variable_section(tokens, start);

    if (section_type == TokenType::Events) {
        return ParseError {
            "unimplemented",
            "create a parse_events_section function",
            section_token
        };
    }

    return ParseError {
        "unknown section name",
        "expected one of ['Variables', 'Events']",
        section_token 
    };
}

static ParseSingleItemResult parse_variable(Tokens const& token,
    u32 start);

static ParseSingleItemResult parse_variable_section(
    Tokens const& tokens,
    u32 start)
{
    VariableSection variable_section;
    variable_section.name = tokens[1];
    u32 end = start + 4;
    for (;end < tokens.size();) {
        auto name_token = tokens[end];
        if (name_token.type != TokenType::Identifier) {
            return ParseError {
                "unexpected token",
                "expected variable name",
                name_token
            };
        }
        auto parse_result = parse_variable(tokens, end);
        if (parse_result.is_error())
            return parse_result.release_error();
        auto generic_variable = parse_result.release_value();
        auto variable = generic_variable.as_variable();
        variable_section.variables.push_back(std::move(variable));
        end = generic_variable.end_token_index;
    }
    return ParsedItem(std::move(variable_section), start, end);
}

static ParseSingleItemResult parse_variable(Tokens const& tokens,
    u32 start)
{
    auto equal_sign = tokens[start + 1];
    if (equal_sign.type != TokenType::EqualSign) {
        return ParseError {
            "unexpected token",
            "expected '='",
            equal_sign
        };
    }
    auto end = start + 2;
    for (;end < tokens.size(); end++) {
        if (tokens[end].type == TokenType::NewLine)
            break;
    }

    auto first_token = tokens[start + 2];
    auto last_token = tokens[end - 1];

    Variable variable;
    variable.name = tokens[start];
    variable.start_index = first_token.start_index;
    variable.end_index = last_token.end_index;
    return ParsedItem(std::move(variable), start, end + 1);
}

}
