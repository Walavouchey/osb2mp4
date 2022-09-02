#pragma once
#include <Core/ErrorOr.hpp>
#include <Lexer/SourceFile.hpp>
#include <OSB/OSBFile.hpp>
#include <OSB/Token.hpp>

namespace OSB {

enum class ParsedItemType {
    Variable,
    VariableSection,

    Invalid,
};

struct Variable {
    Token name {};
    u32 start_index { 0 };
    u32 end_index { 0 };

    void dump(std::string_view source, u32 indent) const;
};

struct VariableSection {
    Token name {};
    std::vector<Variable> variables {};

    void dump(std::string_view source, u32 indent) const;
};

struct ParsedItem {
    ParsedItem(Variable&& value, u32 start_index, u32 end_index)
        : m_storage(std::move(value))
        , start_token_index(start_index)
        , end_token_index(end_index)
    {
    }

    ParsedItem(VariableSection&& value, u32 start_index, u32 end_index)
        : m_storage(std::move(value))
        , start_token_index(start_index)
        , end_token_index(end_index)
    {
    }

    ParsedItemType type() const
    {
        return (ParsedItemType)m_storage.index();
    }

    Variable const& as_variable() const
    {
        return std::get<Variable>(m_storage);
    }

    VariableSection const& as_variable_section() const
    {
        return std::get<VariableSection>(m_storage);
    }

    void dump(std::string_view source, u32 indent = 0) const;

private:
    std::variant<Variable, VariableSection> m_storage {};

public:
    u32 start_token_index { 0 };
    u32 end_token_index { 0 };
};
using ParsedItems = std::vector<ParsedItem>;

struct ParseError {
    std::string_view message {};
    std::string_view hint {};
    Token offending_token;

    Core::ErrorOr<void> show(Lexer::SourceFile source) const;
};

using ParseResult = Core::ErrorOr<ParsedItems, ParseError>;
ParseResult parse_osb(Tokens const& tokens);

}
