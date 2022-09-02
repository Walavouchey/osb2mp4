#pragma once
#include <Core/ErrorOr.hpp>
#include <Lexer/SourceFile.hpp>
#include <OSB/Token.hpp>
#include <unordered_map>

namespace OSB {

struct FileError {
    std::string_view message {};
    Token offending_token;

    Core::ErrorOr<void> show(Lexer::SourceFile source) const;
};
using FetchVariableResult = Core::ErrorOr<Tokens const*, FileError>;

struct OSBFile {
    void dump() const;
    
private:
    FetchVariableResult fetch_variable(std::string_view source,
        Token name_token) const
    {
        auto variable_name = name_token.text(source);
        if (m_variables.find(variable_name) == m_variables.end())
            return FileError { "could not find variable", name_token };
        return &m_variables.at(variable_name);
    }
    std::unordered_map<std::string_view, Tokens> m_variables;

};

}
