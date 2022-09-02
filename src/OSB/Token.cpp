#include "Lexer/util.hpp"
#include <OSB/Token.hpp>
#include <iostream>

namespace OSB {

static std::string_view token_type_string(TokenType type);

void Token::dump(std::string_view source) const
{
    auto text = source.substr(start_index, end_index - start_index);
    auto start = *Lexer::line_and_column_for(source, start_index);
    start.line += 1;
    start.column += 1;
    auto end = *Lexer::line_and_column_for(source, end_index);
    end.line += 1;
    end.column += 1;
    std::cout << "Token"
              << '['
              ;
    auto old_width = std::cout.width(12);
    std::cout << token_type_string(type);
#if 0
    std::cout.width(1);
    std::cout << '(';
    std::cout.width(3);
    std::cout << start.line;
    std::cout.width(1);
    std::cout << ' ';
    std::cout.width(2);
    std::cout << start.column;
    std::cout.width(1);
    std::cout << ' ';
    std::cout.width(3);
    std::cout << end.line;
    std::cout.width(1);
    std::cout << ' ';
    std::cout.width(2);
    std::cout << end.column;
    std::cout.width(1);
    std::cout << ')';
#endif
    std::cout << ' ';
    std::cout << '\'' << (text == "\n" ? "\\n" : text) << '\'';
              
    std::cout << ']' << '\n';
    std::cout.width(old_width);
    // std::cout << value << value;
}

static std::string_view token_type_string(TokenType type)
{
    switch (type) {
#define CASE_RETURN(variant) case TokenType::variant: return #variant
        CASE_RETURN(LeftBracket);
        CASE_RETURN(RightBracket);

        CASE_RETURN(Comma);
        CASE_RETURN(EqualSign);
        CASE_RETURN(NewLine);
        CASE_RETURN(Number);
        CASE_RETURN(Space);

        CASE_RETURN(Identifier);
        CASE_RETURN(QuotedString);
        CASE_RETURN(String);

        CASE_RETURN(Invalid);
    
        CASE_RETURN(Variables);
        CASE_RETURN(Events);
#undef CASE_RETURN
    }
}

}
