#pragma once
#include <Core/ErrorOr.hpp>
#include <Lexer/SourceFile.hpp>
#include <OSB/Token.hpp>
#include <Types.hpp>
#include <string_view>
#include <vector>

namespace OSB {

struct LexError {
    std::string_view message {};
    u32 source_index { 0 };

    Core::ErrorOr<void> show(Lexer::SourceFile source) const;
};

using LexResult = Core::ErrorOr<Tokens, LexError>;
LexResult lex_osb(std::string_view source);

}
