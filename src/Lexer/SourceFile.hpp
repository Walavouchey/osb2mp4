#pragma once
#include <string_view>

namespace Lexer {

struct SourceFile {
    std::string_view file_name;
    std::string_view text;
};

}
