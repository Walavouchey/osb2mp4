#include <OSB/OSBFile.hpp>
#include <iostream>

namespace OSB {

Core::ErrorOr<void> FileError::show(Lexer::SourceFile source) const
{
    (void)source;
    return Core::Error::from_string_literal("unimplemented");
}

void OSBFile::dump() const
{
    std::cout << "unimplemented " << __PRETTY_FUNCTION__;
}

}
