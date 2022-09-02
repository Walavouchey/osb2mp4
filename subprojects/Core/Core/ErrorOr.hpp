#pragma once
#include <Core/Error.hpp>
#include <variant>
#include <optional>

namespace Core {

template <typename T, typename E = Error>
struct [[nodiscard]] ErrorOr {
    constexpr ErrorOr(T&& value)
        : m_storage(std::move(value))
    {
    }

    constexpr ErrorOr(E&& error)
        : m_storage(std::move(error))
    {
    }

    constexpr bool is_error() const
    {
        return m_storage.index() == 1;
    }

    T release_value()
    {
        return std::get<T>(std::move(m_storage));
    }

    E release_error()
    {
        return std::get<E>(std::move(m_storage));
    }

    T const& release_value() const
    {
        return std::get<T>(m_storage);
    }

    E const& error() const
    {
        return std::get<E>(m_storage);
    }

private:
    std::variant<T, E> m_storage {};
};

template <typename E>
struct ErrorOr<void, E> {
    constexpr ErrorOr() = default;

    constexpr ErrorOr(E error)
        : m_error(error)
    {
    }

    constexpr bool is_error() const
    {
        return m_error.has_value();
    }

    void release_value() const { }

    Error release_error() const
    {
        return std::move(*m_error);
    }

    E const& error() const
    {
        return *m_error;
    }

private:
    std::optional<E> m_error {};
};

}
