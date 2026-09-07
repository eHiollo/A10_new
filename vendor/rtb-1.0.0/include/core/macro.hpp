#ifndef GENERNAL_MACRO_HPP_
#define GENERNAL_MACRO_HPP_

// 原子量操作
#define ATOMIC_LOAD(ato_var) ato_var.load(std::memory_order_acquire)
#define ATOMIC_STORE(ato_var, val) ato_var.store(val, std::memory_order_release)

// HPP 中使用
#define HPP_DECLARE_BIG_FOUR(type_name)           \
    type_name(const type_name& other);            \
    type_name(type_name&& other);                 \
    type_name& operator=(const type_name& other); \
    type_name& operator=(type_name&& other)

#define HPP_DECLARE_BIG_FOUR_NOEXCEPT(type_name)  \
    type_name(const type_name& other);            \
    type_name(type_name&& other) noexcept;        \
    type_name& operator=(const type_name& other); \
    type_name& operator=(type_name&& other) noexcept

#define HPP_DEFINE_BIG_FOUR(type_name)                      \
    type_name(const type_name& other) = default;            \
    type_name(type_name&& other) = default;                 \
    type_name& operator=(const type_name& other) = default; \
    type_name& operator=(type_name&& other) = default

#define HPP_DEFINE_BIG_FOUR_NOEXCEPT(type_name)             \
    type_name(const type_name& other) = default;            \
    type_name(type_name&& other) noexcept = default;        \
    type_name& operator=(const type_name& other) = default; \
    type_name& operator=(type_name&& other) noexcept = default

#define HPP_DELETE_BIG_FOUR(type_name)                     \
    type_name(const type_name& other) = delete;            \
    type_name(type_name&& other) = delete;                 \
    type_name& operator=(const type_name& other) = delete; \
    type_name& operator=(type_name&& other) = delete

// CPP 中使用
#define CPP_DEFINE_BIG_FOUR(type_name)                                 \
    type_name::type_name(const type_name& other) = default;            \
    type_name::type_name(type_name&& other) = default;                 \
    type_name& type_name::operator=(const type_name& other) = default; \
    type_name& type_name::operator=(type_name&& other) = default

#define CPP_DEFINE_BIG_FOUR_NOEXCEPT(type_name)                        \
    type_name::type_name(const type_name& other) = default;            \
    type_name::type_name(type_name&& other) noexcept = default;        \
    type_name& type_name::operator=(const type_name& other) = default; \
    type_name& type_name::operator=(type_name&& other) noexcept = default

#endif  // GENERNAL_MACRO_HPP_