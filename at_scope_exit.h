#pragma once

template <class Lambda> class AtScopeExit {
    Lambda& m_lambda;
public:
    AtScopeExit(Lambda& action) : m_lambda(action) {}
    ~AtScopeExit() { m_lambda(); }
};

#define TOKEN_PASTEx(x, y) x ## y
#define TOKEN_PASTE(x, y) TOKEN_PASTEx(x, y)

#define AT_SCOPE_EXIT_INTERNAL1(lname, aname, ...) \
    auto lname = [&]() { __VA_ARGS__; }; \
    AtScopeExit<decltype(lname)> aname(lname);

#define AT_SCOPE_EXIT_INTERNAL2(ctr, ...) \
    AT_SCOPE_EXIT_INTERNAL1(TOKEN_PASTE(AT_SCOPE_EXIT_func_, ctr), \
                   TOKEN_PASTE(AT_SCOPE_EXIT_instance_, ctr), __VA_ARGS__)

#define AT_SCOPE_EXIT(...) AT_SCOPE_EXIT_INTERNAL2(__COUNTER__, __VA_ARGS__)