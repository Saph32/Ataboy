// This file is public domain
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#pragma once

template<class Lambda>
class AtScopeExit {
    Lambda& m_lambda;

public:
    AtScopeExit(Lambda& action) : m_lambda(action) {}
    ~AtScopeExit() { m_lambda(); }
};

#define TOKEN_PASTEx(x, y) x##y
#define TOKEN_PASTE(x, y) TOKEN_PASTEx(x, y)

#define AT_SCOPE_EXIT_INTERNAL1(lname, aname, ...)                                                           \
    auto                         lname = [&]() { __VA_ARGS__; };                                             \
    AtScopeExit<decltype(lname)> aname(lname);

#define AT_SCOPE_EXIT_INTERNAL2(ctr, ...)                                                                    \
    AT_SCOPE_EXIT_INTERNAL1(                                                                                 \
        TOKEN_PASTE(AT_SCOPE_EXIT_func_, ctr), TOKEN_PASTE(AT_SCOPE_EXIT_instance_, ctr), __VA_ARGS__)

#define AT_SCOPE_EXIT(...) AT_SCOPE_EXIT_INTERNAL2(__COUNTER__, __VA_ARGS__)