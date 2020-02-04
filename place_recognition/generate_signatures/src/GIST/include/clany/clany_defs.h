/////////////////////////////////////////////////////////////////////////////////
// The MIT License(MIT)
//
// Copyright (c) 2014 Tiangang Song
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files(the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and / or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions :
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
/////////////////////////////////////////////////////////////////////////////////

#ifndef CLANY_MACROS_H
#define CLANY_MACROS_H

#define _CLANY_BEGIN namespace cls {
#define _CLANY_END }

#if defined CLANYAPI_EXPORTS
#define CLANY_EXPORTS __declspec(dllexport)
#else
#define CLANY_EXPORTS
#endif

// Define some macros for debugging use
#undef ASSERT
#undef VERIFY
#undef DBGVAR
#undef DBGMSG
#undef TRACE

#ifdef _MSC_VER
#define NOMINMAX
#define _WINSOCKAPI_
#include <windows.h>
#define ASSERT _ASSERTE
#else
#include <cassert>
#define ASSERT assert
#endif

#ifndef NDEBUG
#define VERIFY ASSERT
#define DBGVAR(os, var)                                                        \
  (os) << "Debug: " << __FILE__ << "(" << __LINE__ << "): " << #var << " = ["  \
       << (var) << "]" << std::endl
#define DBGMSG(os, msg)                                                        \
  (os) << "Debug: " << __FILE__ << "(" << __LINE__ << "): " << msg << std::endl
#define _TRACE(format, ...)                                                    \
  char buffer[256];                                                            \
  sprintf(buffer, (format), ##__VA_ARGS__);                                    \
  stringstream ss;                                                             \
  ss << "Debug: " << __FILE__ << "(" << __LINE__ << "): " << buffer << endl
#ifdef _MSC_VER
#define TRACE(format, ...)                                                     \
  {                                                                            \
    _TRACE((format), ##__VA_ARGS__);                                           \
    OutputDebugString(ss.str().c_str());                                       \
  }
#else
#define TRACE(format, ...)                                                     \
  {                                                                            \
    _TRACE((format), ##__VA_ARGS__);                                           \
    cout << ss.str();                                                          \
  }
#endif
#else
#define VERIFY(expression) (expression)
#define DBGVAR(os, var) ((void)0)
#define DBGMSG(os, msg) ((void)0)
#define TRACE(...) ((void)0)
#endif

#undef NOEXCEPT
#undef CONSTEXPR
#if !(__cplusplus >= 201103L)
#define NOEXCEPT throw()
#define CONSTEXPR const
#else
#define NOEXCEPT noexcept
#define CONSTEXPR constexpr
#endif

// Some useful typedef
using ushort = unsigned short;
using uchar = unsigned char;
using uint = unsigned int;
using ulong = unsigned long;
using ullong = unsigned long long;
using llong = long long;

// Define GCC and Clang Version
#define GCC_VERSION (__GNUC__ * 100 + __GNUC_MINOR__)
#define CLANG_VERSION (__clang_major__ * 100 + __clang_minor__)
#define CPP11_SUPPORT                                                          \
  (GCC_VERSION >= 408 || CLANG_VERSION >= 303 || _MSC_VER >= 1900)
#define CPP14_SUPPORT (GCC_VERSION >= 409 || CLANG_VERSION >= 304)

_CLANY_BEGIN
using namespace std;
_CLANY_END

#endif // CLANY_MACROS_H
