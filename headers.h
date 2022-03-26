#pragma once

#if defined(_WIN32) && !defined(_XBOX) && !defined(__HAVOK_PARSER__)
// Do a full windows incl so that Com etc included.
#    if !defined(HK_PLATFORM_WINRT) && (!defined(WINAPI_FAMILY) || (WINAPI_FAMILY == 1))
#        define _WIN32_WINNT 0x0500 // Windows2000 or higher
#        if (_MSC_VER >= 1400)
#            define _CRT_SECURE_NO_DEPRECATE  1
#            define _CRT_NONSTDC_NO_DEPRECATE 1
#        endif
#        define NOMINMAX
#        ifdef HK_ENABLE_SCRIPT
#            include <winsock2.h>
#        endif
#    endif
#    include <windows.h>
#endif

#include <Common/Base/hkBase.h>

#include <Common/Base/KeyCode.h>