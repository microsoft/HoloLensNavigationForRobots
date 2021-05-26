//*********************************************************
//
// Copyright (c) Microsoft. All rights reserved.
// This code is licensed under the MIT License (MIT).
// THIS CODE IS PROVIDED *AS IS* WITHOUT WARRANTY OF
// ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING ANY
// IMPLIED WARRANTIES OF FITNESS FOR A PARTICULAR
// PURPOSE, MERCHANTABILITY, OR NON-INFRINGEMENT.
//
//*********************************************************

#include "pch.h"
#include <StrSafe.h>

using namespace HoloLensNavigation;

DX::StepTimer *     _pGlobalTimer = nullptr;

/*************************************************************************************************/
/*                                                                                               */
/*************************************************************************************************/
void HoloLensNavigation::DebugMsgW(PCWSTR pwszMessage, ...)
{
    WCHAR       ach[2048];
    va_list     vArgs;

    va_start(vArgs, pwszMessage);

    StringCchVPrintfW(ach, _countof(ach), pwszMessage, vArgs);

    va_end(vArgs);

    StringCchCatW(ach, _countof(ach), L"\r\n");

    OutputDebugStringW(ach);
}
/***********************************************************************************************************/
/*                                                                                                         */
/***********************************************************************************************************/
StaticMemoryBuffer::StaticMemoryBuffer(void)
{
    InitializeCriticalSection(&m_cs);
    m_pvBuffer = nullptr;
    m_uAllocatedBufferSize = 0;
    m_uRequestedBufferSize = 0;
}
/***********************************************************************************************************/
/*                                                                                                         */
/***********************************************************************************************************/
StaticMemoryBuffer::~StaticMemoryBuffer(void)
{
    LockRWAccess();
    if (m_pvBuffer) {
        free(m_pvBuffer);
        m_pvBuffer = nullptr;
    }
    m_uAllocatedBufferSize = 0;
    m_uRequestedBufferSize = 0;
    UnlockRWAccess();

    DeleteCriticalSection(&m_cs);
}
/***********************************************************************************************************/
/*                                                                                                         */
/***********************************************************************************************************/
void* StaticMemoryBuffer::realloc(unsigned int uBufferSize)
{
    LockRWAccess();

    if (m_uAllocatedBufferSize < uBufferSize) {
        if (m_pvBuffer == nullptr)
            m_pvBuffer = ::malloc(uBufferSize);
        else
            m_pvBuffer = ::realloc(m_pvBuffer, uBufferSize);

        //
        // out of memory?
        if (m_pvBuffer == nullptr) {
            m_uAllocatedBufferSize = 0;
            m_uRequestedBufferSize = 0;
            UnlockRWAccess();
            return nullptr;
        }

        m_uAllocatedBufferSize = uBufferSize;
    }

    m_uRequestedBufferSize = uBufferSize;

    UnlockRWAccess();

    return m_pvBuffer;
}
