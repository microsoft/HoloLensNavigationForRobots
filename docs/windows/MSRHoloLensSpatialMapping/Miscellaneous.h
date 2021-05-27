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

#pragma once

namespace DX
{
    class StepTimer;
};

extern DX::StepTimer *  _pGlobalTimer;

namespace HoloLensNavigation
{
    void DebugMsgW(PCWSTR pwszMessage, ...);

    class CSLock
    {
    public:
        //----------------------------------------------------------
        CSLock(CRITICAL_SECTION* pcs)
        {
            m_pcs = pcs;
            EnterCriticalSection(m_pcs);
        }
        //----------------------------------------------------------
        ~CSLock()
        {
            LeaveCriticalSection(m_pcs);
        }

    private:
        CRITICAL_SECTION* m_pcs;
    };

    class StaticMemoryBuffer
    {
    public:
        StaticMemoryBuffer(void);
        ~StaticMemoryBuffer(void);

        void* realloc(unsigned int uBufferSize);
        _inline void LockRWAccess(void) { EnterCriticalSection(&m_cs); }
        _inline void UnlockRWAccess(void) { LeaveCriticalSection(&m_cs); }
        _inline void* getPointer(void) { return m_pvBuffer; }
        _inline unsigned int getBufferSize(void) { return m_uRequestedBufferSize; }

    private:
        CRITICAL_SECTION                                    m_cs;
        void* m_pvBuffer;
        unsigned int                                        m_uAllocatedBufferSize;
        unsigned int                                        m_uRequestedBufferSize;
    };

} // namespace HoloLensNavigation
