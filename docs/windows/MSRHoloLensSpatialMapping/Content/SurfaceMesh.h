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

#include "Common\DeviceResources.h"
#include "ShaderStructures.h"
#include <ppltasks.h>

using namespace Windows::Storage::Streams;

namespace HoloLensNavigation
{
    #define SMSH_SIGNATURE      "SMSHSIG_"

    struct SurfaceMeshStreamHeader
    {
        unsigned char   signature[8];
        float           scale;
        float           center_x;
        float           center_y;
        float           center_z;
        float           orientation_x;
        float           orientation_y;
        float           orientation_z;
        float           orientation_w;
        unsigned int    uVertexBufferSize;
        unsigned int    uNumVertices;
    };

    struct SurfaceMeshProperties
    {
        Windows::Perception::Spatial::SpatialCoordinateSystem^ coordinateSystem = nullptr;
        Windows::Foundation::Numerics::float3 vertexPositionScale = Windows::Foundation::Numerics::float3::one();
        unsigned int vertexStride   = 0;
        unsigned int normalStride   = 0;
        unsigned int indexCount     = 0;
        DXGI_FORMAT  indexFormat    = DXGI_FORMAT_UNKNOWN;
    };

    struct SurfaceMeshUpdateData
    {
        Windows::Perception::Spatial::Surfaces::SpatialSurfaceMesh^     surfaceMesh;
        ID3D11Device*                                                   pd3dDevice;
    };


    class SurfaceMesh final
    {
    public:
        SurfaceMesh();
        ~SurfaceMesh();

        void UpdateSurface(Windows::Perception::Spatial::Surfaces::SpatialSurfaceMesh^ surface);
        void UpdateTransform(
            ID3D11Device* device, 
            ID3D11DeviceContext* context,
            DX::StepTimer const& timer,
            Windows::Perception::Spatial::SpatialCoordinateSystem^ baseCoordinateSystem
            );

        void Draw(ID3D11Device* device, ID3D11DeviceContext* context, bool usingVprtShaders, bool isStereo);

        void UpdateVertexResources(ID3D11Device* device);
        void CreateDeviceDependentResources(ID3D11Device* device);
        void ReleaseVertexResources();
        void ReleaseDeviceDependentResources();

        const bool&                             GetIsActive()       const { return m_isActive;       }
        const float&                            GetLastActiveTime() const { return m_lastActiveTime; }
        const Windows::Foundation::DateTime&    GetLastUpdateTime() const { return m_lastUpdateTime; }

        void SetIsActive(const bool& isActive)          { m_isActive = isActive;                                    }
        void SetColorFadeTimer(const float& duration)   { m_colorFadeTimeout = duration; m_colorFadeTimer = 0.f;    }

    public:
        _inline StaticMemoryBuffer& GetVertexMemoryBuffer(void) { return m_memBuffer; }

    private:
        void SwapVertexBuffers();
        void CreateDirectXBuffer(
            ID3D11Device* device,
            D3D11_BIND_FLAG binding,
            Windows::Storage::Streams::IBuffer^ buffer,
            ID3D11Buffer** target
            );

        Windows::Perception::Spatial::Surfaces::SpatialSurfaceMesh^ m_pendingSurfaceMesh = nullptr;

        Microsoft::WRL::ComPtr<ID3D11Buffer> m_vertexPositions;
        Microsoft::WRL::ComPtr<ID3D11Buffer> m_vertexNormals;
        Microsoft::WRL::ComPtr<ID3D11Buffer> m_triangleIndices;
        Microsoft::WRL::ComPtr<ID3D11Buffer> m_updatedVertexPositions;
        Microsoft::WRL::ComPtr<ID3D11Buffer> m_updatedVertexNormals;
        Microsoft::WRL::ComPtr<ID3D11Buffer> m_updatedTriangleIndices;
        Microsoft::WRL::ComPtr<ID3D11Buffer> m_modelTransformBuffer;

        Windows::Foundation::DateTime m_lastUpdateTime;

        SurfaceMeshProperties m_meshProperties;
        SurfaceMeshProperties m_updatedMeshProperties;

        ModelNormalConstantBuffer m_constantBufferData;

        bool   m_constantBufferCreated = false;
        bool   m_loadingComplete    = false;
        bool   m_updateReady        = false;
        bool   m_isActive           = false;
        float  m_lastActiveTime     = -1.f;
        float  m_colorFadeTimer     = -1.f;
        float  m_colorFadeTimeout   = -1.f;

    private:
        Windows::Perception::Spatial::SpatialCoordinateSystem^ m_baseCoordinateSystem = nullptr;

        //
        // we'll use a memory buffer which only grows in size as needed. Because we collect
        // vertex data over and over, re-using previously allocated memory will improve
        // performance.
        StaticMemoryBuffer                                  m_memBuffer;
        SurfaceMeshStreamHeader                             m_emptySMSH;

        //
        // To increase overall performance, use a worker thread to update new vertex, normal and index buffers.
        static DWORD WINAPI _ThreadProc(LPVOID lpParameter) { return ((SurfaceMesh*)lpParameter)->UpdateVertexResourcesWorkerThread(); }
        DWORD UpdateVertexResourcesWorkerThread(void);
        void CreateUpdateVertexResourcesWorkerThread();
        void ExitUpdateVertexResourcesWorkerThread();
        void UpdateVertexResourcesTask(ID3D11Device* device, Windows::Perception::Spatial::Surfaces::SpatialSurfaceMesh^ surfaceMesh);

        HANDLE                                              m_hThread;
        DWORD                                               m_dwThreadID;
        HANDLE                                              m_hEventExitUpdateVertexResourcesWorkerThread;

        CRITICAL_SECTION                                    m_csMeshResources;
        CRITICAL_SECTION                                    m_csUpdateVertexResources;
        bool                                                m_fClosing;

        SurfaceMeshUpdateData                               m_smud;
    };

} // namespace HoloLensNavigation
