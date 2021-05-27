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

#include <ppltasks.h>

#include "Common\DirectXHelper.h"
#include "Common\StepTimer.h"
#include "GetDataFromIBuffer.h"
#include "SurfaceMesh.h"

using namespace HoloLensNavigation;
using namespace DirectX;
using namespace Windows::Perception::Spatial;
using namespace Windows::Perception::Spatial::Surfaces;
using namespace Windows::Foundation::Numerics;

/***********************************************************************************************************/
/*                                                                                                         */
/***********************************************************************************************************/
SurfaceMesh::SurfaceMesh()
{
    InitializeCriticalSection(&m_csMeshResources);
    InitializeCriticalSection(&m_csUpdateVertexResources);

    CSLock lock(&m_csMeshResources);

    ReleaseDeviceDependentResources();
    m_lastUpdateTime.UniversalTime = 0;
    m_fClosing = false;

    m_lastActiveTime = static_cast<float>(_pGlobalTimer->GetTotalSeconds());

    m_smud.surfaceMesh = nullptr;
    m_smud.pd3dDevice = nullptr;

    CreateUpdateVertexResourcesWorkerThread();
}
/***********************************************************************************************************/
/*                                                                                                         */
/***********************************************************************************************************/
SurfaceMesh::~SurfaceMesh()
{
    ExitUpdateVertexResourcesWorkerThread();

    {
        CSLock lock(&m_csMeshResources);

        ReleaseDeviceDependentResources();
    }

    {
        CSLock lock(&m_csUpdateVertexResources);

        m_fClosing = true;
    }

    DeleteCriticalSection(&m_csUpdateVertexResources);
    DeleteCriticalSection(&m_csMeshResources);
}
/***********************************************************************************************************/
/*                                                                                                         */
/***********************************************************************************************************/
void SurfaceMesh::UpdateSurface(SpatialSurfaceMesh ^ surfaceMesh)
{
    m_pendingSurfaceMesh = surfaceMesh;
}
/***********************************************************************************************************/
/*                                                                                                         */
/***********************************************************************************************************/
void SurfaceMesh::UpdateTransform(ID3D11Device * device,  ID3D11DeviceContext * context, DX::StepTimer const & timer, SpatialCoordinateSystem ^ baseCoordinateSystem)
{
    //
    // Spatial Mapping surface meshes each have a transform. This transform is updated every frame.

    UpdateVertexResources(device);

    {
        CSLock      csLock(&m_csMeshResources);

        m_baseCoordinateSystem = baseCoordinateSystem;

        if (m_updateReady)
        {
            // Surface mesh resources are created off-thread so that they don't affect rendering latency.
            // When a new update is ready, we should begin using the updated vertex position, normal, and 
            // index buffers.
            SwapVertexBuffers();
            m_updateReady = false;
        }
    }

    XMMATRIX transform;
    if (m_isActive)
    {
        // In this example, new surfaces are treated differently by highlighting them in a different
        // color. This allows you to observe changes in the spatial map that are due to new meshes,
        // as opposed to mesh updates.
        if (m_colorFadeTimeout > 0.f)
        {
            m_colorFadeTimer += static_cast<float>(timer.GetElapsedSeconds());
            if (m_colorFadeTimer < m_colorFadeTimeout)
            {
                float colorFadeFactor = min(1.f, m_colorFadeTimeout - m_colorFadeTimer);
                m_constantBufferData.colorFadeFactor = XMFLOAT4(colorFadeFactor, colorFadeFactor, colorFadeFactor, 1.f);
            }
            else
            {
                m_constantBufferData.colorFadeFactor = XMFLOAT4(0.f, 0.f, 0.f, 0.f);
                m_colorFadeTimer = m_colorFadeTimeout = -1.f;
            }
        }

        // The transform is updated relative to a SpatialCoordinateSystem. In the SurfaceMesh class, we
        // expect to be given the same SpatialCoordinateSystem that will be used to generate view
        // matrices, because this class uses the surface mesh for rendering.
        // Other applications could potentially involve using a SpatialCoordinateSystem from a stationary
        // reference frame that is being used for physics simulation, etc.
        auto tryTransform = m_meshProperties.coordinateSystem ? m_meshProperties.coordinateSystem->TryGetTransformTo(baseCoordinateSystem) : nullptr;
        if (tryTransform != nullptr)
        {
            // If the transform can be acquired, this spatial mesh is valid right now and
            // we have the information we need to draw it this frame.
            transform = XMLoadFloat4x4(&tryTransform->Value);
            m_lastActiveTime = static_cast<float>(timer.GetTotalSeconds());
        }
        else
        {
            // If the transform cannot be acquired, the spatial mesh is not valid right now
            // because its location cannot be correlated to the current space.
            m_isActive = false;
        }
    }

    if (!m_isActive)
    {
        // If for any reason the surface mesh is not active this frame - whether because
        // it was not included in the observer's collection, or because its transform was
        // not located - we don't have the information we need to update it.
        return;
    }

    // Set up a transform from surface mesh space, to world space.
    XMMATRIX scaleTransform = XMMatrixScalingFromVector(XMLoadFloat3(&m_meshProperties.vertexPositionScale));

    XMStoreFloat4x4(&m_constantBufferData.modelToWorld, XMMatrixTranspose(scaleTransform * transform));

    // Surface meshes come with normals, which are also transformed from surface mesh space, to world space.
    XMMATRIX normalTransform = transform;

    // Normals are not translated, so we remove the translation component here.
    normalTransform.r[3] = XMVectorSet(0.f, 0.f, 0.f, XMVectorGetW(normalTransform.r[3]));
    XMStoreFloat4x4(&m_constantBufferData.normalToWorld, XMMatrixTranspose(normalTransform));

    if (!m_constantBufferCreated)
    {
        // If loading is not yet complete, we cannot actually update the graphics resources.
        // This return is intentionally placed after the surface mesh updates so that this
        // code may be copied and re-used for CPU-based processing of surface data.
        CreateDeviceDependentResources(device);
        return;
    }

    context->UpdateSubresource(m_modelTransformBuffer.Get(), 0, NULL, &m_constantBufferData, 0, 0);
}
/***********************************************************************************************************/
/*                                                                                                         */
/***********************************************************************************************************/
void SurfaceMesh::Draw(ID3D11Device* device, ID3D11DeviceContext* context, bool usingVprtShaders, bool isStereo)
{
    //
    // Does an indexed, instanced draw call after setting the IA stage to use the mesh's geometry, and
    // after setting up the constant buffer for the surface mesh.
    // The caller is responsible for the rest of the shader pipeline.

    if (!m_constantBufferCreated || !m_loadingComplete)
    {
        // Resources are still being initialized.
        return;
    }
    
    if (!m_isActive)
    {
        // Mesh is not active this frame, and should not be drawn.
        return;
    }

    // The vertices are provided in {vertex, normal} format

    UINT strides [] = { m_meshProperties.vertexStride, m_meshProperties.normalStride };
    UINT offsets [] = { 0, 0 };
    ID3D11Buffer* buffers [] = { m_vertexPositions.Get(), m_vertexNormals.Get() };

    context->IASetVertexBuffers(0, ARRAYSIZE(buffers), buffers, strides, offsets);
    context->IASetIndexBuffer(m_triangleIndices.Get(), m_meshProperties.indexFormat, 0);
    context->VSSetConstantBuffers(0, 1, m_modelTransformBuffer.GetAddressOf());

    if (!usingVprtShaders)
    {
        context->GSSetConstantBuffers(0, 1, m_modelTransformBuffer.GetAddressOf());
    }

    context->PSSetConstantBuffers(0, 1, m_modelTransformBuffer.GetAddressOf());

    context->DrawIndexedInstanced(
        m_meshProperties.indexCount,       // Index count per instance.
        isStereo ? 2 : 1,   // Instance count.
        0,                  // Start index location.
        0,                  // Base vertex location.
        0                   // Start instance location.
        );
}
/***********************************************************************************************************/
/*                                                                                                         */
/***********************************************************************************************************/
void SurfaceMesh::CreateDirectXBuffer(ID3D11Device * device, D3D11_BIND_FLAG binding, IBuffer ^ buffer, ID3D11Buffer ** target)
{
    auto length = buffer->Length;

    CD3D11_BUFFER_DESC bufferDescription(buffer->Length, binding);
    D3D11_SUBRESOURCE_DATA bufferBytes = { GetDataFromIBuffer(buffer), 0, 0 };
    device->CreateBuffer(&bufferDescription, &bufferBytes, target);
}
/***********************************************************************************************************/
/*                                                                                                         */
/***********************************************************************************************************/
void SurfaceMesh::CreateUpdateVertexResourcesWorkerThread()
{
    HRESULT         hr;

    m_hEventExitUpdateVertexResourcesWorkerThread = CreateEventW(NULL, TRUE, FALSE, NULL);
    if (!m_hEventExitUpdateVertexResourcesWorkerThread) {
        hr = HRESULT_FROM_WIN32(GetLastError());
    }

    m_hThread = CreateThread(NULL, 0, _ThreadProc, this, 0, &m_dwThreadID);
    if (!m_hThread) {
        hr = HRESULT_FROM_WIN32(GetLastError());
    }
}
/***********************************************************************************************************/
/*                                                                                                         */
/***********************************************************************************************************/
void SurfaceMesh::ExitUpdateVertexResourcesWorkerThread()
{
    HRESULT         hr = S_OK;
    DWORD           dwTimeout = INFINITE;
    DWORD           rc;

    if ((!m_hEventExitUpdateVertexResourcesWorkerThread) || (!m_hThread))
        return;

    rc = SignalObjectAndWait(m_hEventExitUpdateVertexResourcesWorkerThread, m_hThread, dwTimeout, FALSE);
    switch (rc) {
        case WAIT_OBJECT_0:
            hr = S_OK;
            break;

        case WAIT_ABANDONED:
        case WAIT_IO_COMPLETION:
        case WAIT_TIMEOUT:
            hr = E_FAIL;
            break;

        case WAIT_FAILED:
            hr = HRESULT_FROM_WIN32(GetLastError());
            break;
    }

    CloseHandle(m_hThread);
    m_hThread = nullptr;

    CloseHandle(m_hEventExitUpdateVertexResourcesWorkerThread);
    m_hEventExitUpdateVertexResourcesWorkerThread = nullptr;
}
/***********************************************************************************************************/
/*                                                                                                         */
/***********************************************************************************************************/
DWORD SurfaceMesh::UpdateVertexResourcesWorkerThread(void)
{
    const DWORD dwSleep = 0;

    for (;;) {
        if (WaitForSingleObject(m_hEventExitUpdateVertexResourcesWorkerThread, dwSleep) == WAIT_OBJECT_0) {
            break;
        }

        SpatialSurfaceMesh^ surfaceMesh = nullptr;
        ID3D11Device* device = nullptr;

        EnterCriticalSection(&m_csMeshResources);
        if (m_smud.surfaceMesh != nullptr) {
            surfaceMesh = std::move(m_smud.surfaceMesh);
            device = m_smud.pd3dDevice;

            m_smud.surfaceMesh = nullptr;
            m_smud.pd3dDevice = nullptr;
        }
        LeaveCriticalSection(&m_csMeshResources);

        if (surfaceMesh && device) {
            UpdateVertexResourcesTask(device, surfaceMesh);
            surfaceMesh = nullptr;
        }

        Sleep(1);
    }

    return 0;
}
/***********************************************************************************************************/
/*                                                                                                         */
/***********************************************************************************************************/
inline byte* GetPointerToBufferData(IBuffer^ buffer)
{
    ComPtr<IInspectable> insp(reinterpret_cast<IInspectable*>(buffer));

    // Query the IBufferByteAccess interface.
    ComPtr<IBufferByteAccess> bufferByteAccess;
    insp.As(&bufferByteAccess);

    byte* pixels = nullptr;
    bufferByteAccess->Buffer(&pixels);
    return pixels;
}
/***********************************************************************************************************/
/*                                                                                                         */
/***********************************************************************************************************/
void SurfaceMesh::UpdateVertexResourcesTask(ID3D11Device* device, Windows::Perception::Spatial::Surfaces::SpatialSurfaceMesh^ surfaceMesh)
{
    // Before updating the meshes, check to ensure that there wasn't a more recent update.
    auto meshUpdateTime = surfaceMesh->SurfaceInfo->UpdateTime;

    if (meshUpdateTime.UniversalTime <= m_lastUpdateTime.UniversalTime)
        return;

    // Create new Direct3D device resources for the updated buffers. These will be set aside
    // for now, and then swapped into the active slot next time the render loop is ready to draw.

    // First, we acquire the raw data buffers.
    Windows::Storage::Streams::IBuffer^ positions = surfaceMesh->VertexPositions->Data;
    Windows::Storage::Streams::IBuffer^ normals = surfaceMesh->VertexNormals->Data;
    Windows::Storage::Streams::IBuffer^ indices = surfaceMesh->TriangleIndices->Data;

    // Then, we create Direct3D device buffers with the mesh data provided by HoloLens.
    Microsoft::WRL::ComPtr<ID3D11Buffer> updatedVertexPositions;
    Microsoft::WRL::ComPtr<ID3D11Buffer> updatedVertexNormals;
    Microsoft::WRL::ComPtr<ID3D11Buffer> updatedTriangleIndices;

    CreateDirectXBuffer(device, D3D11_BIND_VERTEX_BUFFER, positions, updatedVertexPositions.GetAddressOf());
    CreateDirectXBuffer(device, D3D11_BIND_VERTEX_BUFFER, normals, updatedVertexNormals.GetAddressOf());
    CreateDirectXBuffer(device, D3D11_BIND_INDEX_BUFFER, indices, updatedTriangleIndices.GetAddressOf());

    {
        CSLock lock(&m_csMeshResources);

        // Prepare to swap in the new meshes.
        // Here, we use ComPtr.Swap() to avoid unnecessary overhead from ref counting.
        m_updatedVertexPositions.Swap(updatedVertexPositions);
        m_updatedVertexNormals.Swap(updatedVertexNormals);
        m_updatedTriangleIndices.Swap(updatedTriangleIndices);

        // Cache properties for the buffers we will now use.
        m_updatedMeshProperties.coordinateSystem = surfaceMesh->CoordinateSystem;
        m_updatedMeshProperties.vertexPositionScale = surfaceMesh->VertexPositionScale;
        m_updatedMeshProperties.vertexStride = surfaceMesh->VertexPositions->Stride;
        m_updatedMeshProperties.normalStride = surfaceMesh->VertexNormals->Stride;
        m_updatedMeshProperties.indexCount = surfaceMesh->TriangleIndices->ElementCount;
        m_updatedMeshProperties.indexFormat = static_cast<DXGI_FORMAT>(surfaceMesh->TriangleIndices->Format);

        // Send a signal to the render loop indicating that new resources are available to use.
        m_updateReady = true;
        m_lastUpdateTime = meshUpdateTime;
        m_loadingComplete = true;

        //
        // retrieving and saving vertex data from the GPU is expensive. When debugging and 
        // to ease CPU and GPU impact, disable the below block if not part of investigation.
#ifndef DISABLE_SAVE_VERTEX_DATA
        //
        // save vertex data information
        bool fSuccess = false; // pessimistic
        auto bound = surfaceMesh->SurfaceInfo->TryGetBounds(m_baseCoordinateSystem);
        if (bound != nullptr) {
            //
            // get the raw vertex information
            unsigned int    uVertexBufferSize = positions->Length;
            unsigned int    uNormalBufferSize = normals->Length;
            unsigned int    uIndexBufferSize = indices->Length;

            unsigned int    uBufferSize = sizeof(SurfaceMeshStreamHeader) + uVertexBufferSize;

            //
            // TODO: add normals and indices as needed
            // uBufferSize += (uNormalBufferSize + uIndexBufferSize);

            m_memBuffer.LockRWAccess();

            unsigned char* ptr = (unsigned char*)m_memBuffer.realloc(uBufferSize);

            if (ptr) {
                SurfaceMeshStreamHeader* hdr = (SurfaceMeshStreamHeader*)ptr;

                memcpy(hdr->signature, (void*)SMSH_SIGNATURE, sizeof(hdr->signature));

                hdr->scale = 1.26f / 504.0f;
                hdr->center_x = bound->Value.Center.x;
                hdr->center_y = bound->Value.Center.y;
                hdr->center_z = bound->Value.Center.z;
                hdr->orientation_x = bound->Value.Orientation.x;
                hdr->orientation_y = bound->Value.Orientation.y;
                hdr->orientation_z = bound->Value.Orientation.z;
                hdr->orientation_w = bound->Value.Orientation.w;

                //
                // copy vertices
                hdr->uVertexBufferSize = uVertexBufferSize;
                hdr->uNumVertices = (uVertexBufferSize / sizeof(short)) / 4;    // one vertex equals to 4 positions of type short
                memcpy(ptr + sizeof(SurfaceMeshStreamHeader), GetPointerToBufferData(positions), uVertexBufferSize);

                //
                // TODO: copy normals and indices as needed
                //hdr->uNormalBufferSize = uNormalBufferSize;
                //hdr->uNumNormals = (uNormalBufferSize / sizeof(char));
                // memcpy(ptr + sizeof(SurfaceMeshStreamHeader) + uVertexBufferSize, GetPointerToBufferData(normals), uNormalBufferSize);

                //hdr->uIndexBufferSize = uIndexBufferSize;
                //hdr->uNumIndices = (uIndexBufferSize / sizeof(short));
                // memcpy(ptr + sizeof(SurfaceMeshStreamHeader) + uVertexBufferSize, GetPointerToBufferData(indices), uIndexBufferSize);

                fSuccess = true;
            }

            m_memBuffer.UnlockRWAccess();
        }
#else // #ifndef DISABLE_SAVE_VERTEX_DATA
        #pragma message("====================== saving vertex data disabled ======================")
#endif // #ifndef DISABLE_SAVE_VERTEX_DATA
    }
}
/***********************************************************************************************************/
/*                                                                                                         */
/***********************************************************************************************************/
void SurfaceMesh::UpdateVertexResources(ID3D11Device * device)
{
    SpatialSurfaceMesh^ surfaceMesh = std::move(m_pendingSurfaceMesh);

    if (!surfaceMesh || surfaceMesh->TriangleIndices->ElementCount < 3)
    {
        // Not enough indices to draw a triangle.
        return;
    }

    CSLock          lock(&m_csMeshResources);

    //
    // update data block for worker thread to pick up
    m_smud.surfaceMesh = surfaceMesh; //  std::move(surfaceMesh);
    m_smud.pd3dDevice = device;
}
/***********************************************************************************************************/
/*                                                                                                         */
/***********************************************************************************************************/
void SurfaceMesh::CreateDeviceDependentResources(ID3D11Device * device)
{
    UpdateVertexResources(device);

    // Create a constant buffer to control mesh position.
    CD3D11_BUFFER_DESC constantBufferDesc(sizeof(ModelNormalConstantBuffer), D3D11_BIND_CONSTANT_BUFFER);
    DX::ThrowIfFailed(
        device->CreateBuffer(&constantBufferDesc, nullptr, &m_modelTransformBuffer
        )
    );

    m_constantBufferCreated = true;
}
/***********************************************************************************************************/
/*                                                                                                         */
/***********************************************************************************************************/
void SurfaceMesh::ReleaseVertexResources()
{
    m_pendingSurfaceMesh = nullptr;

    m_meshProperties = {};
    m_vertexPositions.Reset();
    m_vertexNormals.Reset();
    m_triangleIndices.Reset();
}
/***********************************************************************************************************/
/*                                                                                                         */
/***********************************************************************************************************/
void SurfaceMesh::SwapVertexBuffers()
{
    // Swap out the previous vertex position, normal, and index buffers, and replace
    // them with up-to-date buffers.
    m_vertexPositions = m_updatedVertexPositions;
    m_vertexNormals   = m_updatedVertexNormals;
    m_triangleIndices = m_updatedTriangleIndices;

    // Swap out the metadata: index count, index format, .
    m_meshProperties  = m_updatedMeshProperties;
    
    m_updatedMeshProperties = {};
    m_updatedVertexPositions.Reset();
    m_updatedVertexNormals.Reset();
    m_updatedTriangleIndices.Reset();
}
/***********************************************************************************************************/
/*                                                                                                         */
/***********************************************************************************************************/
void SurfaceMesh::ReleaseDeviceDependentResources()
{
    // Clear out any pending resources.
    SwapVertexBuffers();

    // Clear out active resources.
    ReleaseVertexResources();

    m_modelTransformBuffer.Reset();

    m_constantBufferCreated = false;
    m_loadingComplete = false;
}
