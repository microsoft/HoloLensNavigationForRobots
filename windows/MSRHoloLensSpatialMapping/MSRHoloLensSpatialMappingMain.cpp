﻿//*********************************************************
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
#include "MSRHololensSpatialMappingMain.h"
#include "Common\DirectXHelper.h"

#include <windows.graphics.directx.direct3d11.interop.h>
#include <Collection.h>

using namespace HoloLensNavigation;

using namespace concurrency;
using namespace Microsoft::WRL;
using namespace Platform;
using namespace Windows::Foundation;
using namespace Windows::Foundation::Collections;
using namespace Windows::Foundation::Numerics;
using namespace Windows::Graphics::DirectX;
using namespace Windows::Graphics::DirectX::Direct3D11;
using namespace Windows::Graphics::Holographic;
using namespace Windows::Perception::Spatial;
using namespace Windows::Perception::Spatial::Surfaces;
using namespace Windows::UI::Input::Spatial;
using namespace std::placeholders;

/***********************************************************************************************************/
/*                                                                                                         */
/***********************************************************************************************************/
HoloLensNavigationMain::HoloLensNavigationMain(const std::shared_ptr<DX::DeviceResources>& deviceResources) :
    m_deviceResources(deviceResources)
{
    //
    // Loads and initializes application assets when the application is loaded.

    _pGlobalTimer = &m_timer;

    m_fVerbose = false;

    m_deviceResources->RegisterDeviceNotify(this);

    //
    // initialize vertices array
    // m_vert = ref new Platform::Array<unsigned char>(1);

    //
    // set up socket listener
    m_listenercontext = ref new ListenerContext(this);

    m_listener = StartListener(ref new ListenerContext::OnConnectionEvent(m_listenercontext, &ListenerContext::OnConnection), PORT);
}
/***********************************************************************************************************/
/*                                                                                                         */
/***********************************************************************************************************/
HoloLensNavigationMain::~HoloLensNavigationMain()
{
    // Deregister device notification.
    m_deviceResources->RegisterDeviceNotify(nullptr);

    UnregisterHolographicEventHandlers();
}
/***********************************************************************************************************/
/*                                                                                                         */
/***********************************************************************************************************/
void HoloLensNavigationMain::SetHolographicSpace(HolographicSpace^ holographicSpace)
{
    UnregisterHolographicEventHandlers();

    m_holographicSpace = holographicSpace;

    m_meshRenderer = std::make_unique<RealtimeSurfaceMeshRenderer>(m_deviceResources);

    m_spatialInputHandler = std::make_unique<SpatialInputHandler>();

    // Use the default SpatialLocator to track the motion of the device.
    m_locator = SpatialLocator::GetDefault();

    m_locatabilityChangedToken = m_locator->LocatabilityChanged +=
        ref new Windows::Foundation::TypedEventHandler<SpatialLocator^, Object^>(
            std::bind(&HoloLensNavigationMain::OnLocatabilityChanged, this, _1, _2)
            );

    // respond to changes in the positional tracking state by cancelling deactivation 
    // of positional tracking.
    m_positionalTrackingDeactivatingToken = m_locator->PositionalTrackingDeactivating +=
        ref new Windows::Foundation::TypedEventHandler<SpatialLocator^, SpatialLocatorPositionalTrackingDeactivatingEventArgs^>(
            std::bind(&HoloLensNavigationMain::OnPositionalTrackingDeactivating, this, _1, _2));

    // Respond to camera added events by creating any resources that are specific
    // to that camera, such as the back buffer render target view.
    // When we add an event handler for CameraAdded, the API layer will avoid putting
    // the new camera in new HolographicFrames until we complete the deferral we created
    // for that handler, or return from the handler without creating a deferral. This
    // allows the app to take more than one frame to finish creating resources and
    // loading assets for the new holographic camera.
    // This function should be registered before the app creates any HolographicFrames.
    m_cameraAddedToken = m_holographicSpace->CameraAdded +=
        ref new Windows::Foundation::TypedEventHandler<HolographicSpace^, HolographicSpaceCameraAddedEventArgs^>(
            std::bind(&HoloLensNavigationMain::OnCameraAdded, this, _1, _2));

    // Respond to camera removed events by releasing resources that were created for that
    // camera.
    // When the app receives a CameraRemoved event, it releases all references to the back
    // buffer right away. This includes render target views, Direct2D target bitmaps, and so on.
    // The app must also ensure that the back buffer is not attached as a render target, as
    // shown in DeviceResources::ReleaseResourcesForBackBuffer.
    m_cameraRemovedToken = m_holographicSpace->CameraRemoved +=
        ref new Windows::Foundation::TypedEventHandler<HolographicSpace^, HolographicSpaceCameraRemovedEventArgs^>(
            std::bind(&HoloLensNavigationMain::OnCameraRemoved, this, _1, _2));

    // This code sample uses a DeviceAttachedFrameOfReference to have the Spatial Mapping surface observer
    // follow along with the device's location.
    m_referenceFrame = m_locator->CreateAttachedFrameOfReferenceAtCurrentHeading();

    // Notes on spatial tracking APIs:
    // * Stationary reference frames are designed to provide a best-fit position relative to the
    //   overall space. Individual positions within that reference frame are allowed to drift slightly
    //   as the device learns more about the environment.
    // * When precise placement of individual holograms is required, a SpatialAnchor should be used to
    //   anchor the individual hologram to a position in the real world - for example, a point the user
    //   indicates to be of special interest. Anchor positions do not drift, but can be corrected; the
    //   anchor will use the corrected position starting in the next frame after the correction has
    //   occurred.

    // Create a frame of reference that remains stationary relative to the user's surroundings, 
    // with its initial origin at the SpatialLocator's current location.
    m_stationaryReferenceFrame = m_locator->CreateStationaryFrameOfReferenceAtCurrentLocation();

    // Create a spatial anchor helper
    m_spatialAnchorHelper = std::shared_ptr<SpatialAnchorHelper>(new SpatialAnchorHelper(nullptr));
}
/***********************************************************************************************************/
/*                                                                                                         */
/***********************************************************************************************************/
void HoloLensNavigationMain::UnregisterHolographicEventHandlers()
{
    if (m_holographicSpace != nullptr)
    {
        // Clear previous event registrations.

        if (m_cameraAddedToken.Value != 0)
        {
            m_holographicSpace->CameraAdded -= m_cameraAddedToken;
            m_cameraAddedToken.Value = 0;
        }

        if (m_cameraRemovedToken.Value != 0)
        {
            m_holographicSpace->CameraRemoved -= m_cameraRemovedToken;
            m_cameraRemovedToken.Value = 0;
        }
    }

    if (m_locator != nullptr)
    {
        m_locator->LocatabilityChanged -= m_locatabilityChangedToken;
        m_locator->PositionalTrackingDeactivating -= m_positionalTrackingDeactivatingToken;
    }

    if (m_surfaceObserver != nullptr)
    {
        m_surfaceObserver->ObservedSurfacesChanged -= m_surfacesChangedToken;
    }
}
/***********************************************************************************************************/
/*                                                                                                         */
/***********************************************************************************************************/
void HoloLensNavigationMain::OnSurfacesChanged(SpatialSurfaceObserver^ sender, Object^ args)
{
    IMapView<Guid, SpatialSurfaceInfo^>^ const& surfaceCollection = sender->GetObservedSurfaces();

    // Process surface adds and updates.
    for (const auto& pair : surfaceCollection)
    {
        auto id = pair->Key;
        auto surfaceInfo = pair->Value;

        // Choose whether to add, or update the surface.
        // In this example, new surfaces are treated differently by highlighting them in a different
        // color. This allows you to observe changes in the spatial map that are due to new meshes,
        // as opposed to mesh updates.
        // In your app, you might choose to process added surfaces differently than updated
        // surfaces. For example, you might prioritize processing of added surfaces, and
        // defer processing of updates to existing surfaces.
        if (m_meshRenderer->HasSurface(id))
        {
            if (m_meshRenderer->GetLastUpdateTime(id).UniversalTime < surfaceInfo->UpdateTime.UniversalTime)
            {
                // Update existing surface.
                m_meshRenderer->UpdateSurface(id, surfaceInfo);
            }
        }
        else
        {
            // New surface.
            m_meshRenderer->AddSurface(id, surfaceInfo);
        }
    }

    // Sometimes, a mesh will fall outside the area that is currently visible to
    // the surface observer. In this code sample, we "sleep" any meshes that are
    // not included in the surface collection to avoid rendering them.
    // The system can including them in the collection again later, in which case
    // they will no longer be hidden.
    m_meshRenderer->HideInactiveMeshes(surfaceCollection);
}
/***********************************************************************************************************/
/*                                                                                                         */
/***********************************************************************************************************/
HolographicFrame^ HoloLensNavigationMain::Update()
{
    //
    // Updates the application state once per frame.

    // Before doing the timer update, there is some work to do per-frame
    // to maintain holographic rendering. First, we will get information
    // about the current frame.

    // The HolographicFrame has information that the app needs in order
    // to update and render the current frame. The app begins each new
    // frame by calling CreateNextFrame.
    HolographicFrame^ holographicFrame = m_holographicSpace->CreateNextFrame();

    // Get a prediction of where holographic cameras will be when this frame
    // is presented.
    HolographicFramePrediction^ prediction = holographicFrame->CurrentPrediction;

    // Back buffers can change from frame to frame. Validate each buffer, and recreate
    // resource views and depth buffers as needed.
    m_deviceResources->EnsureCameraResources(holographicFrame, prediction);

    // Next, we get a coordinate system from the attached frame of reference that is
    // associated with the current frame. Later, this coordinate system is used for
    // for creating the stereo view matrices when rendering the sample content.
    SpatialCoordinateSystem^ currentCoordinateSystem = m_referenceFrame->GetStationaryCoordinateSystemAtTimestamp(prediction->Timestamp);

    if (!InternalUpdate(holographicFrame))
        return holographicFrame;

    // Only create a surface observer when you need to - do not create a new one each frame.
    if (m_surfaceObserver == nullptr)
    {
        // Initialize the Surface Observer using a valid coordinate system.
        if (!m_spatialPerceptionAccessRequested)
        {
            // The spatial mapping API reads information about the user's environment. The user must
            // grant permission to the app to use this capability of the Windows Holographic device.
            auto initSurfaceObserverTask = create_task(SpatialSurfaceObserver::RequestAccessAsync());
            initSurfaceObserverTask.then([this, currentCoordinateSystem](Windows::Perception::Spatial::SpatialPerceptionAccessStatus status)
            {
                switch (status)
                {
                    case SpatialPerceptionAccessStatus::Allowed:
                        m_surfaceAccessAllowed = true;
                        break;
                    default:
                        // Access was denied. This usually happens because your AppX manifest file does not declare the
                        // spatialPerception capability.
                        // For info on what else can cause this, see: http://msdn.microsoft.com/library/windows/apps/mt621422.aspx
                        m_surfaceAccessAllowed = false;
                        break;
                }
            });

            m_spatialPerceptionAccessRequested = true;
        }
    }

    if (m_surfaceAccessAllowed)
    {
        SpatialBoundingBox axisAlignedBoundingBox =
        {
            {  0.f,  0.f, 0.f },
            { 20.f, 20.f, 5.f },
        };
        SpatialBoundingVolume^ bounds = SpatialBoundingVolume::FromBox(currentCoordinateSystem, axisAlignedBoundingBox);

        // If status is Allowed, we can create the surface observer.
        if (m_surfaceObserver == nullptr)
        {
            // First, we'll set up the surface observer to use our preferred data formats.
            // In this example, a "preferred" format is chosen that is compatible with our precompiled shader pipeline.
            m_surfaceMeshOptions = ref new SpatialSurfaceMeshOptions();
            IVectorView<DirectXPixelFormat>^ supportedVertexPositionFormats = m_surfaceMeshOptions->SupportedVertexPositionFormats;
            unsigned int formatIndex = 0;
            if (supportedVertexPositionFormats->IndexOf(DirectXPixelFormat::R16G16B16A16IntNormalized, &formatIndex))
            {
                m_surfaceMeshOptions->VertexPositionFormat = DirectXPixelFormat::R16G16B16A16IntNormalized;
            }
            IVectorView<DirectXPixelFormat>^ supportedVertexNormalFormats = m_surfaceMeshOptions->SupportedVertexNormalFormats;
            if (supportedVertexNormalFormats->IndexOf(DirectXPixelFormat::R8G8B8A8IntNormalized, &formatIndex))
            {
                m_surfaceMeshOptions->VertexNormalFormat = DirectXPixelFormat::R8G8B8A8IntNormalized;
            }

            // If you are using a very high detail setting with spatial mapping, it can be beneficial
            // to use a 32-bit unsigned integer format for indices instead of the default 16-bit. 
            // Uncomment the following code to enable 32-bit indices.
            //IVectorView<DirectXPixelFormat>^ supportedTriangleIndexFormats = m_surfaceMeshOptions->SupportedTriangleIndexFormats;
            //if (supportedTriangleIndexFormats->IndexOf(DirectXPixelFormat::R32UInt, &formatIndex))
            //{
            //    m_surfaceMeshOptions->TriangleIndexFormat = DirectXPixelFormat::R32UInt;
            //}

            // Create the observer.
            m_surfaceObserver = ref new SpatialSurfaceObserver();
            if (m_surfaceObserver)
            {
                m_surfaceObserver->SetBoundingVolume(bounds);

                // If the surface observer was successfully created, we can initialize our
                // collection by pulling the current data set.
                auto mapContainingSurfaceCollection = m_surfaceObserver->GetObservedSurfaces();
                for (auto const& pair : mapContainingSurfaceCollection)
                {
                    // Store the ID and metadata for each surface.
                    auto const& id = pair->Key;
                    auto const& surfaceInfo = pair->Value;
                    m_meshRenderer->AddSurface(id, surfaceInfo);
                }

                // We then subcribe to an event to receive up-to-date data.
                m_surfacesChangedToken = m_surfaceObserver->ObservedSurfacesChanged +=
                    ref new TypedEventHandler<SpatialSurfaceObserver^, Platform::Object^>(bind(&HoloLensNavigationMain::OnSurfacesChanged, this, _1, _2));
            }
        }

        // Keep the surface observer positioned at the device's location.
        m_surfaceObserver->SetBoundingVolume(bounds);

        // Note that it is possible to set multiple bounding volumes. Pseudocode:
        //     m_surfaceObserver->SetBoundingVolumes(/* iterable collection of bounding volumes*/);
        //
        // It is also possible to use other bounding shapes - such as a view frustum. Pseudocode:
        //     SpatialBoundingVolume^ bounds = SpatialBoundingVolume::FromFrustum(coordinateSystem, viewFrustum);
        //     m_surfaceObserver->SetBoundingVolume(bounds);
    }

    // Check for new input state since the last frame.
    SpatialInteractionSourceState^ pointerState = m_spatialInputHandler->CheckForInput();
    if (pointerState != nullptr)
    {
        // When a Pressed gesture is detected, the rendering mode will be changed to wireframe.
        m_drawWireframe = !m_drawWireframe;
    }

    m_timer.Tick([&] ()
    {
#ifdef TODO
        if (m_baseAnchor != nullptr) {
            const auto tryTransform = m_baseAnchor->CoordinateSystem->TryGetTransformTo(stationaryCoordinateSystem);
            m_meshRenderer->Update(m_timer, m_baseAnchor->CoordinateSystem);
        }
        else
#endif // #ifdef TODO
        {
            m_meshRenderer->Update(m_timer, currentCoordinateSystem);
        }
    });

    //
    // collecting vertex data is expensive. When debugging and to ease CPU and GPU 
    // impact, disable the CollectSpatialMappinginformation() call if not part of 
    // investigation.
#ifndef DISABLE_COLLECTING_SPATIAL_MAPPING_DATA
    CollectSpatialMappinginformation();
#else // #ifndef DISABLE_COLLECTING_SPATIAL_MAPPING_DATA
    #pragma message("====================== CollectSpatialMappinginformation() disabled ======================")
#endif // #ifndef DISABLE_COLLECTING_SPATIAL_MAPPING_DATA

    // This code uses default image stabilization settings, and does not set the focus point.

    // The holographic frame will be used to get up-to-date view and projection matrices and
    // to present the swap chain.
    return holographicFrame;
}
/***********************************************************************************************************/
/*                                                                                                         */
/***********************************************************************************************************/
bool HoloLensNavigationMain::Render(HolographicFrame^ holographicFrame)
{
    //
    // Renders the current frame to each holographic camera, according to the
    // current application and spatial positioning state. Returns true if the
    // frame was rendered to at least one camera.

    // Don't try to render anything before the first Update.
    if (m_timer.GetFrameCount() == 0)
        return false;

    // Lock the set of holographic camera resources, then draw to each camera
    // in this frame.
    return m_deviceResources->UseHolographicCameraResources<bool>(
        [this, holographicFrame](std::map<UINT32, std::unique_ptr<DX::CameraResources>>& cameraResourceMap)
    {
        // Up-to-date frame predictions enhance the effectiveness of image stablization and
        // allow more accurate positioning of holograms.
        holographicFrame->UpdateCurrentPrediction();
        HolographicFramePrediction^ prediction = holographicFrame->CurrentPrediction;
        SpatialCoordinateSystem^ currentCoordinateSystem = m_referenceFrame->GetStationaryCoordinateSystemAtTimestamp(prediction->Timestamp);

        bool atLeastOneCameraRendered = false;
        for (auto cameraPose : prediction->CameraPoses)
        {
            // This represents the device-based resources for a HolographicCamera.
            DX::CameraResources* pCameraResources = cameraResourceMap[cameraPose->HolographicCamera->Id].get();

            // Get the device context.
            const auto context = m_deviceResources->GetD3DDeviceContext();
            const auto depthStencilView = pCameraResources->GetDepthStencilView();

            // Set render targets to the current holographic camera.
            ID3D11RenderTargetView* const targets[1] = { pCameraResources->GetBackBufferRenderTargetView() };
            context->OMSetRenderTargets(1, targets, depthStencilView);

            // Clear the back buffer and depth stencil view.
            context->ClearRenderTargetView(targets[0], DirectX::Colors::Transparent);
            context->ClearDepthStencilView(depthStencilView, D3D11_CLEAR_DEPTH | D3D11_CLEAR_STENCIL, 1.0f, 0);

            // The view and projection matrices for each holographic camera will change
            // every frame. This function refreshes the data in the constant buffer for
            // the holographic camera indicated by cameraPose.
            pCameraResources->UpdateViewProjectionBuffer(m_deviceResources, cameraPose, currentCoordinateSystem);

            // Attach the view/projection constant buffer for this camera to the graphics pipeline.
            bool cameraActive = pCameraResources->AttachViewProjectionBuffer(m_deviceResources);

            // Only render world-locked content when positional tracking is active.
            if (cameraActive)
            {
                // Draw the hologram.
                m_meshRenderer->Render(pCameraResources->IsRenderingStereoscopic(), m_drawWireframe);

                // On versions of the platform that support the CommitDirect3D11DepthBuffer API, we can 
                // provide the depth buffer to the system, and it will use depth information to stabilize 
                // the image at a per-pixel level.
                static const bool canCommitDirect3D11DepthBuffer =
                    Windows::Foundation::Metadata::ApiInformation::IsMethodPresent("Windows.Graphics.Holographic.HolographicCameraRenderingParameters", "CommitDirect3D11DepthBuffer");

                if (canCommitDirect3D11DepthBuffer)
                {
                    HolographicCameraRenderingParameters^ renderingParameters = holographicFrame->GetRenderingParameters(cameraPose);
                    ComPtr<ID3D11Texture2D> spDepthStencil = pCameraResources->GetDepthStencilTexture2D();

                    // Direct3D interop APIs are used to provide the buffer to the WinRT API.
                    ComPtr<IDXGIResource1> depthStencilResource;
                    DX::ThrowIfFailed(spDepthStencil.As(&depthStencilResource));
                    ComPtr<IDXGISurface2> depthDxgiSurface;
                    DX::ThrowIfFailed(depthStencilResource->CreateSubresourceSurface(0, &depthDxgiSurface));
                    IDirect3DSurface^ depthD3DSurface = CreateDirect3DSurface(depthDxgiSurface.Get());

                    // Calling CommitDirect3D11DepthBuffer causes the system to queue Direct3D commands to 
                    // read the depth buffer. It will then use that information to stabilize the image as
                    // the HolographicFrame is presented.
                    renderingParameters->CommitDirect3D11DepthBuffer(depthD3DSurface);
                }
            }

            atLeastOneCameraRendered = true;
        }

        return atLeastOneCameraRendered;
    });
}
/***********************************************************************************************************/
/*                                                                                                         */
/***********************************************************************************************************/
void HoloLensNavigationMain::SaveAppState()
{
    if (m_spatialAnchorHelper != nullptr)
    {
        m_spatialAnchorHelper->TrySaveToAnchorStore();
    }
}
/***********************************************************************************************************/
/*                                                                                                         */
/***********************************************************************************************************/
void HoloLensNavigationMain::LoadAppState()
{
    LoadAnchorStore();
}
/***********************************************************************************************************/
/*                                                                                                         */
/***********************************************************************************************************/
void HoloLensNavigationMain::OnDeviceLost()
{
    // Notifies classes that use Direct3D device resources that the device resources
    // need to be released before this method returns.

    m_meshRenderer->ReleaseDeviceDependentResources();
}
/***********************************************************************************************************/
/*                                                                                                         */
/***********************************************************************************************************/
void HoloLensNavigationMain::OnDeviceRestored()
{
    // Notifies classes that use Direct3D device resources that the device resources
    // may now be recreated.

    m_meshRenderer->CreateDeviceDependentResources();
}
/***********************************************************************************************************/
/*                                                                                                         */
/***********************************************************************************************************/
void HoloLensNavigationMain::OnPositionalTrackingDeactivating(SpatialLocator^ sender, SpatialLocatorPositionalTrackingDeactivatingEventArgs^ args)
{
    // Without positional tracking, spatial meshes will not be locatable.
    args->Canceled = true;
}
/***********************************************************************************************************/
/*                                                                                                         */
/***********************************************************************************************************/
void HoloLensNavigationMain::OnCameraAdded(HolographicSpace^ sender, HolographicSpaceCameraAddedEventArgs^ args)
{
    Deferral^ deferral = args->GetDeferral();
    HolographicCamera^ holographicCamera = args->Camera;
    create_task([this, deferral, holographicCamera] ()
    {
        // Create device-based resources for the holographic camera and add it to the list of
        // cameras used for updates and rendering. Notes:
        //   * Since this function may be called at any time, the AddHolographicCamera function
        //     waits until it can get a lock on the set of holographic camera resources before
        //     adding the new camera. At 60 frames per second this wait should not take long.
        //   * A subsequent Update will take the back buffer from the RenderingParameters of this
        //     camera's CameraPose and use it to create the ID3D11RenderTargetView for this camera.
        //     Content can then be rendered for the HolographicCamera.
        m_deviceResources->AddHolographicCamera(holographicCamera);

        // Holographic frame predictions will not include any information about this camera until
        // the deferral is completed.
        deferral->Complete();
    });
}
/***********************************************************************************************************/
/*                                                                                                         */
/***********************************************************************************************************/
void HoloLensNavigationMain::OnCameraRemoved(HolographicSpace^ sender, HolographicSpaceCameraRemovedEventArgs^ args)
{
    // Before letting this callback return, ensure that all references to the back buffer
    // are released.
    // Since this function may be called at any time, the RemoveHolographicCamera function
    // waits until it can get a lock on the set of holographic camera resources before
    // deallocating resources for this camera. At 60 frames per second this wait should
    // not take long.
    m_deviceResources->RemoveHolographicCamera(args->Camera);
}
/***********************************************************************************************************/
/*                                                                                                         */
/***********************************************************************************************************/
void HoloLensNavigationMain::OnLocatabilityChanged(SpatialLocator^ sender, Object^ args)
{
    // String^ mes = sender->Locatability.ToString();

    switch (sender->Locatability)
    {
        case SpatialLocatability::Unavailable:
            // Holograms cannot be rendered.
            break;
        case SpatialLocatability::PositionalTrackingActivating:
            break;
        case SpatialLocatability::OrientationOnly:
            break;
        case SpatialLocatability::PositionalTrackingInhibited:
            break;
        case SpatialLocatability::PositionalTrackingActive:
            break;
    }
}
