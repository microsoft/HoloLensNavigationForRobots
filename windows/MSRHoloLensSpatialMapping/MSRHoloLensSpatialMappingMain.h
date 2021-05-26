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
#include "Common\StepTimer.h"

#include "Content\SpatialInputHandler.h"
#include "Content\RealtimeSurfaceMeshRenderer.h"

#include "Content\SurfaceMesh.h"

#include "SpatialAnchor.h"

#include <sstream>
#include <windows.networking.sockets.h>
#include <windows.storage.streams.h>
#include <Eigen/Eigen>

#define PORT "1234"

namespace HoloLensNavigation
{
    enum InternalStatus {
        UNCONNECTED,
        READY_TO_RECEIVE,
        RECEIVING,
        PROCESSING,
        READY_TO_SEND,
        SENDING
    };

    enum EnumAskedState {
        Default_Ask_Pose,
        Refresh_Anchor_And_Render_Map,
        Update_Anchor
    };

    enum EnumRepliedState {
        Reply_Pose,
        Reply_Anchor_Pose,
        Reply_Anchor_Pose_Map
    };

    class HoloLensNavigationMain;

    ref class ListenerContext
    {
    internal:
        ListenerContext(HoloLensNavigationMain* mappingmain)
        {
            m_mappingmain = mappingmain;
        }
        delegate void OnConnectionEvent(Windows::Networking::Sockets::StreamSocketListener^ listener, Windows::Networking::Sockets::StreamSocketListenerConnectionReceivedEventArgs^ object);
    public:
        void OnConnection(Windows::Networking::Sockets::StreamSocketListener^ listener, Windows::Networking::Sockets::StreamSocketListenerConnectionReceivedEventArgs^ object);

    internal:
        Windows::Storage::Streams::DataWriter ^             m_writer;
        Windows::Storage::Streams::DataReader ^             m_reader;
        Windows::Networking::Sockets::StreamSocket ^        m_socket;
        std::mutex                                          m_sockLock;

    private:
        HoloLensNavigationMain *                            m_mappingmain;
    };

    //
    // Updates, renders, and presents holographic content using Direct3D.
    class HoloLensNavigationMain : public DX::IDeviceNotify
    {
    public:
        HoloLensNavigationMain(const std::shared_ptr<DX::DeviceResources>& deviceResources);
        ~HoloLensNavigationMain();

        // Sets the holographic space. This is our closest analogue to setting a new window
        // for the app.
        void SetHolographicSpace(Windows::Graphics::Holographic::HolographicSpace^ holographicSpace);

        // Starts the holographic frame and updates the content.
        Windows::Graphics::Holographic::HolographicFrame^ Update();

        // Renders holograms, including world-locked content.
        bool Render(Windows::Graphics::Holographic::HolographicFrame^ holographicFrame);

        // Handle saving and loading of app state owned by AppMain.
        void SaveAppState();
        void LoadAppState();

        // IDeviceNotify
        virtual void OnDeviceLost();
        virtual void OnDeviceRestored();

        // Handle surface change events.
        void OnSurfacesChanged(Windows::Perception::Spatial::Surfaces::SpatialSurfaceObserver^ sender, Platform::Object^ args);

    public:
        //
        // socket connection state handlers
        Windows::Networking::Sockets::StreamSocketListener^ StartListener(ListenerContext::OnConnectionEvent^ onConnectionEvent, Platform::String^ port);
        void StateSender();
        void StateReceiver();

    private:
        // Asynchronously creates resources for new holographic cameras.
        void OnCameraAdded(
            Windows::Graphics::Holographic::HolographicSpace^ sender,
            Windows::Graphics::Holographic::HolographicSpaceCameraAddedEventArgs^ args);

        // Synchronously releases resources for holographic cameras that are no longer
        // attached to the system.
        void OnCameraRemoved(
            Windows::Graphics::Holographic::HolographicSpace^ sender,
            Windows::Graphics::Holographic::HolographicSpaceCameraRemovedEventArgs^ args);

        // Used to prevent the device from deactivating positional tracking, which is 
        // necessary to continue to receive spatial mapping data.
        void OnPositionalTrackingDeactivating(
            Windows::Perception::Spatial::SpatialLocator^ sender,
            Windows::Perception::Spatial::SpatialLocatorPositionalTrackingDeactivatingEventArgs^ args);

        void OnLocatabilityChanged(
            Windows::Perception::Spatial::SpatialLocator^ sender,
            Platform::Object^ args);

        // Clears event registration state. Used when changing to a new HolographicSpace
        // and when tearing down AppMain.
        void UnregisterHolographicEventHandlers();

        // Listens for the Pressed spatial input event.
        std::shared_ptr<SpatialInputHandler>                                m_spatialInputHandler;

        // A data handler for surface meshes.
        std::unique_ptr<RealtimeSurfaceMeshRenderer>                        m_meshRenderer;

    private:
        // Cached pointer to device resources.
        std::shared_ptr<DX::DeviceResources>                                m_deviceResources;

        // Render loop timer.
        DX::StepTimer                                                       m_timer;

        // Represents the holographic space around the user.
        Windows::Graphics::Holographic::HolographicSpace ^                  m_holographicSpace;

        // SpatialLocator that is attached to the primary camera.
        Windows::Perception::Spatial::SpatialLocator ^                      m_locator;

        // A reference frame attached to the holographic camera.
        Windows::Perception::Spatial::SpatialLocatorAttachedFrameOfReference^ m_referenceFrame;

        // Event registration tokens.
        Windows::Foundation::EventRegistrationToken                         m_cameraAddedToken;
        Windows::Foundation::EventRegistrationToken                         m_cameraRemovedToken;
        Windows::Foundation::EventRegistrationToken                         m_positionalTrackingDeactivatingToken;
        Windows::Foundation::EventRegistrationToken                         m_surfacesChangedToken;
        Windows::Foundation::EventRegistrationToken                         m_locatabilityChangedToken;

        // Indicates whether access to spatial mapping data has been granted.
        bool                                                                m_surfaceAccessAllowed = false;

        // Indicates whether the surface observer initialization process was started.
        bool                                                                m_spatialPerceptionAccessRequested = false;

        // Obtains spatial mapping data from the device in real time.
        Windows::Perception::Spatial::Surfaces::SpatialSurfaceObserver ^    m_surfaceObserver;
        Windows::Perception::Spatial::Surfaces::SpatialSurfaceMeshOptions ^ m_surfaceMeshOptions;

        // Determines the rendering mode.
        bool                                                                m_drawWireframe = true;


        //
        // hololens navigation
private:
        // A frame of reference that remains stationary relative to the user's surroundings at a point in time.
        Windows::Perception::Spatial::SpatialStationaryFrameOfReference^    m_stationaryReferenceFrame;

        bool InternalUpdate(Windows::Graphics::Holographic::HolographicFrame^ holographicFrame);
        void CollectSpatialMappinginformation();
        void FloorDetection(unsigned char* buffer, unsigned int uBufferSize, double& HoloHeight, Eigen::Vector3d& floorpt);

        std::shared_ptr<SpatialAnchorHelper>                                m_spatialAnchorHelper;
        Windows::Perception::Spatial::SpatialAnchor ^                       m_baseAnchor;
        Windows::Perception::Spatial::SpatialAnchor ^                       m_nextAnchor;
        Windows::Perception::Spatial::SpatialAnchor ^                       m_anchor;

        void LoadAnchorStore();
        Platform::String ^                                                  m_newKey;
        Platform::String ^                                                  m_AnchorKey;
        int                                                                 m_spatialId = 0;
        Windows::Foundation::Numerics::float4x4                             m_initNewAnchorPositionFromPrevAnchor;

        // kept current state
        Windows::Foundation::Numerics::float4x4                             m_curPositionFromAnchor;
        float                                                               m_floorAndHeight[4] = { 0,0,0,0 };

        // send functions
        void SendPose();
        void SendInitAnchorPose();
        void SendAnchorID();
        void SendPointCloud();

        //
        // socket management
        friend ListenerContext;
        ListenerContext ^                                                   m_listenercontext;
        Windows::Networking::Sockets::StreamSocketListener ^                m_listener;

        //
        // connection status
        bool                                                                m_connecting = false;
        bool                                                                m_bCommunicating = false, m_bNextSend = false;
        EnumAskedState                                                      m_askedState = EnumAskedState::Default_Ask_Pose;
        EnumRepliedState                                                    m_replyedState = EnumRepliedState::Reply_Pose;
        InternalStatus                                                      m_internalState = InternalStatus::UNCONNECTED;

        //
        // log output flag
        bool                                                                m_fVerbose;
    };

} // namespace HoloLensNavigation
