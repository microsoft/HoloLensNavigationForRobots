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
#include "MSRHololensSpatialMappingMain.h"
#include "Common\DirectXHelper.h"

#include <windows.graphics.directx.direct3d11.interop.h>
#include <Collection.h>

using namespace HoloLensNavigation;

using namespace concurrency; 
using namespace Microsoft::WRL;
using namespace Platform;
using namespace Windows::Foundation::Collections;
using namespace Windows::Foundation::Numerics;
using namespace Windows::Graphics::Holographic;
using namespace Windows::Networking;
using namespace Windows::Perception::Spatial;
using namespace Windows::System::Threading;

/***********************************************************************************************************/
/*                                                                                                         */
/***********************************************************************************************************/
Windows::Networking::Sockets::StreamSocketListener^ HoloLensNavigationMain::StartListener(ListenerContext::OnConnectionEvent^ onConnectionEvent, Platform::String^ port)
{
    Sockets::StreamSocketListener^ listener = ref new Sockets::StreamSocketListener();
    listener->ConnectionReceived += ref new Windows::Foundation::TypedEventHandler<Windows::Networking::Sockets::StreamSocketListener^, Windows::Networking::Sockets::StreamSocketListenerConnectionReceivedEventArgs^>(onConnectionEvent);
    listener->Control->KeepAlive = false;

    create_task(listener->BindServiceNameAsync(port)).then([this](task<void> previousTask)
    {
        try
        {
            // Try getting an exception.
            previousTask.get();
        }
        catch (COMException^ exception)
        {
            // HRESULT:0x80072740 Only one usage of each socket address (protocol/network address/port) is normally permitted.
            if (exception->HResult == 0x80072740)
            {
                throw;
            }
        }
        catch (Exception^ exception)
        {
        }
    });

    return listener;
}
/***********************************************************************************************************/
/*                                                                                                         */
/***********************************************************************************************************/
void ListenerContext::OnConnection(Windows::Networking::Sockets::StreamSocketListener^ listener, Windows::Networking::Sockets::StreamSocketListenerConnectionReceivedEventArgs^ object)
{
    DebugMsgW(L"Socket connected");

    m_writer = ref new Windows::Storage::Streams::DataWriter(object->Socket->OutputStream);
    m_reader = ref new Windows::Storage::Streams::DataReader(object->Socket->InputStream);
    m_socket = object->Socket;

    m_mappingmain->m_internalState = InternalStatus::READY_TO_RECEIVE;
}
/***********************************************************************************************************/
/*                                                                                                         */
/***********************************************************************************************************/
bool HoloLensNavigationMain::InternalUpdate(HolographicFrame^ holographicFrame)
{
    HolographicFramePrediction^     prediction = holographicFrame->CurrentPrediction;
    SpatialCoordinateSystem^        stationaryCoordinateSystem = m_stationaryReferenceFrame->CoordinateSystem;
    SpatialCoordinateSystem^        currentCoordinateSystem = m_referenceFrame->GetStationaryCoordinateSystemAtTimestamp(prediction->Timestamp);

    if ((m_internalState != PROCESSING) && (m_internalState != UNCONNECTED)) {
        return false;
    }

    // create new anchor
    EnumAskedState tempState = EnumAskedState::Default_Ask_Pose;
    if (m_askedState != EnumAskedState::Default_Ask_Pose) {
        // for async processing of data receiving
        // should be checked the connection timing
        m_replyedState = EnumRepliedState::Reply_Anchor_Pose_Map;
        tempState = m_askedState;
        m_askedState = EnumAskedState::Default_Ask_Pose;
    }

    // HoloLens pose update
    auto cameraPose = prediction->CameraPoses->GetAt(0);
    Platform::IBox<HolographicStereoTransform>^ viewTransformContainerStatic = cameraPose->TryGetViewTransform(stationaryCoordinateSystem);
    bool viewTransformAcquired = viewTransformContainerStatic != nullptr;

    if (viewTransformAcquired) //get Camera pose in m_baseAnchor coordinate system, 
    {
        HolographicStereoTransform viewCoordinateSystemTransform = viewTransformContainerStatic->Value;
        float4x4 camPosition = viewCoordinateSystemTransform.Left;

        if (m_baseAnchor != nullptr) {
            float4x4 anchorSpaceToCurrentCoordinateSystem;
            SpatialCoordinateSystem^ anchorSpace = m_baseAnchor->CoordinateSystem;
            const auto tryTransform = anchorSpace->TryGetTransformTo(stationaryCoordinateSystem);

            if (tryTransform != nullptr)
            {
                anchorSpaceToCurrentCoordinateSystem = tryTransform->Value;
                camPosition = anchorSpaceToCurrentCoordinateSystem * camPosition;//->camPotition: base coordinates(spatial anchor coordinates)2camera coordinates 
            }
        }

        bool invertible = Windows::Foundation::Numerics::invert(camPosition, &m_curPositionFromAnchor);

        // distance from anchor to camera check.
        // base anchor is newly created or changed another one created before
        if (invertible)
        {
            float xp = m_curPositionFromAnchor.m41, yp = m_curPositionFromAnchor.m42, zp = m_curPositionFromAnchor.m43;
            float thres = 4.0 * 4.0;

            if (m_baseAnchor != nullptr && (xp * xp + yp * yp + zp * zp > thres)) {
                // get anchor map
                auto anchorMap = m_spatialAnchorHelper->GetAnchorMap();
                double minimumerr = -1;
                IKeyValuePair<String^, SpatialAnchor^>^ bestPair;

                for each (auto & pair in anchorMap) {
                    SpatialAnchor^ candidateAnchor = pair->Value;
                    float4x4 anchorSpaceToCurrentCoordinateSystem;
                    SpatialCoordinateSystem^ anchorSpace = candidateAnchor->CoordinateSystem;
                    const auto tryTransform = anchorSpace->TryGetTransformTo(stationaryCoordinateSystem);
                    float4x4 camPos_inAnchor;

                    if (tryTransform != nullptr)
                    {
                        anchorSpaceToCurrentCoordinateSystem = tryTransform->Value;
                        camPos_inAnchor = anchorSpaceToCurrentCoordinateSystem * viewCoordinateSystemTransform.Left;//->camPotition: base coordinates(spatial anchor coordinates)2camera coordinates 	
                        float4x4 camPosInv;
                        bool invertible_ = Windows::Foundation::Numerics::invert(camPos_inAnchor, &camPosInv);

                        if (invertible_) {
                            float xp = camPosInv.m41, yp = camPosInv.m42, zp = camPosInv.m43;
                            double err = xp * xp + yp * yp + zp * zp;

                            if (err < minimumerr || minimumerr < 0) {
                                bestPair = pair;
                                minimumerr = err;
                            }
                        }
                    }
                }

                if (minimumerr > 0 && minimumerr < thres) {
                    // update anchor position pre-created
                    m_baseAnchor = bestPair->Value;
                    m_AnchorKey = bestPair->Key;
                    m_replyedState = EnumRepliedState::Reply_Anchor_Pose;
                }
                else {
                    // create new anchor and rendering map
                    m_replyedState = EnumRepliedState::Reply_Anchor_Pose_Map;
                }
            }
        }

        // new anchor create
        if (m_replyedState == EnumRepliedState::Reply_Anchor_Pose_Map) {
            Platform::IBox<HolographicStereoTransform>^ viewTransformContainer = cameraPose->TryGetViewTransform(currentCoordinateSystem);
            bool viewTransformAcquired = viewTransformContainer != nullptr;

            if (viewTransformAcquired)
            {
                HolographicStereoTransform viewCoordinateSystemTransform = viewTransformContainer->Value;
                float4x4 camPosition2 = viewCoordinateSystemTransform.Left;
                float4x4 viewInverse;
                bool invertible = Windows::Foundation::Numerics::invert(camPosition2, &viewInverse);

                if (invertible)
                {
                    const float3 campos(viewInverse.m41, viewInverse.m42, viewInverse.m43);
                    float rad = sqrt(viewInverse.m31 * viewInverse.m31 + viewInverse.m33 * viewInverse.m33);
                    float theta = acos(viewInverse.m33 / rad);
                    theta = viewInverse.m31 < 0 ? -theta : theta;
                    const float3 camdirection(viewInverse.m31 / rad, 0, viewInverse.m33 / rad);
                    const quaternion q(0, sin(theta / 2), 0, cos(theta / 2));
                    SpatialAnchor^ anchor = SpatialAnchor::TryCreateRelativeTo(currentCoordinateSystem, campos, q);

                    if ((anchor != nullptr) && (m_spatialAnchorHelper != nullptr))
                    {
                        if (tempState == EnumAskedState::Refresh_Anchor_And_Render_Map)
                        {
                            m_spatialAnchorHelper->ClearAnchorStore();
                            m_spatialId = 0;
                        }
                        else {
                            m_spatialId++;
                        }

                        auto anchorMap = m_spatialAnchorHelper->GetAnchorMap();

                        // Create an identifier for the anchor.
                        std::wstringstream ss;
                        ss << "anchor_" << m_spatialId;
                        std::wstring w_char = ss.str();

                        m_AnchorKey = ref new String(w_char.c_str());

                        if (m_fVerbose) {
                            OutputDebugStringA((std::string("State: ") + std::to_string(m_internalState) + "\n").c_str());
                            OutputDebugStringA((std::string("Anchor: ") + std::to_string(m_spatialId) + "\n").c_str());
                        }

                        SaveAppState();

                        {
                            if (m_baseAnchor == nullptr) {
                                m_baseAnchor = anchor;
                            }
                            const auto tryTransform = anchor->CoordinateSystem->TryGetTransformTo(m_baseAnchor->CoordinateSystem);
                            m_initNewAnchorPositionFromPrevAnchor = tryTransform->Value;
                            m_baseAnchor = anchor;
                            currentCoordinateSystem = m_baseAnchor->CoordinateSystem;
                            anchorMap->Insert(m_AnchorKey->ToString(), anchor);
                            m_replyedState = EnumRepliedState::Reply_Anchor_Pose_Map;
                        }
                    }
                }

            }
        }

        Windows::Foundation::Numerics::invert(camPosition, &m_curPositionFromAnchor);
    }

    return true;
}
/***********************************************************************************************************/
/*                                                                                                         */
/***********************************************************************************************************/
void HoloLensNavigationMain::CollectSpatialMappinginformation()
{
    {
        m_meshRenderer->CollectVertexData();

        StaticMemoryBuffer& memBuffer = m_meshRenderer->GetVertexDataCollectionBuffer();

        memBuffer.LockRWAccess();

        unsigned int        uBufferSize = memBuffer.getBufferSize();
        unsigned char *     ptr = (unsigned char* )memBuffer.getPointer();

        if ((uBufferSize > 0) && (ptr != nullptr)) {
            double              holoHeight;
            Eigen::Vector3d     holo2floor;

            //
            // FloorDetection() is very CPU intensive. When debugging and to ease CPU impact, disable the
            // call and assign temporary fixed values.
#ifndef DISABLE_FLOORDETECTION
            FloorDetection(ptr, uBufferSize, holoHeight, holo2floor);
#else // #ifndef DISABLE_FLOORDETECTION
            #pragma message("====================== FloorDetection() disabled ======================")
            holoHeight = -1.31;
            holo2floor << -0.163, 0.345, 1.251;
#endif // #ifndef DISABLE_FLOORDETECTION

            if (m_fVerbose) {
                OutputDebugStringA((std::string("Height and floor: ") + std::to_string(holoHeight) + ","
                    + std::to_string(holo2floor(0)) + ","
                    + std::to_string(holo2floor(1)) + ","
                    + std::to_string(holo2floor(2)) + ","
                    + "\n").c_str());
            }

            m_floorAndHeight[0] = (float)holo2floor(0);
            m_floorAndHeight[1] = (float)holo2floor(1);
            m_floorAndHeight[2] = (float)holo2floor(2);
            m_floorAndHeight[3] = (float)holoHeight;
        }

        memBuffer.UnlockRWAccess();
    }

    if (m_internalState == PROCESSING) {
        m_internalState = READY_TO_SEND;
    }
}
/***********************************************************************************************************/
/*                                                                                                         */
/***********************************************************************************************************/
void HoloLensNavigationMain::LoadAnchorStore()
{
    m_spatialAnchorHelper = create_task(SpatialAnchorManager::RequestStoreAsync())
        .then([](task<SpatialAnchorStore^> previousTask)
    {
        std::shared_ptr<SpatialAnchorHelper> newHelper = nullptr;


        try
        {
            SpatialAnchorStore^ anchorStore = previousTask.get();
            newHelper = std::shared_ptr<SpatialAnchorHelper>(new SpatialAnchorHelper(anchorStore));
            newHelper->LoadFromAnchorStore();

        }
        catch (Exception^ exception)
        {
            OutputDebugStringA((std::string(__FUNCTION__) + " " + std::to_string(__LINE__) + "\n").c_str());
            OutputDebugStringW(exception->Message->Data());
            OutputDebugStringA("\n");
        }

        // Return the initialized class instance.
        return newHelper;
    }).get();
}
/***********************************************************************************************************/
/*                                                                                                         */
/***********************************************************************************************************/
void HoloLensNavigationMain::StateReceiver()
{
    if (m_internalState != READY_TO_RECEIVE)
        return;

    // receive data first, then send data
    m_internalState = InternalStatus::RECEIVING;

    DataReader^ reader = m_listenercontext->m_reader;
    Windows::Networking::Sockets::StreamSocket^ socket = m_listenercontext->m_socket;

    create_task(reader->LoadAsync(sizeof(UINT32))).then([this, reader, socket](unsigned int size)
    {
        if (size < sizeof(UINT32))
        {
            // The underlying socket was closed before we were able to read the whole data.
            cancel_current_task();
        }

        std::lock_guard<std::mutex> guard(m_listenercontext->m_sockLock);
        int receivedValue = -1;

        try
        {
            receivedValue = m_listenercontext->m_reader->ReadInt32();
        }
        catch (Platform::COMException^ exception)
        {
            DebugMsgW(L"exception reading socket buffer!");
            return;
        }

        this->m_askedState = EnumAskedState(receivedValue);
        this->m_internalState = InternalStatus::PROCESSING;

    }).then([this](task<void> readTask)
    {
        try
        {
            // Try getting an exception.
            readTask.get();
        }
        catch (Platform::COMException^ exception)
        {
            this->m_listenercontext->m_reader = nullptr;
            this->m_listenercontext->m_writer = nullptr;
            this->m_listenercontext->m_socket = nullptr;
        }
        catch (task_canceled&)
        {
            // this will usually happen because user closed the client socket.

            // Explicitly close the socket.
            delete this->m_listenercontext->m_socket;

            this->m_listenercontext->m_reader = nullptr;
            this->m_listenercontext->m_writer = nullptr;
            this->m_listenercontext->m_socket = nullptr;
        }
    });

    return;
}
/***********************************************************************************************************/
/*                                                                                                         */
/***********************************************************************************************************/
void HoloLensNavigationMain::StateSender()
{
    if (m_internalState != READY_TO_SEND)
        return;

    m_listenercontext->m_writer->WriteInt32(m_replyedState);

    switch (m_replyedState) {
        case EnumRepliedState::Reply_Pose:
            SendPose();
            break;

        case EnumRepliedState::Reply_Anchor_Pose:
            SendAnchorID();
            SendPose();
            break;

        case EnumRepliedState::Reply_Anchor_Pose_Map:
            SendInitAnchorPose();
            SendPointCloud();
            SendAnchorID();
            SendPose();
            break;

        default:
            return;
    }

    this->m_internalState = InternalStatus::SENDING;

    create_task(m_listenercontext->m_writer->StoreAsync()).then([this](task<unsigned int> writeTask)
    {
        std::lock_guard<std::mutex> guard(m_listenercontext->m_sockLock);
        try
        {
            writeTask.get();
            this->m_internalState = InternalStatus::READY_TO_RECEIVE;
        }
        catch (Platform::COMException^ exception)
        {
            OutputDebugStringA((std::string(__FUNCTION__) + " " + std::to_string(__LINE__) + "\n").c_str());
            OutputDebugStringW(exception->Message->Data());
            OutputDebugStringA("\n");
            m_listenercontext->m_reader = nullptr;
            m_listenercontext->m_writer = nullptr;
            m_listenercontext->m_socket = nullptr;
        }
    });

    m_replyedState = Reply_Pose;

    return;
}
/***********************************************************************************************************/
/*                                                                                                         */
/***********************************************************************************************************/
void HoloLensNavigationMain::SendPose()
{
    Platform::Array<unsigned char>^ buffer = ref new Platform::Array<unsigned char>(sizeof(float) * 20);
    float datamat[20] =
    { m_curPositionFromAnchor.m11, m_curPositionFromAnchor.m12, m_curPositionFromAnchor.m13, m_curPositionFromAnchor.m14,
      m_curPositionFromAnchor.m21, m_curPositionFromAnchor.m22, m_curPositionFromAnchor.m23, m_curPositionFromAnchor.m24,
      m_curPositionFromAnchor.m31, m_curPositionFromAnchor.m32, m_curPositionFromAnchor.m33, m_curPositionFromAnchor.m34,
      m_curPositionFromAnchor.m41, m_curPositionFromAnchor.m42, m_curPositionFromAnchor.m43, m_curPositionFromAnchor.m44,
      m_floorAndHeight[0], m_floorAndHeight[1], m_floorAndHeight[2], m_floorAndHeight[3] };
    void* p = datamat;

    memcpy(buffer->Data, p, sizeof(float) * 20);

    m_listenercontext->m_writer->WriteBytes(buffer);
}
/***********************************************************************************************************/
/*                                                                                                         */
/***********************************************************************************************************/
void HoloLensNavigationMain::SendInitAnchorPose()
{
    Platform::Array<unsigned char>^ buffer = ref new Platform::Array<unsigned char>(sizeof(float) * 16);
    float datamat[16] =
    { m_initNewAnchorPositionFromPrevAnchor.m11, m_initNewAnchorPositionFromPrevAnchor.m12, m_initNewAnchorPositionFromPrevAnchor.m13, m_initNewAnchorPositionFromPrevAnchor.m14,
      m_initNewAnchorPositionFromPrevAnchor.m21, m_initNewAnchorPositionFromPrevAnchor.m22, m_initNewAnchorPositionFromPrevAnchor.m23, m_initNewAnchorPositionFromPrevAnchor.m24,
      m_initNewAnchorPositionFromPrevAnchor.m31, m_initNewAnchorPositionFromPrevAnchor.m32, m_initNewAnchorPositionFromPrevAnchor.m33, m_initNewAnchorPositionFromPrevAnchor.m34,
      m_initNewAnchorPositionFromPrevAnchor.m41, m_initNewAnchorPositionFromPrevAnchor.m42, m_initNewAnchorPositionFromPrevAnchor.m43, m_initNewAnchorPositionFromPrevAnchor.m44, };
    void* p = datamat;

    memcpy(buffer->Data, p, sizeof(float) * 16);

    m_listenercontext->m_writer->WriteBytes(buffer);
}
/***********************************************************************************************************/
/*                                                                                                         */
/***********************************************************************************************************/
void HoloLensNavigationMain::SendAnchorID()
{
    m_listenercontext->m_writer->WriteInt32(m_listenercontext->m_writer->MeasureString(m_AnchorKey));

    m_listenercontext->m_writer->WriteString(m_AnchorKey);
}
/***********************************************************************************************************/
/*                                                                                                         */
/***********************************************************************************************************/
void HoloLensNavigationMain::SendPointCloud()
{
    StaticMemoryBuffer& memBuffer = m_meshRenderer->GetVertexDataCollectionBuffer();

    memBuffer.LockRWAccess();

    unsigned int uBufferSize = memBuffer.getBufferSize();
    unsigned char* ptr = (unsigned char*)memBuffer.getPointer();

    m_listenercontext->m_writer->WriteInt32(uBufferSize);
    m_listenercontext->m_writer->WriteBytes(Platform::ArrayReference<BYTE>(ptr, uBufferSize));

    // DebugMsgW(L"======================= sent total buffer size: %u", uBufferSize);

    memBuffer.UnlockRWAccess();
}
/***********************************************************************************************************/
/*                                                                                                         */
/***********************************************************************************************************/
void HoloLensNavigationMain::FloorDetection(unsigned char* buffer, unsigned int uBufferSize, double& HoloHeight, Eigen::Vector3d& floorpt)
{
    //// get plane and Height
    //// around 1m x 1m

    //// ransac
    int                             ransac_max = 100;
    std::vector<Eigen::Vector3d>    points;
    std::vector<unsigned int>       indices;
    Eigen::Vector3d                 bestn, bestp;
    unsigned int                    cnt = 0;
    int                             maxcnt = -1;
    unsigned int                    uBufferPosition = 0;

    bestn << 1, 1, 1;

    while (true)
    {
        SurfaceMeshStreamHeader*    hdr = (SurfaceMeshStreamHeader*)(buffer);

        if (memcmp(hdr->signature, (void*)SMSH_SIGNATURE, sizeof(hdr->signature)) != 0) {
            DebugMsgW(L"FloorDetection() buffer misalignment at offset %u!\r\n", uBufferPosition);
            break;
        }

        Eigen::Vector3d             center;
        Eigen::Quaterniond          orientation = Eigen::Quaterniond(hdr->orientation_w, hdr->orientation_x, hdr->orientation_y, hdr->orientation_z);
        unsigned int                uNumVertices = hdr->uNumVertices;

        center << hdr->center_x, hdr->center_y, hdr->center_z;

        if (false) {
            OutputDebugStringA((std::to_string(uNumVertices) + "\n").c_str());
        }

        buffer += sizeof(SurfaceMeshStreamHeader);

        short *     vertexBuffer = (short *)buffer;
        for (unsigned int i = 0; i < uNumVertices; i++)
        {
            Eigen::Vector3d p;
            p << vertexBuffer[i * 4] * 0.0025f, vertexBuffer[i * 4 + 1] * 0.0025f, vertexBuffer[i * 4 + 2] * 0.0025f;
            p = orientation * p + center;

            if ((p(0) < 1.0) && (p(0) > -1.0) && (p(1) < 1.0) && (p(1) > -1.0) && (p(2) < 0)) {
                points.push_back(p);
                indices.push_back(cnt);
                cnt++;
            }
        }

        buffer += hdr->uVertexBufferSize;;
        uBufferPosition += sizeof(SurfaceMeshStreamHeader) + hdr->uVertexBufferSize;
        if (uBufferPosition >= uBufferSize)
            break;
    }

    if (points.size() < 50)
        return;

    std::vector<unsigned int> bestlist;
    for (int ransac_t = 0; ransac_t < ransac_max; ransac_t++) {
        random_shuffle(indices.begin(), indices.end());

        std::vector<unsigned int>   candlist;
        Eigen::Vector3d             v01, v02, nfloor, cand_p;
        int                         numInliers = 0;

        cand_p = points.at(indices.at(0));
        v01 = points.at(indices.at(1)) - cand_p;
        v02 = points.at(indices.at(2)) - cand_p;
        nfloor = v01.cross(v02);
        nfloor = nfloor.normalized();

        for (int idx = 3; idx < (int)indices.size(); idx++) {
            Eigen::Vector3d     targp = points.at(indices.at(idx)) - cand_p;
            double              err = abs(targp.dot(nfloor));

            if (err < 0.005) {
                //inlier
                candlist.push_back(indices.at(idx));
                numInliers++;
            }
        }

        if (maxcnt < numInliers) {
            maxcnt = numInliers;
            bestn = nfloor;
            bestp = cand_p;
            bestlist = std::vector<unsigned int>(candlist);
        }
    }

    // plane fitting
    // solve least square problem
    // ax+by+z+d=0: ax+by+d=-z
    Eigen::MatrixXd     A(bestlist.size(), 3);
    Eigen::VectorXd     B(bestlist.size());

    for (int idx = 0; idx < (int)bestlist.size(); idx++) {
        Eigen::Vector3d targp = points.at(indices.at(idx));

        A(idx, 0) = targp(0);   // x
        A(idx, 1) = targp(1);   // y
        A(idx, 2) = 1;          // 1
        B(idx) = -targp(2);     // -z
    }

    Eigen::Vector3d ansX = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B);

    // rendering: 1.0m upper from hololens - 2.0m lower from hololens (3.0m range)
    // hololens point (scale*(dwidth/2),scale*(dwidth/2),1.0)
    // ax+by+z+d=0: n<<a,b,1
    Eigen::Vector3d     phl;

    bestn << ansX(0), ansX(1), 1;
    bestp << 0, 0, -ansX(2);
    bestn = bestn.normalized();

    if (bestn(2) < 0)
        bestn = -bestn;

    // rendering: 1.0m upper from hololens - 2.0m lower from hololens (3.0m range)
    // hololens point (scale*(dwidth/2),scale*(dwidth/2),1.0)
    phl << 0, 0, 0.0;

    HoloHeight = -(phl - bestp).dot(bestn);
    floorpt = -HoloHeight * bestn; // hololens 2 floor
}
