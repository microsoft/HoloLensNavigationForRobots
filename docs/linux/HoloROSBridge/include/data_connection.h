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

#include <sys/socket.h>
#include <netinet/in.h>

//
// needs to match SurfaceMeshStreamHeader in the HoloLensSpatialMapping Windows application
#define SMSH_SIGNATURE      "SMSHSIG_"

struct SurfaceMeshStreamHeader
{
    unsigned char           signature[8];
    float                   scale;
    float                   center_x;
    float                   center_y;
    float                   center_z;
    float                   orientation_x;
    float                   orientation_y;
    float                   orientation_z;
    float                   orientation_w;
    unsigned int            uVertexBufferSize;
    unsigned int            uNumVertices;
};

enum EnumAskState
{
    Default_Ask_Pose,
    Refresh_Anchor_And_Render_Map,
    Update_Anchor
};

enum EnumRepliedState
{
    Recv_Pose,
    Recv_Anchor_Pose,
    Recv_Anchor_Pose_Map
};

void recvMessage(int sock,unsigned int size,char* pointer);
int reverseEndian(int n);
