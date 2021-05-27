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

#include <Collection.h>

namespace HoloLensNavigation
{
    class SpatialAnchorHelper
    {
    public:
        SpatialAnchorHelper(Windows::Perception::Spatial::SpatialAnchorStore^ anchorStore);
        Windows::Foundation::Collections::IMap<Platform::String^, Windows::Perception::Spatial::SpatialAnchor^>^ GetAnchorMap() { return m_anchorMap; };
        void LoadFromAnchorStore();
        void ClearAnchorStore();
        bool TrySaveToAnchorStore();

    private:
        Windows::Perception::Spatial::SpatialAnchorStore^ m_anchorStore;
        Windows::Foundation::Collections::IMap<Platform::String^, Windows::Perception::Spatial::SpatialAnchor^>^ m_anchorMap;
    };

} // namespace HoloLensNavigation
