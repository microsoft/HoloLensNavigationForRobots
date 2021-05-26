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
#include "SpatialAnchor.h"


using namespace HoloLensNavigation;
using namespace Windows::Perception::Spatial;
using namespace Platform;

/***********************************************************************************************************/
/*                                                                                                         */
/***********************************************************************************************************/
SpatialAnchorHelper::SpatialAnchorHelper(SpatialAnchorStore^ anchorStore)
{
    m_anchorStore = anchorStore;
    m_anchorMap = ref new Platform::Collections::Map<String^, SpatialAnchor^>();
}
/***********************************************************************************************************/
/*                                                                                                         */
/***********************************************************************************************************/
bool SpatialAnchorHelper::TrySaveToAnchorStore()
{
    // This function returns true if all the anchors in the in-memory collection are saved to the anchor
    // store. If zero anchors are in the in-memory collection, we will still return true because the
    // condition has been met.
    bool success = true;

    // If access is denied, 'anchorStore' will not be obtained.
    if (m_anchorStore != nullptr)
    {
        for each (auto & pair in m_anchorMap)
        {
            auto const& id = pair->Key;
            auto const& anchor = pair->Value;

            // Try to save the anchors.
            if (!m_anchorStore->TrySave(id, anchor))
            {
                // This may indicate the anchor ID is taken, or the anchor limit is reached for the app.
                success = false;
            }
        }
    }

    return success;
}
/***********************************************************************************************************/
/*                                                                                                         */
/***********************************************************************************************************/
void SpatialAnchorHelper::LoadFromAnchorStore()
{
    // If access is denied, 'anchorStore' will not be obtained.
    if (m_anchorStore != nullptr)
    {
        // Get all saved anchors.
        auto anchorMapView = m_anchorStore->GetAllSavedAnchors();
        for each (auto const& pair in anchorMapView)
        {
            auto const& id = pair->Key;
            auto const& anchor = pair->Value;
            m_anchorMap->Insert(id, anchor);
        }
    }
}
/***********************************************************************************************************/
/*                                                                                                         */
/***********************************************************************************************************/
void SpatialAnchorHelper::ClearAnchorStore()
{
    // If access is denied, 'anchorStore' will not be obtained.
    if (m_anchorStore != nullptr)
    {
        // Clear all anchors from the store.
        m_anchorMap->Clear();
        m_anchorStore->Clear();
    }
}
