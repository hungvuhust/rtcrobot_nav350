#pragma once

#include <algorithm>
#include <iostream>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace sick_nav350 {

typedef std::pair<std::string, std::string> SickNav350CommandPair;

typedef std::map<std::string, SickNav350CommandPair> SickNav350Command;

static const SickNav350Command SICK_NAV350_COMMANDS= {
    {"READ_DEVICE_IDENT", {"sRN", "DeviceIdent"}},
    {"READ_SERIAL_NUMBER", {"sRN", "SerialNumber"}},
    {"READ_FW_VERSION", {"sRN", "FirmwareVersion"}},
    {"READ_MEASUREMENT_FW", {"sRN", "MMDeviceInfo"}},
    // Setting Layer
    {"SET_LAYER", {"sWN", "NEVACurrLayer"}},
    {"READ_LAYER", {"sRN", "NEVACurrLayer"}},
    // Setting Reflector Window
    {"DEFINE_REFLECTOR_WINDOW", {"sWN", "NCORIdentWindow"}},
    {"READ_REFLECTOR_WINDOW", {"sRN", "NCORIdentWindow"}},
    // Setting mapping
    {"CONFIGURE_MAPPING", {"sWN", "NMAPMapCfg"}},
    {"READ_MAPPING_CONFIG", {"sRN", "NMAPMapCfg"}},
    // Setting detect data
    {"SET_SLIDING_MEAN", {"sWN", "NPOSSlidingMean"}},
    {"READ_SLIDING_MEAN", {"sRN", "NPOSSlidingMean"}},
    {"SET_POS_DATA_FORMAT", {"sWN", "NPOSPoseDataFormat"}},
    {"READ_POS_DATA_FORMAT", {"sRN", "NPOSPoseDataFormat"}},
    {"SET_LANDMARK_DATA_FORMAT", {"sWN", "NLMDLandmarkDataFormat"}},
    {"READ_LANDMARK_DATA_FORMAT", {"sRN", "NLMDLandmarkDataFormat"}},
    {"SET_SCAN_DATA_FORMAT", {"sWN", "NAVScanDataFormat"}},
    {"READ_SCAN_DATA_FORMAT", {"sRN", "NAVScanDataFormat"}},
    // Set Time Sync
    {"SET_HARDWARE_TIME_SYNC", {"sWN", "NAVHardwareTimeSync"}},
    {"READ_HARDWARE_TIME_SYNC", {"sRN", "NAVHardwareTimeSync"}},
    // Reflector Size
    {"SET_REFLECTOR_SIZE", {"sWN", "NLMDReflSize"}},
    {"READ_REFLECTOR_SIZE", {"sRN", "NLMDReflSize"}},
    {"SET_REFLECTOR_TYPE", {"sWN", "NLMDReflType"}},
    {"READ_REFLECTOR_TYPE", {"sRN", "NLMDReflType"}},
    // Set Landmark Matching
    {"SET_LANDMARK_MATCHING", {"sWN", "NLMDLandmarkMatching"}},
    {"READ_LANDMARK_MATCHING", {"sRN", "NLMDLandmarkMatching"}},
    {"SET_SECTOR_MUTING", {"sWN", "NLMDMutedSectors"}},
    {"READ_SECTOR_MUTING", {"sRN", "NLMDMutedSectors"}},
    // Set Orientation Coordinate System
    {"SET_COORD_ORIENTATION", {"sWN", "NEVACoordOrientation"}},
    {"READ_COORD_ORIENTATION", {"sRN", "NEVACoordOrientation"}},
    // Set N Closest Reflectors
    {"SET_N_CLOSEST_REFLECTORS", {"sWN", "NLMDnClosest"}},
    {"READ_N_CLOSEST_REFLECTORS", {"sRN", "NLMDnClosest"}},
    // Set Action Radius
    {"SET_ACTION_RADIUS", {"sWN", "NLMDActionRadius"}},
    {"READ_ACTION_RADIUS", {"sRN", "NLMDActionRadius"}},
    // Set Reflector Threshold
    {"SET_REFLECTOR_THRESHOLD", {"sWN", "NLMDReflThreshold"}},
    {"READ_REFLECTOR_THRESHOLD", {"sRN", "NLMDReflThreshold"}},
    // Set Operating Mode
    {"SET_MODE", {"sMN", "mNEVAChangeState"}},
    // USER LEVEL
    // {main: B21ACE26}
    // {client: F4724744}
    {"SET_ACCESS", {"sMN", "SetAccessMode"}},
    {"STORE_DATA", {"sMN", "mEEwriteall"}},
    {"SYNC_TIMESTAMP", {"sMN", "mNAVGetTimestamp"}},
    {"BREAK_ASYNC", {"sMN", "mNAVBreak"}},
    {"DEVICE_RESET", {"sMN", "mNAVReset"}},
    // METHOD IN STANDBY MODE
    {"CHANGE_SERIAL_CONFIG", {"sMN", "mChangeSerialCfg"}},
    {"CHANGE_IP_CONFIG", {"sMN", "mChangeIPCfg"}},
    {"CHANGE_ETH_CONFIG", {"sMN", "mChangeEthrCfg"}},
    {"ENABLE_DHCP", {"sMN", "mEnableDHCP"}},
    {"ADD_LANDMARK", {"sMN", "mNLAYAddLandmark"}},
    {"EDIT_LANDMARK", {"sMN", "mNLAYSetLandmark"}},
    {"DELETE_LANDMARK", {"sMN", "mNLAYDelLandmark"}},
    {"READ_LANDMARK", {"sMN", "mNLAYGetLandmark"}},
    {"READ_LAYER", {"sMN", "mNLAYGetLayer"}},
    {"READ_LAYOUT", {"sMN", "mNLAYGetLayout"}},
    {"ERASE_LAYOUT", {"sMN", "mNLAYEraseLayout"}},
    {"STORE_LAYOUT", {"sMN", "mNLAYStoreLayout"}},
    // METHOD IN MAPPING MODE
    {"DO_MAPPING", {"sMN", "mNMAPDoMapping"}},
    // METHOD IN DETECT LANMARK MODE
    {"GET_LANDMARK_DATA", {"sMN", "mNLMDGetData"}},
    // METHODS IN NAVIGATION MODE
    {"REQUEST_POSITION", {"sMN", "mNPOSGetPose"}},
    {"REQUEST_POSITION_DATA", {"sMN", "mNPOSGetData"}},
    {"VELOCITY_INPUT", {"sMN", "mNPOSSetSpeed"}},
    {"SET_POSITION", {"sMN", "mNPOSSetPose"}},
    {"SET_POSITION_BY_LANDMARK", {"sMN", "mNPOSSetPoseID"}}};

} // namespace sick_nav350