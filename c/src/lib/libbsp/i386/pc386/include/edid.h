/**
 * @file edid.h
 *
 * @ingroup i386_pc386
 *
 * @brief VESA EDID definitions.
 */

/*
 * edid.h  - This file contains definitions for constants related to
 *           VESA Extended Display Identification Data.
 *          More information can be found at
 *      <http://www.vesa.org/vesa-standards/free-standards/>
 *          VESA public standards may be found at
 *      <http://www.vesa.org/wp-content/uploads/2010/12/thankspublic.htm>
 *
 * Copyright (C) 2014  Jan Doležal (dolezj21@fel.cvut.cz)
 *                     CTU in Prague.
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.org/license/LICENSE.
 */

#ifndef _EDID_H
#define _EDID_H

#ifndef ASM /* ASM */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */


/* VESA Extended Display Identification Data (EDID) Standard Version 3, Revision Date: November 13, 1997 */
/* VESA Enhanced Extended Display Identification Data (E-EDID) Standard Release A, Revision 2, September 25, 2006 */
/* VESA Enhanced Extended Display Identification Data (E-EDID) Proposed Release A, March 27, 2007 */


/**** structs and defines shared by both EDIDv1 and EDIDv2 ****/

/* Video Input Definition */
/* Analog Interface Data Format - Signal Level Standard */
#define EDIDx_SLS_0700_0300_1000Vpp             0x0
#define EDIDx_SLS_0714_0286_1000Vpp             0x1
#define EDIDx_SLS_1000_0400_1400Vpp             0x2
#define EDIDx_SLS_0700_0000_0700Vpp             0x3

struct DetailedTimingDescriptor{
    uint16_t PixelClock_div10000;
    uint8_t HorizontalActiveLow;
    uint8_t HorizontalBlankingLow;
    uint8_t HorizontalBlankingHigh          : 4;
    uint8_t HorizontalActiveHigh            : 4;
    uint8_t VerticalActiveLow;
    uint8_t VerticalBlankingLow;
    uint8_t VerticalBlankingHigh            : 4;
    uint8_t VerticalActiveHigh              : 4;
    uint8_t HorizontalSyncOffsetLow;
    uint8_t HorizontalSyncPulseWidthLow;    
    uint8_t VerticalSyncPulseWidthLow       : 4;
    uint8_t VerticalSyncOffsetLow           : 4;
    uint8_t VerticalSyncPulseWidthHigh      : 2;
    uint8_t VerticalSyncOffsetHigh          : 2;
    uint8_t HorizontalSyncPulseWidthHigh    : 2;
    uint8_t HorizontalSyncOffsetHigh        : 2;
    uint8_t HorizontalImageSizeLow;
    uint8_t VerticalImageSizeLow;
    uint8_t VerticalImageSizeHigh           : 4;
    uint8_t HorizontalImageSizeHigh         : 4;
    uint8_t HorizontalBorder;
    uint8_t VerticalBorder;
    uint8_t Flag_StereoMode0                : 1;
/* next four flags has different purpose with analog
   and digital interfaces for EDID version 2; first
   goes analog, after underscore there is digital name of bit */
    uint8_t Flag_SyncPolarity_LPPolarity    : 1;
    uint8_t Flag_Serration_FLMPolarity      : 1;
    uint8_t Flag_Composition_Serration      : 1;
    uint8_t Flag_Composition_ShiftClockUse  : 1;

    uint8_t Flag_StereoMode1                : 2;
    uint8_t Flag_Interlaced                 : 1;
}__attribute__((__packed__));

struct ColorPointData {
    uint8_t ColorPointWhitePointIndexNumber;
    uint8_t ColorPointWhiteLowBits;
    uint8_t ColorPointWhite_x;
    uint8_t ColorPointWhite_y;
    uint8_t ColorPointWhiteGamma;
}__attribute__((__packed__));


/**** structs and defines for EDIDv1 ****/

/* Basic Display Parameters */
    /* Feature Support */
        /* analog - Display Color Type */
#define EDID_DisplayType_Monochrome         0
#define EDID_DisplayType_RGBcolor           1
#define EDID_DisplayType_nonRGBcolor        2
#define EDID_DisplayType_undef              3
        /* digital - Supported Color Encoding Formats */
#define EDID_DisplayType_RGB444                 0
#define EDID_DisplayType_RGB444YCrCb444         1
#define EDID_DisplayType_RGB444YCrCb422         2
#define EDID_DisplayType_RGB444YCrCb444YCrCb422 3

/* Monitor Descriptor - Data Type Tag */
#define EDID_DTT_MonitorSerialNumber        0xFF

#define EDID_DTT_ASCIIString                0xFE

#define EDID_DTT_MonitorRangeLimits         0xFD
struct MonitorRangeLimits {
    uint8_t MinVerticalRateInHz;
    uint8_t MaxVerticalRateInHz;
    uint8_t MinHorizontalInKHz;
    uint8_t MaxHorizontalInKHz;
    uint8_t MaxSupportedPixelClockIn10MHz;
/* see VESA, Generalized Timing Formula Standard - GTF, Version 1.0, December 18, 1996 */
    uint8_t GTFStandard[8];
}__attribute__((__packed__));

#define EDID_DTT_MonitorName                0xFC

#define EDID_DTT_AdditionalColorPointData   0xFB

#define EDID_DTT_AdditionalSTI              0xFA /* Standard Timing Identification */

#define EDID_DTT_DisplayColorManagement     0xF9

#define EDID_DTT_CVT3ByteTimingCodes        0xF8
#define EDID_CVT_AspectRatio_4_3            0
#define EDID_CVT_AspectRatio_16_9           1
#define EDID_CVT_AspectRatio_16_10          2
#define EDID_CVT_AspectRatio_15_9           3
#define EDID_CVT_PrefVertRate50Hz           0
#define EDID_CVT_PrefVertRate60Hz           1
#define EDID_CVT_PrefVertRate75Hz           2
#define EDID_CVT_PrefVertRate85Hz           3
struct CVT3ByteCodeDescriptor {
    uint8_t AddressableLinesLow;
    uint8_t Reserved0               : 2;
    uint8_t AspectRatio             : 2;
    uint8_t AddressableLinesHigh    : 4;
    /* next 5 bits indicate supported vertical rates */
    uint8_t VerticalRate60HzRB      : 1;
    uint8_t VerticalRate85Hz        : 1;
    uint8_t VerticalRate75Hz        : 1;
    uint8_t VerticalRate60Hz        : 1;
    uint8_t VerticalRate50Hz        : 1;
    uint8_t PreferredVerticalRate   : 2;
    uint8_t Reserved1               : 1;
}__attribute__((__packed__));
struct CVTTimingCodes3B {
    uint8_t VersionNumber;
    struct CVT3ByteCodeDescriptor cvt[4];
}__attribute__((__packed__));

#define EDID_DTT_EstablishedTimingsIII      0xF7
struct EstablishedTimingsIII {
    uint8_t RevisionNumber;

    uint8_t EST_1152x864_75Hz   : 1;
    uint8_t EST_1024x768_85Hz   : 1;
    uint8_t EST_800x600_85Hz    : 1;
    uint8_t EST_848x480_60Hz    : 1;
    uint8_t EST_640x480_85Hz    : 1;
    uint8_t EST_720x400_85Hz    : 1;
    uint8_t EST_640x400_85Hz    : 1;
    uint8_t EST_640x350_85Hz    : 1;

    uint8_t EST_1280x1024_85Hz  : 1;
    uint8_t EST_1280x1024_60Hz  : 1;
    uint8_t EST_1280x960_85Hz   : 1;
    uint8_t EST_1280x960_60Hz   : 1;
    uint8_t EST_1280x768_85Hz   : 1;
    uint8_t EST_1280x768_75Hz   : 1;
    uint8_t EST_1280x768_60Hz   : 1;
    uint8_t EST_1280x768_60HzRB : 1;

    uint8_t EST_1400x1050_75Hz  : 1;
    uint8_t EST_1400x1050_60Hz  : 1;
    uint8_t EST_1400x1050_60HzRB: 1;
    uint8_t EST_1400x900_85Hz   : 1;
    uint8_t EST_1400x900_75Hz   : 1;
    uint8_t EST_1400x900_60Hz   : 1;
    uint8_t EST_1400x900_60HzRB : 1;
    uint8_t EST_1360x768_60Hz   : 1;

    uint8_t EST_1600x1200_70Hz  : 1;
    uint8_t EST_1600x1200_65Hz  : 1;
    uint8_t EST_1600x1200_60Hz  : 1;
    uint8_t EST_1680x1050_85Hz  : 1;
    uint8_t EST_1680x1050_75Hz  : 1;
    uint8_t EST_1680x1050_60Hz  : 1;
    uint8_t EST_1680x1050_60HzRB: 1;
    uint8_t EST_1400x1050_85Hz  : 1;

    uint8_t EST_1920x1200_60Hz  : 1;
    uint8_t EST_1920x1200_60HzRB: 1;
    uint8_t EST_1856x1392_75Hz  : 1;
    uint8_t EST_1856x1392_60Hz  : 1;
    uint8_t EST_1792x1344_75Hz  : 1;
    uint8_t EST_1792x1344_60Hz  : 1;
    uint8_t EST_1600x1200_85Hz  : 1;
    uint8_t EST_1600x1200_75Hz  : 1;

    uint8_t EST_Reserved0       : 4;
    uint8_t EST_1920x1440_75Hz  : 1;
    uint8_t EST_1920x1440_60Hz  : 1;
    uint8_t EST_1920x1200_85Hz  : 1;
    uint8_t EST_1920x1200_75Hz  : 1;

    uint8_t EST_Reserved1[6];
}__attribute__((__packed__));

#define EDID_DTT_DescriptorSpaceUnused      0x10

struct MonitorDescriptor {
    uint16_t Flag0;
    uint8_t Flag1;
    uint8_t DataTypeTag;
    uint8_t Flag2;
    uint8_t DescriptorData[13];
}__attribute__((__packed__));

union DTD_MD {
    struct DetailedTimingDescriptor dtd;
    struct MonitorDescriptor md;
};

#define EDID_STI_DescriptorUnused           0x0101
#define EDID_STI_AspectRatio_16_10          0
#define EDID_STI_AspectRatio_4_3            1
#define EDID_STI_AspectRatio_5_4            2
#define EDID_STI_AspectRatio_16_9           3
struct StandardTimingIdentification {
    uint8_t HorizontalActivePixels;
    uint8_t ImageAspectRatio    : 2;
    uint8_t RefreshRate         : 6;
}__attribute__((__packed__));

struct VID_Analog {
    uint8_t SerationOnVerticalSync  : 1;
    uint8_t SyncSignalOnGreen       : 1;
    uint8_t SyncSignalOnHorizontal  : 1;
    uint8_t SeparateSyncHandVSignals: 1;
    uint8_t VideoSetupBlank         : 1;
    uint8_t SignalLevelStandard     : 2;
    uint8_t DigitalSignalLevel      : 1; /* Analog = 0, Digital = 1*/
}__attribute__((__packed__));

struct VID_Digital {
    uint8_t DigitalVideoStandardSupp: 4;
    uint8_t ColorBitDepth           : 3;
    uint8_t DigitalSignalLevel      : 1;
}__attribute__((__packed__));

/* Color Bit Depths */
#define CBD_undef               0x0
#define CBD_6bPerPrimaryColor   0x1
#define CBD_8bPerPrimaryColor   0x2
#define CBD_10bPerPrimaryColor  0x3
#define CBD_12bPerPrimaryColor  0x4
#define CBD_14bPerPrimaryColor  0x5
#define CBD_16bPerPrimaryColor  0x6
#define CBD_reserved            0x7

/* Digital Video Standard Supported */
#define DVS_undef               0x0
#define DVS_DVI                 0x1
#define DVS_HDMI-a              0x2
#define DVS_HDMI-b              0x3
#define DVS_MDDI                0x4
#define DVS_DiplayPort          0x5

union VideoInputDefinition {
    struct VID_Analog analog;
    struct VID_Digital digital;
};

struct edid1{
    uint8_t Header[8];
/*  Vendor Product Identification */
    uint16_t IDManufacturerName;
    uint16_t IDProductCode;
    uint32_t IDSerialNumber;
    uint8_t WeekofManufacture;
    uint8_t YearofManufacture;
/*  EDID Structure Version Revision Level */
    uint8_t Version;
    uint8_t Revision;
/*  Basic Display Parameters Features */
    /* Video Input Definition */
    union VideoInputDefinition vid;
    uint8_t MaxHorizontalImageSize;
    uint8_t MaxVerticalImageSize;
    uint8_t DisplayTransferCharacteristic;
    /* Feature Support */
    uint8_t Feature_GTFSupported                : 1;
    uint8_t Feature_PreferredTimingMode         : 1;
    uint8_t Feature_StandardDefaultColorSpace   : 1;
    uint8_t Feature_DisplayType                 : 2;
        /* Refer to VESA DPMS Specification */
    uint8_t Feature_ActiveOff                   : 1;
    uint8_t Feature_Suspend                     : 1;
    uint8_t Feature_StandBy                     : 1;
/*  Color Characteristics */
    uint8_t GreenYLow   : 2;
    uint8_t GreenXLow   : 2;
    uint8_t RedYLow     : 2;
    uint8_t RedXLow     : 2;
    uint8_t WhiteYLow   : 2;
    uint8_t WhiteXLow   : 2;
    uint8_t BlueYLow    : 2;
    uint8_t BlueXLow    : 2;
    uint8_t RedXHigh;
    uint8_t RedYHigh;
    uint8_t GreenXHigh;
    uint8_t GreenYHigh;
    uint8_t BlueXHigh;
    uint8_t BlueYHigh;
    uint8_t WhiteXHigh;
    uint8_t WhiteYHigh;
/*  Established Timings I */
    uint8_t EST_800x600_60Hz    : 1;
    uint8_t EST_800x600_56Hz    : 1;
    uint8_t EST_640x480_75Hz    : 1;
    uint8_t EST_640x480_72Hz    : 1;
    uint8_t EST_640x480_67Hz    : 1;
    uint8_t EST_640x480_60Hz    : 1;
    uint8_t EST_720x400_88Hz    : 1;
    uint8_t EST_720x400_70Hz    : 1;
/*  Established Timings II */
    uint8_t EST_1280x1024_75Hz  : 1;
    uint8_t EST_1024x768_75Hz   : 1;
    uint8_t EST_1024x768_70Hz   : 1;
    uint8_t EST_1024x768_60Hz   : 1;
    uint8_t EST_1024x768_87Hz   : 1;
    uint8_t EST_832x624_75Hz    : 1;
    uint8_t EST_800x600_75Hz    : 1;
    uint8_t EST_800x600_72Hz    : 1;
/*  Manufacturer's Timings */
    uint8_t EST_Reserved        : 7;
    uint8_t EST_1152x870_75Hz   : 1;
/*  Standard Timing Identification */
    struct StandardTimingIdentification STI[8];
/*  Detailed Timing Descriptions / Monitor Descriptions */
    union DTD_MD dtd_md[4];
    uint8_t ExtensionFlag;
    uint8_t Checksum;
}__attribute__((__packed__));

/**** structs and defines for EDIDv2 ****/

struct EDID2_AnalogInterfaceDataFormatDescription{
    uint8_t SyncInputsSupported0            : 1;
    uint8_t SyncInputsSupported1            : 1;
    uint8_t SyncInputsSupported2            : 1;
    uint8_t SyncInputsSupported3            : 1;
    uint8_t Setup                           : 1;
    uint8_t SignalLevelStandard             : 2;
    uint8_t Reserved0                       : 1;

    uint8_t Reserved1                       : 7;
    uint8_t PixelClockSupported             : 1;
    uint8_t Reserved2;
    uint8_t Reserved3;
}__attribute__((__packed__));

struct EDID2_DigitalInterfaceDataFormatDescription{
    uint8_t ChannelSpeedExponent    : 4;
    uint8_t NumberOfReceiverUnits   : 2;
    uint8_t EdgeOfShiftClockUsed    : 1;
    uint8_t DisplayEnablePolarity   : 1;

    uint8_t MinimumChannelSpeed;
    uint8_t MaximumChannelSpeed;
    uint8_t DigitalInterfaceDataFormat;
}__attribute__((__packed__));

union EDID2_InterfaceDataFormatDescription{
    struct EDID2_AnalogInterfaceDataFormatDescription analog;
    struct EDID2_DigitalInterfaceDataFormatDescription digital;
};

struct edid2{
/*  EDID Structure Version/Revision */
    uint8_t Revision : 4;
    uint8_t Version : 4;
/*  Vendor / Product Identification */
    uint16_t IDManufacturerName;
    uint16_t IDProductCode;
    uint8_t WeekOfManufacture;
    uint16_t YearOfManufacture;
/*  Manufacturer/Product Name String */
    char ManufacturerProductIDstring[32];
/*  Serial Number String */
    char SerialNumberString[16];
/*  Unused (Reserved) */
    uint8_t Unused0[8];
/*  Display Interface Parameters */
    /* Physical Connector Type */
    uint8_t PhysicalInterfaceSecondary  : 4; /* see Physical Connector Types */
    uint8_t PhysicalInterfaceDefault    : 4; /* see Physical Connector Types */
    uint8_t VideoInterfaceTypeSecondary     : 4;
    uint8_t VideoInterfaceTypeDefault       : 4;
    union EDID2_InterfaceDataFormatDescription IDFDefault;
    union EDID2_InterfaceDataFormatDescription IDFSecondary;
    /* Interface Color Luminance Encoding */
    uint8_t CLE_ColorencodingSecondary      : 4;
    uint8_t CLE_ColorencodingDefault        : 4;
        /* SC - sub-channel; 0 - red, 1 - green, 2 - blue */
    uint8_t CLE_supportedBitDepthSC1Default     : 4;
    uint8_t CLE_supportedBitDepthSC0Default     : 4;
    uint8_t CLE_supportedBitDepthSC3Default     : 4;
    uint8_t CLE_supportedBitDepthSC2Default     : 4;
    uint8_t CLE_supportedBitDepthSC1Secondary   : 4;
    uint8_t CLE_supportedBitDepthSC0Secondary   : 4;
    uint8_t CLE_supportedBitDepthSC3Secondary   : 4;
    uint8_t CLE_supportedBitDepthSC2Secondary   : 4;
/*  Display Device Description */
    uint8_t DisplayTechnologySubtype        : 4;
    uint8_t DisplayTechnologyType           : 4;
    /* Major Display Characteristics */
    uint8_t MDC_PhysicalImplementation      : 2;
    uint8_t MDC_DisplayBackground           : 1;
    uint8_t MDC_ScanOrientation             : 2;
    uint8_t MDC_Conditionalupdate           : 1;
    uint8_t MDC_SelectableDisplayChromacity : 1;
    uint8_t MDC_Color                       : 1;
    /* Features */
    uint8_t Features_Reserved0              : 1;
    uint8_t Features_StereoSupport          : 3;
    uint8_t Features_Off                    : 1;
    uint8_t Features_ActiveOff              : 1;
    uint8_t Features_Suspend                : 1;
    uint8_t Features_StandBy                : 1;

    uint8_t Features_VideoInput             : 2;
    uint8_t Features_AudioOutputInterface   : 2;
    uint8_t Features_AudioOutput            : 1;
    uint8_t Features_AudioInputInterface    : 2;
    uint8_t Features_AudioInput             : 1;

    uint8_t Features_Reserved1              : 3;
    uint8_t Features_AdjustableOrientation  : 1;
    uint8_t Features_Colorimeter            : 1;
    uint8_t Features_LuminanceProbe         : 1;
    uint8_t Features_LightPen               : 1;
    uint8_t Features_TouchScreen            : 1;
/*  Display Response Time; Seconds - Range 0-15 * 10^(-n), n=timeexp */
    uint8_t DRT_RiseTimeResponseInSeconds   : 4;
    uint8_t DRT_RiseTimeExponent            : 4;
    uint8_t DRT_FallTimeResponseInSeconds   : 4;
    uint8_t DRT_FallTimeExponent            : 4;
/*  Color Luminance Description */
    uint8_t DisplayTransferCharacteristic_WhiteGamma;
    uint8_t DisplayTransferCharacteristic_Color0Gamma;
    uint8_t DisplayTransferCharacteristic_Color1Gamma;
    uint8_t DisplayTransferCharacteristic_Color2Gamma;
    uint16_t MaximumLuminanceWhite;
    uint8_t LuminanceFlag_OffsetSignBit     : 1;
    uint8_t LuminanceFlags_Reserved         : 5;
    uint8_t LuminanceFlag_AdjustableGamma   : 1;
    uint8_t LuminanceFlag_StandardRGBmodel  : 1;
    uint8_t LuminanceOffset;
    /* Chromacity and Default White Point Coordinates */
    uint8_t ColorCharacteristicsGreenYLow   : 2;
    uint8_t ColorCharacteristicsGreenXLow   : 2;
    uint8_t ColorCharacteristicsRedYLow     : 2;
    uint8_t ColorCharacteristicsRedXLow     : 2;
    uint8_t ColorCharacteristicsWhiteYLow   : 2;
    uint8_t ColorCharacteristicsWhiteXLow   : 2;
    uint8_t ColorCharacteristicsBlueYLow    : 2;
    uint8_t ColorCharacteristicsBlueXLow    : 2;
    uint8_t ColorCharacteristicsRedXHigh;
    uint8_t ColorCharacteristicsRedYHigh;
    uint8_t ColorCharacteristicsGreenXHigh;
    uint8_t ColorCharacteristicsGreenYHigh;
    uint8_t ColorCharacteristicsBlueXHigh;
    uint8_t ColorCharacteristicsBlueYHigh;
    uint8_t ColorCharacteristicsWhiteXHigh;
    uint8_t ColorCharacteristicsWhiteYHigh;
    /* Additional White Points And Gamma */
    struct ColorPointData cpd[2];
/*  Display Spatial Description */
    uint16_t MaxHorizontalImageSize;
    uint16_t MaxVerticalImageSize;
    uint16_t MaxHorizontalAddressibility;
    uint16_t MaxVerticalAddressibility;
    uint8_t HorizontalPixelPitch;
    uint8_t VerticalPixelPitch;
/*  Unused (Reserved) */
    uint8_t Unused1;
/*  GTF Support Information */
    uint8_t GTFSupportInformationSecond : 4;
    uint8_t GTFSupportInformation       : 4;
/*  Map Of Timing Information */
    uint8_t MTI_NumberOfDetailedRangeLimitsListed   : 2;
    uint8_t MTI_NumberOfFrequencyRangesListed       : 3;
    uint8_t MTI_LuminanceTableProvided              : 1;
    uint8_t MTI_PreferredTimingMode                 : 1;
    uint8_t MTI_ExtensionFlag                       : 1;
    uint8_t MTI_NumberOfDetailedTimingDescriptions  : 3;
    uint8_t MTI_NumberOf4byteTimingCodesListed      : 5;
/* 80h  Luminance Table Timing Descriptions */
    uint8_t LuminanceTableTimingDescriptions[0x7F];
/*  x*A Luminance Table */
/*  8*B Range Limits */
/* 27*C 27-Byte Detailed Range Limits */
/*  4*D 4-Byte Timing Codes */
/* 18*E 18-Byte Detailed Timing Descriptions */
/* zeros up to FEh */
    uint8_t Checksum;
}__attribute__((__packed__));

/* Luminance Table */
struct LuminanceTable {
    uint8_t NumberOfLuminanceEntries        : 5; /* n */
    uint8_t Reserved                        : 2;
    uint8_t SeparateSubChannels             : 1; /* s */
    uint8_t LuminanceValues; /* if s==0 there are n else 3*n bytes here */
}__attribute__((__packed__));
/* Range Limits */
struct DisplayTimingRangeLimits {
    uint8_t MinFrameFieldRateInHzHigh;
    uint8_t MaxFrameFieldRateInHzHigh;
    uint8_t MinLineRateInkHzHigh;
    uint8_t MaxLineRateInkHzHigh;
    /* XXX low values are not correctly specified in used standard - document */
    uint8_t MaxLineRateInkHzLow             : 2; 
    uint8_t MinLineRateInkHzLow             : 2;
    uint8_t MaxFrameFieldRateInHzLow        : 2;
    uint8_t MinFrameFieldRateInHzLow        : 2;
    uint8_t MinPixelRateInMHzLow;
    uint8_t MaxPixelRateInMHzLow;
    uint8_t MaxPixelRateInMHzHigh           : 4;
    uint8_t MinPixelRateInMHzHigh           : 4;
}__attribute__((__packed__));
/* 27-Byte Detailed Range Limits */
struct DetailedTimingRangeFormat27B {
    uint16_t MinPixelClock_div10000;
    uint8_t MinHorizontalBlankingLow;
    uint8_t MinVerticalBlankingLow;
    uint8_t MinVerticalBlankingHigh         : 4;
    uint8_t MinHorizontalBlankingHigh       : 4;
    uint8_t MinHorizontalSyncOffsetLow;
    uint8_t MinHorizontalSyncPulseWidthLow;
    uint8_t MinVerticalSyncPulseWidthLow    : 4;
    uint8_t MinVerticalSyncOffsetLow        : 4;
    uint8_t MinVerticalSyncPulseWidthHigh   : 2;
    uint8_t MinVerticalSyncOffsetHigh       : 2;
    uint8_t MinHorizontalSyncPulseWidthHigh : 2;
    uint8_t MinHorizontalSyncOffsetHigh     : 2;
    uint16_t MaxPixelClock_div10000;
    uint8_t MaxHorizontalBlankingLow;
    uint8_t MaxVerticalBlankingLow;
    uint8_t MaxVerticalBlankingHigh         : 4;
    uint8_t MaxHorizontalBlankingHigh       : 4;
    uint8_t MaxHorizontalSyncOffsetLow;
    uint8_t MaxHorizontalSyncPulseWidthLow;
    uint8_t MaxVerticalSyncPulseWidthLow    : 4;
    uint8_t MaxVerticalSyncOffsetLow        : 4;
    uint8_t MaxVerticalSyncPulseWidthHigh   : 2;
    uint8_t MaxVerticalSyncOffsetHigh       : 2;
    uint8_t MaxHorizontalSyncPulseWidthHigh : 2;
    uint8_t MaxHorizontalSyncOffsetHigh     : 2;
    uint8_t HorizontalImageSizeLow;
    uint8_t VerticalImageSizeLow;
    uint8_t VerticalImageSizeHigh           : 4;
    uint8_t HorizontalImageSizeHigh         : 4;
    uint8_t HorizontalActiveLow;
    uint8_t VerticalActiveLow;
    uint8_t VerticalActiveHigh              : 4;
    uint8_t HorizontalActiveHigh            : 4;
    uint8_t HorizontalBorder;
    uint8_t VerticalBorder;
    uint8_t Flags_Set0                      : 1;
    /* Next four bits specified for digital interfaces. Meaning for analog interfaces shall be found in the document mentioned above. */
    uint8_t Flags_LPPolarity                : 1;
    uint8_t Flags_FLMPolarity               : 1;
    uint8_t Flags_Reserved                  : 1;
    uint8_t Flags_ShiftClockUse             : 1;
    uint8_t Flags_Set1                      : 2;
    uint8_t Flags_Interlaced                : 1;
}__attribute__((__packed__));
/* 4-Byte Timing Codes */
struct TimingCodeFormat4B {
    uint8_t ActivePixelsPerLine;
    uint8_t Reserved                        : 1;
    uint8_t SHFTCLKedgesUsed                : 1;
    uint8_t FLMPolarity                     : 1;
    uint8_t LPPolarity                      : 1;
    uint8_t Portrait                        : 1;
    uint8_t Stereo                          : 1;
    uint8_t InterlacedTiming                : 1;
    uint8_t ReducedBlanking                 : 1;
    uint8_t FormatAspectRatio;
    uint8_t RefreshRate;
}__attribute__((__packed__));
/* 18-Byte Detailed Timing Descriptions */
    /* see struct DetailedTimingDescriptor */

/* Physical Connector Types (Default/SecondaryPhysicalInterface) */
#define EDID2_PCT_None                          0x0
#define EDID2_PCT_BNC                           0x1
#define EDID2_PCT_15pinVGA                      0x2
#define EDID2_PCT_13w3                          0x3
#define EDID2_PCT_VESAEVC                       0x4
#define EDID2_PCT_VESAPDD                       0x5
#define EDID2_PCT_Microribbon                   0x6
#define EDID2_PCT_IEEE1394                      0x7
#define EDID2_PCT_VESAFPDI2                     0x8
/* Reserved 0x9-0xE */
#define EDID2_PCT_Nonstandard                   0xF

/* Video Interface Type */
#define EDID2_VIT_None                          0x0
#define EDID2_VIT_Analog                        0x1
#define EDID2_VIT_Analogwsampledpixelclock      0x2
#define EDID2_VIT_TMDS                          0x3 /* Transition Minimized Differential Signaling */
#define EDID2_VIT_IEEE1394_1995                 0x4
#define EDID2_VIT_LVDS                          0x5
#define EDID2_VIT_Parallel                      0x6
/* Reserved 0x7-0xF */

/* digital interface data format */
/* ... */
#define EDID2_didf_RGB_STN-DD_8BitOver8Bit      0x15
#define EDID2_didf_RGB_STN-DD_12BitOver12Bit    0x19
/* ... */
#define EDID2_didf_MSBAlignedRGBTFT24Bit        0x24
/* ... */

/* Color/Luminance Encoding Description */
        /* nibbles of byte 0 - color encodings */
#define EDID2_CLE_CE_Monochrome                 0x0
#define EDID2_CLE_CE_RGBadd                     0x1
#define EDID2_CLE_CE_CMYsub                     0x2
#define EDID2_CLE_CE_CMYKsub                    0x3
#define EDID2_CLE_CE_YC_NTSC                    0x4
#define EDID2_CLE_CE_YC_PAL                     0x5
#define EDID2_CLE_CE_YC_SECAM                   0x6
#define EDID2_CLE_CE_YIQ_NTSC                   0x7
#define EDID2_CLE_CE_YIQ_PAL                    0x8
#define EDID2_CLE_CE_YIQ_SECAM                  0x9
#define EDID2_CLE_CE_YCrCb                      0xA
#define EDID2_CLE_CE_YPrPb                      0xB
#define EDID2_CLE_CE_XYZ                        0xC
#define EDID2_CLE_CE_CIELuv                     0xD
#define EDID2_CLE_CE_CIELab                     0xE
#define EDID2_CLE_CE_Digital_ATSC               0xF

/* Display Device Description; T-type, ST-subtype */
#define EDID2_DDD_T_CRT         0x0 /* Cathode Ray Tube */
#define EDID2_DDD_ST_CRT_Monochrome             0x0
#define EDID2_DDD_ST_CRT_ShadowmaskColor        0x1
#define EDID2_DDD_ST_CRT_BeamIndexColor         0x2
#define EDID2_DDD_ST_CRT_BeamPenetrationColor   0x3

#define EDID2_DDD_T_LCD         0x1 /* Liquid Crystal Display */
#define EDID2_DDD_ST_LCD_STN                    0x0
#define EDID2_DDD_ST_LCD_DSTN                   0x1
#define EDID2_DDD_ST_LCD_FLCD                   0x2
#define EDID2_DDD_ST_LCD_TFTconv                0x3
#define EDID2_DDD_ST_LCD_TFTinplaneswtch        0x4
#define EDID2_DDD_ST_LCD_PDLCD                  0x5
#define EDID2_DDD_ST_LCD_PSCLCD                 0x6
#define EDID2_DDD_ST_LCD_PALCD                  0x7

#define EDID2_DDD_T_EL          0x2 /* Electroluminiscent Displ */
#define EDID2_DDD_ST_EL_ACThin                  0x0
#define EDID2_DDD_ST_EL_ACThick                 0x1
#define EDID2_DDD_ST_EL_DCThin                  0x2
#define EDID2_DDD_ST_EL_DCThick                 0x3

#define EDID2_DDD_T_PDP         0x3 /* Plasma Display Panel */
#define EDID2_DDD_ST_PDP_AC                     0x0
#define EDID2_DDD_ST_PDP_DC                     0x1

#define EDID2_DDD_T_FED         0x4 /* Field Emission Displays */
#define EDID2_DDD_ST_FED_TBD                    0x0

#define EDID2_DDD_T_LED         0x5 /* Light Emitting Diode */
#define EDID2_DDD_ST_LED_TBD                    0x0

#define EDID2_DDD_T_IRD         0x6 /* Integrated Reflective Display */
#define EDID2_DDD_ST_IRD_1Chip                  0x0
#define EDID2_DDD_ST_IRD_2Chip                  0x1
#define EDID2_DDD_ST_IRD_3Chip                  0x2

    /* Types 0x7-0xF Reserved */

/* MajorDisplayCharacteristics */
#define EDID2_MDC_C_Color               1
#define EDID2_MDC_C_Mono                0
#define EDID2_MDC_SO_notscanned         0x0
#define EDID2_MDC_SO_fastLRslowTB       0x1
#define EDID2_MDC_SO_fastTBslowLR       0x2
#define EDID2_MDC_SO_fastTBslowRL       0x3
#define EDID2_MDC_DB_NonTransp          0
#define EDID2_MDC_DB_Transparent        1
#define EDID2_MDC_PI_Large              0x0
#define EDID2_MDC_PI_Desktop            0x1
#define EDID2_MDC_PI_Eyepiece           0x2

/* FeaturesSupport */
        /* byte 0 - refer to VESA DPMS specification */
            /* Stereo Support */
#define EDID2_FS_SS_nodirectstereo      0x0
#define EDID2_FS_SS_fieldseqstereo      0x1
#define EDID2_FS_SS_directviewcolumnilv 0x2
#define EDID2_FS_SS_directviewlineilv   0x3
        /* byte 1 */
            /* VideoInputValues */
#define EDID2_FX_VI_None                0x0
#define EDID2_FX_VI_AnalogYCconnEVC     0x1
#define EDID2_FX_VI_USBCam              0x2
#define EDID2_FX_VI_IEEE1394Cam         0x3
            /* AudioOutputInterfaceValues */
#define EDID2_FS_AOI_None               0x0
#define EDID2_FS_AOI_Analog             0x1
#define EDID2_FS_AOI_USB                0x2
#define EDID2_FS_AOI_IEEE1394           0x3
            /* Audio Input Interface Values */
#define EDID2_FS_AII_None               0x0
#define EDID2_FS_AII_Analog             0x1
#define EDID2_FS_AII_USB                0x2
#define EDID2_FS_AII_IEEE1394           0x3

/* GTFSupportInformation values */
#define EDID2_GTFSI_nosupport       0x0
#define EDID2_GTFSI_stdCRTparams    0x1
#define EDID2_GTFSI_defaultTBD      0x2
#define EDID2_GTFSI_custom          0xF


union edid {
    struct edid1 edid1;
    struct edid2 edid2;
};

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* ASM */

#endif /* _VBE_H */
