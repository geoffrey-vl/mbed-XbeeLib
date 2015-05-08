/**
 * Digi XBee library for mbed. This is the only header you have to include from your application to use the library.
 *
 * Copyright (c) 2015 Digi International Inc.,
 * All rights not expressly granted are reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Digi International Inc. 11001 Bren Road East, Minnetonka, MN 55343.
 */
 
/** @file
 */

#if !defined(__XBEE_H_)
#define __XBEE_H_

#define XB_LIBRARY_VERSION          0x00010500U
#define XB_MAJOR_VERSION            ((XB_LIBRARY_VERSION >> 24) & 0xFFU)
#define XB_MINOR_VERSION            ((XB_LIBRARY_VERSION >> 16) & 0xFFU)
#define XB_PATCH_LEVEL              ((XB_LIBRARY_VERSION >> 8) & 0xFFU)
#define XB_BUILD_ID                 (XB_LIBRARY_VERSION & 0xFFU)
 
/**/
#define XB_LIB_BANNER               "\r\n\r\nmbed Digi Radio library v%d.%d.%d\r\n", \
                                        XB_MAJOR_VERSION, XB_MINOR_VERSION, XB_PATCH_LEVEL

#include "XBeeZB/XBeeZB.h"
#include "IO/IOSampleZB.h"
#include "XBee802/XBee802.h"
#include "IO/IOSample802.h"
#include "XBee/Addresses.h"
#include "RemoteXBee/RemoteXBee.h"

#endif /* __XBEE_H_ */


