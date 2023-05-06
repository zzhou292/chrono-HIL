// =============================================================================
// CHRONO-HIL - https://github.com/zzhou292/chrono-HIL
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Jason Zhou
// =============================================================================

#ifndef CH_API_HIL_H
#define CH_API_HIL_H

#include "chrono/core/ChPlatform.h"

// When compiling the Chrono HIL libraries, remember to define
// CH_API_COMPILE_HIL (so that the symbols with 'CH_MODELS_API' in front of
// them will be marked as exported). Otherwise, just do not define it if you
// link the library to your code, and the symbols will be imported.

#if defined(CH_API_COMPILE_HIL)
#define CH_HIL_API ChApiEXPORT
#else
#define CH_HIL_API ChApiIMPORT
#endif

#endif