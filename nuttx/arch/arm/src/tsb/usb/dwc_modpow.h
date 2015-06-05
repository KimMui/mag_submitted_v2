/**
 * Copyright (c) 2014-2015 Google Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * dwc_modpow.h
 * See dwc_modpow.c for license and changes
 */
#ifndef _DWC_MODPOW_H
#define _DWC_MODPOW_H

#ifdef __cplusplus
extern "C" {
#endif

#include "dwc_os.h"

/** @file
 *
 * This file defines the module exponentiation function which is only used
 * internally by the DWC UWB modules for calculation of PKs during numeric
 * association.  The routine is taken from the PUTTY, an open source terminal
 * emulator.  The PUTTY License is preserved in the dwc_modpow.c file.
 *
 */

typedef uint32_t BignumInt;
typedef uint64_t BignumDblInt;
typedef BignumInt *Bignum;

/* Compute modular exponentiaion */
extern Bignum dwc_modpow(void *mem_ctx, Bignum base_in, Bignum exp, Bignum mod);

#ifdef __cplusplus
}
#endif

#endif /* _LINUX_BIGNUM_H */
