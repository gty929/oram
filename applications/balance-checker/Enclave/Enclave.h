#pragma once

#include <assert.h>
#include <stdlib.h>
#ifndef ENCLAVE_MODE_ENCLAVE
#define ENCLAVE_MODE_ENCLAVE
#endif
#if defined(__cplusplus)
extern "C" {
#endif

void printf(const char *fmt, ...);

#if defined(__cplusplus)
}
#endif
