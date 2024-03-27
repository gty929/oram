#pragma once
#include <cinttypes>

typedef void* omap_t;
typedef void* initializer_t;

struct OMapBindingSingleton {
  omap_t omap;
  initializer_t initializer;
  OMapBindingSingleton();
  void InitEmpty(uint32_t size);
  void InitEmptyExternal(uint32_t size, uint64_t cacheBytes);
  bool Insert(uint64_t key, uint64_t val);
  bool OInsert(uint64_t key, uint64_t val);
  bool Find(uint64_t key, uint64_t& val);
  bool Erase(uint64_t key);
  bool OErase(uint64_t key);
  void StartInit(uint32_t size);
  void StartInitExternal(uint32_t size, uint64_t cacheBytes);
  void FinishInit();
  ~OMapBindingSingleton();
};
