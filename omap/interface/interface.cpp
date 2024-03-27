#include "interface/recoram_interface.hpp"
#include "odsl/recursive_oram.hpp"

using namespace ODSL;

ORAMBindingSingleton::ORAMBindingSingleton() {
  uint64_t BackendSize = 1e10;
  EM::Backend::g_DefaultBackend =
      new EM::Backend::MemServerBackend(BackendSize);
  oram = nullptr;
}

void ORAMBindingSingleton::InitORAM(uint32_t size) {
  // ASSERT(oram == nullptr);
  oram = (void*)(new RecursiveORAM<uint64_t, uint32_t>(size));
  ((RecursiveORAM<uint64_t, uint32_t>*)oram)->InitDefault(0);
}

void ORAMBindingSingleton::Write(uint32_t addr, uint64_t val) {
  ((RecursiveORAM<uint64_t, uint32_t>*)oram)->Write(addr, val);
}

uint64_t ORAMBindingSingleton::Read(uint32_t addr) {
  uint64_t ret;
  ((RecursiveORAM<uint64_t, uint32_t>*)oram)->Read(addr, ret);
  return ret;
}

ORAMBindingSingleton::~ORAMBindingSingleton() {
  if (oram) {
    delete (RecursiveORAM<uint64_t, uint32_t>*)oram;
  }
}

#include "interface/omap_interface.hpp"
#include "odsl/omap.hpp"

using namespace ODSL;

using K = uint64_t;
using V = uint64_t;
using MapType = OHashMap<K, V, true, uint32_t>;
using InitializerType = typename MapType::InitContext;

OMapBindingSingleton::OMapBindingSingleton() {
  uint64_t BackendSize = 1e10;
  EM::Backend::g_DefaultBackend =
      new EM::Backend::MemServerBackend(BackendSize);
  omap = nullptr;
  initializer = nullptr;
}

void OMapBindingSingleton::InitEmpty(uint32_t size) {
  omap = (void*)(new MapType(size));
  ((MapType*)omap)->Init();
}

void OMapBindingSingleton::StartInit(uint32_t size) {
  omap = (void*)(new MapType(size));
  initializer = (void*)(((MapType*)omap)->NewInitContext());
}

void OMapBindingSingleton::FinishInit() {
  Assert(initializer, "FinishInit without StartInit");
  ((InitializerType*)initializer)->Finalize();
  delete (InitializerType*)initializer;
  initializer = nullptr;
}

bool OMapBindingSingleton::Insert(K key, V val) {
  if (initializer) {
    ((InitializerType*)initializer)->Insert(key, val);
    return false;
  }
  return ((MapType*)omap)->Insert(key, val);
}

bool OMapBindingSingleton::OInsert(K key, V val) {
  if (initializer) {
    ((InitializerType*)initializer)->Insert(key, val);
    return false;
  }
  return ((MapType*)omap)->OInsert(key, val);
}

bool OMapBindingSingleton::Find(K key, V& val) {
  Assert(!initializer, "Find during initialization");
  return ((MapType*)omap)->Find(key, val);
}

bool OMapBindingSingleton::Erase(K key) {
  Assert(!initializer, "Erase during initialization");
  return ((MapType*)omap)->Erase(key);
}

bool OMapBindingSingleton::OErase(K key) {
  Assert(!initializer, "Erase during initialization");
  return ((MapType*)omap)->OErase(key);
}

OMapBindingSingleton::~OMapBindingSingleton() {
  if (omap) {
    delete (MapType*)omap;
  }
  if (initializer) {
    delete (InitializerType*)initializer;
  }
}
