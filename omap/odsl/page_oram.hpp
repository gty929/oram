#pragma once

#include "common/utils.hpp"
#include "external_memory/server/serverFrontend.hpp"
#include "external_memory/virtualvector.hpp"
#include "oram_common.hpp"

namespace ODSL {

template <typename T, typename UidType = uint64_t,
          const uint64_t page_size = 4096>
struct Page {
  static constexpr uint64_t item_per_page =
      divRoundUp(page_size, sizeof(UidType) + sizeof(T));
  UidType uids[item_per_page];
  T data[item_per_page];
  using Encrypted_t = FreshEncrypted<Page>;
  Page() {
    for (uint64_t i = 0; i < item_per_page; i++) {
      uids[i] = DUMMY<UidType>();
    }
  }
};

template <typename T, typename UidType = uint64_t,
          typename PageIdxType = uint32_t, const bool includePosMap = true,
          const uint64_t page_size = 4096>
struct PageORAM {
  using Page_ = Page<T, UidType, page_size>;
  using FrontEndType = EM::MemoryServer::NonCachedServerFrontendInstance<
      Page_, EM::Backend::MemServerBackend,
      EM::MemoryServer::EncryptType::ENCRYPT_AND_AUTH_FRESH>;
  using BackendType = typename FrontEndType::BackendType;
  static constexpr uint64_t item_per_page = Page_::item_per_page;

 private:
  std::vector<PageIdxType> posMap;
  struct LinkedNode {
    T data;
    UidType uid;
    uint64_t nextNodeIdx;
  };
  std::vector<LinkedNode> stash;  // A fairly large stash
  // Each page can have a linked list of nodes
  // Since the size of each node is the same, we avoid using malloc. Instead, we
  // manage the free list by ourself. After eviction, we add the head of the
  // linked list to the free list.
  uint64_t freeListHead, freeListTail;
  static constexpr uint64_t listEnd = DUMMY<uint64_t>();
  PageIdxType numPages;
  std::vector<uint64_t> pageLists;
  FrontEndType frontend;
  T defaultVal = T();

 public:
  PageORAM(BackendType& _backend = *::EM::Backend::g_DefaultBackend)
      : frontend(_backend) {}

  PageORAM(uint64_t size,
           BackendType& _backend = *::EM::Backend::g_DefaultBackend)
      : frontend(_backend) {
    SetSize(size);
  }

  PageORAM(uint64_t size, uint64_t cacheBytes,
           BackendType& _backend = *::EM::Backend::g_DefaultBackend)
      : frontend(_backend) {
    SetSize(size, cacheBytes);
  }

  void SetSize(uint64_t size, uint64_t cacheBytes = 0) {
    numPages = divRoundUp(size, item_per_page) * 1.3;
    // std::cout << "size: " << size << " numPages: " << numPages << std::endl;
    frontend.SetSize(numPages);
    if constexpr (includePosMap) {
      posMap.resize(size);
      for (uint64_t i = 0; i < size; i++) {
        posMap[i] = UniformRandom(numPages - 1);
      }
    }
    stash.reserve(numPages);
    stash.resize(1);
    stash[0].nextNodeIdx = listEnd;
    pageLists.resize(numPages, listEnd);
    freeListHead = 0;
    freeListTail = 0;
  }

  PageIdxType GetRandPos() const {
    static_assert(!includePosMap);
    return UniformRandom(numPages - 1);
  }

  /**
   * @brief Update a block in the ORAM and assign it to a new position
   *
   * @tparam Func The type of the update function. The update function should
   * take a reference to the block and return a bool. If the return value is
   * true, the block is kept, otherwise it is deleted.
   * @param pos The current position of the block
   * @param uid The unique id of the block
   * @param newPos The new position of the block
   * @param updateFunc The update function
   * @return PageIdxType
   */
  template <class Func>
    requires UpdateOrRemoveFunction<Func, T>
  PageIdxType Update(PageIdxType pos, const UidType& uid, PageIdxType newPos,
                     const Func& updateFunc) {
    static_assert(!includePosMap);
    Page_ page;
    frontend.Read(pos, page);
    // add remove
    access(uid, updateFunc, pos, page, newPos);
    frontend.Write(pos, page);
    return newPos;
  }

  /**
   * @brief Read a block from the ORAM and assign it to a new position
   *
   * @param pos The current position of the block
   * @param uid The unique id of the block
   * @param out The output block
   * @param newPos The new position of the block
   * @return PageIdxType The new position of the block
   */
  PageIdxType Read(PageIdxType pos, const UidType& uid, T& out,
                   PageIdxType newPos) {
    static_assert(!includePosMap);
    auto accessor = [&](const T& data) {
      out = data;
      return true;
    };
    return Update(pos, uid, newPos, accessor);
  }

  template <class Func>
    requires UpdateFunction<Func, T>
  void Access(UidType address, const Func& accessor) {
    static_assert(includePosMap);
    PageIdxType pageIdx = posMap[address];
    Page_ page;
    frontend.Read(pageIdx, page);
    PageIdxType newPageIdx = UniformRandom(numPages - 1);
    access(address, accessor, pageIdx, page, newPageIdx);
    frontend.Write(pageIdx, page);
  }

  void PauseWorker() {}

  void ResumeWorker() {}

  void PauseWorkers() {}

  void ResumeWorkers() {}

  uint64_t GetMemoryUsage() const {
    return sizeof(PageORAM) + sizeof(PageIdxType) * numPages +
           sizeof(UidType) * posMap.size() + sizeof(LinkedNode) * stash.size() +
           sizeof(UidType) * pageLists.size();
  }

  /**
   * @brief Read the data at a given address
   *
   * @param address The address to read
   * @param out The output data
   */
  void Read(UidType address, T& out) {
    static_assert(includePosMap);
    Access(address, [&](const T& data) { out = data; });
  }

  /**
   * @brief Write the data at a given address
   *
   * @param address The address to write
   * @param in The input data
   */
  void Write(UidType address, const T& in) {
    static_assert(includePosMap);
    Access(address, [&](T& data) { data = in; });
  }

  template <typename Reader>
    requires Readable<Reader, T>
  void InitFromReader(Reader& reader,
                      uint64_t cacheBytes = DEFAULT_HEAP_SIZE / 5) {
    static_assert(includePosMap);
    // first load everything to the external array
    UidType uid = 0;
    uint64_t initSize = reader.size();
    for (PageIdxType i = 0; i < numPages; i++) {
      if (reader.eof()) {
        break;
      }
      Page_ page;
      frontend.Read(i, page);
      for (uint64_t j = 0; j < item_per_page; j++) {
        if (reader.eof()) {
          break;
        }
        page.data[j] = reader.read();
        page.uids[j] = uid++;
      }
      frontend.Write(i, page);
    }
    EM::VirtualVector::VirtualWriter<UidBlock<T, UidType>> overflowWriter(
        initSize, [&](const uint64_t idx, const UidBlock<T, UidType>& block) {
          uint64_t slot = getFreeSlot();
          stash[slot].data = block.data;
          stash[slot].uid = block.uid;
          addSlotToPageList(posMap[block.uid], slot);
        });
    partitionHelper(0, numPages, cacheBytes, overflowWriter);
  }

  void InitDefault(const T& defaultVal) { this->defaultVal = defaultVal; }

  void PrintMemoryUsage() {
    const double MBinv = 1.0 / 1024 / 1024;
    printf(
        "PageORAM: %lu elements, %lu pages, maximum of %lu elements in stash\n",
        posMap.size(), (uint64_t)numPages, stash.size());
    printf("Raw data size: %f MB\n", posMap.size() * sizeof(T) * MBinv);
    uint64_t stashBytes = stash.size() * sizeof(LinkedNode);
    uint64_t posMapBytes = posMap.size() * sizeof(PageIdxType);
    uint64_t pageListPtrsBytes = pageLists.size() * sizeof(UidType);
    uint64_t freshnessCheckBytes = numPages * sizeof(uint32_t);

    printf("Heap usage: %f MB\n", (stashBytes + posMapBytes +
                                   pageListPtrsBytes + freshnessCheckBytes) *
                                      MBinv);
    printf(
        "(Stash: %f MB, posMap: %f MB, page list heads: %f MB, "
        "freshnessCheck: %f MB)\n",
        stashBytes * MBinv, posMapBytes * MBinv, pageListPtrsBytes * MBinv,
        freshnessCheckBytes * MBinv);
    printf("External memory usage: %f MB\n",
           numPages * sizeof(typename Page_::Encrypted_t) * MBinv);
  }

 private:
  template <class Func>
    requires UpdateFunction<Func, T>
  void access(UidType address, const Func& accessor, PageIdxType pageIdx,
              Page_& page, PageIdxType newPageIdx) {
    bool foundInStash = false;
    bool foundInPage = false;

    T* dataPtr = nullptr;
    UidType* addressPtr = nullptr;
    uint64_t* prevIdxPtr = &pageLists[pageIdx];
    uint64_t currIdx = *prevIdxPtr;
    while (currIdx != listEnd) {
      if (stash[currIdx].uid == address) {
        dataPtr = &stash[currIdx].data;
        if constexpr (!includePosMap) {
          addressPtr = &stash[currIdx].uid;
        }
        // remove the node from the page linked list
        *prevIdxPtr = stash[currIdx].nextNodeIdx;
        // add the node to the head of the new page linked list
        addSlotToPageList(newPageIdx, currIdx);
        foundInStash = true;
        // break;
      }
      prevIdxPtr = &stash[currIdx].nextNodeIdx;
      currIdx = *prevIdxPtr;
    }

    for (uint64_t i = 0; i < item_per_page; i++) {
      if (page.uids[i] == address) {
        dataPtr = &page.data[i];
        page.uids[i] = DUMMY<UidType>();
        foundInPage = true;
        // break;
      }
    }
    if (!foundInStash) {
      uint64_t freeSlot = getFreeSlot();
      stash[freeSlot].uid = address;
      const T* src = foundInPage ? dataPtr : &defaultVal;

      memcpy(&stash[freeSlot].data, src, sizeof(T));

      dataPtr = &stash[freeSlot].data;
      if constexpr (!includePosMap) {
        addressPtr = &stash[freeSlot].uid;
      }
      addSlotToPageList(newPageIdx, freeSlot);
    }

    bool keepFlag = true;
    // Write back and maintainence
    if constexpr (includePosMap) {
      accessor(*dataPtr);
      posMap[address] = newPageIdx;
    } else {
      keepFlag = accessor(*dataPtr);
      if (!keepFlag) {
        *addressPtr = DUMMY<UidType>();
        // first set the data to dummy
      }
    }

    uint32_t freeSlotIndices[item_per_page];
    uint32_t numFreeSlots = 0;
    for (uint64_t i = 0; i < item_per_page; i++) {
      if (page.uids[i] == DUMMY<UidType>()) {
        freeSlotIndices[numFreeSlots++] = i;
      }
    }
    // evict the data of the current page
    uint64_t prev = listEnd;
    uint64_t curr = pageLists[pageIdx];
    for (uint64_t i = 0; i < numFreeSlots; i++) {
      if (curr == listEnd) {
        break;
      }

      uint32_t freeSlotIdx = freeSlotIndices[i];
      if constexpr (!includePosMap) {
        if (stash[curr].uid == DUMMY<UidType>()) {
          // the stash slot is dummy, skip
          prev = curr;
          curr = stash[curr].nextNodeIdx;
          continue;
        }
      }
      page.uids[freeSlotIdx] = stash[curr].uid;
      memcpy(&page.data[freeSlotIdx], &stash[curr].data, sizeof(T));

      prev = curr;
      curr = stash[curr].nextNodeIdx;
    }
    if (prev != listEnd) {
      // add the evicted slots to the free list
      stash[prev].nextNodeIdx = freeListHead;
      freeListHead = pageLists[pageIdx];
    }
    pageLists[pageIdx] = curr;
  }

  template <class OverflowWriter>
  void partitionHelper(PageIdxType beginPageIdx, PageIdxType endPageIdx,
                       uint64_t cacheBytes, OverflowWriter& overflowWriter) {
    static_assert(includePosMap);
    PageIdxType pageCounts = endPageIdx - beginPageIdx;
    if (pageCounts <= 1) {
      return;
    }
    PageIdxType maxWay = cacheBytes / sizeof(Page_);
    int levelNeeded = 1;
    for (PageIdxType totalWay = maxWay; totalWay < pageCounts;
         totalWay *= maxWay) {
      levelNeeded++;
    }
    PageIdxType way = ceil(pow(pageCounts, 1.0 / levelNeeded));
    PageIdxType stepSize = cacheBytes / (way * sizeof(Page_));
    if (stepSize == 0) {
      stepSize = 1;
    }
    PageIdxType partitionSize = divRoundUp(pageCounts, way);
    stepSize = std::min(stepSize, partitionSize);
    PageIdxType numSteps = divRoundUp(partitionSize, stepSize);
    {
      std::vector<std::vector<UidBlock<T, UidType>>> partitions(way);
      for (PageIdxType i = 0; i < way; i++) {
        partitions[i].reserve(stepSize * item_per_page * 2);
      }
      std::vector<PageIdxType> stepBeginIndices(way);
      std::vector<PageIdxType> stepEndIndices(way);
      for (PageIdxType i = 0; i < numSteps; i++) {
        for (PageIdxType wayIdx = 0; wayIdx < way; wayIdx++) {
          stepBeginIndices[wayIdx] = std::min(
              beginPageIdx + i * stepSize + wayIdx * partitionSize, endPageIdx);
          stepEndIndices[wayIdx] =
              std::min(std::min(stepBeginIndices[wayIdx] + stepSize,
                                beginPageIdx + (wayIdx + 1) * partitionSize),
                       endPageIdx);
        }
        for (PageIdxType wayIdx = 0; wayIdx < way; wayIdx++) {
          PageIdxType stepBeginPageIdx = stepBeginIndices[wayIdx];
          PageIdxType stepEndPageIdx = stepEndIndices[wayIdx];
          for (PageIdxType j = stepBeginPageIdx; j < stepEndPageIdx; j++) {
            Page_ page;
            frontend.Read(j, page);
            for (uint32_t k = 0; k < item_per_page; k++) {
              UidType uid = page.uids[k];
              if (uid != DUMMY<UidType>()) {
                PageIdxType pageIdx = posMap[uid];
                PageIdxType partitionIdx =
                    (pageIdx - beginPageIdx) / partitionSize;
                uint64_t partitionMaxSize = (stepEndIndices[partitionIdx] -
                                             stepBeginIndices[partitionIdx]) *
                                            item_per_page;
                // printf("partitionIdx: %u, partitionMaxSize: %lu\n",
                //        partitionIdx, partitionMaxSize);
                if (partitions[partitionIdx].size() < partitionMaxSize) {
                  partitions[partitionIdx].emplace_back(page.data[k], uid);
                } else {
                  overflowWriter.write(UidBlock<T, UidType>(page.data[k], uid));
                }
              }
            }
          }
        }
        for (PageIdxType wayIdx = 0; wayIdx < way; wayIdx++) {
          PageIdxType stepBeginPageIdx = stepBeginIndices[wayIdx];
          PageIdxType stepEndPageIdx = stepEndIndices[wayIdx];
          uint64_t wayOffset = 0;
          for (PageIdxType j = stepBeginPageIdx; j < stepEndPageIdx; j++) {
            Page_ page = Page_();
            for (uint32_t k = 0; k < item_per_page; k++) {
              if (wayOffset < partitions[wayIdx].size()) {
                page.uids[k] = partitions[wayIdx][wayOffset].uid;
                page.data[k] = partitions[wayIdx][wayOffset].data;
                wayOffset++;
              }
            }
            frontend.Write(j, page);
          }
          Assert(wayOffset == partitions[wayIdx].size());
          partitions[wayIdx].clear();
        }
      }
    }
    for (PageIdxType wayIdx = 0; wayIdx < way; wayIdx++) {
      PageIdxType wayBeginPageIdx = beginPageIdx + wayIdx * partitionSize;
      PageIdxType wayEndPageIdx =
          std::min(wayBeginPageIdx + partitionSize, endPageIdx);
      partitionHelper(wayBeginPageIdx, wayEndPageIdx, cacheBytes,
                      overflowWriter);
    }
  }

  uint64_t getFreeSlot() {
    if (freeListHead == freeListTail) {  // provision a new slot
      stash.emplace_back();
      freeListTail = stash[freeListTail].nextNodeIdx = stash.size() - 1;
      stash[freeListTail].nextNodeIdx = listEnd;
    }
    uint64_t slot = freeListHead;
    freeListHead = stash[slot].nextNodeIdx;
    return slot;
  }

  void addSlotToPageList(PageIdxType pageIdx, uint64_t slot) {
    stash[slot].nextNodeIdx = pageLists[pageIdx];
    pageLists[pageIdx] = slot;
  }

  void printFreeList() {
    printf("Free list: ");
    uint64_t curr = freeListHead;
    while (curr != listEnd) {
      printf("%lu ", curr);
      curr = stash[curr].nextNodeIdx;
    }
    printf("\n");
  }
};

}  // namespace ODSL