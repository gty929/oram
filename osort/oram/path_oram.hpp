#pragma once

#include "external_memory/algorithm/bitonic.hpp"
#include "oram_common.hpp"
#include "tree.hpp"

namespace ORAM::PathORAM {
template <typename T, const int Z = 5, const int stashSize = 63,
          typename PositionType = uint64_t, typename UidType = uint64_t>
struct PathORAM {
  // using Node_ = Node<T, Z, PositionType, UidType>;
  using Stash = Bucket<T, stashSize, PositionType, UidType>;
  using Block_ = Block<T, PositionType, UidType>;
  using Bucket_ = Bucket<T, Z, PositionType, UidType>;
  using UidBlock_ = UidBlock<T, UidType>;
  using HeapTree_ = HeapTree<Bucket_, PositionType>;
  // Node_* root = nullptr;
  HeapTree_ tree;
  Stash* stash = nullptr;
  int depth = 0;
  PositionType _size = 0;

  PathORAM() {}

  // PathORAM(const PathORAM& other) = default;
  PathORAM(PathORAM&& other) {
    _size = other._size;
    tree = std::move(other.tree);
    stash = other.stash;
    other.stash = nullptr;
    depth = other.depth;
  }

  PathORAM(PositionType size, size_t cacheBytes = 1UL << 62) {
    SetSize(size, cacheBytes);
  }

  void SetSize(PositionType size, size_t cacheBytes = 1UL << 62) {
    if (_size) {
      throw std::runtime_error("Path ORAM double initialization");
    }

    _size = size;
    int cacheLevel = GetMaxCacheLevel(size, cacheBytes);
    if (cacheLevel < 0) {
      throw std::runtime_error("Path ORAM cache size too small");
    }
    // printf(
    //     "tree size = %lu, cacheBytes = %lu, cacheLevel = %d, element size = "
    //     "%d\n",
    //     size, cacheBytes, cacheLevel, sizeof(T));
    Bucket_ dummyBucket;
    for (int i = 0; i < Z; ++i) {
      dummyBucket.blocks[i].uid = DUMMY<UidType>();
    }
    tree.InitWithDefault(size, dummyBucket, cacheLevel);
    depth = GetLogBaseTwo(size - 1) + 2;
    stash = new Stash();
  }

  static int GetMaxCacheLevel(PositionType size,
                              size_t cacheBytes = 1UL << 62) {
    int maxCacheLevel = 0;
    if (cacheBytes >= HeapTree_::GetMemoryUsage(size, 62)) {
      // default value
      return 62;
    }
    if (cacheBytes < sizeof(Stash)) {
      return -1;
    }
    cacheBytes -= sizeof(Stash);
    for (;; ++maxCacheLevel) {
      size_t treeUsage = HeapTree_::GetMemoryUsage(size, maxCacheLevel);
      if (cacheBytes < treeUsage) {
        break;
      }
    }
    return maxCacheLevel - 1;
  }

  static size_t GetMemoryUsage(PositionType size,
                               size_t cacheBytes = 1UL << 62) {
    return sizeof(Stash) +
           HeapTree_::getMemoryUsage(size, GetMaxCacheLevel(size, cacheBytes));
  }

  size_t GetMemoryUsage() const {
    return sizeof(Stash) + tree.GetMemoryUsage();
  }

  template <typename Reader, class PosMapWriter>
  void InitFromReader(Reader& reader, PosMapWriter& posMapWriter) {
    size_t initSize = reader.size();
    if (initSize * 5 < size()) {
      for (UidType uid = 0; uid != (UidType)initSize; ++uid) {
        PositionType newPos = Write(uid, reader.read());
        posMapWriter.write(UidBlock<PositionType, UidType>(newPos, uid));
      }
      return;
    }
    FastInitFromReader<Reader, PosMapWriter, T, Z, stashSize, PositionType,
                       UidType>(reader, posMapWriter, tree);
  }

  PositionType size() const { return _size; }

  ~PathORAM() {
    if (stash) {
      delete stash;
      stash = nullptr;
    }
  }

  PathORAM& operator=(const PathORAM& other) = default;
  PathORAM& operator=(PathORAM&& other) = default;

  PositionType Read(PositionType pos, const UidType& uid, T& out,
                    PositionType newPos) {
    std::vector<Block_> path = ReadPath(pos);

    ReadElementAndRemoveFromPath(path, uid, out);

    Block_ newBlock(out, newPos, uid);
    WriteNewBlockToPath(path, newBlock);
    EvictPath(path, pos);
    WriteBackPath(path, pos);
    return newPos;
  }

  PositionType Read(PositionType pos, const UidType& uid, T& out) {
    PositionType newPos = UniformRandom(size() - 1);
    return Read(pos, uid, out, newPos);
  }

  PositionType Write(const UidType& uid, const T& in, PositionType newPos) {
    PositionType pos = UniformRandom(size() - 1);
    std::vector<Block_> path = ReadPath(pos);
    Block_ newBlock(in, newPos, uid);
    WriteNewBlockToPath(path, newBlock);
    EvictPath(path, pos);
    WriteBackPath(path, pos);
    return newPos;
  }

  PositionType Write(const UidType& uid, const T& in) {
    PositionType newPos = UniformRandom(size() - 1);
    return Write(uid, in, newPos);
  }

  PositionType Update(PositionType pos, const UidType& uid,
                      std::function<void(T&)> updateFunc) {
    T out;
    return Update(pos, uid, updateFunc, out);
  }

  PositionType Update(PositionType pos, const UidType& uid, PositionType newPos,
                      std::function<void(T&)> updateFunc) {
    T out;
    return Update(pos, uid, newPos, updateFunc, out);
  }

  PositionType Update(PositionType pos, const UidType& uid,
                      std::function<void(T&)> updateFunc, T& out) {
    return Update(pos, uid, updateFunc, out, uid);  // does not change uid
  }

  PositionType Update(PositionType pos, const UidType& uid, PositionType newPos,
                      std::function<void(T&)> updateFunc, T& out) {
    return Update(pos, uid, newPos, updateFunc, out,
                  uid);  // does not change uid
  }

  PositionType Update(PositionType pos, const UidType& uid,
                      std::function<void(T&)> updateFunc, T& out,
                      const UidType& updatedUid) {
    PositionType newPos = UniformRandom(size() - 1);
    return Update(pos, uid, newPos, updateFunc, out, updatedUid);
  }

  PositionType Update(PositionType pos, const UidType& uid, PositionType newPos,
                      std::function<void(T&)> updateFunc, T& out,
                      const UidType& updatedUid) {
    std::vector<Block_> path = ReadPath(pos);

    ReadElementAndRemoveFromPath(path, uid, out);
    updateFunc(out);
    Block_ newBlock(out, newPos, updatedUid);
    WriteNewBlockToPath(path, newBlock);
    EvictPath(path, pos);

    WriteBackPath(path, pos);
    return newPos;
  }

  std::vector<Block_> ReadPath(PositionType pos) {
    std::vector<Block_> path(stashSize + Z * depth);

    memcpy(&path[0], stash->blocks, stashSize * sizeof(Block_));

    int level = tree.ReadPath(pos, (Bucket_*)(&path[stashSize]));
    path.resize(stashSize + Z * level);
#ifndef NDEBUG
    for (int i = stashSize; i < path.size(); ++i) {
      int lv = (i - stashSize) / Z;
      size_t mask = (1UL << lv) - 1;
      Assert(path[i].uid == DUMMY<UidType>() ||
                 (path[i].position & mask) == (pos & mask),
             "Path position mismatch");
    }
#endif
    return path;
  }

  void WriteBackPath(const std::vector<Block_>& path, PositionType pos) {
    memcpy(stash->blocks, &path[0], stashSize * sizeof(Block_));
    tree.WritePath(pos, (Bucket_*)(&path[stashSize]));
  }

  static void EvictPath(std::vector<Block_>& path, PositionType pos) {
    std::vector<int> deepest(path.size());
    std::vector<int> deeperDummyCount(path.size());
    int depth_ = (path.size() - stashSize) / Z;
    for (int i = 0; i < path.size(); i++) {
      deepest[i] = commonSuffixLength(path[i].position, pos);
      obliMove(path[i].isDummy(), deepest[i], -1);
      obliMove(deepest[i] >= depth_, deepest[i], depth_ - 1);
    }
    EM::Algorithm::BitonicSortSepPayload(deepest.begin(), deepest.end(),
                                         path.begin());
    int rank = 0;
    int numDummy = 0;
    for (int i = path.size() - 1; i >= 0; i--) {
      numDummy += path[i].isDummy();
      // least number of blocks deeper than this block
      int minRank = (depth_ - 1 - deepest[i]) * Z;
      obliMove(rank < minRank, rank, minRank);
      // # dummies that needs to be added deeper than this block
      deeperDummyCount[i] = rank - (path.size() - 1 - i);
      if ((deepest[i] >= 0) & (rank >= path.size())) {
        throw std::runtime_error("Stash overflow");
      }
      obliMove(deepest[i] == -1, deeperDummyCount[i],
               0);  // mark dummy as 0, so that it won't be moved
      ++rank;
    }

    int distriLevel = GetLogBaseTwo(path.size() - 1) + 1;

    for (int level = distriLevel - 1; level >= 0; --level) {
      int pow2Level = 1 << level;
      for (int i = pow2Level; i < path.size(); ++i) {
        bool shiftFlag =
            (deeperDummyCount[i] & (2 * pow2Level - 1)) >= pow2Level;
        obliSwap(shiftFlag, deeperDummyCount[i - pow2Level],
                 deeperDummyCount[i]);
        // could uncomment this instead of OrDistribute
        // causes twice the swap
        // obliSwap(shiftFlag, path[i - pow2Level], path[i]);
      }
    }
    std::vector<int> markArr(path.size() + 1);
    markArr[0] = path.size() - numDummy;
    for (int i = 0; i < path.size(); ++i) {
      --numDummy;
      obliMove(numDummy < deeperDummyCount[i], numDummy, deeperDummyCount[i]);
      markArr[i + 1] = path.size() - 1 - i - numDummy;
    }
    EM::Algorithm::OrDistributeSeparateMark(path.rbegin(), path.rend(),
                                            markArr.rbegin());
  }
};
}  // namespace ORAM::PathORAM