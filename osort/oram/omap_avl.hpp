#pragma once

#include "oram.hpp"
/**
 * @brief An oblivious map based on B+ tree.
 * Each level of the tree is stored in a separate ORAM.
 * The number of levels is fixed. If a level has only one node, then the node
 * can contain any number of entries, otherwise, the node must contain at least
 * min_fan_out entries.
 */
namespace ODSL {


#define NO_REBALANCE 0
#define DIRECT_REBALANCE 1
#define ROT2 2
#define ROT3 3

typedef uint32_t Dir_t;
constexpr inline Dir_t B_LEFT = 0;
constexpr inline Dir_t B_RIGHT = 1;
constexpr inline Dir_t B_BALANCED = 2;

INLINE std::string Dir_t_to_string(const Dir_t& t) {
  if (t == B_LEFT) { return "B_LEFT"; }
  if (t == B_RIGHT) { return "B_RIGHT"; }
  if (t == B_BALANCED) { return "B_BALANCED"; }
  Assert(false);
  return "B_ERRORBALANCE";
}

template <typename K, typename V,
          typename PositionType = uint64_t, typename UidType = uint64_t>
struct AVLNode {
  K key;
  V v;
  UidBlock<PositionType, UidType> child[2];
  
  Dir_t balance;

    INLINE bool IsBalanced() { return balance == B_BALANCED; }
    auto operator==(const AVLNode &other) const {
      return true * (key == other.key) * (v == other.v) * (child[0] == other.child[0]) * (child[1] == other.child[1]) * (balance == other.balance);
    }

    #ifndef ENCLAVE_MODE
    // friend with cout
    friend std::ostream &operator<<(std::ostream &o, const AVLNode &x) {      
      o << "AVLNode  ";
      o << "(" << std::to_string(x.key) << ", " << std::to_string(x.v) << ", " << x.child[0] << ", " << x.child[1] << "," << Dir_t_to_string(x.balance) <<  ")";
      return o;
    }
    #endif

    static consteval INLINE AVLNode DUMMY() { 
      return AVLNode{
        K{0},
        V{0},
        {DUMMY<UidBlock<PositionType, UidType>>(), DUMMY<UidBlock<PositionType, UidType>>()},
        Dir_t{B_BALANCED}
      };
    }

    INLINE Dir_t Direction(const K& k, bool isFakeNode) {
      Dir_t ret = B_LEFT;
      CMOV((k == this->key) * (!isFakeNode), ret, B_BALANCED);
      CMOV(k > this->key, ret, B_RIGHT);
      return ret;
    }
};

/**
 * @tparam K must overload < and == operators, should be trivially copyable
 * @tparam V should be trivially copyable
 * @tparam PositionType
 * @tparam UidType
 */
template <typename K, typename V, 
          typename PositionType = uint64_t, typename UidType = uint64_t>
struct OMap {
  using UidPosition = UidBlock<PositionType, UidType>;
  using AVLNode_ = AVLNode<K, V, PositionType, UidType>;

  // UNDONE(): fix this to have a separate ORAM for values:
  using InternalORAM_ = ORAM<AVLNode_, PositionType, UidType>;
  InternalORAM_ internalOram;
  PositionType oramSize;
  uint64_t maxDepth = 0;
  uint64_t maxNodes = 0;
  uint64_t count = 0;
  UidPosition root;
  UidPosition FAKE_NODE;

  OMap() {}

  OMap(PositionType maxSize,
       size_t cacheBytes = ((uint64_t)ENCLAVE_SIZE << 20) * 3UL / 4UL) {
    SetSize(maxSize, cacheBytes);
  }

  void SetSize(PositionType maxSize,
               size_t cacheBytes = ((uint64_t)ENCLAVE_SIZE << 20) * 3UL / 4UL) {
    this->maxNodes = maxSize + 2;
    this->maxDepth = GetLogBaseTwo(GetNextPowerOfTwo(maxNodes ));
    this->maxDepth = this->maxDepth + (this->maxDepth>>1) + 1;    
    this->oramSize = this->maxNodes;

    internalOram.SetSize(this->oramSize, cacheBytes);
  }

  ~OMap() {}

  void Init() {
    FAKE_NODE.uid = internalOram.GetNextUid();
    FAKE_NODE.data = UniformRandom(oramSize - 1);
    AVLNode_ node;
    node = AVLNode_ { K{0}, V{0}, {FAKE_NODE, FAKE_NODE}, Dir_t{B_BALANCED} };
    PositionType newPos;
    newPos = internalOram.Write(FAKE_NODE.uid, node, FAKE_NODE.data);
    Assert(newPos == FAKE_NODE.data);
    
    root.uid = internalOram.GetNextUid();
    root.data = UniformRandom(oramSize - 1);
    node = AVLNode_ { K{0}, V{0}, {FAKE_NODE, FAKE_NODE}, Dir_t{B_BALANCED} };
    newPos = internalOram.Write(root.uid, node, root.data);
    Assert(newPos == root.data);

    Assert(FAKE_NODE.uid == 0);
    Assert(root.uid == 1);
  }

  template <class Reader>
  void InitFromReader(Reader& reader) {
    Init();
    // UNDONE(): fix this to actually initialize
    return;
    while(!reader.eof()) {
      std::pair<K, V> kv = reader.read();
      insert(kv.first, kv.second);
    }
  }

  template <class Reader>
  void InitFromReaderInPlace(Reader& reader) {
    InitFromReader(reader);
  }

  #define FIX_FAKE_NODE(uidPos, newPos) \
    if (uidPos.uid == FAKE_NODE.uid) { \
      uidPos.data = FAKE_NODE.data; \
      FAKE_NODE.data = newPos; \
    }

  template<bool isUpdate>
  bool GetUpdate(const K& key, V& valInOut) {
    bool foundFlag = false;

    PositionType newPos = UniformRandom(oramSize - 1);
    UidPosition curr = root;
    root.data = newPos;

    for (int i = 0; i <= maxDepth; i++) {
      PositionType newChildPos = UniformRandom(oramSize - 1);
      UidPosition nextChild;
      FIX_FAKE_NODE(curr, newPos);

      std::function<bool(AVLNode_&)> updateFunc =
          [&](AVLNode_& node) -> bool {
        Dir_t dir = node.Direction(key, curr.uid <= 1);
        bool match = dir == B_BALANCED;
        bool right = dir == B_RIGHT;
        Assert((!match) + (!foundFlag)); // no duplicate keys

        obliMove(!right, nextChild, node.child[0]);
        obliMove(!right, node.child[0].data, newChildPos);
        obliMove(right, nextChild, node.child[1]);
        obliMove(right, node.child[1].data, newChildPos);

        if constexpr (isUpdate) {
          obliSwap(match, node.v, valInOut);
        } else {
          obliMove(match, valInOut, node.v);
        }

        obliMove(match, foundFlag, true);        

        return true;
      };

      DEBUG_ONLY(PositionType ok =) 
        internalOram.Update(curr.data, curr.uid, newPos, updateFunc);
      Assert(ok == newPos);

      curr = nextChild;
      newPos = newChildPos;
    }
    Assert(curr.uid == FAKE_NODE.uid);
    
    return foundFlag;
  }

  bool find(const K& key, V& valOut) {
    return GetUpdate<false>(key, valOut);
  }

  bool update(const K& key, const V& val) {
    V valOut;
    valOut = val;
    return GetUpdate<true>(key, valOut);
  }

  void GO_NEXT(
    const K& key,
    Dir_t& dir, Dir_t& pDir1, Dir_t& pDir2,
    bool& match,
    bool& right,
    UidPosition& curr, UidPosition& currP,
    AVLNode_& node,
    UidPosition& other,
    PositionType& newChildPos,
    PositionType& newPos
  ) {
    dir = node.Direction(key, curr.uid <= 1);
    match = dir == B_BALANCED;
    right = dir == B_RIGHT;
    pDir2 = pDir1; pDir1 = dir; currP = curr; currP.data = newPos;
    
    obliMove(!right, curr, node.child[0]);
    obliMove(!right, other, node.child[1]);
    obliMove(!right, node.child[0].data, newChildPos);
    obliMove(right, curr, node.child[1]);
    obliMove(right, other, node.child[0]);
    obliMove(right, node.child[1].data, newChildPos);
  }

  void GO_NEXT_COND(
    bool cond,
    const K& key,
    Dir_t& dir, Dir_t& pDir1, Dir_t& pDir2,
    bool& match,
    bool& right,
    UidPosition& curr, UidPosition& currP,
    AVLNode_& node,
    UidPosition& other,
    PositionType& newChildPos,
    PositionType& newPos
  ) {
    obliMove(cond, dir, node.Direction(key, curr.uid <= 1));
    obliMove(cond, match, dir == B_BALANCED);
    obliMove(cond, right, dir == B_RIGHT);
    obliMove(cond, pDir2, pDir1);
    obliMove(cond, pDir1, dir);
    obliMove(cond, currP, curr);
    obliMove(cond, currP.data, newPos);
    
    obliMove((cond)*(!right), curr, node.child[0]);
    obliMove((cond)*(!right), other, node.child[1]);
    obliMove((cond)*(!right), node.child[0].data, newChildPos);
    obliMove((cond)*(right), curr, node.child[1]);
    obliMove((cond)*(right), other, node.child[0]);
    obliMove((cond)*(right), node.child[1].data, newChildPos);
  }
  
  #define NO_REBALANCE 0
  #define DIRECT_REBALANCE 1
  #define ROT2 2
  #define ROT3 3

  /**
   * @brief insert a key-value pair into the map, if the key already exist,
   update the value
   *
   * @param key
   * @param val
   * @return true if key found
   * @return false if key not found
   */
  bool insert(const K &key, const V &value) {
    PositionType newPos = UniformRandom(oramSize - 1), newPos0 = UniformRandom(oramSize - 1), newPos1 = UniformRandom(oramSize - 1);
    UidPosition curr = root;
    UidPosition currP = FAKE_NODE;
    UidPosition newNodeUP = UidPosition{internalOram.GetNextUid(), UniformRandom(oramSize - 1)};
    AVLNode_ newNode = AVLNode_ { key, value, {FAKE_NODE, FAKE_NODE}, Dir_t{B_BALANCED} };
    internalOram.Write(newNodeUP.uid, newNode, newNodeUP.data);

    root.data = newPos;

    Dir_t dir = B_BALANCED, pDir1 = B_BALANCED, pDir2 = B_BALANCED;
    UidPosition A, B, C, D, E, F, G, X;
    V vB, vF, vD;
    K kB, kF, kD;

    // Values saved for second pass:
    //
    bool match = false;
    int rotType = NO_REBALANCE;
    int unState = 10;
    Dir_t dirArg1 = B_BALANCED;
    Dir_t dirArg2 = B_BALANCED;
    UidPosition other;

    bool contained = false;
    bool inserted = false;
    bool right = false;

    for (int i = 0; i <= maxDepth; i++) {
      PositionType newChildPos = UniformRandom(oramSize - 1);
      FIX_FAKE_NODE(curr, newPos);

      std::function<bool(AVLNode_&)> updateFunc =
          [&](AVLNode_& node) -> bool {
        GO_NEXT(key, dir, pDir1, pDir2, match, right, curr, currP, node, other, newChildPos, newPos);        
        
        if (i > 0) {
          bool case0 = (node.balance != B_BALANCED) * (currP.uid != FAKE_NODE.uid);
          bool case0_0 = case0 * (node.balance != pDir1);
          bool case1 = !case0 * (unState == 1);
          bool case1_1 = case1 * (pDir1 == pDir2);
          bool case1_1_1 = case1_1 * (curr.uid == FAKE_NODE.uid);
          bool case2 = !case0 * !case1 * (unState == 2);
          bool case2_1 = case2 * (D.uid != FAKE_NODE.uid);
          bool case2_1_1 = case2_1 * (curr.uid == FAKE_NODE.uid);
          bool case2_1_2 = case2_1 * (dir != dirArg1);
          bool case2_2 = case2 * !case2_1;
          // if (case0) {
            obliMove(case0,   unState, 0);
            obliMove(case0,   B,       currP);
            obliMove(case0,   A,       other);
            obliMove(case0,   F,       curr);
            obliMove(case0,   vB,      node.v);
            obliMove(case0,   kB,      node.key);
            obliMove(case0_0, rotType, DIRECT_REBALANCE);
            obliMove(case0_0, unState, 3);
          // } else {
            // if (unState == 1) {
              obliMove(case1, G,       other);
              obliMove(case1, D,       curr);
              obliMove(case1, vF,      node.v);
              obliMove(case1, kF,      node.key);
              obliMove(case1, dirArg1, pDir2);
              // if (pDir1 == pDir2) {
                obliMove(case1_1, rotType, ROT2);
                obliMove(case1_1, unState, 3);
                // if (D.uid == FAKE_NODE.uid) {
                  obliMove(case1_1_1, D, newNodeUP);
                // }
              // }
            // } else if (unState == 2) {
              // if (D.uid != FAKE_NODE.uid) {
                obliMove(case2_1, C, other);
                obliMove(case2_1, E, curr);

                // if (curr.uid == FAKE_NODE.uid) {
                  obliMove(case2_1_1, E, newNodeUP);
                // }

                // if (dir != dirArg1) {
                  obliSwap(case2_1_2, C, E);
                // }
                obliMove(case2_1, vD,      node.v);
                obliMove(case2_1, kD,      node.key);
                obliMove(case2_1, rotType, ROT3);
                obliMove(case2_1, dirArg2, dir);
              // } else {
                obliMove(case2_2, E,       FAKE_NODE);
                obliMove(case2_2, C,       FAKE_NODE);
                obliMove(case2_2, D,       newNodeUP);
                obliMove(case2_2, vD,      value);
                obliMove(case2_2, kD,      key);
                obliMove(case2_2, rotType, ROT3);
                obliMove(case2_2, dirArg2, B_BALANCED);
              // }
            // }
          // }
          unState++;
        }

        obliMove(match, contained, true);
        obliMove(match, node.v, value);
        bool cond0 = !contained * !inserted * (curr.uid == FAKE_NODE.uid);
        obliMove(cond0 * !right, node.child[0], newNodeUP);
        obliMove(cond0 * right, node.child[1], newNodeUP);
        obliMove(cond0, curr, FAKE_NODE);
        obliMove(cond0, inserted, true);

        return true;
      };

      UidPosition aux = curr;
      DEBUG_ONLY(PositionType ok =) 
        internalOram.Update(aux.data, aux.uid, newPos, updateFunc);
      Assert(ok == newPos);

      newPos = newChildPos;
    }
    Assert(curr.uid == FAKE_NODE.uid);
    Assert(unState >= 3 || !inserted);

    // Handle update case -> delete block from tree:
    //
    obliMove(!inserted, curr, newNodeUP);
    obliMove(inserted, curr.uid, DUMMY<UidType>());
    obliMove(inserted, curr.data, UniformRandom(oramSize - 1));
    {
      std::function<bool(AVLNode_&)> updateFunc =
      [&](AVLNode_& node) -> bool {
        return false;
      };
      internalOram.Update(curr.data, curr.uid, updateFunc);
    }

   
    // Second stage:
    //
    bool rebalancing = false;
    newPos = UniformRandom(oramSize - 1);
    obliMove(inserted, curr, root);
    obliMove(inserted, root.data, newPos);
    obliMove(!inserted, curr, FAKE_NODE);

    for (int i = 0; i <= maxDepth; i++) {
      PositionType newChildPos = UniformRandom(oramSize - 1);
      UidPosition nextChild;
      FIX_FAKE_NODE(curr, newPos);

      std::function<bool(AVLNode_&)> updateFunc =
          [&](AVLNode_& node) -> bool {
        obliMove(curr.uid == newNodeUP.uid, rebalancing, false);

        bool cond_dr = (rotType == DIRECT_REBALANCE) * (curr.uid == B.uid);
        bool cond_r2_1 = (!cond_dr)  * (rotType == ROT2)  * (curr.uid == B.uid);
        bool cond_r2_2 = (!cond_dr) * (!cond_r2_1) * (rotType == ROT2) * (curr.uid == F.uid);
        bool cond_r3_1 = (!cond_dr) * (!cond_r2_1) * (!cond_r2_2) * (rotType == ROT3) * (curr.uid == B.uid);
        bool cond_r3_2 = (!cond_dr) * (!cond_r2_1) * (!cond_r2_2) * (!cond_r3_1) * (rotType == ROT3) * (curr.uid == F.uid);
        bool cond_r3_2_1 = cond_r3_2 * (dirArg2 != B_BALANCED);
        bool cond_r3_2_1_1 = cond_r3_2_1 * (dirArg1 == dirArg2);
        bool cond_r3_2_1_2 = cond_r3_2_1 * (!cond_r3_2_1_1);
        bool cond_r3_2_2 = cond_r3_2 * (!cond_r3_2_1);
        bool cond_r3_3 = (!cond_dr) * (!cond_r2_1) * (!cond_r2_2) * (!cond_r3_1) * (!cond_r3_2) * (rotType == ROT3) * (curr.uid == D.uid);
        bool cond_r3_3_1 = cond_r3_3 * (dirArg2 != dirArg1) * (dirArg2 != B_BALANCED);
        bool cond_r3_3_2 = cond_r3_3 * (!cond_r3_3_1);
        bool cond_update = (!cond_dr) * (!cond_r2_1) * (!cond_r2_2) * (!cond_r3_1) * (!cond_r3_2) * (!cond_r3_3);
        bool cond_update_1 = cond_update * rebalancing;
        bool cond_update_2 = cond_update * (!cond_update_1) * (i == 0) * (rotType == NO_REBALANCE);
        // if (rotType == DIRECT_REBALANCE && curr.uid == B.uid) {
          obliMove(cond_dr, node.balance, B_BALANCED);
          obliMove(cond_dr, rebalancing, true);
          obliMove(cond_dr, newPos, UniformRandom(oramSize - 1));
          GO_NEXT_COND(cond_dr, key, dir, pDir1, pDir2, match, right, curr, currP, node, other, newChildPos, newPos);        

          // node.data.balance = dir;
        // } else if (rotType == ROT2 && curr.uid == B.uid) {
          obliMove(cond_r2_1, node.v, vF);
          obliMove(cond_r2_1, node.key, kF);
          obliMove(cond_r2_1, node.balance, B_BALANCED);
          obliMove(cond_r2_1, curr, F);
          obliMove(cond_r2_1, newPos, UniformRandom(oramSize - 1));
          obliMove(cond_r2_1, F.data, newPos);
          obliMove(cond_r2_1, newPos0, UniformRandom(oramSize - 1));
          obliMove(cond_r2_1, newPos1, D.data);
          obliMove(cond_r2_1, D.data, newPos0);
          obliMove(cond_r2_1*dirArg1, node.child[0], F);
          obliMove(cond_r2_1*dirArg1, node.child[1], D);
          obliMove(cond_r2_1*(!dirArg1), node.child[0], D);
          obliMove(cond_r2_1*(!dirArg1), node.child[1], F);
        // } else if (rotType == ROT2 && curr.uid == F.uid) {
          obliMove(cond_r2_2, node.v, vB);
          obliMove(cond_r2_2, node.key, kB);
          obliMove(cond_r2_2, node.balance, B_BALANCED);
          obliMove(cond_r2_2, curr, D);
          obliMove(cond_r2_2, curr.data, newPos1);
          obliMove(cond_r2_2, newPos, newPos0);

          obliMove(cond_r2_2*dirArg1, node.child[0], A);
          obliMove(cond_r2_2*dirArg1, node.child[1], G);
          obliMove(cond_r2_2*(!dirArg1), node.child[0], G);
          obliMove(cond_r2_2*(!dirArg1), node.child[1], A);
          obliMove(cond_r2_2, rebalancing, true);
        // } else if (rotType == ROT3 && curr.uid == B.uid) {
          obliMove(cond_r3_1, node.v, vD);
          obliMove(cond_r3_1, node.key, kD);
          obliMove(cond_r3_1, node.balance, B_BALANCED);
          obliMove(cond_r3_1, curr, F);
          obliMove(cond_r3_1, newPos, UniformRandom(oramSize - 1));
          obliMove(cond_r3_1, F.data, newPos);
          obliMove(cond_r3_1, newPos0, D.data);
          obliMove(cond_r3_1, D.data, UniformRandom(oramSize - 1));
          obliMove(cond_r3_1*dirArg1, node.child[0], F);
          obliMove(cond_r3_1*dirArg1, node.child[1], D);
          obliMove(cond_r3_1*(!dirArg1), node.child[0], D);
          obliMove(cond_r3_1*(!dirArg1), node.child[1], F);
        // } else if (rotType == ROT3 && curr.uid == F.uid) {
          obliMove(cond_r3_2, node.v, vB);
          obliMove(cond_r3_2, node.key, kB);
          // if (dirArg2 != B_BALANCED) {
            // if (dirArg1 == dirArg2) {
              obliMove(cond_r3_2_1_1, node.balance, 1 - dirArg1);
              obliMove(cond_r3_2_1_1, X, E);
              obliMove(cond_r3_2_1_1, newPos1, UniformRandom(oramSize - 1));
              obliMove(cond_r3_2_1_1, E.data, newPos1);
            // } else {
              obliMove(cond_r3_2_1_2, node.balance, B_BALANCED);
              obliMove(cond_r3_2_1_2, X, C);
              obliMove(cond_r3_2_1_2, newPos1, UniformRandom(oramSize - 1));
              obliMove(cond_r3_2_1_2, C.data, newPos1);
            // }
          // } else {
            obliMove(cond_r3_2_2, node.balance, B_BALANCED);
            obliMove(cond_r3_2_2, X, FAKE_NODE);
          // }
          obliMove(cond_r3_2, newPos, D.data);
          obliMove(cond_r3_2, curr, D);
          obliMove(cond_r3_2, curr.data, newPos0);
          obliMove(cond_r3_2*dirArg1, node.child[0], A);
          obliMove(cond_r3_2*dirArg1, node.child[1], C);
          obliMove(cond_r3_2*(!dirArg1), node.child[0], C);
          obliMove(cond_r3_2*(!dirArg1), node.child[1], A);
        // } else if (rotType == ROT3 && curr.uid == D.uid) {
          obliMove(cond_r3_3, node.v, vF);
          obliMove(cond_r3_3, node.key, kF);
          // if (dirArg2 != dirArg1 && dirArg2 != B_BALANCED) {
            obliMove(cond_r3_3_1, node.balance, dirArg1);
          // } else {
            obliMove(cond_r3_3_2, node.balance, B_BALANCED);
          // }
          obliMove(cond_r3_3, curr, X);
          obliMove(cond_r3_3, newPos, newPos1);
          obliMove(cond_r3_3*dirArg1, node.child[0], E);
          obliMove(cond_r3_3*dirArg1, node.child[1], G);
          obliMove(cond_r3_3*(!dirArg1), node.child[0], G);
          obliMove(cond_r3_3*(!dirArg1), node.child[1], E);
          obliMove(cond_r3_3, rebalancing, true);
        // } else {
          obliMove(cond_update, newPos, UniformRandom(oramSize - 1));
          GO_NEXT_COND(cond_update, key, dir, pDir1, pDir2, match, right, curr, currP, node, other, newChildPos, newPos);
          
          // if (rebalancing) {
            obliMove(cond_update_1, node.balance, dir);
          // } else if (i == 0 && rotType == NO_REBALANCE) {
            obliMove(cond_update_2, rebalancing, true);
          // }
        // }

        return true;
      };

      DEBUG_ONLY(PositionType ok =) 
        internalOram.Update(curr.data, curr.uid, newPos, updateFunc);
      Assert(ok == newPos);

      curr = nextChild;
      newPos = newChildPos;
    }
    Assert(curr.uid == FAKE_NODE.uid);

    return !inserted;
  }
};
}  // namespace ODSL