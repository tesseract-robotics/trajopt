#include <gtest/gtest.h>

#include <cstdint>
#include <set>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "trajopt_common/cache.h"  // <-- adjust include path/name to your header

namespace
{
struct TestValue
{
  int marker{ 0 };
};

using Key = int;
using CacheT = trajopt_common::Cache<Key, TestValue>;

std::shared_ptr<TestValue> acquireAndPut(CacheT& cache, Key key, int marker)
{
  auto [ptr, hit] = cache.getOrAcquire(key);
  if (!hit)
  {
    ptr->marker = marker;
    cache.put(key, ptr);
  }
  return ptr;
}

std::unordered_set<const TestValue*> pooledPointersSeen(CacheT& cache, const std::vector<Key>& keys)
{
  std::unordered_set<const TestValue*> seen;
  for (Key k : keys)
  {
    auto [ptr, hit] = cache.getOrAcquire(k);
    // On miss the cache does not associate with key until put().
    if (!hit)
      cache.put(k, ptr);
    seen.insert(ptr.get());
  }
  return seen;
}

}  // namespace

TEST(CacheTest, StartsEmpty)
{
  CacheT cache(3);
  EXPECT_EQ(cache.capacity(), 3U);
  EXPECT_EQ(cache.size(), 0U);

  EXPECT_EQ(cache.get(1), nullptr);
  EXPECT_EQ(cache.get(42), nullptr);
}

TEST(CacheTest, GetOrAcquireMissDoesNotCreateKeyMappingUntilPut)
{
  CacheT cache(2);

  auto [ptr, hit] = cache.getOrAcquire(10);
  EXPECT_FALSE(hit);
  ASSERT_NE(ptr, nullptr);

  // On miss, key is not yet associated until caller puts it back.
  EXPECT_EQ(cache.get(10), nullptr);
  EXPECT_EQ(cache.size(), 0U);

  cache.put(10, ptr);
  EXPECT_EQ(cache.size(), 1U);
  EXPECT_EQ(cache.get(10).get(), ptr.get());
}

TEST(CacheTest, GetHitTouchesRecencySoLRUEvictionChanges)
{
  CacheT cache(2);

  auto p1 = acquireAndPut(cache, 1, 101);  // miss -> put
  auto p2 = acquireAndPut(cache, 2, 202);  // miss -> put
  ASSERT_EQ(cache.size(), 2U);

  // After puts, MRU is key=2, LRU is key=1.

  // Touch key=1 (now key=2 becomes LRU).
  auto g1 = cache.get(1);
  ASSERT_NE(g1, nullptr);
  EXPECT_EQ(g1.get(), p1.get());

  // Miss on key=3 should evict LRU (key=2).
  auto [p3, hit3] = cache.getOrAcquire(3);
  EXPECT_FALSE(hit3);
  ASSERT_NE(p3, nullptr);

  // Eviction happens during acquire on miss; key=2 should be gone immediately.
  EXPECT_EQ(cache.get(2), nullptr);
  // key=3 still not mapped until put.
  EXPECT_EQ(cache.get(3), nullptr);

  cache.put(3, p3);
  EXPECT_NE(cache.get(3), nullptr);
  EXPECT_EQ(cache.size(), 2U);
}

TEST(CacheTest, GetOrAcquireHitDoesNotRequirePutAndPreservesKey)
{
  CacheT cache(2);

  auto p1 = acquireAndPut(cache, 1, 111);
  ASSERT_EQ(cache.size(), 1U);

  auto [p1b, hit] = cache.getOrAcquire(1);
  EXPECT_TRUE(hit);
  ASSERT_NE(p1b, nullptr);
  EXPECT_EQ(p1b.get(), p1.get());

  // Key should still be present without calling put().
  auto g1 = cache.get(1);
  ASSERT_NE(g1, nullptr);
  EXPECT_EQ(g1.get(), p1.get());
  EXPECT_EQ(cache.size(), 1U);
}

TEST(CacheTest, LRUEvictionOrderWithThreeKeys)
{
  CacheT cache(3);

  auto p1 = acquireAndPut(cache, 1, 1);
  auto p2 = acquireAndPut(cache, 2, 2);
  auto p3 = acquireAndPut(cache, 3, 3);

  // After puts: MRU=3, then 2, LRU=1.
  // Touch 1 -> MRU=1, then 3, LRU=2.
  ASSERT_NE(cache.get(1), nullptr);

  // Miss key=4 should evict key=2.
  auto [p4, hit4] = cache.getOrAcquire(4);
  EXPECT_FALSE(hit4);
  cache.put(4, p4);

  EXPECT_NE(cache.get(1), nullptr);
  EXPECT_EQ(cache.get(2), nullptr);  // evicted
  EXPECT_NE(cache.get(3), nullptr);
  EXPECT_NE(cache.get(4), nullptr);

  EXPECT_EQ(cache.size(), 3U);
  (void)p1;
  (void)p2;
  (void)p3;
}

TEST(CacheTest, PutSameKeySamePtrTouchDoesNotChangeSize)
{
  CacheT cache(2);

  auto p1 = acquireAndPut(cache, 1, 7);
  ASSERT_EQ(cache.size(), 1U);

  // Put same key with same ptr should keep size == 1.
  cache.put(1, p1);
  EXPECT_EQ(cache.size(), 1U);
  EXPECT_EQ(cache.get(1).get(), p1.get());
}

TEST(CacheTest, PutExistingKeyWithDifferentPtrEvictsOldMapping)
{
  CacheT cache(2);

  auto p1 = acquireAndPut(cache, 1, 111);
  auto p2 = acquireAndPut(cache, 2, 222);
  ASSERT_EQ(cache.size(), 2U);

  // Touch key=2 so key=1 becomes LRU.
  ASSERT_NE(cache.get(2), nullptr);

  // Acquire miss for key=3 evicts LRU key=1 and returns its pooled object (likely p1's slot).
  auto [p3, hit3] = cache.getOrAcquire(3);
  EXPECT_FALSE(hit3);
  ASSERT_NE(p3, nullptr);
  EXPECT_EQ(cache.get(1), nullptr);

  // Now intentionally map key=2 to the acquired pointer p3 (different slot than current key=2).
  // This should evict the old key=2 mapping and replace it with p3.
  cache.put(2, p3);

  auto g2 = cache.get(2);
  ASSERT_NE(g2, nullptr);
  EXPECT_EQ(g2.get(), p3.get());

  // key=3 is still not mapped (we never put it).
  EXPECT_EQ(cache.get(3), nullptr);
}

TEST(CacheTest, PutPtrPreviouslyMappedToDifferentKeyRemovesOldKey)
{
  CacheT cache(2);

  auto p1 = acquireAndPut(cache, 1, 1);
  auto p2 = acquireAndPut(cache, 2, 2);
  ASSERT_EQ(cache.size(), 2U);

  // Re-map p1's pooled object to key=99.
  cache.put(99, p1);

  // Old key=1 mapping should be removed.
  EXPECT_EQ(cache.get(1), nullptr);
  EXPECT_NE(cache.get(99), nullptr);
  EXPECT_EQ(cache.get(99).get(), p1.get());

  // Key=2 should remain.
  EXPECT_NE(cache.get(2), nullptr);
  EXPECT_EQ(cache.size(), 2U);
  (void)p2;
}

TEST(CacheTest, EraseRemovesKeyAndDecrementsSize)
{
  CacheT cache(3);

  auto p1 = acquireAndPut(cache, 1, 1);
  auto p2 = acquireAndPut(cache, 2, 2);
  (void)p1;
  (void)p2;

  ASSERT_EQ(cache.size(), 2U);

  cache.erase(1);
  EXPECT_EQ(cache.get(1), nullptr);
  EXPECT_EQ(cache.size(), 1U);

  // Erasing non-existent key should be no-op.
  cache.erase(1234);
  EXPECT_EQ(cache.size(), 1U);
}

TEST(CacheTest, PoolNeverExceedsCapacityUniqueObjects)
{
  constexpr std::size_t kCapacity = 4;
  CacheT cache(kCapacity);

  std::vector<Key> keys;
  keys.reserve(100);
  for (int i = 0; i < 100; ++i)
    keys.push_back(1000 + i);

  const auto seen = pooledPointersSeen(cache, keys);

  // Even after many misses/puts, pool object identities should be capped by capacity.
  EXPECT_EQ(seen.size(), kCapacity);
}

TEST(CacheTest, RepeatedHitsDoNotChangePoolIdentityForKey)
{
  CacheT cache(2);

  auto p1 = acquireAndPut(cache, 1, 10);
  ASSERT_NE(p1, nullptr);

  for (int i = 0; i < 50; ++i)
  {
    auto g = cache.get(1);
    ASSERT_NE(g, nullptr);
    EXPECT_EQ(g.get(), p1.get());
  }
}

// After you erase keys, those slots are NOT added back to a reusable "available slot"
// list; once free_slots_ is exhausted and lru_ becomes empty, acquireSlot() falls into
// the degenerate path and always returns slot 0.
//
// If you decide that erased slots should be reusable, enable this test after fixing
// acquireSlot()/erase() to track available unassigned slots.
TEST(CacheTest, ErasedSlotsShouldBeReusableOnMiss)
{
  CacheT cache(2);

  auto p1 = acquireAndPut(cache, 1, 1);
  auto p2 = acquireAndPut(cache, 2, 2);
  ASSERT_EQ(cache.size(), 2U);

  cache.erase(1);
  cache.erase(2);
  ASSERT_EQ(cache.size(), 0U);

  // Now two misses should ideally return two different pooled objects (reuse both slots).
  auto [a, ha] = cache.getOrAcquire(10);
  auto [b, hb] = cache.getOrAcquire(11);
  EXPECT_FALSE(ha);
  EXPECT_FALSE(hb);

  ASSERT_NE(a, nullptr);
  ASSERT_NE(b, nullptr);

  // Expect different objects if slots are properly reusable.
  EXPECT_NE(a.get(), b.get());

  (void)p1;
  (void)p2;
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
