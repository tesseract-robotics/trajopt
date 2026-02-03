#ifndef TRAJOPT_COMMON_CACHE_H
#define TRAJOPT_COMMON_CACHE_H

#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <Eigen/Eigen>
#include <algorithm>
#include <cstddef>
#include <list>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>
TRAJOPT_IGNORE_WARNINGS_POP

namespace trajopt_common
{
// Cache: fixed-capacity LRU pool of ValueT objects stored as shared_ptr<ValueT>.
// - Not thread-safe: callers must synchronize externally if using from multiple threads.
// - Usage pattern:
//     auto [ptr, hit] = cache.getOrAcquire(key);
//     if (!hit) { /* populate ptr (returned from pool) */ cache.put(key, ptr); }
// - get(key) returns Ptr on hit, nullptr on miss.
// - getOrAcquire(key) returns {ptr, true} on hit, {ptr, false} on miss (ptr from pool).
//   If the pool is exhausted (no free or evictable slots), getOrAcquire returns {nullptr, false}.
// - put(key, ptr) associates a pooled ptr with a key and marks it MRU. Returns true on success
//   (ptr belonged to this pool), false otherwise.
// - erase(key) removes the mapping but leaves pooled object allocated.

template <typename KeyT, typename ValueT, typename Hash = std::hash<KeyT>, typename KeyEqual = std::equal_to<KeyT>>
class Cache
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<ValueT>;
  using AcquireResult = std::pair<Ptr, bool>;  // {ptr, hit}

  explicit Cache(std::size_t capacity = 666)
    : capacity_(capacity)
    , pool_(capacity)
    , slot_key_(capacity)
    , assigned_(capacity, false)
    , checked_out_(capacity, false)
    , slot_it_(capacity)
  {
    free_slots_.reserve(capacity_);
    for (std::size_t i = 0; i < capacity_; ++i)
    {
      pool_[i] = std::make_shared<ValueT>();
      ptr_to_slot_[pool_[i].get()] = i;
      free_slots_.push_back(i);
    }
  }

  // Returns nullptr on miss. On hit, updates LRU recency.
  Ptr get(const KeyT& key)
  {
    const auto it = key_to_slot_.find(key);
    if (it == key_to_slot_.end())
      return Ptr{};

    const std::size_t slot = it->second;
    touch(slot);
    return pool_[slot];
  }

  // Returns {ptr, true} on hit (and updates LRU).
  // Returns {ptr, false} on miss, where ptr is taken from the pool.
  // On miss, the returned ptr is NOT associated with `key`. The caller should
  // rebuild it and then call put(key, ptr) to cache it.
  // If the pool is exhausted (no free or evictable slots), returns {nullptr, false}.
  AcquireResult getOrAcquire(const KeyT& key)
  {
    const auto it = key_to_slot_.find(key);
    if (it != key_to_slot_.end())
    {
      const std::size_t slot = it->second;
      touch(slot);
      return { pool_[slot], true };
    }

    const std::size_t slot = acquireSlot();
    if (slot == kNpos)
      return { Ptr{}, false };
    return { pool_[slot], false };
  }

  // Associates ptr with key and marks it as most-recently-used.
  //
  // If ptr belongs to this cache's pool, it is stored in its pool slot.
  // If key already exists, the previous slot is evicted (key removed).
  // Returns true if mapping established; false if ptr not from this pool.
  bool put(const KeyT& key, const Ptr& value)
  {
    if (!value)
      return false;

    const std::size_t slot = slotFromPtr(value);
    if (slot == kNpos)
    {
      assert(false && "Cache::put called with pointer not from pool");
      return false;
    }

    // If the key is already mapped to a different slot, evict that entry.
    auto existing = key_to_slot_.find(key);
    if (existing != key_to_slot_.end() && existing->second != slot)
      evictSlot(existing->second);

    // If this slot is currently assigned to a different key, remove that mapping.
    if (assigned_[slot])
    {
      if (!KeyEqual{}(slot_key_[slot], key))
      {
        key_to_slot_.erase(slot_key_[slot]);
        removeFromLru(slot);
        assigned_[slot] = false;

        // Slot is no longer assigned; make it available if not checked out.
        if (!checked_out_[slot])
          free_slots_.push_back(slot);
      }
    }

    // If this slot was checked out, it is now being returned to the cache.
    checked_out_[slot] = false;
    removeFromFreeSlots(slot);

    // Associate and mark MRU.
    slot_key_[slot] = key;
    key_to_slot_[key] = slot;

    if (!assigned_[slot])
    {
      lru_.push_front(slot);
      slot_it_[slot] = lru_.begin();
      assigned_[slot] = true;
    }
    else
    {
      touch(slot);
    }

    return true;
  }

  // Removes a key mapping from the cache (the underlying pooled object remains).
  void erase(const KeyT& key)
  {
    const auto it = key_to_slot_.find(key);
    if (it == key_to_slot_.end())
      return;

    evictSlot(it->second);
  }

  std::size_t capacity() const { return capacity_; }

  std::size_t size() const { return key_to_slot_.size(); }

private:
  static constexpr std::size_t kNpos = static_cast<std::size_t>(-1);

  // Moves a slot to most-recently-used.
  void touch(std::size_t slot)
  {
    if (!assigned_[slot])
      return;

    lru_.splice(lru_.begin(), lru_, slot_it_[slot]);
    slot_it_[slot] = lru_.begin();
  }

  void removeFromLru(std::size_t slot)
  {
    if (!assigned_[slot])
      return;

    lru_.erase(slot_it_[slot]);
  }

  void removeFromFreeSlots(std::size_t slot)
  {
    auto it = std::find(free_slots_.begin(), free_slots_.end(), slot);
    if (it != free_slots_.end())
      free_slots_.erase(it);
  }

  // Evicts the given slot from the cache (removes key mapping and LRU membership).
  // The pooled object stays allocated; the slot becomes unassigned.
  void evictSlot(std::size_t slot)
  {
    if (!assigned_[slot])
      return;

    key_to_slot_.erase(slot_key_[slot]);
    removeFromLru(slot);
    assigned_[slot] = false;

    // Make this slot available for reuse if it isn't currently checked out.
    if (!checked_out_[slot])
      free_slots_.push_back(slot);
  }

  // Returns a slot index for use on a miss:
  //  - Prefer available/free slots first.
  //  - Otherwise evict the LRU cached slot.
  //
  // The returned slot is marked checked_out_ = true and has no key mapping.
  // If no slot can be provided (all slots checked out and nothing evictable), returns kNpos.
  std::size_t acquireSlot()
  {
    std::size_t slot = kNpos;

    // Prefer unassigned slots that are available for reuse.
    while (!free_slots_.empty())
    {
      const std::size_t candidate = free_slots_.back();
      free_slots_.pop_back();

      // Skip stale entries (can happen if a slot was pushed multiple times).
      if (!assigned_[candidate] && !checked_out_[candidate])
      {
        slot = candidate;
        break;
      }
    }

    if (slot == kNpos)
    {
      if (!lru_.empty())
      {
        // Evict least-recently-used cached entry.
        slot = lru_.back();
        evictSlot(slot);
      }
      else
      {
        // No available free slots and no assigned (evictable) slots:
        // indicate exhaustion by returning kNpos.
        return kNpos;
      }
    }

    // Ensure the returned object has no key association.
    checked_out_[slot] = true;
    return slot;
  }

  std::size_t slotFromPtr(const Ptr& p) const
  {
    const ValueT* raw = p.get();
    const auto it = ptr_to_slot_.find(raw);
    return (it == ptr_to_slot_.end()) ? kNpos : it->second;
  }

  std::size_t capacity_;
  std::vector<Ptr> pool_;  // Fixed pool of objects.

  // Key -> slot lookup.
  std::unordered_map<KeyT, std::size_t, Hash, KeyEqual> key_to_slot_;

  // Raw pointer -> slot lookup (pool membership).
  std::unordered_map<const ValueT*, std::size_t> ptr_to_slot_;

  // Slot metadata.
  std::vector<KeyT> slot_key_;     // Valid only when assigned_[slot] is true.
  std::vector<bool> assigned_;     // Slot is currently mapped to a key (in LRU list).
  std::vector<bool> checked_out_;  // Slot has been acquired on miss but not yet put().
  std::vector<std::list<std::size_t>::iterator> slot_it_;

  // LRU list of assigned slots: front = MRU, back = LRU.
  std::list<std::size_t> lru_;

  // Slots that are available for reuse (unassigned and not checked out).
  // Note: may contain duplicates/stale entries; acquireSlot() filters them.
  std::vector<std::size_t> free_slots_;
};

}  // namespace trajopt_common

#endif  // TRAJOPT_COMMON_CACHE_H
