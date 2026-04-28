#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <mutex>

namespace printsphere {

enum class ResourceKind : uint8_t {
  kTlsLocalMqtt,
  kTlsCloudMqtt,
  kHttpCloudRest,
  kCameraTls,
  kJpegDecode,
  kPngDecode,
  // Reserved for a future LVGL flush hook. Today the display is protected by
  // the existing LVGL lock / DisplayDecodeGuard path.
  kDisplayProtected,
  kCount,
};

class ResourceArbiter {
 public:
  class Lease {
   public:
    Lease() = default;
    Lease(const Lease&) = delete;
    Lease& operator=(const Lease&) = delete;
    Lease(Lease&& other) noexcept;
    Lease& operator=(Lease&& other) noexcept;
    ~Lease();

    explicit operator bool() const { return arbiter_ != nullptr; }
    bool acquired() const { return arbiter_ != nullptr; }
    void reset();

   private:
    friend class ResourceArbiter;
    Lease(ResourceArbiter* arbiter, ResourceKind kind, uint32_t lease_id);

    ResourceArbiter* arbiter_ = nullptr;
    ResourceKind kind_ = ResourceKind::kCount;
    uint32_t lease_id_ = 0;
  };

  Lease try_acquire(ResourceKind kind, const char* owner, const char* reason);
  bool can_acquire(ResourceKind kind) const;
  bool is_held(ResourceKind kind) const;

  static const char* to_string(ResourceKind kind);

 private:
  static constexpr size_t kResourceCount = static_cast<size_t>(ResourceKind::kCount);

  void release(ResourceKind kind, uint32_t lease_id);
  bool can_acquire_locked(ResourceKind kind) const;
  ResourceKind first_conflict_locked(ResourceKind kind) const;
  static uint32_t bit(ResourceKind kind);
  static uint32_t conflict_mask(ResourceKind kind);

  mutable std::mutex mutex_{};
  uint32_t held_mask_ = 0;
  uint32_t next_lease_id_ = 1;
  std::array<uint32_t, kResourceCount> lease_ids_{};
  std::array<const char*, kResourceCount> owners_{};
  std::array<const char*, kResourceCount> reasons_{};
};

}  // namespace printsphere
