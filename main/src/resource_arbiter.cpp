#include "printsphere/resource_arbiter.hpp"

#include <utility>

#include "esp_log.h"

namespace printsphere {

namespace {
constexpr char kTag[] = "printsphere.arbiter";

bool log_grant_release_at_info(ResourceKind kind) {
  switch (kind) {
    case ResourceKind::kJpegDecode:
    case ResourceKind::kDisplayProtected:
      return false;
    case ResourceKind::kTlsLocalMqtt:
    case ResourceKind::kTlsCloudMqtt:
    case ResourceKind::kHttpCloudRest:
    case ResourceKind::kCameraTls:
    case ResourceKind::kPngDecode:
    case ResourceKind::kCount:
      break;
  }
  return true;
}

void log_grant(ResourceKind kind, const char* owner, const char* reason) {
  if (log_grant_release_at_info(kind)) {
    ESP_LOGI(kTag, "grant %s owner=%s reason=%s", ResourceArbiter::to_string(kind),
             owner != nullptr ? owner : "-", reason != nullptr ? reason : "-");
  } else {
    ESP_LOGD(kTag, "grant %s owner=%s reason=%s", ResourceArbiter::to_string(kind),
             owner != nullptr ? owner : "-", reason != nullptr ? reason : "-");
  }
}

void log_release(ResourceKind kind, const char* owner, const char* reason) {
  if (log_grant_release_at_info(kind)) {
    ESP_LOGI(kTag, "release %s owner=%s reason=%s", ResourceArbiter::to_string(kind),
             owner != nullptr ? owner : "-", reason != nullptr ? reason : "-");
  } else {
    ESP_LOGD(kTag, "release %s owner=%s reason=%s", ResourceArbiter::to_string(kind),
             owner != nullptr ? owner : "-", reason != nullptr ? reason : "-");
  }
}
}

ResourceArbiter::Lease::Lease(ResourceArbiter* arbiter, ResourceKind kind, uint32_t lease_id)
    : arbiter_(arbiter), kind_(kind), lease_id_(lease_id) {}

ResourceArbiter::Lease::Lease(Lease&& other) noexcept
    : arbiter_(std::exchange(other.arbiter_, nullptr)),
      kind_(std::exchange(other.kind_, ResourceKind::kCount)),
      lease_id_(std::exchange(other.lease_id_, 0)) {}

ResourceArbiter::Lease& ResourceArbiter::Lease::operator=(Lease&& other) noexcept {
  if (this != &other) {
    reset();
    arbiter_ = std::exchange(other.arbiter_, nullptr);
    kind_ = std::exchange(other.kind_, ResourceKind::kCount);
    lease_id_ = std::exchange(other.lease_id_, 0);
  }
  return *this;
}

ResourceArbiter::Lease::~Lease() {
  reset();
}

void ResourceArbiter::Lease::reset() {
  if (arbiter_ != nullptr) {
    arbiter_->release(kind_, lease_id_);
    arbiter_ = nullptr;
    kind_ = ResourceKind::kCount;
    lease_id_ = 0;
  }
}

const char* ResourceArbiter::to_string(ResourceKind kind) {
  switch (kind) {
    case ResourceKind::kTlsLocalMqtt:
      return "TLS_LOCAL_MQTT";
    case ResourceKind::kTlsCloudMqtt:
      return "TLS_CLOUD_MQTT";
    case ResourceKind::kHttpCloudRest:
      return "HTTP_CLOUD_REST";
    case ResourceKind::kCameraTls:
      return "CAMERA_TLS";
    case ResourceKind::kJpegDecode:
      return "JPEG_DECODE";
    case ResourceKind::kPngDecode:
      return "PNG_DECODE";
    case ResourceKind::kDisplayProtected:
      return "DISPLAY_PROTECTED";
    case ResourceKind::kCount:
      break;
  }
  return "UNKNOWN";
}

uint32_t ResourceArbiter::bit(ResourceKind kind) {
  return 1U << static_cast<uint8_t>(kind);
}

uint32_t ResourceArbiter::conflict_mask(ResourceKind kind) {
  const uint32_t local_mqtt = bit(ResourceKind::kTlsLocalMqtt);
  const uint32_t cloud_mqtt = bit(ResourceKind::kTlsCloudMqtt);
  const uint32_t http = bit(ResourceKind::kHttpCloudRest);
  const uint32_t camera_tls = bit(ResourceKind::kCameraTls);
  const uint32_t jpeg = bit(ResourceKind::kJpegDecode);
  const uint32_t png = bit(ResourceKind::kPngDecode);
  const uint32_t display = bit(ResourceKind::kDisplayProtected);
  const uint32_t network = local_mqtt | cloud_mqtt | http | camera_tls;
  const uint32_t media = jpeg | png;

  switch (kind) {
    case ResourceKind::kTlsLocalMqtt:
      return network | media;
    case ResourceKind::kTlsCloudMqtt:
      return network | media;
    case ResourceKind::kHttpCloudRest:
      return network | media;
    case ResourceKind::kCameraTls:
      return network | media;
    case ResourceKind::kJpegDecode:
      return network | media | display;
    case ResourceKind::kPngDecode:
      return network | media | display;
    case ResourceKind::kDisplayProtected:
      return media | display;
    case ResourceKind::kCount:
      break;
  }
  return UINT32_MAX;
}

bool ResourceArbiter::can_acquire_locked(ResourceKind kind) const {
  return (held_mask_ & conflict_mask(kind)) == 0U;
}

ResourceKind ResourceArbiter::first_conflict_locked(ResourceKind kind) const {
  const uint32_t conflicts = held_mask_ & conflict_mask(kind);
  for (uint8_t i = 0; i < static_cast<uint8_t>(ResourceKind::kCount); ++i) {
    const auto candidate = static_cast<ResourceKind>(i);
    if ((conflicts & bit(candidate)) != 0U) {
      return candidate;
    }
  }
  return ResourceKind::kCount;
}

ResourceArbiter::Lease ResourceArbiter::try_acquire(ResourceKind kind, const char* owner,
                                                    const char* reason) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!can_acquire_locked(kind)) {
    const ResourceKind conflict = first_conflict_locked(kind);
    const size_t conflict_index = static_cast<size_t>(conflict);
    ESP_LOGI(kTag, "deny %s owner=%s reason=%s conflict=%s conflict_owner=%s",
             to_string(kind), owner != nullptr ? owner : "-",
             reason != nullptr ? reason : "-", to_string(conflict),
             conflict_index < kResourceCount && owners_[conflict_index] != nullptr
                 ? owners_[conflict_index]
                 : "-");
    return {};
  }

  uint32_t lease_id = next_lease_id_++;
  if (next_lease_id_ == 0) {
    next_lease_id_ = 1;
  }
  const size_t index = static_cast<size_t>(kind);
  held_mask_ |= bit(kind);
  lease_ids_[index] = lease_id;
  owners_[index] = owner;
  reasons_[index] = reason;
  log_grant(kind, owner, reason);
  return Lease(this, kind, lease_id);
}

bool ResourceArbiter::can_acquire(ResourceKind kind) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return can_acquire_locked(kind);
}

bool ResourceArbiter::is_held(ResourceKind kind) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return (held_mask_ & bit(kind)) != 0U;
}

void ResourceArbiter::release(ResourceKind kind, uint32_t lease_id) {
  std::lock_guard<std::mutex> lock(mutex_);
  const size_t index = static_cast<size_t>(kind);
  if (index >= kResourceCount || (held_mask_ & bit(kind)) == 0U ||
      lease_ids_[index] != lease_id) {
    ESP_LOGW(kTag, "release mismatch %s lease=%u", to_string(kind),
             static_cast<unsigned>(lease_id));
    return;
  }

  log_release(kind, owners_[index], reasons_[index]);
  held_mask_ &= ~bit(kind);
  lease_ids_[index] = 0;
  owners_[index] = nullptr;
  reasons_[index] = nullptr;
}

}  // namespace printsphere
