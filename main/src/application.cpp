#include "printsphere/application.hpp"

#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_pm.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "bsp/esp32_s3_touch_amoled_1_75.h"
#include "printsphere/error_lookup.hpp"
#include "printsphere/status_resolver.hpp"

namespace printsphere {

namespace {
constexpr char kTag[] = "printsphere.app";
constexpr TickType_t kStopBannerDuration = pdMS_TO_TICKS(12000);
constexpr TickType_t kHybridCloudFallbackDelayLocalFirst = pdMS_TO_TICKS(35000);
constexpr TickType_t kHybridCloudFallbackDelayCloudFirst = pdMS_TO_TICKS(12000);
constexpr TickType_t kHybridCameraCloudCooldown = pdMS_TO_TICKS(8000);
constexpr TickType_t kLocalMqttHandoffCooldown = pdMS_TO_TICKS(3000);
constexpr TickType_t kScreenOffTouchWakePollSlice = pdMS_TO_TICKS(25);
constexpr size_t kLocalMqttStartMinInternalLargest = 16U * 1024U;
constexpr size_t kLocalMqttStartMinDmaLargest = 16U * 1024U;
constexpr size_t kCameraStartMinInternalLargest = 12U * 1024U;
constexpr size_t kCameraStartMinDmaLargest = 12U * 1024U;
constexpr size_t kPreviewFetchMinInternalLargest = 4U * 1024U;
constexpr size_t kPreviewFetchMinDmaLargest = 4U * 1024U;
constexpr uint64_t kChamberLightOverrideMs = 6000;
constexpr uint32_t kSourceFreshMs = 15000;
constexpr uint32_t kSourceRecentMs = 60000;
constexpr uint32_t kSourceOldMs = 180000;

struct SourceConfidence {
  uint8_t local_score = 0;
  uint8_t cloud_score = 0;
  uint32_t local_age_ms = UINT32_MAX;
  uint32_t cloud_age_ms = UINT32_MAX;
};

esp_err_t configure_power_management() {
#if CONFIG_PM_ENABLE
  esp_pm_config_t pm_config = {};
  pm_config.max_freq_mhz = 240;
  pm_config.min_freq_mhz = 80;
  pm_config.light_sleep_enable = false;
  ESP_RETURN_ON_ERROR(esp_pm_configure(&pm_config), kTag, "esp_pm_configure failed");
  ESP_LOGI(kTag, "Power management enabled: DFS 80-240 MHz, light sleep off");
#else
  ESP_LOGI(kTag, "Power management disabled in sdkconfig (CONFIG_PM_ENABLE=n)");
#endif
  return ESP_OK;
}

bool local_print_is_live(const PrinterSnapshot& snapshot) {
  return snapshot.print_active || snapshot.lifecycle == PrintLifecycleState::kPreparing ||
         snapshot.lifecycle == PrintLifecycleState::kPrinting ||
         snapshot.lifecycle == PrintLifecycleState::kPaused;
}

bool cloud_print_is_live(const BambuCloudSnapshot& snapshot) {
  return snapshot.lifecycle == PrintLifecycleState::kPreparing ||
         snapshot.lifecycle == PrintLifecycleState::kPrinting ||
         snapshot.lifecycle == PrintLifecycleState::kPaused;
}

bool tick_deadline_active(TickType_t deadline, TickType_t now) {
  return deadline != 0 && static_cast<int32_t>(deadline - now) > 0;
}

const char* to_string(DominantLane lane) {
  switch (lane) {
    case DominantLane::kStatus:
      return "status";
    case DominantLane::kPreview:
      return "preview";
    case DominantLane::kCamera:
      return "camera";
    case DominantLane::kConfig:
      return "config";
    case DominantLane::kPowerSave:
      return "power-save";
  }
  return "unknown";
}

const char* to_string(SourceCoordinatorState state) {
  switch (state) {
    case SourceCoordinatorState::kLocalCold:
      return "LocalCold";
    case SourceCoordinatorState::kLocalProbing:
      return "LocalProbing";
    case SourceCoordinatorState::kLocalConnected:
      return "LocalConnected";
    case SourceCoordinatorState::kLocalSleeping:
      return "LocalSleeping";
    case SourceCoordinatorState::kCloudFallback:
      return "CloudFallback";
    case SourceCoordinatorState::kHandoffToLocal:
      return "HandoffToLocal";
    case SourceCoordinatorState::kLocalPrimary:
      return "LocalPrimary";
    case SourceCoordinatorState::kCloudPrimary:
      return "CloudPrimary";
  }
  return "Unknown";
}

DominantLane dominant_lane_for_ui(bool config_page_active, bool preview_page_active,
                                  bool preview_page_visible, bool camera_page_active,
                                  bool camera_page_visible, bool page_transition_active,
                                  ScreenPowerMode screen_power_mode) {
  if (screen_power_mode == ScreenPowerMode::kOff) {
    return DominantLane::kPowerSave;
  }
  if (config_page_active) {
    return DominantLane::kConfig;
  }
  if (camera_page_active) {
    return DominantLane::kCamera;
  }
  if (preview_page_active || (page_transition_active && preview_page_visible)) {
    return DominantLane::kPreview;
  }
  return DominantLane::kStatus;
}

bool local_connect_allowed_for_lane(DominantLane lane, SourceMode source_mode,
                                    bool local_network_ready,
                                    bool local_printer_enabled,
                                    const PrinterSnapshot& local_snapshot,
                                    bool local_mqtt_handoff_active) {
  if (!local_network_ready || !local_printer_enabled || source_mode == SourceMode::kCloudOnly ||
      local_mqtt_handoff_active) {
    return false;
  }
  if (local_snapshot.local_connected) {
    return true;
  }

  switch (lane) {
    case DominantLane::kStatus:
    case DominantLane::kCamera:
      return true;
    case DominantLane::kPreview:
      return source_mode == SourceMode::kLocalOnly;
    case DominantLane::kConfig:
      return source_mode == SourceMode::kLocalOnly;
    case DominantLane::kPowerSave:
      return local_snapshot.print_active;
  }
  return false;
}

SourceCoordinatorState source_state_for(DominantLane lane, SourceMode source_mode,
                                        bool local_printer_enabled,
                                        bool local_network_ready,
                                        bool local_connect_allowed,
                                        bool local_mqtt_handoff_active,
                                        bool hybrid_local_path_healthy,
                                        bool hybrid_prefers_cloud,
                                        bool cloud_network_ready,
                                        const PrinterSnapshot& local_snapshot,
                                        const BambuCloudSnapshot& cloud_snapshot) {
  if (local_mqtt_handoff_active) {
    return SourceCoordinatorState::kHandoffToLocal;
  }
  if (source_mode == SourceMode::kCloudOnly) {
    return SourceCoordinatorState::kCloudPrimary;
  }
  if (!local_printer_enabled || !local_network_ready) {
    if (source_mode == SourceMode::kHybrid && cloud_network_ready &&
        (cloud_snapshot.connected || cloud_snapshot.session_connected)) {
      return SourceCoordinatorState::kCloudFallback;
    }
    return SourceCoordinatorState::kLocalCold;
  }
  if (hybrid_local_path_healthy) {
    return SourceCoordinatorState::kLocalPrimary;
  }
  if (local_snapshot.local_connected) {
    return SourceCoordinatorState::kLocalConnected;
  }
  if (source_mode == SourceMode::kHybrid && cloud_network_ready &&
      (cloud_snapshot.connected || cloud_snapshot.session_connected)) {
    return hybrid_prefers_cloud ? SourceCoordinatorState::kCloudPrimary
                                : SourceCoordinatorState::kCloudFallback;
  }
  if (local_printer_enabled && local_network_ready && local_connect_allowed) {
    return SourceCoordinatorState::kLocalProbing;
  }
  if (local_printer_enabled && (lane == DominantLane::kPreview ||
                                lane == DominantLane::kConfig ||
                                lane == DominantLane::kPowerSave)) {
    return SourceCoordinatorState::kLocalSleeping;
  }
  return SourceCoordinatorState::kCloudFallback;
}

uint32_t coordinator_signature(DominantLane lane, SourceCoordinatorState state,
                               bool local_connect_allowed, bool camera_enabled,
                               bool camera_stream_connected,
                               bool cloud_network_ready, bool cloud_live_mqtt_enabled,
                               bool pause_cloud_fetches, bool preview_fetch_enabled,
                               bool deferred_local_probe, bool local_mqtt_budget_ok,
                               bool camera_start_budget_ok, bool preview_fetch_budget_ok,
                               const SourceConfidence& confidence) {
  uint32_t sig = static_cast<uint32_t>(lane);
  sig |= static_cast<uint32_t>(state) << 4;
  sig |= (local_connect_allowed ? 1U : 0U) << 8;
  sig |= (camera_enabled ? 1U : 0U) << 9;
  sig |= (camera_stream_connected ? 1U : 0U) << 10;
  sig |= (cloud_network_ready ? 1U : 0U) << 11;
  sig |= (cloud_live_mqtt_enabled ? 1U : 0U) << 12;
  sig |= (pause_cloud_fetches ? 1U : 0U) << 13;
  sig |= (preview_fetch_enabled ? 1U : 0U) << 14;
  sig |= (deferred_local_probe ? 1U : 0U) << 15;
  sig |= (local_mqtt_budget_ok ? 1U : 0U) << 16;
  sig |= (camera_start_budget_ok ? 1U : 0U) << 17;
  sig |= (preview_fetch_budget_ok ? 1U : 0U) << 18;
  sig |= ((static_cast<uint32_t>(confidence.local_score) / 25U) & 0x7U) << 19;
  sig |= ((static_cast<uint32_t>(confidence.cloud_score) / 25U) & 0x7U) << 22;
  return sig;
}

uint32_t age_ms_or_unknown(uint64_t now_ms, uint64_t update_ms) {
  if (update_ms == 0 || update_ms > now_ms) {
    return UINT32_MAX;
  }
  const uint64_t age = now_ms - update_ms;
  return age > UINT32_MAX ? UINT32_MAX : static_cast<uint32_t>(age);
}

uint8_t source_score(bool configured, bool network_ready, bool connected, uint32_t age_ms) {
  if (!configured || !network_ready) {
    return 0;
  }
  uint8_t score = connected ? 50 : 10;
  if (age_ms <= kSourceFreshMs) {
    score += 40;
  } else if (age_ms <= kSourceRecentMs) {
    score += 25;
  } else if (age_ms <= kSourceOldMs) {
    score += 10;
  }
  return score > 100 ? 100 : score;
}

int32_t age_seconds_for_log(uint32_t age_ms) {
  return age_ms == UINT32_MAX ? -1 : static_cast<int32_t>(age_ms / 1000U);
}

SourceConfidence source_confidence(bool local_printer_enabled, bool local_network_ready,
                                   const PrinterSnapshot& local_snapshot,
                                   bool cloud_network_ready,
                                   const BambuCloudSnapshot& cloud_snapshot,
                                   uint64_t now_ms) {
  SourceConfidence confidence;
  confidence.local_age_ms = age_ms_or_unknown(now_ms, local_snapshot.local_last_update_ms);
  const uint64_t cloud_update_ms = cloud_snapshot.live_data_last_update_ms != 0
                                       ? cloud_snapshot.live_data_last_update_ms
                                       : cloud_snapshot.last_update_ms;
  confidence.cloud_age_ms = age_ms_or_unknown(now_ms, cloud_update_ms);
  confidence.local_score =
      source_score(local_printer_enabled, local_network_ready, local_snapshot.local_connected,
                   confidence.local_age_ms);
  confidence.cloud_score =
      source_score(cloud_snapshot.configured, cloud_network_ready,
                   cloud_snapshot.connected || cloud_snapshot.session_connected,
                   confidence.cloud_age_ms);
  return confidence;
}

bool hybrid_local_status_ready(const PrinterSnapshot& snapshot) {
  return !snapshot.raw_status.empty() || !snapshot.raw_stage.empty() ||
         snapshot.progress_percent > 0.0f || snapshot.current_layer > 0U ||
         snapshot.total_layers > 0U || snapshot.nozzle_temp_c > 0.0f ||
         snapshot.bed_temp_c > 0.0f || snapshot.chamber_temp_c > 0.0f ||
         snapshot.secondary_nozzle_temp_c > 0.0f || snapshot.print_error_code != 0 ||
         !snapshot.hms_codes.empty() ||
         snapshot.hms_alert_count > 0U;
}

PrinterModel preferred_model_for_routing(const PrinterSnapshot& local_snapshot,
                                         const BambuCloudSnapshot& cloud_snapshot) {
  if (cloud_snapshot.model != PrinterModel::kUnknown) {
    return cloud_snapshot.model;
  }
  return local_snapshot.local_model;
}

bool hybrid_prefers_cloud_status(const PrinterSnapshot& local_snapshot,
                                 const BambuCloudSnapshot& cloud_snapshot) {
  return printer_model_prefers_cloud_status(
      preferred_model_for_routing(local_snapshot, cloud_snapshot));
}

bool hybrid_local_status_supported(const PrinterSnapshot& local_snapshot,
                                   const BambuCloudSnapshot& cloud_snapshot) {
  return printer_model_supports_local_status(
      preferred_model_for_routing(local_snapshot, cloud_snapshot));
}

TickType_t hybrid_cloud_fallback_delay(const PrinterSnapshot& local_snapshot,
                                       const BambuCloudSnapshot& cloud_snapshot) {
  return hybrid_prefers_cloud_status(local_snapshot, cloud_snapshot)
             ? kHybridCloudFallbackDelayCloudFirst
             : kHybridCloudFallbackDelayLocalFirst;
}

struct ChamberLightCommandPlan {
  bool try_local = false;
  bool try_cloud = false;
};

ChamberLightCommandPlan chamber_light_command_plan(SourceMode source_mode,
                                                   bool hybrid_prefers_cloud,
                                                   bool hybrid_local_status_supported_now,
                                                   bool local_network_ready,
                                                   bool local_printer_enabled,
                                                   bool cloud_network_ready,
                                                   const PrinterSnapshot& local_snapshot,
                                                   const BambuCloudSnapshot& cloud_snapshot) {
  ChamberLightCommandPlan plan;
  switch (source_mode) {
    case SourceMode::kLocalOnly:
      plan.try_local = true;
      break;
    case SourceMode::kCloudOnly:
      plan.try_cloud = true;
      break;
    case SourceMode::kHybrid:
    default:
      plan.try_local =
          !hybrid_prefers_cloud && hybrid_local_status_supported_now && local_network_ready &&
          local_printer_enabled &&
          (local_snapshot.local_connected ||
           printer_model_has_chamber_light(local_snapshot.local_model));
      plan.try_cloud =
          cloud_network_ready &&
          (cloud_snapshot.connected || printer_model_has_chamber_light(cloud_snapshot.model));
      break;
  }
  return plan;
}

void mark_chamber_light_state(PrinterSnapshot& snapshot, bool on) {
  snapshot.chamber_light_supported = true;
  snapshot.chamber_light_state_known = true;
  snapshot.chamber_light_on = on;
}

void mark_chamber_light_state(BambuCloudSnapshot& snapshot, bool on) {
  snapshot.chamber_light_supported = true;
  snapshot.chamber_light_state_known = true;
  snapshot.chamber_light_on = on;
}

void wait_for_next_iteration(Ui& ui, TickType_t delay) {
  TickType_t remaining = delay;
  while (remaining > 0) {
    const bool touch_wake_poll_active = ui.screen_power_mode() == ScreenPowerMode::kOff;
    const TickType_t slice =
        (touch_wake_poll_active && remaining > kScreenOffTouchWakePollSlice)
            ? kScreenOffTouchWakePollSlice
            : remaining;
    vTaskDelay(slice);
    remaining -= slice;

    if (touch_wake_poll_active && gpio_get_level(BSP_LCD_TOUCH_INT) == 0) {
      // The LVGL worker is paused while the screen is off, so a short tap can
      // be missed if the main loop sleeps for the full low-power interval.
      // Poll the raw touch IRQ in short slices so wake feels immediate.
      ui.request_wake_display();
      break;
    }
  }
}
}

Application::Application()
    : setup_portal_(config_store_, wifi_manager_, cloud_client_, printer_client_, camera_client_,
                    ui_, pmu_manager_) {
  cloud_client_.set_config_store(&config_store_);
  cloud_client_.set_resource_arbiter(&resource_arbiter_);
  printer_client_.set_resource_arbiter(&resource_arbiter_);
  camera_client_.set_resource_arbiter(&resource_arbiter_);
  ui_.set_resource_arbiter(&resource_arbiter_);
  // Route printer online/offline events from the Bambu Cloud MQTT feed to the
  // local PrinterClient so it can collapse its reconnect backoff the moment the
  // printer is known to be reachable again. Avoids blind TCP-probe cycles while
  // the printer is powered off or roaming on the LAN.
  cloud_client_.set_printer_presence_callback(
      [this](bool online) {
        if (!online) {
          printer_client_.notify_cloud_presence(false);
          return;
        }
        const auto lane =
            static_cast<DominantLane>(dominant_lane_.load(std::memory_order_relaxed));
        if (lane == DominantLane::kPreview || lane == DominantLane::kConfig ||
            lane == DominantLane::kPowerSave) {
          deferred_local_probe_ = true;
          ESP_LOGI(kTag, "Cloud presence hint deferred while %s lane is dominant",
                   to_string(lane));
          return;
        }
        printer_client_.notify_cloud_presence(true);
      });
  printer_client_.set_pre_local_mqtt_callback([this]() -> uint32_t {
    ESP_LOGI(kTag, "Local MQTT handoff: pausing cloud live MQTT and camera before TLS start");
    local_mqtt_handoff_until_tick_ = xTaskGetTickCount() + kLocalMqttHandoffCooldown;
    cloud_client_.set_live_mqtt_enabled(false);
    cloud_client_.set_fetch_paused(true);
    camera_client_.set_enabled(false);
    return 650U;
  });
}

void Application::run() {
  esp_log_level_set("mbedtls", ESP_LOG_WARN);
  ESP_LOGI(kTag, "Bootstrapping native PrintSphere project");

  ESP_ERROR_CHECK(config_store_.initialize());
  ESP_ERROR_CHECK(configure_power_management());
  ESP_ERROR_CHECK(wifi_manager_.initialize_network_stack());
  ESP_ERROR_CHECK(wifi_manager_.start_setup_access_point(config_store_.load_device_name()));

  const WifiCredentials wifi_credentials = config_store_.load_wifi_credentials();
  if (wifi_credentials.is_configured()) {
    const esp_err_t wifi_err = wifi_manager_.connect_station(wifi_credentials);
    if (wifi_err != ESP_OK) {
      ESP_LOGW(kTag, "Stored Wi-Fi connect failed: %s", esp_err_to_name(wifi_err));
    }
  }

  ESP_ERROR_CHECK(setup_portal_.start());
  ESP_ERROR_CHECK(pmu_manager_.initialize());
  ESP_LOGI(kTag, "Heap status: internal=%u bytes psram=%u bytes",
           static_cast<unsigned int>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)),
           static_cast<unsigned int>(heap_caps_get_free_size(MALLOC_CAP_SPIRAM)));
  ui_.set_arc_color_scheme(config_store_.load_arc_color_scheme());
  ui_.set_display_rotation(config_store_.load_display_rotation());
  ui_.set_battery_display_policy(config_store_.load_battery_display_policy());
  filament_wake_enabled_ = config_store_.load_filament_wake_enabled();
  filament_anim_enabled_ = config_store_.load_filament_anim_enabled();
  ESP_ERROR_CHECK(ui_.initialize());
  if (!initialize_error_lookup_storage()) {
    ESP_LOGW(kTag, "Embedded error lookup unavailable; falling back to generic error text");
  }

  const BambuCloudCredentials cloud_credentials = config_store_.load_cloud_credentials();
  source_mode_ = config_store_.load_source_mode();
  const PrinterConnection printer_connection = config_store_.load_active_printer_profile().to_connection();
  cloud_client_.configure(cloud_credentials, printer_connection.serial);
  ESP_ERROR_CHECK(cloud_client_.start());

  printer_client_.configure(printer_connection);
  ESP_ERROR_CHECK(printer_client_.start());
  camera_client_.configure(printer_connection);
  ESP_ERROR_CHECK(camera_client_.start());

  ESP_LOGI(kTag, "Bootstrap complete");

  while (true) {
    const TickType_t now_tick = xTaskGetTickCount();
    const uint64_t now_ms = static_cast<uint64_t>(esp_timer_get_time() / 1000ULL);
    if (ui_.consume_portal_unlock_request()) {
      setup_portal_.request_unlock_pin();
    }
    const int switch_idx = ui_.consume_printer_switch_request();
    if (switch_idx >= 0 &&
        static_cast<uint8_t>(switch_idx) != config_store_.load_active_printer_index()) {
      config_store_.save_active_printer_index(static_cast<uint8_t>(switch_idx));
      const PrinterConnection new_conn = config_store_.load_active_printer_profile().to_connection();
      printer_client_.configure(new_conn);
      camera_client_.configure(new_conn);
      cloud_client_.configure(config_store_.load_cloud_credentials(), new_conn.serial);
      ESP_LOGI(kTag, "Switched active printer to profile %d", switch_idx);
    }
    if (ui_.is_config_page_active()) {
      const auto profiles = config_store_.load_printer_profiles();
      const uint8_t active_idx = config_store_.load_active_printer_index();
      const bool local_connected = printer_client_.snapshot().local_connected;
      std::vector<Ui::PrinterCardInfo> cards;
      cards.reserve(profiles.size());
      for (const auto& p : profiles) {
        Ui::PrinterCardInfo ci;
        ci.index = p.index;
        ci.name = p.display_name;
        ci.model = p.model;
        ci.host = p.host;
        ci.active = (p.index == active_idx);
        ci.connected = ci.active && local_connected;
        cards.push_back(std::move(ci));
      }
      ui_.update_printer_cards(cards);
    }
    const PortalAccessSnapshot portal_access = setup_portal_.access_snapshot();
    const bool wifi_connected = wifi_manager_.is_station_connected();
    const std::string wifi_ip = wifi_manager_.station_ip();
    const bool page_transition_active = ui_.is_page_transition_active();
    const bool config_page_active = ui_.is_config_page_active();
    const bool preview_page_active = ui_.is_page2_active();
    const bool preview_page_visible = ui_.is_page2_visible();
    const bool camera_page_active = ui_.is_camera_page_active();
    const bool camera_page_visible = ui_.is_camera_page_visible();
    const DominantLane dominant_lane = dominant_lane_for_ui(
        config_page_active, preview_page_active, preview_page_visible, camera_page_active,
        camera_page_visible, page_transition_active, ui_.screen_power_mode());
    dominant_lane_.store(static_cast<uint8_t>(dominant_lane), std::memory_order_relaxed);
    source_mode_ = config_store_.load_source_mode();
    const bool local_network_ready = wifi_connected && source_mode_ != SourceMode::kCloudOnly;
    const bool local_mqtt_handoff_active =
        tick_deadline_active(local_mqtt_handoff_until_tick_.load(), now_tick);
    local_printer_enabled_ = printer_client_.is_configured();
    PrinterSnapshot local_snapshot = printer_client_.snapshot();
    const size_t internal_largest = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
    const size_t dma_largest = heap_caps_get_largest_free_block(MALLOC_CAP_DMA);
    const bool local_mqtt_budget_ok =
        internal_largest >= kLocalMqttStartMinInternalLargest &&
        dma_largest >= kLocalMqttStartMinDmaLargest;
    const bool camera_start_budget_ok =
        internal_largest >= kCameraStartMinInternalLargest &&
        dma_largest >= kCameraStartMinDmaLargest;
    const bool preview_fetch_budget_ok =
        internal_largest >= kPreviewFetchMinInternalLargest &&
        dma_largest >= kPreviewFetchMinDmaLargest;
    const bool local_connect_base_allowed = local_connect_allowed_for_lane(
        dominant_lane, source_mode_, local_network_ready, local_printer_enabled_, local_snapshot,
        local_mqtt_handoff_active);
    const bool local_connect_allowed =
        local_connect_base_allowed && (local_snapshot.local_connected || local_mqtt_budget_ok);
    printer_client_.set_connect_allowed(local_connect_allowed);
    printer_client_.set_network_ready(local_network_ready);
    camera_client_.set_network_ready(local_network_ready);
    const bool camera_stream_connected = camera_client_.is_stream_connected();
    const bool camera_lane_allowed =
        source_mode_ != SourceMode::kCloudOnly && local_printer_enabled_ && wifi_connected &&
        local_snapshot.local_connected && !local_mqtt_handoff_active &&
        !page_transition_active &&
        dominant_lane == DominantLane::kCamera &&
        ui_.screen_power_mode() != ScreenPowerMode::kOff;
    const bool camera_enabled =
        camera_lane_allowed && (camera_start_budget_ok || camera_stream_connected);
    camera_client_.set_enabled(camera_enabled);
    if (ui_.consume_camera_refresh_request()) {
      camera_client_.request_refresh();
    }

    local_snapshot.wifi_connected = wifi_connected;
    local_snapshot.wifi_ip = wifi_ip;
    local_snapshot.setup_ap_active = wifi_manager_.is_setup_access_point_active();
    local_snapshot.setup_ap_ssid = wifi_manager_.setup_access_point_ssid();
    local_snapshot.setup_ap_password = wifi_manager_.setup_access_point_password();
    local_snapshot.setup_ap_ip = wifi_manager_.setup_access_point_ip();
    camera_client_.observe_printer_snapshot(local_snapshot);
    if (last_local_print_live_ && local_snapshot.non_error_stop) {
      stop_banner_until_tick_ = now_tick + kStopBannerDuration;
    } else if (source_mode_ != SourceMode::kCloudOnly && !local_snapshot.non_error_stop) {
      stop_banner_until_tick_ = 0;
    }
    local_snapshot.show_stop_banner =
        local_snapshot.non_error_stop && tick_deadline_active(stop_banner_until_tick_, now_tick);
    resolve_ui_state(local_snapshot);

    const bool source_mode_changed = source_mode_ != last_source_mode_;
    const bool wifi_reconnected = wifi_connected && !last_wifi_connected_;
    const bool wifi_lost = !wifi_connected && last_wifi_connected_;
    if (source_mode_ == SourceMode::kHybrid && last_camera_page_active_ && !camera_page_visible &&
        wifi_connected) {
      hybrid_camera_cooldown_deadline_ = now_tick + kHybridCameraCloudCooldown;
      ESP_LOGD(kTag, "Hybrid mode: delaying cloud path briefly after camera activity");
    }
    if (source_mode_changed || wifi_lost) {
      hybrid_cloud_gate_open_ = false;
      hybrid_cloud_gate_deadline_ = 0;
      hybrid_camera_cooldown_deadline_ = 0;
    }

    BambuCloudSnapshot cloud_snapshot = cloud_client_.snapshot();
    const bool hybrid_prefers_cloud =
        source_mode_ == SourceMode::kHybrid &&
        hybrid_prefers_cloud_status(local_snapshot, cloud_snapshot);
    const bool hybrid_local_status_supported_now =
        source_mode_ != SourceMode::kCloudOnly &&
        hybrid_local_status_supported(local_snapshot, cloud_snapshot);

    bool cloud_network_ready = wifi_connected;
    const bool hybrid_camera_cooldown_active =
        source_mode_ == SourceMode::kHybrid &&
        tick_deadline_active(hybrid_camera_cooldown_deadline_, now_tick);
    if (source_mode_ == SourceMode::kLocalOnly) {
      cloud_network_ready = false;
    } else if (source_mode_ == SourceMode::kHybrid) {
      if (!local_printer_enabled_) {
        hybrid_cloud_gate_open_ = true;
        hybrid_cloud_gate_deadline_ = 0;
      } else if (!wifi_connected) {
        hybrid_cloud_gate_open_ = false;
      } else if (hybrid_prefers_cloud || !hybrid_local_status_supported_now) {
        hybrid_cloud_gate_open_ = true;
        hybrid_cloud_gate_deadline_ = 0;
      } else {
        if (source_mode_changed || wifi_reconnected || hybrid_cloud_gate_deadline_ == 0) {
          hybrid_cloud_gate_open_ = false;
          hybrid_cloud_gate_deadline_ =
              now_tick + hybrid_cloud_fallback_delay(local_snapshot, cloud_snapshot);
          ESP_LOGI(kTag, "Hybrid mode: delaying cloud path until local status or fallback timeout");
        }

        if (!hybrid_cloud_gate_open_ && hybrid_local_status_ready(local_snapshot)) {
          hybrid_cloud_gate_open_ = true;
          ESP_LOGI(kTag, "Hybrid mode: local status received, enabling cloud path");
        } else if (!hybrid_cloud_gate_open_ &&
                   !tick_deadline_active(hybrid_cloud_gate_deadline_, now_tick)) {
          hybrid_cloud_gate_open_ = true;
          ESP_LOGW(kTag, "Hybrid mode: local status timeout, enabling cloud path fallback");
        }

        cloud_network_ready = hybrid_cloud_gate_open_;
      }
    }
    const bool hybrid_local_path_healthy =
        source_mode_ == SourceMode::kHybrid && local_network_ready && local_printer_enabled_ &&
        local_snapshot.local_connected && hybrid_local_status_supported_now && !hybrid_prefers_cloud;
    if (source_mode_ == SourceMode::kHybrid && hybrid_local_path_healthy &&
        dominant_lane != DominantLane::kPreview && dominant_lane != DominantLane::kConfig) {
      cloud_network_ready = false;
    }
    const bool may_release_deferred_probe =
        deferred_local_probe_.load() && local_connect_allowed &&
        dominant_lane != DominantLane::kPreview && dominant_lane != DominantLane::kConfig &&
        dominant_lane != DominantLane::kPowerSave;
    if (may_release_deferred_probe) {
      deferred_local_probe_ = false;
      ESP_LOGI(kTag, "Cloud presence hint released in %s lane", to_string(dominant_lane));
      printer_client_.notify_cloud_presence(true);
    }
    const bool cloud_status_needed =
        source_mode_ == SourceMode::kCloudOnly ||
        (source_mode_ == SourceMode::kHybrid &&
         (hybrid_prefers_cloud || !hybrid_local_path_healthy));
    const bool cloud_live_mqtt_enabled =
        cloud_network_ready && !local_mqtt_handoff_active && cloud_status_needed &&
        dominant_lane != DominantLane::kCamera &&
        dominant_lane != DominantLane::kConfig &&
        dominant_lane != DominantLane::kPowerSave;
    const bool pause_cloud_fetches =
        !cloud_network_ready || local_mqtt_handoff_active ||
        dominant_lane == DominantLane::kCamera ||
        dominant_lane == DominantLane::kPowerSave ||
        page_transition_active || hybrid_camera_cooldown_active ||
        (source_mode_ == SourceMode::kHybrid && hybrid_local_path_healthy &&
         dominant_lane != DominantLane::kPreview &&
         dominant_lane != DominantLane::kConfig);
    const bool preview_fetch_enabled =
        source_mode_ != SourceMode::kLocalOnly &&
        dominant_lane == DominantLane::kPreview &&
        !local_mqtt_handoff_active && !page_transition_active && preview_fetch_budget_ok;
    const SourceCoordinatorState source_state = source_state_for(
        dominant_lane, source_mode_, local_printer_enabled_, local_network_ready,
        local_connect_allowed, local_mqtt_handoff_active, hybrid_local_path_healthy,
        hybrid_prefers_cloud, cloud_network_ready, local_snapshot, cloud_snapshot);
    const SourceConfidence confidence =
        source_confidence(local_printer_enabled_, local_network_ready, local_snapshot,
                          cloud_network_ready, cloud_snapshot, now_ms);
    const uint32_t signature = coordinator_signature(
        dominant_lane, source_state, local_connect_allowed, camera_enabled,
        camera_stream_connected, cloud_network_ready, cloud_live_mqtt_enabled,
        pause_cloud_fetches, preview_fetch_enabled,
        deferred_local_probe_.load(), local_mqtt_budget_ok, camera_start_budget_ok,
        preview_fetch_budget_ok, confidence);
    if (signature != last_coordinator_signature_) {
      last_coordinator_signature_ = signature;
      ESP_LOGI(kTag,
               "Coordinator: lane=%s source=%s local_connect=%d camera=%d "
               "camera_stream=%d cloud_net=%d cloud_live=%d cloud_fetch_paused=%d preview=%d "
               "deferred_probe=%d local_budget=%d camera_budget=%d preview_budget=%d "
               "local_score=%u cloud_score=%u local_age_s=%d cloud_age_s=%d "
               "int_largest=%u dma_largest=%u",
               to_string(dominant_lane), to_string(source_state), local_connect_allowed ? 1 : 0,
               camera_enabled ? 1 : 0, camera_stream_connected ? 1 : 0,
               cloud_network_ready ? 1 : 0, cloud_live_mqtt_enabled ? 1 : 0,
               pause_cloud_fetches ? 1 : 0,
               preview_fetch_enabled ? 1 : 0, deferred_local_probe_.load() ? 1 : 0,
               local_mqtt_budget_ok ? 1 : 0, camera_start_budget_ok ? 1 : 0,
               preview_fetch_budget_ok ? 1 : 0,
               static_cast<unsigned>(confidence.local_score),
               static_cast<unsigned>(confidence.cloud_score),
               age_seconds_for_log(confidence.local_age_ms),
               age_seconds_for_log(confidence.cloud_age_ms),
               static_cast<unsigned>(internal_largest), static_cast<unsigned>(dma_largest));
    }
    cloud_client_.set_network_ready(cloud_network_ready);
    cloud_client_.set_live_mqtt_enabled(cloud_live_mqtt_enabled);
    cloud_client_.set_fetch_paused(pause_cloud_fetches);

    cloud_snapshot = cloud_client_.snapshot();
    if (source_mode_ == SourceMode::kCloudOnly) {
      if (last_cloud_print_live_ && cloud_snapshot.non_error_stop) {
        stop_banner_until_tick_ = now_tick + kStopBannerDuration;
      } else if (!cloud_snapshot.non_error_stop) {
        stop_banner_until_tick_ = 0;
      }
    }
    auto build_merged_snapshot = [&](const PrinterSnapshot& current_local_snapshot,
                                     const BambuCloudSnapshot& current_cloud_snapshot) {
      PrinterSnapshot merged =
          merge_status_sources(current_local_snapshot, local_printer_enabled_, current_cloud_snapshot,
                               source_mode_, now_ms, wifi_connected, wifi_ip);
      merged.setup_ap_active = current_local_snapshot.setup_ap_active;
      merged.setup_ap_ssid = current_local_snapshot.setup_ap_ssid;
      merged.setup_ap_password = current_local_snapshot.setup_ap_password;
      merged.setup_ap_ip = current_local_snapshot.setup_ap_ip;
      merged.show_stop_banner =
          merged.non_error_stop && tick_deadline_active(stop_banner_until_tick_, now_tick);
      merged.preview_page_available = source_mode_ != SourceMode::kLocalOnly;
      merged.camera_page_available = source_mode_ != SourceMode::kCloudOnly;
      return merged;
    };
    auto apply_chamber_light_override = [&](PrinterSnapshot* target_snapshot) {
      if (target_snapshot == nullptr) {
        return;
      }
      if (!chamber_light_override_active_) {
        return;
      }
      if (now_ms >= chamber_light_override_until_ms_) {
        chamber_light_override_active_ = false;
        chamber_light_override_until_ms_ = 0;
        return;
      }
      target_snapshot->chamber_light_supported = true;
      target_snapshot->chamber_light_state_known = true;
      target_snapshot->chamber_light_on = chamber_light_override_on_;
    };
    PrinterSnapshot snapshot = build_merged_snapshot(local_snapshot, cloud_snapshot);
    apply_chamber_light_override(&snapshot);

    if (ui_.consume_chamber_light_toggle_request()) {
      const bool requested_on =
          !snapshot.chamber_light_state_known || !snapshot.chamber_light_on;
      bool command_sent = false;
      const ChamberLightCommandPlan light_plan =
          chamber_light_command_plan(source_mode_, hybrid_prefers_cloud,
                                     hybrid_local_status_supported_now, local_network_ready,
                                     local_printer_enabled_, cloud_network_ready,
                                     local_snapshot, cloud_snapshot);

      if (light_plan.try_local) {
        command_sent = printer_client_.set_chamber_light(requested_on);
        if (command_sent) {
          mark_chamber_light_state(local_snapshot, requested_on);
        }
      }
      if (!command_sent && light_plan.try_cloud) {
        command_sent = cloud_client_.set_chamber_light(requested_on);
        if (command_sent) {
          mark_chamber_light_state(cloud_snapshot, requested_on);
        }
      }

      if (!command_sent) {
        ESP_LOGW(kTag, "Chamber light toggle failed in %s mode", to_string(source_mode_));
      } else {
        chamber_light_override_active_ = true;
        chamber_light_override_on_ = requested_on;
        chamber_light_override_until_ms_ = now_ms + kChamberLightOverrideMs;
        snapshot = build_merged_snapshot(local_snapshot, cloud_snapshot);
        apply_chamber_light_override(&snapshot);
      }
    }

    const PowerSnapshot power = pmu_manager_.sample();
    if (power.available) {
      snapshot.battery_percent = power.battery_percent;
      snapshot.battery_present = power.battery_present;
      snapshot.charging = power.charging;
      snapshot.usb_present = power.usb_present;
      snapshot.pmu_temp_c = power.temperature_c;
    }

    const P1sCameraSnapshot camera_snapshot = camera_client_.snapshot();
    if (source_mode_ == SourceMode::kCloudOnly || !local_printer_enabled_) {
      snapshot.camera_connected = false;
      snapshot.camera_detail = source_mode_ == SourceMode::kCloudOnly
                                   ? "Camera unavailable in cloud-only mode"
                                   : "Local camera not configured";
      snapshot.camera_blob.reset();
      snapshot.camera_width = 0;
      snapshot.camera_height = 0;
      snapshot.camera_source = FieldSource::kNone;
    } else {
      snapshot.camera_connected = camera_snapshot.connected;
      snapshot.camera_detail = camera_snapshot.detail;
      snapshot.camera_blob = camera_snapshot.frame_blob;
      snapshot.camera_width = camera_snapshot.width;
      snapshot.camera_height = camera_snapshot.height;
      if (dominant_lane != DominantLane::kCamera) {
        snapshot.camera_blob.reset();
        snapshot.camera_width = 0;
        snapshot.camera_height = 0;
      }
    }

    // Detect filament stage before resolve_ui_state for animation suppression and wake logic.
    const bool is_filament = is_filament_stage(snapshot.stage);
    const bool is_external_spool = snapshot.tray_tar == 254;

    // When filament animation is disabled, suppress the loading/unloading stage for AMS auto
    // changes so resolve_ui_state treats it as normal printing (no arc animation).
    if (!filament_anim_enabled_ && is_filament && !is_external_spool) {
      snapshot.stage.clear();
      snapshot.raw_stage.clear();
    }

    resolve_ui_state(snapshot);
    // Store portal state first (lock-free), then apply_snapshot uses it
    // inside the same LVGL lock section — eliminates a separate lock acquisition.
    ui_.set_portal_access_state(portal_access.lock_enabled,
                                portal_access.request_authorized, portal_access.session_active,
                                portal_access.pin_active, portal_access.pin_code,
                                portal_access.pin_remaining_s, portal_access.session_remaining_s);
    ui_.apply_snapshot(snapshot);
    last_local_print_live_ = local_print_is_live(local_snapshot);
    last_cloud_print_live_ = cloud_print_is_live(cloud_snapshot);

    const bool on_battery = power.available && power.battery_present && !power.usb_present;
    cloud_client_.set_preview_fetch_enabled(preview_fetch_enabled);
    bool keep_screen_awake;
    if (filament_wake_enabled_ && is_filament && !is_external_spool) {
      // AMS auto filament change: suppress wake, let display sleep
      keep_screen_awake = dominant_lane == DominantLane::kCamera || page_transition_active;
    } else {
      keep_screen_awake =
          snapshot.print_active || dominant_lane == DominantLane::kCamera || page_transition_active;
    }
    if (filament_wake_enabled_ && is_filament && is_external_spool) {
      ui_.request_wake_display();
    }
    ui_.update_power_save(on_battery, keep_screen_awake);

    cloud_client_.set_low_power_mode(dominant_lane == DominantLane::kCamera ||
                                     page_transition_active ||
                                     (on_battery && ui_.is_low_power_mode_active() &&
                                      !snapshot.print_active));

    const TickType_t loop_delay =
        (snapshot.print_active || camera_page_active || page_transition_active ||
         !ui_.is_low_power_mode_active())
            ? pdMS_TO_TICKS(page_transition_active ? 100 : 500)
            : pdMS_TO_TICKS(1500);
    last_source_mode_ = source_mode_;
    last_wifi_connected_ = wifi_connected;
    last_camera_page_active_ = camera_page_visible;
    wait_for_next_iteration(ui_, loop_delay);
  }
}

}  // namespace printsphere
