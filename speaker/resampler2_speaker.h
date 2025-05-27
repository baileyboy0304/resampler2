#pragma once

#ifdef USE_ESP32

#include "esphome/components/audio/audio.h"
#include "esphome/components/audio/audio_transfer_buffer.h"
#include "esphome/components/speaker/speaker.h"

#include "esphome/core/component.h"

#include <freertos/event_groups.h>
#include <freertos/FreeRTOS.h>

namespace esphome {
namespace resampler2 {

class Resampler2Speaker : public Component, public speaker::Speaker {
 public:
  float get_setup_priority() const override { return esphome::setup_priority::DATA; }
  void setup() override;
  void loop() override;

  size_t play(const uint8_t *data, size_t length, TickType_t ticks_to_wait) override;
  //size_t play(const uint8_t *data, size_t length) override; { return this->play(data, length, 0); }
  size_t play(const uint8_t *data, size_t length) override;

  void start() override;
  void stop() override;
  void finish() override;

  void set_pause_state(bool pause_state) override { this->output_speaker_->set_pause_state(pause_state); }
  bool get_pause_state() const override { return this->output_speaker_->get_pause_state(); }

  bool has_buffered_data() const override;
  //bool has_buffered_data2() const override;

  /// @brief Mute state changes are passed to the parent's output speaker
  void set_mute_state(bool mute_state) override;

  /// @brief Volume state changes are passed to the parent's output speaker
  void set_volume(float volume) override;

  void set_output_speaker(speaker::Speaker *speaker) { this->output_speaker_ = speaker; }
  void set_output_speaker2(speaker::Speaker *speaker) { this->output_speaker_2_ = speaker; }
  void set_task_stack_in_psram(bool task_stack_in_psram) { this->task_stack_in_psram_ = task_stack_in_psram; }
  void set_task_stack_in_psram2(bool task_stack_in_psram) { this->task_stack_in_psram_2_ = task_stack_in_psram; }

  void set_target_bits_per_sample(uint8_t target_bits_per_sample) { this->target_bits_per_sample_ = target_bits_per_sample; }
  void set_target_sample_rate(uint32_t target_sample_rate) { this->target_sample_rate_ = target_sample_rate; }
  void set_target_num_channels(uint32_t target_num_channels) { this->target_num_channels_ = target_num_channels; }

  void set_target_bits_per_sample_2(uint8_t target_bits_per_sample_2) { this->target_bits_per_sample_2_ = target_bits_per_sample_2; }
  void set_target_sample_rate_2(uint32_t target_sample_rate_2) { this->target_sample_rate_2_ = target_sample_rate_2; }
  void set_target_num_channels_2(uint32_t target_num_channels_2) { this->target_num_channels_2_ = target_num_channels_2; }

  void set_filters(uint16_t filters) { this->filters_ = filters; }
  void set_taps(uint16_t taps) { this->taps_ = taps; }

  void set_buffer_duration(uint32_t buffer_duration_ms) { this->buffer_duration_ms_ = buffer_duration_ms; }

 protected:
  /// @brief Starts the output speaker after setting the resampled stream info. If resampling is required, it starts the
  /// task.
  /// @return ESP_OK if resampling is required
  ///         return value of start_task_() if resampling is required
  esp_err_t start_();
  esp_err_t start_2_();

  /// @brief Starts the resampler task after allocating the task stack
  /// @return ESP_OK if successful,
  ///         ESP_ERR_NO_MEM if the task stack couldn't be allocated
  ///         ESP_ERR_INVALID_STATE if the task wasn't created
  esp_err_t start_task_();
  esp_err_t start_task_2_();

  /// @brief Stops the output speaker. If the resampling task is running, it sends the stop command.
  void stop_();

  /// @brief Deallocates the task stack and resets the pointers.
  /// @return ESP_OK if successful
  ///         ESP_ERR_INVALID_STATE if the task hasn't stopped itself
  esp_err_t delete_task_();
  esp_err_t delete_task_2_();

  inline bool requires_resampling_() const;
  static void resample_task(void *params);
  inline bool requires_resampling_2_() const;
  static void resample_task2(void *params);

  size_t stereoToMono(const uint8_t* input, uint8_t* output, size_t len, uint8_t bits_per_sample);

  EventGroupHandle_t event_group_{nullptr};
  EventGroupHandle_t event_group_2_{nullptr};

  std::weak_ptr<RingBuffer> ring_buffer_;
  std::weak_ptr<RingBuffer> ring_buffer_2_;

  speaker::Speaker *output_speaker_{nullptr};
  speaker::Speaker *output_speaker_2_{nullptr};

  bool task_stack_in_psram_{false};
  bool task_created_{false};
  bool task_stack_in_psram_2_{false};
  bool task_created_2_{false};

  TaskHandle_t task_handle_{nullptr};
  StaticTask_t task_stack_;
  StackType_t *task_stack_buffer_{nullptr};
  
  TaskHandle_t task_handle_2_{nullptr};
  StaticTask_t task_stack_2_;
  StackType_t *task_stack_buffer_2_{nullptr};

  audio::AudioStreamInfo target_stream_info_;
  audio::AudioStreamInfo target_stream_info_2_;

  uint8_t state_2_;

  uint8_t speakerToStart_ = 0;
  uint8_t speakerToStartPlay_ = 0;
  uint8_t speakerToStop_ = 0;

  uint16_t taps_;
  uint16_t filters_;

  uint8_t target_bits_per_sample_;
  uint32_t target_sample_rate_;
  uint8_t target_num_channels_;

  uint8_t target_bits_per_sample_2_;
  uint32_t target_sample_rate_2_;
  uint8_t target_num_channels_2_;

  uint32_t buffer_duration_ms_;

  int32_t playback_differential_ms_{0};
  int32_t playback_differential_ms_2_{0};
};

}  // namespace resampler
}  // namespace esphome

#endif
