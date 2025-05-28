#include "resampler2_speaker.h"

#ifdef USE_ESP32

#include "esphome/components/audio/audio_resampler.h"

#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#include <algorithm>
#include <cstring>

namespace esphome {
namespace resampler2 {

static const UBaseType_t RESAMPLER_TASK_PRIORITY = 1;

static const uint32_t TRANSFER_BUFFER_DURATION_MS = 50;

static const uint32_t TASK_DELAY_MS = 20;
static const uint32_t TASK_STACK_SIZE = 3072;

static const char *const TAG = "resampler2_speaker";

enum ResamplingEventGroupBits : uint32_t {
  COMMAND_STOP = (1 << 0),  // stops the resampler task
  STATE_STARTING = (1 << 10),
  STATE_RUNNING = (1 << 11),
  STATE_STOPPING = (1 << 12),
  STATE_STOPPED = (1 << 13),
  ERR_ESP_NO_MEM = (1 << 19),
  ERR_ESP_NOT_SUPPORTED = (1 << 20),
  ERR_ESP_FAIL = (1 << 21),
  ALL_BITS = 0x00FFFFFF,  // All valid FreeRTOS event group bits
};

void Resampler2Speaker::setup() {
  this->event_group_ = xEventGroupCreate();
  ESP_LOGD(TAG, "Resampler setup");

  if (this->event_group_ == nullptr) {
    ESP_LOGE(TAG, "Failed to create event group");
    this->mark_failed();
    return;
  }

  // FIXED: Updated callback signature for ESPHome 2025.5.0+
  this->output_speaker_->add_audio_output_callback(
      [this](uint32_t new_playback_ms, int64_t remainder_us) {
        int32_t adjustment = this->playback_differential_ms_;
        this->playback_differential_ms_ -= adjustment;
        int32_t adjusted_playback_ms = static_cast<int32_t>(new_playback_ms) + adjustment;
        this->audio_output_callback_(adjusted_playback_ms, remainder_us);
      });

  
  //Speaker2
  this->event_group_2_ = xEventGroupCreate();

  if (this->event_group_2_ == nullptr) {
    ESP_LOGE(TAG, "Failed to create event group 2");
    this->mark_failed();
    return;
  }

  // FIXED: Updated callback signature for ESPHome 2025.5.0+
  this->output_speaker_2_->add_audio_output_callback(
    [this](uint32_t new_playback_ms, int64_t remainder_us) {
      int32_t adjustment = this->playback_differential_ms_;
      this->playback_differential_ms_2_ -= adjustment;
      int32_t adjusted_playback_ms = static_cast<int32_t>(new_playback_ms) + adjustment;
      this->audio_output_callback_(adjusted_playback_ms, remainder_us);
    });
}

void Resampler2Speaker::loop() {
  //Speaker1
  uint32_t event_group_bits = xEventGroupGetBits(this->event_group_);

  if (event_group_bits & ResamplingEventGroupBits::STATE_STARTING) {
    ESP_LOGD(TAG, "Starting resampler task");
    xEventGroupClearBits(this->event_group_, ResamplingEventGroupBits::STATE_STARTING);
  }

  if (event_group_bits & ResamplingEventGroupBits::ERR_ESP_NO_MEM) {
    this->status_set_error("Resampler task failed to allocate the internal buffers");
    xEventGroupClearBits(this->event_group_, ResamplingEventGroupBits::ERR_ESP_NO_MEM);
    this->state_ = speaker::STATE_STOPPING;
  }
  if (event_group_bits & ResamplingEventGroupBits::ERR_ESP_NOT_SUPPORTED) {
    this->status_set_error("Cannot resample due to an unsupported audio stream");
    xEventGroupClearBits(this->event_group_, ResamplingEventGroupBits::ERR_ESP_NOT_SUPPORTED);
    this->state_ = speaker::STATE_STOPPING;
  }
  if (event_group_bits & ResamplingEventGroupBits::ERR_ESP_FAIL) {
    this->status_set_error("Resampler task failed");
    xEventGroupClearBits(this->event_group_, ResamplingEventGroupBits::ERR_ESP_FAIL);
    this->state_ = speaker::STATE_STOPPING;
  }

  if (event_group_bits & ResamplingEventGroupBits::STATE_RUNNING) {
    ESP_LOGD(TAG, "Started resampler task");
    this->status_clear_error();
    xEventGroupClearBits(this->event_group_, ResamplingEventGroupBits::STATE_RUNNING);
  }
  if (event_group_bits & ResamplingEventGroupBits::STATE_STOPPING) {
    ESP_LOGD(TAG, "Stopping resampler task");
    xEventGroupClearBits(this->event_group_, ResamplingEventGroupBits::STATE_STOPPING);
  }
  if (event_group_bits & ResamplingEventGroupBits::STATE_STOPPED) {
    if (this->delete_task_() == ESP_OK) {
      ESP_LOGD(TAG, "Stopped resampler task");
      xEventGroupClearBits(this->event_group_, ResamplingEventGroupBits::ALL_BITS);
    }
  }

  switch (this->state_) {
    case speaker::STATE_STARTING: {
      esp_err_t err = this->start_();
      if (err == ESP_OK) {
        this->status_clear_error();
        this->state_ = speaker::STATE_RUNNING;
      } else {
        switch (err) {
          case ESP_ERR_INVALID_STATE:
            this->status_set_error("Failed to start resampler: resampler task failed to start");
            break;
          case ESP_ERR_NO_MEM:
            this->status_set_error("Failed to start resampler: not enough memory for task stack");
          default:
            this->status_set_error("Failed to start resampler");
            break;
        }

        this->state_ = speaker::STATE_STOPPING;
      }
      break;
    }
    case speaker::STATE_RUNNING:
      if (this->output_speaker_->is_stopped()) {
        this->state_ = speaker::STATE_STOPPING;
      }

      break;
    case speaker::STATE_STOPPING:
      this->speakerToStop_=1;
      this->stop_();
      this->state_ = speaker::STATE_STOPPED;
      break;
    case speaker::STATE_STOPPED:
      break;
  }

  //Speaker2 - breaks in above code will stop this running if speaker1 fails...
  uint32_t event_group_bits_2 = xEventGroupGetBits(this->event_group_2_);

  if (event_group_bits_2 & ResamplingEventGroupBits::STATE_STARTING) {
    //ESP_LOGD(TAG, "Starting resampler task 2");
    xEventGroupClearBits(this->event_group_2_, ResamplingEventGroupBits::STATE_STARTING);
  }

  if (event_group_bits_2 & ResamplingEventGroupBits::ERR_ESP_NO_MEM) {
    this->status_set_error("Resampler task 2 failed to allocate the internal buffers");
    xEventGroupClearBits(this->event_group_2_, ResamplingEventGroupBits::ERR_ESP_NO_MEM);
    this->state_2_ = speaker::STATE_STOPPING;
  }
  if (event_group_bits_2 & ResamplingEventGroupBits::ERR_ESP_NOT_SUPPORTED) {
    this->status_set_error("Cannot resample 2 due to an unsupported audio stream");
    xEventGroupClearBits(this->event_group_2_, ResamplingEventGroupBits::ERR_ESP_NOT_SUPPORTED);
    this->state_2_ = speaker::STATE_STOPPING;
  }
  if (event_group_bits_2 & ResamplingEventGroupBits::ERR_ESP_FAIL) {
    this->status_set_error("Resampler task 2 failed");
    xEventGroupClearBits(this->event_group_2_, ResamplingEventGroupBits::ERR_ESP_FAIL);
    this->state_2_ = speaker::STATE_STOPPING;
  }

  if (event_group_bits_2 & ResamplingEventGroupBits::STATE_RUNNING) {
    ESP_LOGD(TAG, "Started resampler task 2");
    this->status_clear_error();
    xEventGroupClearBits(this->event_group_2_, ResamplingEventGroupBits::STATE_RUNNING);
  }
  if (event_group_bits_2 & ResamplingEventGroupBits::STATE_STOPPING) {
    ESP_LOGD(TAG, "Stopping resampler task 2");
    xEventGroupClearBits(this->event_group_2_, ResamplingEventGroupBits::STATE_STOPPING);
  }
  if (event_group_bits_2 & ResamplingEventGroupBits::STATE_STOPPED) {
    if (this->delete_task_2_() == ESP_OK) {
      ESP_LOGD(TAG, "Stopped resampler task 2");
      xEventGroupClearBits(this->event_group_2_, ResamplingEventGroupBits::ALL_BITS);
    }
  }

  switch (this->state_2_) {
    case speaker::STATE_STARTING: {
      ESP_LOGD(TAG, ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>::Loop; state_2_; transitioning from STARTING to RUNNING");
      esp_err_t err = this->start_2_();
      if (err == ESP_OK) {
        this->status_clear_error();
        this->state_2_ = speaker::STATE_RUNNING;
        ESP_LOGD(TAG, ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>::Loop; state_2_ = STATE_RUNNING");
      } else {
        switch (err) {
          case ESP_ERR_INVALID_STATE:
            this->status_set_error("Failed to start resampler 2: resampler task failed to start");
            break;
          case ESP_ERR_NO_MEM:
            this->status_set_error("Failed to start resampler 2: not enough memory for task stack");
          default:
            this->status_set_error("Failed to start resampler 2");
            break;
        }

        this->state_2_ = speaker::STATE_STOPPING;
      }
      break;
    }
    case speaker::STATE_RUNNING:
      if (this->output_speaker_2_->is_stopped()) {
        this->state_2_ = speaker::STATE_STOPPING;
      }

      break;
    case speaker::STATE_STOPPING:
      this->speakerToStop_=2;
      this->stop_();
      this->state_2_ = speaker::STATE_STOPPED;
      break;
    case speaker::STATE_STOPPED:
      break;
  }
}

size_t Resampler2Speaker::play(const uint8_t *data, size_t length) { 
  return this->play(data, length, 0); 
}

size_t Resampler2Speaker::play(const uint8_t *data, size_t length, TickType_t ticks_to_wait) {
  // CHECK AVAILABLE MEMORY BEFORE ALLOCATING BUFFERS
  size_t free_dma_mem = heap_caps_get_free_size(MALLOC_CAP_DMA);
  size_t free_internal_mem = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
  
  // Log memory status every few seconds
  static uint32_t last_log_time = 0;
  uint32_t now = millis();
  if (now - last_log_time >= 5000) {
    ESP_LOGD(TAG, "Free DMA memory: %d bytes, Free internal: %d bytes", 
             free_dma_mem, free_internal_mem);
    last_log_time = now;
  }
  
  // PREVENT STARTING IF INSUFFICIENT MEMORY
  if (free_dma_mem < 4096 || free_internal_mem < 8192) {
    ESP_LOGW(TAG, "Insufficient memory for audio buffers. DMA: %d, Internal: %d", 
             free_dma_mem, free_internal_mem);
    return 0;  // Don't start if insufficient memory
  }

  if (this->output_speaker_->is_stopped() || this->output_speaker_2_->is_stopped()) {
    ESP_LOGD(TAG, "::play --> Starting speakers");

    // ADD DELAY BETWEEN SPEAKER STARTS TO PREVENT MEMORY RACE
    if (this->output_speaker_->is_stopped() && this->state_ != speaker::STATE_STARTING) { 
      this->speakerToStart_ = 1;
      this->start();
      vTaskDelay(pdMS_TO_TICKS(100)); // Increase delay to 100ms to prevent memory race
    }

    if (this->output_speaker_->is_running() && this->output_speaker_2_->is_stopped() && this->state_2_ != speaker::STATE_STARTING) { 
        this->speakerToStart_ = 2;
        this->start();
        vTaskDelay(pdMS_TO_TICKS(50)); // Add delay for second speaker
    }
  }
  this->speakerToStartPlay_ = 0;

  // Speaker 1
  size_t bytes_written = 0;
  if ((this->output_speaker_->is_running()) && (!this->requires_resampling_())) {
    this->speakerToStartPlay_ = 1;
    bytes_written = this->output_speaker_->play(data, length, ticks_to_wait*3); // needs to be 3x - is this difference in sample rates?
  } else {
    if (this->ring_buffer_.use_count() == 1) {
      std::shared_ptr<RingBuffer> temp_ring_buffer = this->ring_buffer_.lock();
      if (temp_ring_buffer != nullptr) {  // ADD NULL CHECK
        bytes_written = temp_ring_buffer->write_without_replacement(data, length, ticks_to_wait);
      }
    }
  }

  // Speaker 2 (with stereo to mono conversion if needed)
  size_t bytes_written2 = 0;
  
  // Check if stereo to mono conversion is needed
  const uint8_t source_channels = this->audio_stream_info_.get_channels();
  const uint8_t bits_per_sample = this->audio_stream_info_.get_bits_per_sample();
  bool need_conversion = (source_channels > 1 && this->target_num_channels_2_ == 1);
  
  // Static buffer for mono conversion to avoid frequent allocations
  static uint8_t* mono_buffer = nullptr;
  static size_t mono_buffer_size = 0;
  
  // Convert if needed
  const uint8_t* speaker2_data = data;
  size_t speaker2_length = length;
  
  if (need_conversion) {
    // Calculate mono output size
    size_t bytes_per_sample = bits_per_sample / 8;
    size_t mono_length = length / source_channels;
    
    // Allocate or resize buffer if needed - CHECK MEMORY FIRST
    if (mono_buffer == nullptr || mono_buffer_size < mono_length) {
      // CHECK IF WE HAVE ENOUGH MEMORY FOR MONO BUFFER
      if (heap_caps_get_free_size(MALLOC_CAP_INTERNAL) < mono_length + 1024) {
        ESP_LOGW(TAG, "Insufficient memory for mono conversion buffer");
        need_conversion = false;
      } else {
        if (mono_buffer != nullptr) {
          free(mono_buffer);
        }
        mono_buffer = static_cast<uint8_t*>(malloc(mono_length));
        if (mono_buffer == nullptr) {
          ESP_LOGE(TAG, "Failed to allocate mono conversion buffer");
          need_conversion = false; // Fall back to original data
        } else {
          mono_buffer_size = mono_length;
        }
      }
    }
    
    // Perform conversion if buffer allocation succeeded
    if (need_conversion && mono_buffer != nullptr) {
      // Call the standalone stereoToMono function
      size_t converted_size = stereoToMono(data, mono_buffer, length, bits_per_sample);
      
      if (converted_size > 0) {
        speaker2_data = mono_buffer;
        speaker2_length = converted_size;
      } else {
        // Conversion failed, fall back to original data
        speaker2_data = data;
        speaker2_length = length;
        ESP_LOGW(TAG, "Stereo to mono conversion failed, using original data");
      }
    }
  }
  
  // Now use the potentially converted data
  if ((this->output_speaker_2_->is_running()) && (!this->requires_resampling_2_())) {
    this->speakerToStartPlay_ = 2;
    bytes_written2 = this->output_speaker_2_->play(speaker2_data, speaker2_length, ticks_to_wait);
  } else {
    if (this->ring_buffer_2_.use_count() == 1) {
      std::shared_ptr<RingBuffer> temp_ring_buffer = this->ring_buffer_2_.lock();
      if (temp_ring_buffer != nullptr) {  // ADD NULL CHECK
        bytes_written2 = temp_ring_buffer->write(speaker2_data, speaker2_length);
      }
    }
  }

  // Log occasionally for debugging
  static uint32_t last_debug_time = 0;
  if (now - last_debug_time >= 3000) { // Print only every 3 seconds
    ESP_LOGD(TAG, "length %d, bytes_written %d, byte_written2 %d", length, bytes_written, bytes_written2);
    last_debug_time = now;
  }

  return bytes_written;
}

/**
 * Converts stereo PCM audio samples to mono by averaging the left and right channels.
 * Supports 8, 16, 24, and 32-bit audio with proper handling of different bit depths.
 * 
 * @param input Pointer to the input buffer containing interleaved stereo samples
 * @param output Pointer to the output buffer for mono samples (must be at least half the size of input)
 * @param len Length of the input buffer in bytes
 * @param bits_per_sample Bit depth of each sample (8, 16, 24, or 32)
 * @return The number of bytes written to the output buffer
 */
size_t Resampler2Speaker::stereoToMono(const uint8_t* input, uint8_t* output, size_t len, uint8_t bits_per_sample) {
  if (input == nullptr || output == nullptr || len == 0 || len % 2 != 0) {
      return 0;  // Invalid input parameters
  }

  size_t bytes_per_sample = bits_per_sample / 8;
  size_t bytes_per_stereo_frame = bytes_per_sample * 2;  // Left + Right
  size_t num_stereo_frames = len / bytes_per_stereo_frame;
  size_t output_size = num_stereo_frames * bytes_per_sample;

  switch (bits_per_sample) {
      case 8: {
          // 8-bit audio is typically unsigned (0-255)
          for (size_t i = 0; i < num_stereo_frames; i++) {
              const uint8_t* stereo_frame = input + (i * bytes_per_stereo_frame);
              uint8_t left = stereo_frame[0];
              uint8_t right = stereo_frame[1];
              
              // Average the samples
              output[i] = (uint8_t)((uint16_t)left + (uint16_t)right) / 2;
          }
          break;
      }
      
      case 16: {
          // 16-bit audio is typically signed (-32768 to 32767)
          for (size_t i = 0; i < num_stereo_frames; i++) {
              const uint8_t* stereo_frame = input + (i * bytes_per_stereo_frame);
              
              // Extract left and right samples (assuming little-endian byte order)
              int16_t left = (int16_t)(stereo_frame[0] | (stereo_frame[1] << 8));
              int16_t right = (int16_t)(stereo_frame[2] | (stereo_frame[3] << 8));
              
              // Average the samples
              int16_t mono = (int16_t)(((int32_t)left + (int32_t)right) / 2);
              
              // Write the result (little-endian)
              output[i * 2] = mono & 0xFF;
              output[i * 2 + 1] = (mono >> 8) & 0xFF;
          }
          break;
      }
      
      case 24: {
          // 24-bit audio is typically signed (-8388608 to 8388607)
          for (size_t i = 0; i < num_stereo_frames; i++) {
              const uint8_t* stereo_frame = input + (i * bytes_per_stereo_frame);
              
              // Extract left and right samples (assuming little-endian byte order)
              int32_t left = (int32_t)(
                  stereo_frame[0] |
                  (stereo_frame[1] << 8) |
                  (stereo_frame[2] << 16) |
                  // Sign extension for negative values
                  ((stereo_frame[2] & 0x80) ? 0xFF000000 : 0)
              );
              
              int32_t right = (int32_t)(
                  stereo_frame[3] |
                  (stereo_frame[4] << 8) |
                  (stereo_frame[5] << 16) |
                  // Sign extension for negative values
                  ((stereo_frame[5] & 0x80) ? 0xFF000000 : 0)
              );
              
              // Average the samples
              int32_t mono = (left + right) / 2;
              
              // Write the result (little-endian), keeping only 24 bits
              output[i * 3] = mono & 0xFF;
              output[i * 3 + 1] = (mono >> 8) & 0xFF;
              output[i * 3 + 2] = (mono >> 16) & 0xFF;
          }
          break;
      }
      
      case 32: {
          // 32-bit audio is typically signed (-2147483648 to 2147483647)
          for (size_t i = 0; i < num_stereo_frames; i++) {
              const uint8_t* stereo_frame = input + (i * bytes_per_stereo_frame);
              
              // Extract left and right samples (assuming little-endian byte order)
              int32_t left = (int32_t)(
                  stereo_frame[0] |
                  (stereo_frame[1] << 8) |
                  (stereo_frame[2] << 16) |
                  (stereo_frame[3] << 24)
              );
              
              int32_t right = (int32_t)(
                  stereo_frame[4] |
                  (stereo_frame[5] << 8) |
                  (stereo_frame[6] << 16) |
                  (stereo_frame[7] << 24)
              );
              
              // Average the samples
              int32_t mono = (int32_t)(((int64_t)left + (int64_t)right) / 2);
              
              // Write the result (little-endian)
              output[i * 4] = mono & 0xFF;
              output[i * 4 + 1] = (mono >> 8) & 0xFF;
              output[i * 4 + 2] = (mono >> 16) & 0xFF;
              output[i * 4 + 3] = (mono >> 24) & 0xFF;
          }
          break;
      }
      
      default:
          // Unsupported bit depth
          return 0;
  }
  
  return output_size;
}

void Resampler2Speaker::start() { 
  ESP_LOGD(TAG, "::start .... this->speakerToStart_ = %d", this->speakerToStart_);
  if (this->speakerToStart_ == 1) {
    ESP_LOGD(TAG, "::start =1");
    this->state_ = speaker::STATE_STARTING; 
  }
  // MH: Comment out task2 and speaker1 audio is clean...
  else if (this->speakerToStart_ == 2) {
    ESP_LOGD(TAG, ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>::start; setting state_2_ to STATE_STARTING");
    this->state_2_ = speaker::STATE_STARTING; 
  }
  this->speakerToStart_ = 0;
}

esp_err_t Resampler2Speaker::start_() {
  ESP_LOGD(TAG, "::start_");

  this->target_stream_info_ = audio::AudioStreamInfo(
      this->target_bits_per_sample_, this->audio_stream_info_.get_channels(), this->target_sample_rate_);

  this->output_speaker_->set_audio_stream_info(this->target_stream_info_);
  this->speakerToStart_ = 1;
  this->output_speaker_->start();

  if (this->requires_resampling_()) {
    // Start the resampler task to handle converting sample rates
    return this->start_task_();
  }
  
  return ESP_OK;
}

esp_err_t Resampler2Speaker::start_2_() {
  this->target_stream_info_2_ = audio::AudioStreamInfo(
      this->target_bits_per_sample_2_, this->audio_stream_info_.get_channels(), this->target_sample_rate_2_);

  this->output_speaker_2_->set_audio_stream_info(this->target_stream_info_2_);
    ESP_LOGD(TAG, ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>start_2 -> calling start()");
  this->speakerToStart_ = 2;
  this->output_speaker_2_->start(); //start() of speaker

  if (this->requires_resampling_2_()) {
    // Start the resampler task to handle converting sample rates
    ESP_LOGD(TAG, ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>start_2 -> starting the resampler");
    return this->start_task_2_();
  }

  return ESP_OK;
}

esp_err_t Resampler2Speaker::start_task_() {
  ESP_LOGD(TAG, "::start_task_");
  if (this->task_stack_buffer_ == nullptr) {
    if (this->task_stack_in_psram_) {
      RAMAllocator<StackType_t> stack_allocator(RAMAllocator<StackType_t>::ALLOC_EXTERNAL);
      this->task_stack_buffer_ = stack_allocator.allocate(TASK_STACK_SIZE);
    } else {
      RAMAllocator<StackType_t> stack_allocator(RAMAllocator<StackType_t>::ALLOC_INTERNAL);
      this->task_stack_buffer_ = stack_allocator.allocate(TASK_STACK_SIZE);
    }
  }

  if (this->task_stack_buffer_ == nullptr) {
    return ESP_ERR_NO_MEM;
  }

  if (this->task_handle_ == nullptr) {
    this->task_handle_ = xTaskCreateStatic(resample_task, "sample", TASK_STACK_SIZE, (void *) this,
                                           RESAMPLER_TASK_PRIORITY, this->task_stack_buffer_, &this->task_stack_);
  }

  if (this->task_handle_ == nullptr) {
    return ESP_ERR_INVALID_STATE;
  }

  return ESP_OK;
}

esp_err_t Resampler2Speaker::start_task_2_() {
  ESP_LOGD(TAG, "::start_task_2_");
  if (this->task_stack_buffer_2_ == nullptr) {
    if (this->task_stack_in_psram_2_) {
      RAMAllocator<StackType_t> stack_allocator(RAMAllocator<StackType_t>::ALLOC_EXTERNAL);
      this->task_stack_buffer_2_ = stack_allocator.allocate(TASK_STACK_SIZE);
    } else {
      RAMAllocator<StackType_t> stack_allocator(RAMAllocator<StackType_t>::ALLOC_INTERNAL);
      this->task_stack_buffer_2_ = stack_allocator.allocate(TASK_STACK_SIZE);
    }
  }

  if (this->task_stack_buffer_2_ == nullptr) {
    return ESP_ERR_NO_MEM;
  }

  if (this->task_handle_2_ == nullptr) {
    this->task_handle_2_ = xTaskCreateStatic(resample_task2, "sample2", TASK_STACK_SIZE, (void *) this,
                                           RESAMPLER_TASK_PRIORITY, this->task_stack_buffer_2_, &this->task_stack_2_);
  }

  if (this->task_handle_2_ == nullptr) {
    return ESP_ERR_INVALID_STATE;
  }

  return ESP_OK;
}

void Resampler2Speaker::stop() {  // how to tell which one to stop??
  ESP_LOGD(TAG, "::stop");
  this->state_ = speaker::STATE_STOPPING; 
  this->state_2_ = speaker::STATE_STOPPING; 
}


void Resampler2Speaker::stop_() {
  /*
  ESP_LOGD(TAG, "::stop_");
  if (this->speakerToStop_==1) {
    ESP_LOGD(TAG, "::stop_ ... speakerToStop_==1");
    if (this->task_handle_ != nullptr) {
      xEventGroupSetBits(this->event_group_, ResamplingEventGroupBits::COMMAND_STOP);
    }
    this->output_speaker_->stop();
  }

  if (this->speakerToStop_==2) {
    ESP_LOGD(TAG, "::stop_ ... speakerToStop_==2");
    if (this->task_handle_2_ != nullptr) {
      xEventGroupSetBits(this->event_group_2_, ResamplingEventGroupBits::COMMAND_STOP);
    }
    this->output_speaker_2_->stop();
  }
  this->speakerToStop_=0;
*/
//ChatGPT
  if (this->speakerToStop_ == 1 && this->output_speaker_->is_running()) {
    ESP_LOGD(TAG, "::stop_ ... speakerToStop_==1");
    if (this->task_handle_ != nullptr) {
        xEventGroupSetBits(this->event_group_, ResamplingEventGroupBits::COMMAND_STOP);
    }
    this->output_speaker_->stop();
  }

  if (this->speakerToStop_ == 2 && this->output_speaker_2_->is_running()) {
    ESP_LOGD(TAG, "::stop_ ... speakerToStop_==2");
    if (this->task_handle_2_ != nullptr) {
        xEventGroupSetBits(this->event_group_2_, ResamplingEventGroupBits::COMMAND_STOP);
    }
    this->output_speaker_2_->stop();
  }
}

esp_err_t Resampler2Speaker::delete_task_() {
  ESP_LOGD(TAG, "::delete_task_");
  if (!this->task_created_) {
    this->task_handle_ = nullptr;

    if (this->task_stack_buffer_ != nullptr) {
      if (this->task_stack_in_psram_) {
        RAMAllocator<StackType_t> stack_allocator(RAMAllocator<StackType_t>::ALLOC_EXTERNAL);
        stack_allocator.deallocate(this->task_stack_buffer_, TASK_STACK_SIZE);
      } else {
        RAMAllocator<StackType_t> stack_allocator(RAMAllocator<StackType_t>::ALLOC_INTERNAL);
        stack_allocator.deallocate(this->task_stack_buffer_, TASK_STACK_SIZE);
      }

      this->task_stack_buffer_ = nullptr;
    }

    return ESP_OK;
  }

  return ESP_ERR_INVALID_STATE;
}

esp_err_t Resampler2Speaker::delete_task_2_() {
  ESP_LOGD(TAG, "::delete_task_2_");
  if (!this->task_created_2_) {
    this->task_handle_2_ = nullptr;

    if (this->task_stack_buffer_2_ != nullptr) {
      if (this->task_stack_in_psram_2_) {
        RAMAllocator<StackType_t> stack_allocator(RAMAllocator<StackType_t>::ALLOC_EXTERNAL);
        stack_allocator.deallocate(this->task_stack_buffer_2_, TASK_STACK_SIZE);
      } else {
        RAMAllocator<StackType_t> stack_allocator(RAMAllocator<StackType_t>::ALLOC_INTERNAL);
        stack_allocator.deallocate(this->task_stack_buffer_2_, TASK_STACK_SIZE);
      }

      this->task_stack_buffer_2_ = nullptr;
    }

    return ESP_OK;
  }

  return ESP_ERR_INVALID_STATE;
}

void Resampler2Speaker::finish() {  // how to tell which one to finish?
  ESP_LOGD(TAG, "::finish");
  this->output_speaker_->finish(); 
  this->output_speaker_2_->finish();
}


bool Resampler2Speaker::has_buffered_data() const {  // which buffer?  Which speaker??
  ESP_LOGD(TAG, "::has_buffered_data");
  bool has_ring_buffer_data = false;
  if (this->requires_resampling_() && (this->ring_buffer_.use_count() > 0)) {
    has_ring_buffer_data = (this->ring_buffer_.lock()->available() > 0);
  }
  return (has_ring_buffer_data || this->output_speaker_->has_buffered_data());
}

/*bool Resampler2Speaker::has_buffered_data2() const {
  bool has_ring_buffer_data2 = false;
  if (this->requires_resampling_2() && (this->ring_buffer_2_.use_count() > 0)) {
    has_ring_buffer_data2 = (this->ring_buffer_2.lock()->available() > 0);
  }
  return (has_ring_buffer_data2 || this->output_speaker_2_->has_buffered_data());
}*/

void Resampler2Speaker::set_mute_state(bool mute_state) {
  ESP_LOGD(TAG, "::set_mute_state");
  this->mute_state_ = mute_state;
  this->output_speaker_->set_mute_state(mute_state);
  this->output_speaker_2_->set_mute_state(mute_state);
}

void Resampler2Speaker::set_volume(float volume) {
  ESP_LOGD(TAG, "::set_volume");
  this->volume_ = volume;
  this->output_speaker_->set_volume(volume);
  this->output_speaker_2_->set_volume(volume);
}

bool Resampler2Speaker::requires_resampling_() const {
  //ESP_LOGD(TAG, "::requires_resampling_");
  return (this->audio_stream_info_.get_sample_rate() != this->target_sample_rate_) ||
         (this->audio_stream_info_.get_bits_per_sample() != this->target_bits_per_sample_);
}

bool Resampler2Speaker::requires_resampling_2_() const {
  //ESP_LOGD(TAG, "::requires_resampling_2_");
  return (this->audio_stream_info_.get_sample_rate() != this->target_sample_rate_2_) ||
         (this->audio_stream_info_.get_bits_per_sample() != this->target_bits_per_sample_2_);
}

void Resampler2Speaker::resample_task(void *params) {
  ESP_LOGD(TAG, "::resample_task");
  Resampler2Speaker *this_resampler = (Resampler2Speaker *) params;

  this_resampler->task_created_ = true;
  xEventGroupSetBits(this_resampler->event_group_, ResamplingEventGroupBits::STATE_STARTING);

  std::unique_ptr<audio::AudioResampler> resampler =
      make_unique<audio::AudioResampler>(this_resampler->audio_stream_info_.ms_to_bytes(TRANSFER_BUFFER_DURATION_MS),
                                         this_resampler->target_stream_info_.ms_to_bytes(TRANSFER_BUFFER_DURATION_MS));

  esp_err_t err = resampler->start(this_resampler->audio_stream_info_, this_resampler->target_stream_info_,
                                   this_resampler->taps_, this_resampler->filters_);

  if (err == ESP_OK) {
    std::shared_ptr<RingBuffer> temp_ring_buffer =
        RingBuffer::create(this_resampler->audio_stream_info_.ms_to_bytes(this_resampler->buffer_duration_ms_));

    if (temp_ring_buffer.use_count() == 0) {
      err = ESP_ERR_NO_MEM;
    } else {
      this_resampler->ring_buffer_ = temp_ring_buffer;
      resampler->add_source(this_resampler->ring_buffer_);

      this_resampler->output_speaker_->set_audio_stream_info(this_resampler->target_stream_info_);
      resampler->add_sink(this_resampler->output_speaker_);
    }
  }

  if (err == ESP_OK) {
    xEventGroupSetBits(this_resampler->event_group_, ResamplingEventGroupBits::STATE_RUNNING);
  } else if (err == ESP_ERR_NO_MEM) {
    xEventGroupSetBits(this_resampler->event_group_, ResamplingEventGroupBits::ERR_ESP_NO_MEM);
  } else if (err == ESP_ERR_NOT_SUPPORTED) {
    xEventGroupSetBits(this_resampler->event_group_, ResamplingEventGroupBits::ERR_ESP_NOT_SUPPORTED);
  }

  this_resampler->playback_differential_ms_ = 0;
  while (err == ESP_OK) {
    uint32_t event_bits = xEventGroupGetBits(this_resampler->event_group_);

    if (event_bits & ResamplingEventGroupBits::COMMAND_STOP) {
      break;
    }

    // Stop gracefully if the decoder is done
    int32_t ms_differential = 0;
    audio::AudioResamplerState resampler_state = resampler->resample(false, &ms_differential);

    this_resampler->playback_differential_ms_ += ms_differential;

    if (resampler_state == audio::AudioResamplerState::FINISHED) {
      break;
    } else if (resampler_state == audio::AudioResamplerState::FAILED) {
      xEventGroupSetBits(this_resampler->event_group_, ResamplingEventGroupBits::ERR_ESP_FAIL);
      break;
    }
  }

  xEventGroupSetBits(this_resampler->event_group_, ResamplingEventGroupBits::STATE_STOPPING);
  resampler.reset();
  xEventGroupSetBits(this_resampler->event_group_, ResamplingEventGroupBits::STATE_STOPPED);
  this_resampler->task_created_ = false;
  vTaskDelete(nullptr);
}

void Resampler2Speaker::resample_task2(void *params) {
  ESP_LOGD(TAG, "::resample_task2");
  Resampler2Speaker *this_resampler2 = (Resampler2Speaker *) params;

  this_resampler2->task_created_2_ = true;
  xEventGroupSetBits(this_resampler2->event_group_2_, ResamplingEventGroupBits::STATE_STARTING);

  std::unique_ptr<audio::AudioResampler> resampler2 =
      make_unique<audio::AudioResampler>(this_resampler2->audio_stream_info_.ms_to_bytes(TRANSFER_BUFFER_DURATION_MS),
                                         this_resampler2->target_stream_info_2_.ms_to_bytes(TRANSFER_BUFFER_DURATION_MS));

  esp_err_t err = resampler2->start(this_resampler2->audio_stream_info_, this_resampler2->target_stream_info_2_,
                                   this_resampler2->taps_, this_resampler2->filters_);

  if (err == ESP_OK) {
    std::shared_ptr<RingBuffer> temp_ring_buffer =
        RingBuffer::create(this_resampler2->audio_stream_info_.ms_to_bytes(this_resampler2->buffer_duration_ms_));

    if (temp_ring_buffer.use_count() == 0) {
      err = ESP_ERR_NO_MEM;
    } else {
      this_resampler2->ring_buffer_2_ = temp_ring_buffer;
      resampler2->add_source(this_resampler2->ring_buffer_2_);

      this_resampler2->output_speaker_2_->set_audio_stream_info(this_resampler2->target_stream_info_2_);
      resampler2->add_sink(this_resampler2->output_speaker_2_);
    }
  }

  if (err == ESP_OK) {
    xEventGroupSetBits(this_resampler2->event_group_2_, ResamplingEventGroupBits::STATE_RUNNING);
  } else if (err == ESP_ERR_NO_MEM) {
    xEventGroupSetBits(this_resampler2->event_group_2_, ResamplingEventGroupBits::ERR_ESP_NO_MEM);
  } else if (err == ESP_ERR_NOT_SUPPORTED) {
    xEventGroupSetBits(this_resampler2->event_group_2_, ResamplingEventGroupBits::ERR_ESP_NOT_SUPPORTED);
  }

  this_resampler2->playback_differential_ms_2_ = 0;
  while (err == ESP_OK) {
    uint32_t event_bits = xEventGroupGetBits(this_resampler2->event_group_2_);

    if (event_bits & ResamplingEventGroupBits::COMMAND_STOP) {
      break;
    }

    // Stop gracefully if the decoder is done
    int32_t ms_differential = 0;
    audio::AudioResamplerState resampler_state2 = resampler2->resample(false, &ms_differential);

    this_resampler2->playback_differential_ms_2_ += ms_differential;

    if (resampler_state2 == audio::AudioResamplerState::FINISHED) {
      break;
    } else if (resampler_state2 == audio::AudioResamplerState::FAILED) {
      xEventGroupSetBits(this_resampler2->event_group_2_, ResamplingEventGroupBits::ERR_ESP_FAIL);
      break;
    }
  }

  xEventGroupSetBits(this_resampler2->event_group_2_, ResamplingEventGroupBits::STATE_STOPPING);
  resampler2.reset();
  xEventGroupSetBits(this_resampler2->event_group_2_, ResamplingEventGroupBits::STATE_STOPPED);
  this_resampler2->task_created_2_= false;
  vTaskDelete(nullptr);
}


}  // namespace resampler2
}  // namespace esphome

#endif