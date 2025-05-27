import esphome.codegen as cg
from esphome.components import audio, esp32, speaker
import esphome.config_validation as cv
from esphome.const import (
    CONF_BITS_PER_SAMPLE,
    CONF_BUFFER_DURATION,
    CONF_FILTERS,
    CONF_ID,
    CONF_NUM_CHANNELS,
    CONF_OUTPUT_SPEAKER,
    CONF_SAMPLE_RATE,
    CONF_TASK_STACK_IN_PSRAM,
    PLATFORM_ESP32,
)
from esphome.core.entity_helpers import inherit_property_from

CONF_OUTPUT_SPEAKER2 = "output_speaker2"
CONF_NUM_CHANNELS2 = "num_channels2"
CONF_SAMPLE_RATE2 = "sample_rate2"
CONF_BITS_PER_SAMPLE2 = "bits_per_sample2"

AUTO_LOAD = ["audio"]
CODEOWNERS = ["@kahrendt"]

resampler2_ns = cg.esphome_ns.namespace("resampler2")
Resampler2Speaker = resampler2_ns.class_(
    "Resampler2Speaker", cg.Component, speaker.Speaker
)

CONF_TAPS = "taps"


def _set_stream_limits(config):
    audio.set_stream_limits(
        min_bits_per_sample=16,
        max_bits_per_sample=32,
    )(config)

    return config


def _validate_audio_compatability(config):
    inherit_property_from(CONF_BITS_PER_SAMPLE, CONF_OUTPUT_SPEAKER)(config)
    inherit_property_from(CONF_NUM_CHANNELS, CONF_OUTPUT_SPEAKER)(config)
    inherit_property_from(CONF_SAMPLE_RATE, CONF_OUTPUT_SPEAKER)(config)

    inherit_property_from(CONF_BITS_PER_SAMPLE2, CONF_OUTPUT_SPEAKER2)(config)
    inherit_property_from(CONF_NUM_CHANNELS2, CONF_OUTPUT_SPEAKER2)(config)
    inherit_property_from(CONF_SAMPLE_RATE2, CONF_OUTPUT_SPEAKER2)(config)

    audio.final_validate_audio_schema(
        "source_speaker1",
        audio_device=CONF_OUTPUT_SPEAKER,  #speaker or speaker2?  Any difference?
        bits_per_sample=config.get(CONF_BITS_PER_SAMPLE),
        channels=config.get(CONF_NUM_CHANNELS),
        sample_rate=config.get(CONF_SAMPLE_RATE),
    )(config)

    audio.final_validate_audio_schema(
        "source_speaker2",
        audio_device=CONF_OUTPUT_SPEAKER2,  #speaker or speaker2?  Any difference?
        bits_per_sample=config.get(CONF_BITS_PER_SAMPLE2),
        channels=config.get(CONF_NUM_CHANNELS2),
        sample_rate=config.get(CONF_SAMPLE_RATE2),
    )(config)


def _validate_taps(taps):
    value = cv.int_range(min=16, max=128)(taps)
    if value % 4 != 0:
        raise cv.Invalid("Number of taps must be divisible by 4")
    return value


CONFIG_SCHEMA = cv.All(
    speaker.SPEAKER_SCHEMA.extend(
        {
            cv.GenerateID(): cv.declare_id(Resampler2Speaker),
            cv.Required(CONF_OUTPUT_SPEAKER): cv.use_id(speaker.Speaker),
            cv.Required(CONF_OUTPUT_SPEAKER2): cv.use_id(speaker.Speaker),
            cv.Optional(CONF_NUM_CHANNELS, default=2): cv.int_range(min=1, max=2),
            cv.Optional(CONF_NUM_CHANNELS2, default=2): cv.int_range(min=1, max=2),
            cv.Optional(CONF_SAMPLE_RATE, default=48000): cv.int_,
            cv.Optional(CONF_SAMPLE_RATE2, default=48000): cv.int_,
            cv.Optional(CONF_BITS_PER_SAMPLE, default=16): cv.int_,
            cv.Optional(CONF_BITS_PER_SAMPLE2, default=16): cv.int_,
            cv.Optional(
                CONF_BUFFER_DURATION, default="100ms"
            ): cv.positive_time_period_milliseconds,
            cv.SplitDefault(CONF_TASK_STACK_IN_PSRAM, esp32_idf=False): cv.All(
                cv.boolean, cv.only_with_esp_idf
            ),
            cv.Optional(CONF_FILTERS, default=16): cv.int_range(min=2, max=1024),
            cv.Optional(CONF_TAPS, default=16): _validate_taps,
        }
    ).extend(cv.COMPONENT_SCHEMA),
    cv.only_on([PLATFORM_ESP32]),
    _set_stream_limits,
)


FINAL_VALIDATE_SCHEMA = _validate_audio_compatability


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await speaker.register_speaker(var, config)

    output_spkr = await cg.get_variable(config[CONF_OUTPUT_SPEAKER])
    cg.add(var.set_output_speaker(output_spkr))

    output_spkr2 = await cg.get_variable(config[CONF_OUTPUT_SPEAKER2])
    cg.add(var.set_output_speaker2(output_spkr2))

    cg.add(var.set_buffer_duration(config[CONF_BUFFER_DURATION]))

    if task_stack_in_psram := config.get(CONF_TASK_STACK_IN_PSRAM):
        cg.add(var.set_task_stack_in_psram(task_stack_in_psram))
        cg.add(var.set_task_stack_in_psram_2(task_stack_in_psram_2))
        if task_stack_in_psram:
            if config[CONF_TASK_STACK_IN_PSRAM]:
                esp32.add_idf_sdkconfig_option(
                    "CONFIG_SPIRAM_ALLOW_STACK_EXTERNAL_MEMORY", True
                )

    cg.add(var.set_target_num_channels(config[CONF_NUM_CHANNELS])) 
    cg.add(var.set_target_bits_per_sample(config[CONF_BITS_PER_SAMPLE]))
    cg.add(var.set_target_sample_rate(config[CONF_SAMPLE_RATE]))
    cg.add(var.set_target_num_channels_2(config[CONF_NUM_CHANNELS2]))
    cg.add(var.set_target_bits_per_sample_2(config[CONF_BITS_PER_SAMPLE2]))
    cg.add(var.set_target_sample_rate_2(config[CONF_SAMPLE_RATE2]))

    cg.add(var.set_filters(config[CONF_FILTERS]))
    cg.add(var.set_taps(config[CONF_TAPS]))
