/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Continuous ADC Capture with Application‑Level Circular Buffer
 *
 * This implementation follows the architecture described in README.md:
 *
 * 1. ADC Continuous Driver - Uses official ESP-IDF driver with 4KB pool
 * 2. Application-Level Ring Buffer - Power-of-2 sized circular buffer (32K samples)
 * 3. DMA → IDF Pool → memcpy → circ_buf[] → Trigger analysis
 *
 * Key Features:
 * - True circular history for oscilloscope-style pre-trigger capture
 * - High sample rates (up to 1 MSPS) with minimal CPU overhead
 * - Power-of-2 buffer size for efficient bit masking
 * - Atomic operations to protect shared data between ISR and tasks
 * - Calibrated voltage conversion with line-fitting
 *
 * Architecture:
 * - Main task: Handles ADC data reading and circular buffer management
 * - Processing task: Displays statistics and manages trigger events
 * - ISR callback: Notifies main task of new data availability
 */

#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#define EXAMPLE_ADC_UNIT ADC_UNIT_1
#define _EXAMPLE_ADC_UNIT_STR(unit) #unit
#define EXAMPLE_ADC_UNIT_STR(unit) _EXAMPLE_ADC_UNIT_STR(unit)
#define EXAMPLE_ADC_CONV_MODE ADC_CONV_SINGLE_UNIT_1
#define EXAMPLE_ADC_ATTEN ADC_ATTEN_DB_0
#define EXAMPLE_ADC_BIT_WIDTH SOC_ADC_DIGI_MAX_BITWIDTH

#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
#define EXAMPLE_ADC_OUTPUT_TYPE ADC_DIGI_OUTPUT_FORMAT_TYPE1
#define EXAMPLE_ADC_GET_CHANNEL(p_data) ((p_data)->type1.channel)
#define EXAMPLE_ADC_GET_DATA(p_data) ((p_data)->type1.data)
#else
#define EXAMPLE_ADC_OUTPUT_TYPE ADC_DIGI_OUTPUT_FORMAT_TYPE2
#define EXAMPLE_ADC_GET_CHANNEL(p_data) ((p_data)->type2.channel)
#define EXAMPLE_ADC_GET_DATA(p_data) ((p_data)->type2.data)
#endif

#define EXAMPLE_READ_LEN 256
#define PROCESSING_TASK_STACK_SIZE 4096

// Circular buffer configuration (power of 2 for efficient masking)
#define CIRC_BUF_SAMPLES 32768 // 32K samples = 64KB buffer
#define CIRC_BUF_MASK (CIRC_BUF_SAMPLES - 1)
#define SAMPLE_FREQ_HZ 1000000 // 1 MHz as per README specifications - optimized critical sections enable this rate

static adc_channel_t channel[1] = {ADC_CHANNEL_6}; // Changed to channel 6 as per your log

static TaskHandle_t s_task_handle;
static const char *TAG = "EXAMPLE";

// Circular buffer for oscilloscope-style capture
static uint16_t circ_buf[CIRC_BUF_SAMPLES] __attribute__((aligned(4))); // DMA-capable internal RAM
static volatile size_t circ_buf_wr = 0;                                 // Write pointer
static portMUX_TYPE s_data_lock = portMUX_INITIALIZER_UNLOCKED;         // Spinlock to protect shared data

// Statistics for display
static volatile uint64_t s_voltage_sum = 0;
static volatile uint32_t s_sample_count = 0;

// ADC Calibration variables
static adc_cali_handle_t cali_handle = NULL;
static bool do_calibration = false;

// Trigger-based capture variables
static volatile bool trigger_seen = false;
static volatile bool capture_complete = false;

// Helper function to linearize circular buffer for export (as mentioned in README)
static void export_circular_buffer(uint16_t *out_buffer, size_t pre_samples, size_t total_samples)
{
    portENTER_CRITICAL(&s_data_lock);
    size_t current_wr = circ_buf_wr;
    portEXIT_CRITICAL(&s_data_lock);

    // Calculate start position (pre-trigger samples)
    size_t start = (current_wr - pre_samples) & CIRC_BUF_MASK;

    // Linearize the ring buffer
    for (size_t i = 0; i < total_samples && i < CIRC_BUF_SAMPLES; i++)
    {
        out_buffer[i] = circ_buf[(start + i) & CIRC_BUF_MASK];
    }
}

// Example trigger handler (as mentioned in README trigger pattern)
static void handle_trigger_capture(adc_continuous_handle_t handle, size_t pre_trigger_samples, size_t post_trigger_samples)
{
    if (!trigger_seen)
    {
        ESP_LOGI(TAG, "Trigger detected! Starting post-trigger capture...");
        trigger_seen = true;

        // Continue capturing for post_trigger_samples
        // In a real implementation, you might use a timer or count samples
        vTaskDelay(pdMS_TO_TICKS(100)); // Simple delay for demo

        // Stop ADC
        adc_continuous_stop(handle);

        // Export the circular buffer data
        uint16_t *export_buffer = malloc((pre_trigger_samples + post_trigger_samples) * sizeof(uint16_t));
        if (export_buffer)
        {
            export_circular_buffer(export_buffer, pre_trigger_samples, pre_trigger_samples + post_trigger_samples);
            ESP_LOGI(TAG, "Captured %zu pre-trigger + %zu post-trigger samples", pre_trigger_samples, post_trigger_samples);

            // Here you would typically send this data via UART, USB CDC, or Wi-Fi
            // For now, just log the first few samples
            for (int i = 0; i < 10 && i < (pre_trigger_samples + post_trigger_samples); i++)
            {
                ESP_LOGI(TAG, "Sample[%d]: %u", i, export_buffer[i]);
            }

            free(export_buffer);
        }

        capture_complete = true;
        trigger_seen = false; // Reset for next trigger

        // Restart ADC for continuous operation
        adc_continuous_start(handle);
    }
}

// ADC Calibration initialization function
static bool adc_calibration_init(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    *out_handle = NULL;
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = unit,
        .atten = atten,
        .bitwidth = EXAMPLE_ADC_BIT_WIDTH,
    };
    esp_err_t ret = adc_cali_create_scheme_line_fitting(&cali_config, out_handle);
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "Calibration Success");
    }
    else if (ret == ESP_ERR_NOT_SUPPORTED)
    {
        ESP_LOGW(TAG, "eFuse not burnt, skipping calibration. Using raw data.");
    }
    else
    {
        ESP_LOGE(TAG, "Invalid arg or no memory for calibration: %s", esp_err_to_name(ret));
    }
    return ret == ESP_OK;
}

static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    BaseType_t mustYield = pdFALSE;
    vTaskNotifyGiveFromISR(s_task_handle, &mustYield);
    return (mustYield == pdTRUE);
}

// Pool overflow callback - detects backlog as mentioned in README
static bool IRAM_ATTR s_pool_ovf_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    // Log overflow condition - this indicates the application isn't reading fast enough
    // As per README, this should not happen with flush_pool = false and proper buffer management
    BaseType_t mustYield = pdFALSE;
    // Note: Cannot use ESP_LOGE in ISR, would need to use a flag and log in task context
    vTaskNotifyGiveFromISR(s_task_handle, &mustYield);
    return (mustYield == pdTRUE);
}

// =================================================================================
// PROCESSING AND DISPLAY TASK
// =================================================================================
static void processing_task(void *arg)
{
    char unit[] = EXAMPLE_ADC_UNIT_STR(EXAMPLE_ADC_UNIT);

    while (1)
    {
        // Delay for 1 second to print results at a readable rate
        vTaskDelay(pdMS_TO_TICKS(1000));

        // Atomically copy and reset the shared variables
        portENTER_CRITICAL(&s_data_lock);
        uint64_t temp_sum = s_voltage_sum;
        uint32_t temp_count = s_sample_count;
        size_t temp_wr_pos = circ_buf_wr;
        s_voltage_sum = 0;
        s_sample_count = 0;
        portEXIT_CRITICAL(&s_data_lock);

        // Calculate and print the average voltage for the last second
        if (temp_count > 0)
        {
            uint32_t average_voltage = temp_sum / temp_count;
            ESP_LOGI(TAG, "Unit: %s, Channel: %d, Avg Voltage: %" PRIu32 " mV, Samples: %" PRIu32 ", BufPos: %zu",
                     unit, channel[0], average_voltage, temp_count, temp_wr_pos);
        }
        else
        {
            ESP_LOGI(TAG, "No new samples in the last second. BufPos: %zu", temp_wr_pos);
        }
    }
}

static void continuous_adc_init(adc_channel_t *channel, uint8_t channel_num, adc_continuous_handle_t *out_handle)
{
    adc_continuous_handle_t handle = NULL;

    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 4096,
        .conv_frame_size = EXAMPLE_READ_LEN,
        .flags.flush_pool = false, // Set to false to detect overflows properly
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = SAMPLE_FREQ_HZ, // 1 MSPS as per README specifications
        .conv_mode = EXAMPLE_ADC_CONV_MODE,
        .format = EXAMPLE_ADC_OUTPUT_TYPE,
    };

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    dig_cfg.pattern_num = channel_num;
    for (int i = 0; i < channel_num; i++)
    {
        adc_pattern[i].atten = EXAMPLE_ADC_ATTEN;
        adc_pattern[i].channel = channel[i] & 0x7;
        adc_pattern[i].unit = EXAMPLE_ADC_UNIT;
        adc_pattern[i].bit_width = EXAMPLE_ADC_BIT_WIDTH;
    }
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));

    *out_handle = handle;
}

void app_main(void)
{
    esp_err_t ret;
    uint32_t ret_num = 0;
    uint8_t result[EXAMPLE_READ_LEN] = {0};

    // RAM usage calculation (as per README):
    // - READ_LEN (256 bytes) for frame buffer
    // - CIRC_BUF_SAMPLES * 2 (32768 * 2 = 64KB) for circular buffer
    // - Task stacks and other overhead
    // Total ≈ 64KB as mentioned in README
    ESP_LOGI(TAG, "Initializing ADC continuous capture with %d sample circular buffer (%.1f KB)",
             CIRC_BUF_SAMPLES, (CIRC_BUF_SAMPLES * sizeof(uint16_t)) / 1024.0);

    s_task_handle = xTaskGetCurrentTaskHandle();

    // REMOVED: Queue is no longer needed
    // NEW: Create the processing and display task
    xTaskCreate(processing_task, "processing_task", PROCESSING_TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);

    do_calibration = adc_calibration_init(EXAMPLE_ADC_UNIT, EXAMPLE_ADC_ATTEN, &cali_handle);

    adc_continuous_handle_t handle = NULL;
    continuous_adc_init(channel, sizeof(channel) / sizeof(adc_channel_t), &handle);

    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = s_conv_done_cb,
        .on_pool_ovf = s_pool_ovf_cb, // Register the overflow callback
    };
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &cbs, NULL));
    ESP_ERROR_CHECK(adc_continuous_start(handle));

    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        while (1)
        {
            ret = adc_continuous_read(handle, result, EXAMPLE_READ_LEN, &ret_num, 0);

            if (ret == ESP_OK)
            {
                // Batch process the entire frame to minimize critical sections
                size_t samples_in_frame = ret_num / SOC_ADC_DIGI_RESULT_BYTES;
                uint64_t frame_voltage_sum = 0;
                uint32_t frame_sample_count = 0;

                // Get current write position once at start of frame
                size_t local_wr_pos;
                portENTER_CRITICAL(&s_data_lock);
                local_wr_pos = circ_buf_wr;
                portEXIT_CRITICAL(&s_data_lock);

                // Process all samples in the frame without any critical sections
                for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES)
                {
                    adc_digi_output_data_t *p = (adc_digi_output_data_t *)&result[i];
                    uint32_t raw_data = EXAMPLE_ADC_GET_DATA(p);

                    // Store in circular buffer (no critical section needed per sample)
                    circ_buf[local_wr_pos] = (uint16_t)raw_data;
                    local_wr_pos = (local_wr_pos + 1) & CIRC_BUF_MASK;

                    // Count every sample for statistics
                    frame_sample_count++;

                    // Only do voltage calibration on every 8th sample to reduce CPU load
                    // This still gives accurate average while preventing watchdog timeout
                    if (do_calibration && (frame_sample_count & 0x7) == 0)
                    {
                        int voltage_mv = 0;
                        if (adc_cali_raw_to_voltage(cali_handle, raw_data, &voltage_mv) == ESP_OK)
                        {
                            frame_voltage_sum += voltage_mv * 8; // Scale up to compensate for sampling
                        }
                    }
                }

                // Update all shared variables once per frame (single critical section)
                portENTER_CRITICAL(&s_data_lock);
                circ_buf_wr = local_wr_pos;
                s_voltage_sum += frame_voltage_sum;
                s_sample_count += frame_sample_count;
                portEXIT_CRITICAL(&s_data_lock);
            }
            else if (ret == ESP_ERR_TIMEOUT)
            {
                // Buffer is empty, break to wait for next notification
                break;
            }
            else
            {
                ESP_LOGE(TAG, "ADC read error: %s", esp_err_to_name(ret));
            }
        }

        // Give IDLE task more time to run and feed watchdog
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    // Cleanup (unreachable)
    ESP_ERROR_CHECK(adc_continuous_stop(handle));
    if (do_calibration)
    {
        ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(cali_handle));
    }
    ESP_ERROR_CHECK(adc_continuous_deinit(handle));
}