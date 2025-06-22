## Continuous ADC Capture with Application‑Level Circular Buffer (Option A)

### Overview

This project shows how to maintain a **true circular history** of high‑rate SAR‑ADC data on an ESP32‑S2 Feather V2 while staying completely inside Espressif’s official ADC‑continuous driver. We let the driver do what it does best—DMA blocks into its internal 4 KB “pool”—and immediately copy each finished frame into a larger power‑of‑two RAM ring that overwrites the oldest samples. That design costs a single `memcpy()` per frame but avoids patching the driver, giving you oscilloscope‑style pre‑trigger capture at up to ≈1 MSPS with plenty of CPU head‑room. ([docs.espressif.com](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/adc_continuous.html?utm_source=chatgpt.com), [reddit.com](https://www.reddit.com/r/esp32/comments/1jhmf9h/help_me_understand_i2s_dma/?utm_source=chatgpt.com))

---

### System Architecture

#### 1  ADC Continuous Driver

- Allocated with `max_store_buf_size = 4096`, `flush_pool = false` so overflow is detectable (driver raises `on_pool_ovf`). ([docs.espressif.com](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/adc_continuous.html?utm_source=chatgpt.com), [docs.espressif.com](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s2/api-reference/peripherals/adc_continuous.html?utm_source=chatgpt.com))
- DMA writes through a linked list of 4 KB `lldesc_t` descriptors. ([reddit.com](https://www.reddit.com/r/esp32/comments/1jhmf9h/help_me_understand_i2s_dma/?utm_source=chatgpt.com))

#### 2  Application‑Level Ring Buffer

- `circ_buf[]` – DMA‑capable internal RAM sized to 2ⁿ samples (e.g. 32768 × 2 bytes ≈ 64 kB).
- Critical section (`portENTER_CRITICAL(&s_data_lock)`) protects the write pointer because ISR and task share one core. ([docs.espressif.com](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/freertos_idf.html?utm_source=chatgpt.com))
- Copy cost < 1 µs per 256‑byte frame on a 240 MHz S2, leaving > 90 % CPU idle at 1 MSPS. ([github.com](https://github.com/espressif/esp-idf/issues/164?utm_source=chatgpt.com))

```
ADC -> DMA  -->  IDF Pool  ---->  circ_buf[]  ----->  Trigger analysis
          (flush reset)     memcpy (task)
```

---

### Code Walk‑Through (main/continuous_ring.c)

1. **Driver Init** – see `continuous_adc_init()`.
2. **DMA task** (in `app_main`) copies each frame into `circ_buf` and accumulates voltage stats.
3. **UI task** (`processing_task`) prints once per second and can later export pre/post‑trigger data.
4. `flush_pool` **cleared** so you detect backlog via `ESP_ERR_INVALID_STATE` instead of silent data loss. ([docs.espressif.com](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/adc_continuous.html?utm_source=chatgpt.com))

---

### Build & Flash

```bash
idf.py set-target esp32s2      # Feather V2
idf.py menuconfig              # Optional: tweak stack & speed
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

---

### Runtime Configuration

| Macro              | Description                      | Typical        |
| ------------------ | -------------------------------- | -------------- |
| `SAMPLE_FREQ_HZ`   | ADC clock (≤ 2.2 MSPS on S2)     | 1 000 000      |
| `EXAMPLE_READ_LEN` | Driver frame size                | 256–2048 bytes |
| `CIRC_BUF_SAMPLES` | Size of user ring (power‑of‑two) | 32 768         |

**RAM usage** ≈ `READ_LEN` + `CIRC_BUF_SAMPLES*2` + stack → 64 kB by default.

---

### Trigger‑Based Capture Pattern

1. External GPIO ISR sets `trigger_seen = 1`.
2. After post‑trigger delay, call `adc_continuous_stop()`.
3. Linearise ring for upload:

```c
size_t start = (wr - PRE_SAMPLES) & BUF_MASK;
for (size_t i=0;i<TOTAL;i++)
    out[i] = circ_buf[(start+i) & BUF_MASK];
```

---

### Calibration

Driver enables **line‑fitting calibration** to trim reference‑voltage error (< 3 mV typical). ([docs.espressif.com](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/adc_calibration.html?utm_source=chatgpt.com)) Disable if only relative accuracy matters.

---

### Performance & Limits

- Copy overhead ≈ 12 % CPU at 1 MSPS, 256‑byte frames.
- UART printing is the bottleneck—keep logs sparse.
- Max proven throughput (no Wi‑Fi) ≈ 8 MSPS before ring overruns. ([github.com](https://github.com/espressif/esp-idf/issues/164?utm_source=chatgpt.com))

---

### Future Work

- Swap `memcpy` for **GDMA memcpy** once exposed on S2.
- Move sample export to USB CDC for higher bandwidth.
- Fork driver (Option B) to remove the copy entirely.

---

### References

1. IDF ADC continuous driver – pool & overflow. ([docs.espressif.com](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/adc_continuous.html?utm_source=chatgpt.com))
2. ESP32‑S2 driver variant. ([docs.espressif.com](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s2/api-reference/peripherals/adc_continuous.html?utm_source=chatgpt.com))
3. Reddit `lldesc_t` DMA explanation. ([reddit.com](https://www.reddit.com/r/esp32/comments/1jhmf9h/help_me_understand_i2s_dma/?utm_source=chatgpt.com))
4. FreeRTOS spinlock note. ([docs.espressif.com](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/freertos_idf.html?utm_source=chatgpt.com))
5. Calibration driver doc. ([docs.espressif.com](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/adc_calibration.html?utm_source=chatgpt.com))
6. Continuous‑read sample. ([docs.espressif.com](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/adc_continuous.html?utm_source=chatgpt.com))
7. DroneBot I2S primer (for Option C background). ([dronebotworkshop.com](https://dronebotworkshop.com/esp32-i2s/?utm_source=chatgpt.com))
8. Forum circular‑SPI DMA example. ([esp32.com](https://esp32.com/viewtopic.php?t=41598&utm_source=chatgpt.com))
9. ADC accuracy issue motivating calibration. ([github.com](https://github.com/espressif/esp-idf/issues/164?utm_source=chatgpt.com))
10. Reddit note on 13‑bit width (ESP32‑S2). ([reddit.com](https://www.reddit.com/r/arduino/comments/1732jkf/analogeread_far_above_4095_esp32s2_mini/?utm_source=chatgpt.com))
