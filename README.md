# RotEncoder

A driver for quadrature rotary encoders on ESP-IDF. 

## Features
* 2-bit quadrature, 2- or 4-detent encoders
* Built-in debounce (1 ms)
* Speed-aware acceleration (×2 / ×4)
* Adaptive RPM smoothing for snappy read-outs
* Pure C, no heap, ISR-safe

## Quick Start
```c
#include "encoder.h"

static encoder_t enc;

void app_main(void)
{
    encoder_init(&enc, GPIO_NUM_1, GPIO_NUM_2, true, ENCODER_LATCH_FOUR3);
    while (1) {
        encoder_tick(&enc);
        int32_t pos = encoder_get_position(&enc);
        encoder_direction_t dir = encoder_get_direction(&enc);
        uint32_t rpm = encoder_get_rpm(&enc);
        // ... use the values ...
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
```

## CMakeLists Example
Your project CMakeLists just needs:
```cmake
set(EXTRA_COMPONENT_DIRS "${PROJECT_DIR}/components")
```
The component itself already declares its dependencies (`driver`, `esp_timer`).

## Configuration
Adjust the macros at the top of `encoder.c` if you need different debounce or acceleration thresholds.
