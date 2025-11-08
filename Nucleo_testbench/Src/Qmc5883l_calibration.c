#include "main.h"


float calibrate_heading(float current_heading);
void calibrate_compass(void);

UART_HandleTypeDef huart2;


float current_heading = 0.0f;
float heading_offset = 0.0f;
int offset_calibrated = 0;  // 0 = no, 1 = yes
float calibrated_heading = 0.0f;


void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

void calibrate_compass(void) {
    heading_offset = 0.0f;

    for(int i = 0; i < 10; i++) {
        heading_offset += current_heading;
        HAL_Delay(100);
    }

    heading_offset /= 10.0f;
    offset_calibrated = 1;
}
float calibrate_heading(float current_heading) {
    if (offset_calibrated) {
        calibrated_heading = current_heading - heading_offset;

        if (calibrated_heading < 0.0f) {
            calibrated_heading += 360.0f;
        } else if (calibrated_heading >= 360.0f) {
            calibrated_heading -= 360.0f;
        }
        return calibrated_heading;
    } else {
        return 0 ;
    }
}
