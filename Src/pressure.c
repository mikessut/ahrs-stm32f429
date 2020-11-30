
#include "pressure.h"
#include <math.h>
#include "can_fix.h"
#include "main.h"
#include "utils.h"

extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi3;

IIRFilterDef altitude_filter;
IIRFilterDef vs_filter = {
  {0.00156833, 0.00156833}, // b
  0, 0, // x, y
  -0.99686333 //a1
};

IIRFilterDef ias_filter = {
  {0.04503501, 0.04503501}, // b
  0.0, 0.0, // x, y
  -0.90992999 //a1
};

float dpress_offset = 0.0;

void airspeed_altitude(float abs_press, float diff_press, float alt_setting, float oat,
                       float *altitude, float *ias, float *tas, float *vs)
{
  float p_sealevel = alt_setting*CONV_INHG2PA;
  float h = -(pow(abs_press, CONST_L*CONST_R/CONST_g/CONST_M)
              - pow(p_sealevel, CONST_L*CONST_R/CONST_g/CONST_M))*CONST_T0/CONST_L/pow(CONST_P0, CONST_L*CONST_R/CONST_g/CONST_M);
  float p = pow(pow(p_sealevel, CONST_L*CONST_R/CONST_g/CONST_M)
                - CONST_L/CONST_T0*pow(CONST_P0, CONST_L*CONST_R/CONST_g/CONST_M)*h, CONST_g*CONST_M/CONST_L/CONST_R);

  float rho = p*CONST_M/CONST_R/(273.15 + oat);
  // Filter altitude using IIR
  // from scipy.signal import iirfilter
  // iirfilter(1, .5, btype='lowpass', ftype='butter', output='ba')
  //float new_alt = h * CONV_M2FT;
  //*altitude = 0.01546629 * alt_in_prev + 0.01546629*new_alt - (-0.96906742)*(*altitude);
  //alt_in_prev = new_alt;

  // For VS, use altitude filter's previous input differenced to current alitude
  float alt_ft = h * CONV_M2FT;
  *vs = iir_filter(&vs_filter, (alt_ft - altitude_filter.x) / 0.042 * 60.0);
  *altitude = iir_filter(&altitude_filter, alt_ft);  
  *ias = iir_filter(&ias_filter, sqrt(abs(diff_press)*2/CONST_RHO0)*CONV_MS2KNOTS);
  *tas = (*ias) * sqrt(CONST_RHO0/rho);
}

int abs_pressure(float *pres, float *temp) {
  /* Static Pressure Port Sensor */
    uint8_t abs_press_data[4];
    uint8_t error = HAL_OK;
    error =  HAL_I2C_Master_Receive(&hi2c1, 0x71, abs_press_data, 4, 0xFFFF);
    if (HAL_OK != error) {
      //sprintf((char*)buffer, "Pressure read error: %d\r\n", error);
      //HAL_UART_Transmit(&huart2, buffer, strlen((char*)buffer), 0xFFFF);
	    //printf("error: %u\n\r", error);
      return -1;
    }

    //uint8_t status = (abs_press_data[0] & 0xC0) >> 6;
    uint16_t bridge_data = ((abs_press_data[0] & 0x3F) << 8) + abs_press_data[1];

    *temp = (float)((abs_press_data[2] << 3) + ((abs_press_data[3] & 0xE0) >> 5))/2047.0*200.0 - 50.0;
    //uint16_t temperature_data = (abs_press_data[2] << 3) + ((abs_press_data[3] & 0xE0) >> 5);
    //uint16_t temperature = (temperature_data * 200)/2047 - 50;
    //uint16_t pressure = ((bridge_data - 1638) * 15 * 1000) / (14745-1638);
    *pres = ((float)(bridge_data - 1638)*15.0) / ((float)(14745-1638)) * CONV_PSI2PASCAL;
}

int diff_pressure(float *diff_press, float *diff_press_temp) {

    uint8_t diff_press_data[4];
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
    HAL_SPI_Receive(&hspi3, diff_press_data, sizeof(diff_press_data), 100);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);

    //temperature_data = (diff_press_data[2] << 3) + ((diff_press_data[3] & 0xE0) >> 5);
    //temperature = (temperature_data * 200)/2047 - 50;
    // +/- 100mbar
    //
    uint16_t bridge_data = ((diff_press_data[0] & 0x3F) << 8) + diff_press_data[1];
    *diff_press = (((float)(bridge_data - 1638)*200.0) / ((float)(14745-1638)) - 100.0) * CONV_MBAR2PA;
    *diff_press += dpress_offset;
    *diff_press_temp = (float)((diff_press_data[2] << 3) + ((diff_press_data[3] & 0xE0) >> 5))/2047.0*200.0 - 50.0;
}
