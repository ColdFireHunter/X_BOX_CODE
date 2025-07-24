#include <Arduino.h>
#include <pins.h>
#include <global.h>
#include <AsyncLED.h>

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO_PF2, RCC_MCO1SOURCE_SYSCLK, RCC_MCODIV_1);
}

AsyncLED asyncLED;

void setup()
{
  DEBUG_SERIAL.begin(115200);
  asyncLED.begin();
  asyncLED.setBrightness(160); // Set brightness to 60%
  asyncLED.clearRGBW();        // Clear the LED matrix
  asyncLED.show();

  pinMode(EN_BOOST, OUTPUT);
  pinMode(EN_OUTPUT, OUTPUT);
  pinMode(PG_BOOST, INPUT);

  pinMode(RF_IN1, INPUT_PULLUP);
  pinMode(RF_IN2, INPUT_PULLUP);

  pinMode(KEY_SWITCH_1, INPUT);
  pinMode(KEY_SWITCH_2, INPUT);

  pinMode(FRONT_PANEL_BUTTON, INPUT_PULLUP);

  digitalWrite(EN_BOOST, LOW);
  digitalWrite(EN_OUTPUT, LOW);
}

void loop()
{
  static uint32_t lastButtonPress = 0;
  if (digitalRead(FRONT_PANEL_BUTTON) == LOW)
  {
    uint32_t currentTime = millis();
    if (currentTime - lastButtonPress > 200) // 200 ms debounce time
    {
      lastButtonPress = currentTime;
      // Togle led matrix
      static bool ledOn = false;
      ledOn = !ledOn;
      if (ledOn)
      {
        asyncLED.showChar8x8('X', 0, 255, 0, 0); // Show 'A' in red
      }
      else
      {
        asyncLED.clearRGBW(); // Clear the LED matrix
      }
      asyncLED.show(); // Update the LED matrix
    }
  }
}
