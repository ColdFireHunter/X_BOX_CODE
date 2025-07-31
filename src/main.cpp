#include <Arduino.h>
#include <pins.h>
#include <global.h>
#include <AsyncLED.h>
#include <gpio.h>

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
gpio GPIO;
AsyncLED asyncLED;

// LED DMA1 CH1
// ADC DMA1 CH7

static constexpr float GAMMA = 2.0f;
static uint32_t lastPotTimeout = 0;

static bool ledsActive = false;
static uint32_t cycleEndTime = 0;

static bool doorUp = false; // false = down, true = up
static bool lastWisch1, lastWisch2, retried;

static bool lastDip1State = false;

bool once = false;

// FSM states
enum class DoorState
{
  IDLE,
  BOOST_START,
  WAIT_PG,
  ENABLE_OUTPUT,
  SEND_WISCH,
  MOTOR_WAIT_RESPONSE,
  MOTOR_WAIT_END,
  FINISH_DELAY,
  DISABLE_ALL
};
enum class ErrorReason
{
  NO_PG_SIGNAL,
  MOTOR_NO_RESPONSE,
  MOTOR_NO_END
};

// bit-flags for each possible trigger source
enum InputSource : uint8_t
{
  SRC_NONE = 0,
  SRC_KEY1 = 1 << 0,
  SRC_RF1 = 1 << 1,
  SRC_KEY2 = 1 << 2,
  SRC_RF2 = 1 << 3,
  SRC_FRONTBTN = 1 << 4,
};

static constexpr uint32_t BOOST_STABILIZE_MS = 250;
static constexpr uint32_t TECH_POWERUP_MS = 2000;
static constexpr uint32_t MOTOR_TIMEOUT_MS = 2500;
static constexpr uint32_t END_TIMEOUT_MS = 60000;
static constexpr uint32_t FINISH_DELAY_MS = 5000;
static constexpr uint32_t OPTO_TIMEOUT_MS = 2000;

static Neotimer boostTimer(BOOST_STABILIZE_MS);
static Neotimer techTimer(TECH_POWERUP_MS);
static Neotimer motorTimer(MOTOR_TIMEOUT_MS);
static Neotimer endTimeoutTimer(END_TIMEOUT_MS);
static Neotimer finishTimer(FINISH_DELAY_MS);
static Neotimer optoLowTimer(OPTO_TIMEOUT_MS);
static Neotimer optoHighTimer(OPTO_TIMEOUT_MS);
static Neotimer optoHighReleaseTimer(OPTO_TIMEOUT_MS);

static DoorState state = DoorState::IDLE;

void errorHandler(ErrorReason reason)
{
  DEBUG_SERIAL.print("Error: ");
  switch (reason)
  {
  case ErrorReason::NO_PG_SIGNAL:
    DEBUG_SERIAL.println("Boost-Good (PG) signal never detected");
    break;
  case ErrorReason::MOTOR_NO_RESPONSE:
    DEBUG_SERIAL.println("Motor did not respond to Wisch pulse");
    break;
  case ErrorReason::MOTOR_NO_END:
    DEBUG_SERIAL.println("Motor did not finish moving in time");
    break;
  }
}

bool pollRequest(bool &want1, bool &want2, uint8_t &sourceFlags)
{
  want1 = want2 = false;
  sourceFlags = SRC_NONE;

  if (GPIO.getKeySwitch1())
  {
    want1 = true;
    sourceFlags |= SRC_KEY1;
  }
  if (GPIO.getRF_In1())
  {
    want1 = true;
    sourceFlags |= SRC_RF1;
  }
  if (GPIO.getKeySwitch2())
  {
    want2 = true;
    sourceFlags |= SRC_KEY2;
  }
  if (GPIO.getRF_In2())
  {
    want2 = true;
    sourceFlags |= SRC_RF2;
  }
  if (GPIO.getFrontPanelButton())
  {
    want1 = true; // front panel only ever fires Wisch1
    sourceFlags |= SRC_FRONTBTN;
  }

  return sourceFlags != SRC_NONE;
}

void updateDoorState()
{
  bool req1, req2;
  uint8_t src;
  bool hasReq = pollRequest(req1, req2, src);

  switch (state)
  {
  case DoorState::IDLE:
    if (hasReq)
    {
      DEBUG_SERIAL.print("Request from:");
      if (src & SRC_KEY1)
        DEBUG_SERIAL.print(" KEY1");
      if (src & SRC_RF1)
        DEBUG_SERIAL.print(" RF1");
      if (src & SRC_KEY2)
        DEBUG_SERIAL.print(" KEY2");
      if (src & SRC_RF2)
        DEBUG_SERIAL.print(" RF2");
      if (src & SRC_FRONTBTN)
        DEBUG_SERIAL.print(" FRONTBTN");
      DEBUG_SERIAL.println();

      // kick off fade-in to white over BOOST_STABILIZE_MS
      asyncLED.fillRGBW(0, 0, 0, 0);
      asyncLED.show();
      ledsActive = true;
      cycleEndTime = 0; // reset cycle end time

      // NEW: DIP1 selects full behavior, else everything → Wisch1 only
      if (GPIO.getDip1())
      {
        lastWisch1 = req1;
        lastWisch2 = req2;
      }
      else
      {
        lastWisch1 = true;  // any request fires Wisch1
        lastWisch2 = false; // never fire Wisch2
      }

      retried = false;
      GPIO.setBoostEnable(true);
      boostTimer.reset();
      boostTimer.start();
      state = DoorState::BOOST_START;

      once = true;
    }
    break;

  case DoorState::BOOST_START:
  {
    uint32_t elapsed = boostTimer.getEllapsed();
    uint8_t b;

    /*
    if (elapsed >= BOOST_STABILIZE_MS)
    {
      b = 255;
    }
    else
    {
      // normalized [0..1]
      float f = (float)elapsed / (float)BOOST_STABILIZE_MS;
      // apply gamma (f^GAMMA), you can tweak GAMMA up/down for different curves
      float g = powf(f, GAMMA);
      b = (uint8_t)(g * 255.0f);
    }
    */
    if (once == true)
    {
      b = 255;
      asyncLED.fillRGBW(b, b, b, b);
      asyncLED.show();
      once = false;
    }

    if (elapsed >= BOOST_STABILIZE_MS)
    {
      DEBUG_SERIAL.println("BOOST stabilized → WAIT_PG");
      state = DoorState::WAIT_PG;
    }
    break;
  }

  case DoorState::WAIT_PG:
    if (GPIO.getBoostStatus())
    {
      DEBUG_SERIAL.println("PG high → ENABLE_OUTPUT");
      GPIO.setOutputEnable(true);
      techTimer.reset();
      techTimer.start();
      state = DoorState::ENABLE_OUTPUT;
    }
    break;

  case DoorState::ENABLE_OUTPUT:
    if (techTimer.done())
    {
      DEBUG_SERIAL.println("Control electronics powered → SEND_WISCH");
      state = DoorState::SEND_WISCH;
    }
    break;

  case DoorState::SEND_WISCH:
    if (lastWisch1)
    {
      GPIO.triggerWisch1();
      DEBUG_SERIAL.println("Fired Wisch 1");
    }
    if (lastWisch2)
    {
      GPIO.triggerWisch2();
      DEBUG_SERIAL.println("Fired Wisch 2");
    }
    motorTimer.reset();
    motorTimer.start();
    state = DoorState::MOTOR_WAIT_RESPONSE;
    break;

  case DoorState::MOTOR_WAIT_RESPONSE:
    if (GPIO.getMotorOpto())
    {
      // pin HIGH — cancel any “release” countdown
      if (optoHighReleaseTimer.started())
      {
        optoHighReleaseTimer.stop();
        optoHighReleaseTimer.reset();
      }
      // start/continue the HIGH debounce
      if (!optoHighTimer.started())
      {
        optoHighTimer.reset();
        optoHighTimer.start();
      }
      // only when it’s been HIGH for 2 s do we really start
      if (optoHighTimer.done())
      {
        DEBUG_SERIAL.println("Motor started (stable HIGH 2 s) → movement detected");
        optoHighTimer.stop();
        optoHighTimer.reset();
        doorUp = !doorUp;
        DEBUG_SERIAL.print("Door now ");
        DEBUG_SERIAL.println(doorUp ? "UP" : "DOWN");
        endTimeoutTimer.reset();
        endTimeoutTimer.start();
        state = DoorState::MOTOR_WAIT_END;
      }
    }
    else
    {
      // pin LOW — start/continue the “release” timer but do not
      // cancel HIGH-debounce immediately
      if (optoHighTimer.started() && !optoHighTimer.done())
      {
        if (!optoHighReleaseTimer.started())
        {
          optoHighReleaseTimer.reset();
          optoHighReleaseTimer.start();
        }
        // only if LOW persists for 2 s do we abort the start-debounce
        if (optoHighReleaseTimer.done())
        {
          optoHighTimer.stop();
          optoHighTimer.reset();
          optoHighReleaseTimer.stop();
          optoHighReleaseTimer.reset();
        }
      }
      // still handle your response-timeout retry logic:
      if (motorTimer.done())
      {
        if (!retried)
        {
          DEBUG_SERIAL.println("No response → RETRY Wisch once");
          retried = true;
          if (lastWisch1)
          {
            GPIO.triggerWisch1();
            DEBUG_SERIAL.println("Retry Wisch 1");
          }
          if (lastWisch2)
          {
            GPIO.triggerWisch2();
            DEBUG_SERIAL.println("Retry Wisch 2");
          }
          motorTimer.reset();
          motorTimer.start();
        }
        else
        {
          DEBUG_SERIAL.println("Error: MOTOR_NO_RESPONSE");
          finishTimer.reset();
          finishTimer.start();
          state = DoorState::FINISH_DELAY;
        }
      }
    }
    break;

  case DoorState::MOTOR_WAIT_END:
    // allow new requests mid-move (same as before)…
    if (hasReq)
    {
      DEBUG_SERIAL.println("Queueing next stroke mid-move");
      DEBUG_SERIAL.print("Request from:");
      if (src & SRC_KEY1)
        DEBUG_SERIAL.print(" KEY1");
      if (src & SRC_RF1)
        DEBUG_SERIAL.print(" RF1");
      if (src & SRC_KEY2)
        DEBUG_SERIAL.print(" KEY2");
      if (src & SRC_RF2)
        DEBUG_SERIAL.print(" RF2");
      if (src & SRC_FRONTBTN)
        DEBUG_SERIAL.print(" FRONTBTN");
      DEBUG_SERIAL.println();

      if (GPIO.getDip1())
      {
        lastWisch1 = req1;
        lastWisch2 = req2;
      }
      else
      {
        lastWisch1 = true;
        lastWisch2 = false;
      }

      retried = false;
      state = DoorState::SEND_WISCH;
      cycleEndTime = 0; // reset cycle end time
      break;
    }
    // if pin is low, start/continue the low‐debounce timer
    if (!GPIO.getMotorOpto())
    {
      if (!optoLowTimer.started())
      {
        optoLowTimer.reset();
        optoLowTimer.start();
      }
      // once it’s been low for 2 s, we accept “motor stopped”
      if (optoLowTimer.done())
      {
        DEBUG_SERIAL.println("Motor stopped (stable LOW 2 s) → finish delay");
        finishTimer.reset();
        finishTimer.start();
        state = DoorState::FINISH_DELAY;
      }
    }
    else
    {
      // pin went high again → reset the low‐debounce
      if (optoLowTimer.started())
      {
        optoLowTimer.stop();
        optoLowTimer.reset();
      }
      // you may also choose to restart end‐timeout here if you want
    }

    // overall travel‐time timeout
    if (endTimeoutTimer.done())
    {
      errorHandler(ErrorReason::MOTOR_NO_END);
      finishTimer.reset();
      finishTimer.start();
      state = DoorState::FINISH_DELAY;
    }
    break;

  case DoorState::FINISH_DELAY:
    if (finishTimer.done())
    {
      finishTimer.stop();
      finishTimer.reset();
      DEBUG_SERIAL.println("Finish delay over → DISABLE ALL AFTER POTI TIMEOUT");
      if (ledsActive && cycleEndTime == 0)
      {
        cycleEndTime = millis();
      }
      cycleEndTime = cycleEndTime ?: millis();
    }

    if (hasReq)
    {
      DEBUG_SERIAL.println("Queueing next stroke mid-move");
      DEBUG_SERIAL.print("Request from:");
      if (src & SRC_KEY1)
        DEBUG_SERIAL.print(" KEY1");
      if (src & SRC_RF1)
        DEBUG_SERIAL.print(" RF1");
      if (src & SRC_KEY2)
        DEBUG_SERIAL.print(" KEY2");
      if (src & SRC_RF2)
        DEBUG_SERIAL.print(" RF2");
      if (src & SRC_FRONTBTN)
        DEBUG_SERIAL.print(" FRONTBTN");
      DEBUG_SERIAL.println();

      if (GPIO.getDip1())
      {
        lastWisch1 = req1;
        lastWisch2 = req2;
      }
      else
      {
        lastWisch1 = true;
        lastWisch2 = false;
      }

      retried = false;
      state = DoorState::SEND_WISCH;
      cycleEndTime = 0; // reset cycle end time
      break;
    }
    break;

  case DoorState::DISABLE_ALL:
    GPIO.setOutputEnable(false);
    GPIO.setBoostEnable(false);
    DEBUG_SERIAL.println("OUTPUT & BOOST disabled → back to IDLE");
    // make sure we capture end time if finishDelay was zero
    state = DoorState::IDLE;
    break;
  }
}

void handleLEDDisable()
{
  // --- new: detect & print pot-timeout changes ---
  uint32_t potTime = GPIO.getPotiTime();
  if (potTime != lastPotTimeout)
  {
    lastPotTimeout = potTime;
    DEBUG_SERIAL.print("Pot timeout set to ");
    DEBUG_SERIAL.print(potTime);
    DEBUG_SERIAL.println(" ms");
  }

  if (!ledsActive || cycleEndTime == 0)
    return;

  uint32_t elapsed = millis() - cycleEndTime;
  uint32_t disableMs = GPIO.getPotiTime(); // dynamically updated
  if (elapsed >= disableMs)
  {
    asyncLED.clearRGBW();
    asyncLED.show();
    ledsActive = false;
    DEBUG_SERIAL.println("LEDs disabled after pot timeout");
    state = DoorState::DISABLE_ALL;
  }
}

void handleDip1Change()
{
  bool curr = GPIO.getDip1();
  if (curr != lastDip1State)
  {
    lastDip1State = curr;
    DEBUG_SERIAL.print("DIP1 changed → ");
    if (curr)
    {
      DEBUG_SERIAL.println("Simultaneous mode ENABLED");
    }
    else
    {
      DEBUG_SERIAL.println("Independent mode ENABLED");
    }
  }
}

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
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV3;
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
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
   */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV16;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 6;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_79CYCLES_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_79CYCLES_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */
}
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  /* DMA1_Ch4_7_DMAMUX1_OVR_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Ch4_7_DMAMUX1_OVR_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Ch4_7_DMAMUX1_OVR_IRQn);
}

void setup()
{
  DEBUG_SERIAL.begin(115200);
  asyncLED.begin();
  asyncLED.setBrightness(160); // Set brightness to 60%
  asyncLED.clearRGBW();        // Clear the LED matrix
  asyncLED.show();

  GPIO.init();
  GPIO.setSensEnable(true);

  MX_DMA_Init();
  MX_ADC1_Init();

  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_values, 6);

  DEBUG_SERIAL.println("System initialized successfully.");
}

void loop()
{
  GPIO.handler();
  updateDoorState(); // drive the door-opener FSM
  handleLEDDisable();
  handleDip1Change();
}

extern "C" void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if (hadc->Instance == ADC1)
  {
    /* USER CODE BEGIN ADC1_MspInit 0 */

    /* USER CODE END ADC1_MspInit 0 */

    /** Initializes the peripherals clocks
     */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_SYSCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* Peripheral clock enable */
    __HAL_RCC_ADC_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**ADC1 GPIO Configuration
    PA0     ------> ADC1_IN0
    PA1     ------> ADC1_IN1
    PA2     ------> ADC1_IN2
    PA3     ------> ADC1_IN3
    PA4     ------> ADC1_IN4
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* ADC1 DMA Init */
    /* ADC1 Init */
    hdma_adc1.Instance = DMA1_Channel7;
    hdma_adc1.Init.Request = DMA_REQUEST_ADC1;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(hadc, DMA_Handle, hdma_adc1);

    /* USER CODE BEGIN ADC1_MspInit 1 */

    /* USER CODE END ADC1_MspInit 1 */
  }
}
extern "C" void HAL_ADC_MspDeInit(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    /* USER CODE BEGIN ADC1_MspDeInit 0 */

    /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC_CLK_DISABLE();

    /**ADC1 GPIO Configuration
    PA0     ------> ADC1_IN0
    PA1     ------> ADC1_IN1
    PA2     ------> ADC1_IN2
    PA3     ------> ADC1_IN3
    PA4     ------> ADC1_IN4
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4);

    /* ADC1 DMA DeInit */
    HAL_DMA_DeInit(hadc->DMA_Handle);
    /* USER CODE BEGIN ADC1_MspDeInit 1 */

    /* USER CODE END ADC1_MspDeInit 1 */
  }
}
extern "C" void DMA1_Ch4_7_DMAMUX1_OVR_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Ch4_7_DMAMUX1_OVR_IRQn 0 */

  /* USER CODE END DMA1_Ch4_7_DMAMUX1_OVR_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA1_Ch4_7_DMAMUX1_OVR_IRQn 1 */

  /* USER CODE END DMA1_Ch4_7_DMAMUX1_OVR_IRQn 1 */
}