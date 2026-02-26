/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - Motor Controller with Auto Mode
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx.h"

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// Timer handle typedef
typedef struct {
    TIM_TypeDef *Instance;
} TIM_HandleTypeDef;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DS18B20_CMD_CONVERTTEMP     0x44
#define DS18B20_CMD_RSCRATCHPAD     0xBE
#define DS18B20_CMD_SKIPROM         0xCC
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
TIM_HandleTypeDef htim3;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
// DS18B20 Functions
extern void DS18B20_Init(TIM_HandleTypeDef *timer);
extern float DS18B20_ReadTemp(void);
extern void DS18B20_Write(uint8_t data);
extern uint8_t DS18B20_Read(uint8_t data);
extern uint8_t DS18B20_Start(void);

// Assembly function declarations
extern void Motor_ASM_Init(void);
extern void Motor_ASM_Loop(void);
extern void Motor_ASM_Test_LEDs(void);
extern void Motor_ASM_Set_PWM_C(uint32_t pwm);
extern uint32_t Motor_ASM_Get_Auto_Mode(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void delay(volatile uint32_t count)
{
    while(count--);
}

/* Temperature-based PWM calculation with LED feedback */
uint32_t Calculate_PWM_From_Temp(float temperature)
{
    // LOWERED Temperature thresholds for easier testing
    // Below 15°C: 60% speed (5040) - 1 blink
    // 15-22°C: 85% speed (7140) - 2 blinks
    // Above 22°C: 100% speed (8399) - 3 blinks
    
    // Visual feedback on LEDs based on temperature
    
    if(temperature < 15.0f)
    {
        // Blink PA5 (60% LED) once to show temp < 15°C
        GPIOA->BSRR = (1<<5);  // Turn on PA5
        for(volatile int i=0; i<100000; i++);
        GPIOA->BSRR = (1<<21); // Turn off PA5
        return 5040;  // 60%
    }
    else if(temperature < 22.0f)
    {
        // Blink PA6 (85% LED) twice to show 15-22°C
        for(int j=0; j<2; j++)
        {
            GPIOA->BSRR = (1<<6);
            for(volatile int i=0; i<100000; i++);
            GPIOA->BSRR = (1<<22);
            for(volatile int i=0; i<100000; i++);
        }
        return 7140;  // 85%
    }
    else
    {
        // Blink PA7 (100% LED) three times to show > 22°C
        for(int j=0; j<3; j++)
        {
            GPIOA->BSRR = (1<<7);
            for(volatile int i=0; i<100000; i++);
            GPIOA->BSRR = (1<<23);
            for(volatile int i=0; i<100000; i++);
        }
        return 8399;  // 100%
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    float temperature = 0.0f;
    uint32_t pwm_value = 0;
    uint32_t temp_read_counter = 0;
    
    /* HAL Init */
    HAL_Init();
    
    /* Configure the system clock */
    SystemClock_Config();
    
    /* Initialize TIM3 for DS18B20 microsecond delays */
    MX_TIM3_Init();
    
    /* Initialize DS18B20 sensor */
    DS18B20_Init(&htim3);
    
    /* Initialize motor controller (Assembly) */
    Motor_ASM_Init();
    
    /* Test LEDs on startup */
    Motor_ASM_Test_LEDs();
    
    /* Main loop */
    while(1)
    {
        /* Always check for button presses */
        Motor_ASM_Loop();
        
        /* Check if in auto mode */
        if(Motor_ASM_Get_Auto_Mode() == 1)
        {
            /* Read temperature every ~2 seconds (40 * 50ms) */
            temp_read_counter++;
            if(temp_read_counter >= 40)
            {
                temp_read_counter = 0;
                
                /* Read temperature from DS18B20 */
                temperature = DS18B20_ReadTemp();
                
                /* DIAGNOSTIC: Show actual temperature reading using LEDs */
                /* This will help us see what the sensor is actually reading */
                /* Turn on LEDs based on temperature value: */
                /* LED1 (PA5) = temp > 10°C */
                /* LED2 (PA6) = temp > 20°C */
                /* LED3 (PA7) = temp > 30°C */
                
                GPIOA->BSRR = (1<<21) | (1<<22) | (1<<23); // Turn all LEDs off first
                
                if(temperature > 10.0f)
                    GPIOA->BSRR = (1<<5);  // LED1 on
                if(temperature > 20.0f)
                    GPIOA->BSRR = (1<<6);  // LED2 on
                if(temperature > 30.0f)
                    GPIOA->BSRR = (1<<7);  // LED3 on
                
                /* Wait a moment to see the LEDs */
                for(volatile int i=0; i<2000000; i++);
                
                /* Check if sensor is working (returns -999.0 on error) */
                if(temperature < -500.0f)
                {
                    // Sensor error - blink ALL LEDs rapidly
                    for(int i=0; i<5; i++)
                    {
                        GPIOA->BSRR = (1<<5) | (1<<6) | (1<<7); // All on
                        for(volatile int j=0; j<50000; j++);
                        GPIOA->BSRR = (1<<21) | (1<<22) | (1<<23); // All off
                        for(volatile int j=0; j<50000; j++);
                    }
                    pwm_value = 5040; // Default to 60% on error
                }
                else
                {
                    /* Calculate PWM based on temperature */
                    pwm_value = Calculate_PWM_From_Temp(temperature);
                }
                
                /* Set motor speed */
                Motor_ASM_Set_PWM_C(pwm_value);
            }
        }
        
        delay(50000);  // Small delay for debouncing
    }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    /** Initializes the RCC Oscillators */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 16;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief TIM3 Initialization Function (for DS18B20 microsecond delays)
  * @retval None
  */
static void MX_TIM3_Init(void)
{
    // Enable TIM3 clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    
    // Configure TIM3
    htim3.Instance = TIM3;
    
    // Prescaler = 83 (84MHz / 84 = 1MHz = 1us tick)
    TIM3->PSC = 83;
    
    // Auto-reload value (max count)
    TIM3->ARR = 0xFFFF;
    
    // Enable timer
    TIM3->CR1 |= TIM_CR1_CEN;
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}

#endif
