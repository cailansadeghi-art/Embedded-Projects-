/* DS18B20 Temperature Sensor Driver Implementation */

#include "ds18b20.h"
#include "stm32f4xx.h"

static TIM_HandleTypeDef *htim_delay;

void DS18B20_Init(TIM_HandleTypeDef *timer)
{
    htim_delay = timer;
    
    // Enable GPIOA clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    
    // Start timer
    htim_delay->Instance->CR1 |= TIM_CR1_CEN;
}

void delay_us(uint16_t us)
{
    htim_delay->Instance->CNT = 0;
    while(htim_delay->Instance->CNT < us);
}

void Set_Pin_Output(void)
{
    // Configure PA8 as output
    GPIOA->MODER &= ~(3 << 16);      // Clear mode bits
    GPIOA->MODER |= (1 << 16);       // Set as output (01)
    GPIOA->OTYPER &= ~(1 << 8);      // Push-pull
    GPIOA->OSPEEDR |= (3 << 16);     // High speed
}

void Set_Pin_Input(void)
{
    // Configure PA8 as input
    GPIOA->MODER &= ~(3 << 16);      // Clear mode bits (00 = input)
    GPIOA->PUPDR &= ~(3 << 16);      // No pull-up/pull-down
}

uint8_t DS18B20_Start(void)
{
    uint8_t response = 0;
    
    Set_Pin_Output();
    GPIOA->BSRR = (1 << 24);         // Reset pin (bit 8 + 16)
    delay_us(500);                    // Increased from 480 for waterproof probe
    
    Set_Pin_Input();
    delay_us(70);                     // Slightly adjusted timing
    
    if(!(GPIOA->IDR & (1 << 8))) 
        response = 1;
    else 
        response = 0;
    
    delay_us(430);                    // Adjusted to complete reset cycle
    
    return response;
}

void DS18B20_Write(uint8_t data)
{
    Set_Pin_Output();
    
    for(int i = 0; i < 8; i++)
    {
        if((data & (1 << i)) != 0)
        {
            // Write 1 - adjusted timing for waterproof probe
            Set_Pin_Output();
            GPIOA->BSRR = (1 << 24);         // Reset PA8
            delay_us(10);                     // Increased from 1
            
            Set_Pin_Input();
            delay_us(55);                     // Adjusted
        }
        else
        {
            // Write 0 - adjusted timing for waterproof probe
            Set_Pin_Output();
            GPIOA->BSRR = (1 << 24);         // Reset PA8
            delay_us(65);                     // Increased from 60
            
            Set_Pin_Input();
            delay_us(5);                      // Recovery time
        }
    }
}

uint8_t DS18B20_Read(void)
{
    uint8_t value = 0;
    
    Set_Pin_Input();
    
    for(int i = 0; i < 8; i++)
    {
        Set_Pin_Output();
        GPIOA->BSRR = (1 << 24);         // Reset PA8
        delay_us(3);                      // Increased from 2
        
        Set_Pin_Input();
        delay_us(10);                     // Wait before sampling
        
        if(GPIOA->IDR & (1 << 8))
        {
            value |= (1 << i);
        }
        
        delay_us(55);                     // Complete the time slot
    }
    
    return value;
}

float DS18B20_ReadTemp(void)
{
    uint8_t temp_lsb, temp_msb;
    uint16_t temp;
    float temperature = 0.0;
    
    // Start conversion
    if(!DS18B20_Start())
    {
        return -999.0;  // Error: sensor not responding
    }
    
    DS18B20_Write(DS18B20_CMD_SKIPROM);
    DS18B20_Write(DS18B20_CMD_CONVERTTEMP);
    
    // Wait for conversion - INCREASED for waterproof probe
    // Waterproof probes need more time due to thermal mass
    for(volatile int i = 0; i < 10000000; i++);  // Increased from 8M
    
    // Read temperature
    if(!DS18B20_Start())
    {
        return -999.0;  // Error: sensor not responding
    }
    
    DS18B20_Write(DS18B20_CMD_SKIPROM);
    DS18B20_Write(DS18B20_CMD_RSCRATCHPAD);
    
    temp_lsb = DS18B20_Read();
    temp_msb = DS18B20_Read();
    
    temp = (temp_msb << 8) | temp_lsb;
    
    // Sanity check - if reading 0x0000 or 0xFFFF, something is wrong
    if(temp == 0x0000 || temp == 0xFFFF)
    {
        return -999.0;  // Invalid reading
    }
    
    // Convert to Celsius
    if(temp & 0x8000)  // Negative temperature
    {
        temp = ~temp + 1;
        temperature = -((float)temp * 0.0625);
    }
    else
    {
        temperature = (float)temp * 0.0625;
    }
    
    return temperature;
}
