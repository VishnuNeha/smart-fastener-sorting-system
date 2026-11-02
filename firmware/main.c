#include "stm32f4xx.h"
#include <stdint.h>

/* =============== WATCH WINDOW VARIABLES =============== */

volatile uint32_t objectCount = 0U;
volatile uint32_t metalCount = 0U;
volatile uint32_t distance_cm = 0U;
volatile uint32_t echo_us = 0U;
volatile uint32_t objectHeight_cm = 0U;

volatile uint32_t redCount = 0U;
volatile uint32_t greenCount = 0U;
volatile uint32_t blueCount = 0U;

volatile uint32_t ldrADCValue = 0U;
volatile uint32_t ldrMillivolts = 0U;

volatile uint8_t timeoutFlag = 0U;
volatile uint8_t irDetected = 0U;
volatile uint8_t metalDetected = 0U;
volatile uint8_t inRange = 0U;
volatile uint8_t buzzerState = 0U;
volatile uint8_t objectIsMetal = 0U;
volatile uint8_t accepted = 0U;

volatile uint8_t sortingIRFlag = 0U;

/* =============== SETTINGS =============== */

#define REST_ANGLE          90U
#define ACCEPTED_ANGLE      0U
#define REJECTED_ANGLE      145U
#define PUSH_TIME_MS        1000U

#define METAL_CHECK_TIME_MS 1000U
#define BELT_DISTANCE_CM    20U
#define MIN_HEIGHT_CM       2U

#define LDR_MIN_ADC         2000U
#define LCD_ADDR            0x27U
#define METAL_ACTIVE_LEVEL  0U

static uint8_t metalPreviousState = 1U;
static uint8_t irPreviousState = 1U;

/* =============== FUNCTION DECLARATIONS =============== */

void GPIO_Init(void);
void TIM2_Init(void);
void USART2_Init(void);
void Servo_Init(void);
void Servo_SetAngle(uint16_t angle);

void EXTI4_Init(void);
void ADC1_Init(void);
uint32_t ADC1_Read(void);

void I2C1_Init(void);
void I2C1_Write(uint8_t data);

void LCD_Send(uint8_t value, uint8_t rs);
void LCD_Command(uint8_t command);
void LCD_Data(uint8_t data);
void LCD_Init(void);
void LCD_Clear(void);
void LCD_SetCursor(uint8_t row, uint8_t column);
void LCD_Print(const char *str);
void LCD_PrintNumber(uint32_t number);
void LCD_ShowStatus(const char *status);

void delay_us(uint32_t us);
void delay_ms(uint32_t ms);

void Buzzer_Beep(void);
void Update_Metal_State(void);
uint8_t Check_Metal_Window(void);

uint32_t Read_Distance(void);
uint32_t Calculate_Height(void);

void Select_Filter(uint8_t s2, uint8_t s3);
uint32_t Read_Color(void);

void Bluetooth_SendChar(char ch);
void Bluetooth_SendString(const char *str);
void Bluetooth_SendNumber(uint32_t number);

/* =============== GPIO INITIALIZATION =============== */

void GPIO_Init(void)
{
    RCC->AHB1ENR |= (1U << 0) | (1U << 1);
    (void)RCC->AHB1ENR;

    /* PB3 = red LED, PB4 = green LED */
    GPIOB->MODER &= ~((3U << 6) | (3U << 8));
    GPIOB->MODER |=  (1U << 6) | (1U << 8);

    /* LEDs OFF: active HIGH */
    GPIOB->BSRR = (1U << 19) | (1U << 20);

    /* PA0 = first IR, PA1 = metal sensor */
    GPIOA->MODER &= ~((3U << 0) | (3U << 2));
    GPIOA->PUPDR &= ~((3U << 0) | (3U << 2));
    GPIOA->PUPDR |=  (1U << 0) | (1U << 2);

    /* PA4 = second IR, active LOW */
    GPIOA->MODER &= ~(3U << 8);
    GPIOA->PUPDR &= ~(3U << 8);
    GPIOA->PUPDR |=  (1U << 8);

    /* PA8 = ultrasonic TRIG */
    GPIOA->MODER &= ~(3U << 16);
    GPIOA->MODER |=  (1U << 16);
    GPIOA->BSRR = (1U << 24);

    /* PB1 = ultrasonic ECHO */
    GPIOB->MODER &= ~(3U << 2);
    GPIOB->PUPDR &= ~(3U << 2);

    /* TCS3200: S0=PB0, S1=PB2, S2=PB10, S3=PB12 */
    GPIOB->MODER &= ~((3U << 0) |
                      (3U << 4) |
                      (3U << 20) |
                      (3U << 24));

    GPIOB->MODER |= (1U << 0) |
                    (1U << 4) |
                    (1U << 20) |
                    (1U << 24);

    /* PA6 = TCS3200 OUT */
    GPIOA->MODER &= ~(3U << 12);
    GPIOA->PUPDR &= ~(3U << 12);

    /* TCS3200 frequency scaling: S0=1, S1=0 */
    GPIOB->BSRR = (1U << 0) | (1U << 18);

    /* Initial red filter: S2=0, S3=0 */
    GPIOB->BSRR = (1U << 26) | (1U << 28);

    /* PB5 = active-HIGH buzzer */
    GPIOB->MODER &= ~(3U << 10);
    GPIOB->MODER |=  (1U << 10);
    GPIOB->BSRR = (1U << 21);
}

/* =============== EXTI4: PA4 SORTING IR =============== */

void EXTI4_Init(void)
{
    RCC->APB2ENR |= (1U << 14);

    /* EXTI line 4 source = PA4 */
    SYSCFG->EXTICR[1] &= ~(15U << 0);

    /* Falling-edge interrupt */
    EXTI->RTSR &= ~(1U << 4);
    EXTI->FTSR |=  (1U << 4);

    EXTI->PR = (1U << 4);
    EXTI->IMR |= (1U << 4);

    /* EXTI4 interrupt number = 10 */
    NVIC->ISER[0] = (1U << 10);
}

void EXTI4_IRQHandler(void)
{
    if ((EXTI->PR & (1U << 4)) != 0U)
    {
        EXTI->PR = (1U << 4);
        sortingIRFlag = 1U;
    }
}

/* =============== ADC1: LDR ON PA5 =============== */

void ADC1_Init(void)
{
    RCC->APB2ENR |= (1U << 8);

    /* PA5 analog mode */
    GPIOA->MODER &= ~(3U << 10);
    GPIOA->MODER |=  (3U << 10);
    GPIOA->PUPDR &= ~(3U << 10);

    /* ADC prescaler = PCLK2 / 2 */
    ADC->CCR &= ~(3U << 16);

    ADC1->CR1 = 0U;
    ADC1->CR2 = 0U;
    ADC1->SQR1 = 0U;

    /* ADC channel 5 = PA5 */
    ADC1->SQR3 = 5U;

    /* Channel 5 sample time = 480 cycles */
    ADC1->SMPR2 &= ~(7U << 15);
    ADC1->SMPR2 |=  (7U << 15);

    /* Enable ADC1 */
    ADC1->CR2 |= 1U;
}

uint32_t ADC1_Read(void)
{
    ADC1->CR2 |= (1U << 30);

    while ((ADC1->SR & (1U << 1)) == 0U)
    {
    }

    ldrADCValue = ADC1->DR;
    ldrMillivolts = (ldrADCValue * 3300U) / 4095U;

    return ldrADCValue;
}

/* =============== TIM2 MICROSECOND TIMER =============== */

void TIM2_Init(void)
{
    RCC->APB1ENR |= (1U << 0);

    TIM2->CR1 = 0U;
    TIM2->PSC = 15U;
    TIM2->ARR = 0xFFFFFFFFU;
    TIM2->CNT = 0U;
    TIM2->EGR = 1U;
    TIM2->CR1 = 1U;
}

void delay_us(uint32_t us)
{
    uint32_t start = TIM2->CNT;

    while ((uint32_t)(TIM2->CNT - start) < us)
    {
    }
}

void delay_ms(uint32_t ms)
{
    while (ms > 0U)
    {
        delay_us(1000U);
        ms--;
    }
}

/* =============== USART2 BLUETOOTH =============== */

void USART2_Init(void)
{
    RCC->APB1ENR |= (1U << 17);

    /* PA2 = USART2 TX, PA3 = USART2 RX */
    GPIOA->MODER &= ~((3U << 4) | (3U << 6));
    GPIOA->MODER |=  (2U << 4) | (2U << 6);

    GPIOA->AFR[0] &= ~((15U << 8) | (15U << 12));
    GPIOA->AFR[0] |=  (7U << 8) | (7U << 12);

    USART2->CR1 = 0U;
    USART2->BRR = 0x0683U;
    USART2->CR1 = (1U << 3) | (1U << 13);
}

void Bluetooth_SendChar(char ch)
{
    while ((USART2->SR & (1U << 7)) == 0U)
    {
    }

    USART2->DR = (uint8_t)ch;
}

void Bluetooth_SendString(const char *str)
{
    while (*str != '\0')
    {
        Bluetooth_SendChar(*str++);
    }
}

void Bluetooth_SendNumber(uint32_t number)
{
    char digits[10];
    uint32_t i = 0U;

    if (number == 0U)
    {
        Bluetooth_SendChar('0');
        return;
    }

    while ((number > 0U) && (i < 10U))
    {
        digits[i++] = (char)('0' + (number % 10U));
        number /= 10U;
    }

    while (i > 0U)
    {
        Bluetooth_SendChar(digits[--i]);
    }
}

/* =============== I2C1 LCD: PB8 SCL, PB9 SDA =============== */

void I2C1_Init(void)
{
    RCC->APB1ENR |= (1U << 21);

    /* PB8 and PB9 alternate-function mode */
    GPIOB->MODER &= ~((3U << 16) | (3U << 18));
    GPIOB->MODER |=  (2U << 16) | (2U << 18);

    /* Open-drain */
    GPIOB->OTYPER |= (1U << 8) | (1U << 9);

    GPIOB->OSPEEDR |= (3U << 16) | (3U << 18);

    GPIOB->PUPDR &= ~((3U << 16) | (3U << 18));
    GPIOB->PUPDR |=  (1U << 16) | (1U << 18);

    /* AF4 = I2C1 */
    GPIOB->AFR[1] &= ~((15U << 0) | (15U << 4));
    GPIOB->AFR[1] |=  (4U << 0) | (4U << 4);

    /* Reset I2C peripheral */
    I2C1->CR1 = (1U << 15);
    I2C1->CR1 = 0U;

    /* Assumes PCLK1 = 16 MHz */
    I2C1->CR2 = 16U;
    I2C1->CCR = 80U;
    I2C1->TRISE = 17U;

    I2C1->CR1 = 1U;
}

void I2C1_Write(uint8_t data)
{
    volatile uint32_t temp;
    uint32_t guard;

    guard = 1000000U;

    while (((I2C1->SR2 & (1U << 1)) != 0U) && guard)
    {
        guard--;
    }

    if (guard == 0U)
        return;

    /* START condition */
    I2C1->CR1 |= (1U << 8);

    guard = 1000000U;

    while (((I2C1->SR1 & (1U << 0)) == 0U) && guard)
    {
        guard--;
    }

    if (guard == 0U)
        return;

    /* Send LCD I2C address */
    I2C1->DR = (LCD_ADDR << 1);

    guard = 1000000U;

    while (((I2C1->SR1 & (1U << 1)) == 0U) && guard)
    {
        guard--;
    }

    if (guard == 0U)
    {
        I2C1->CR1 |= (1U << 9);
        return;
    }

    temp = I2C1->SR2;
    (void)temp;

    guard = 1000000U;

    while (((I2C1->SR1 & (1U << 7)) == 0U) && guard)
    {
        guard--;
    }

    if (guard == 0U)
    {
        I2C1->CR1 |= (1U << 9);
        return;
    }

    I2C1->DR = data;

    guard = 1000000U;

    while (((I2C1->SR1 & (1U << 2)) == 0U) && guard)
    {
        guard--;
    }

    /* STOP condition */
    I2C1->CR1 |= (1U << 9);
}

/* =============== LCD LOW-LEVEL FUNCTIONS =============== */

void LCD_Send(uint8_t value, uint8_t rs)
{
    uint8_t high = value & 0xF0U;
    uint8_t low = (value << 4) & 0xF0U;

    /* RS = 1 for data, 0 for command */
    if (rs != 0U)
    {
        high |= 1U;
        low |= 1U;
    }

    /* High nibble */
    I2C1_Write(high | 0x08U);
    I2C1_Write(high | 0x0CU);
    I2C1_Write(high | 0x08U);

    /* Low nibble */
    I2C1_Write(low | 0x08U);
    I2C1_Write(low | 0x0CU);
    I2C1_Write(low | 0x08U);

    delay_ms(2U);
}

void LCD_Command(uint8_t command)
{
    LCD_Send(command, 0U);
}

void LCD_Data(uint8_t data)
{
    LCD_Send(data, 1U);
}

void LCD_Init(void)
{
    delay_ms(50U);

    /* Initialize LCD in 4-bit mode */
    I2C1_Write(0x38U);
    I2C1_Write(0x3CU);
    I2C1_Write(0x38U);
    delay_ms(5U);

    I2C1_Write(0x38U);
    I2C1_Write(0x3CU);
    I2C1_Write(0x38U);
    delay_ms(2U);

    I2C1_Write(0x38U);
    I2C1_Write(0x3CU);
    I2C1_Write(0x38U);
    delay_ms(2U);

    I2C1_Write(0x28U);
    I2C1_Write(0x2CU);
    I2C1_Write(0x28U);

    LCD_Command(0x28U);  /* 4-bit, 2-line mode */
    LCD_Command(0x0CU);  /* Display ON, cursor OFF */
    LCD_Command(0x06U);  /* Cursor moves right */
    LCD_Command(0x01U);  /* Clear display */

    delay_ms(5U);
}

void LCD_Clear(void)
{
    LCD_Command(0x01U);
    delay_ms(3U);
}

void LCD_SetCursor(uint8_t row, uint8_t column)
{
    if (row == 0U)
    {
        LCD_Command(0x80U + column);
    }
    else
    {
        LCD_Command(0xC0U + column);
    }
}

void LCD_Print(const char *str)
{
    while (*str != '\0')
    {
        LCD_Data((uint8_t)*str++);
    }
}

void LCD_PrintNumber(uint32_t number)
{
    char digits[10];
    uint32_t i = 0U;

    if (number == 0U)
    {
        LCD_Data('0');
        return;
    }

    while ((number > 0U) && (i < 10U))
    {
        digits[i++] = (char)('0' + (number % 10U));
        number /= 10U;
    }

    while (i > 0U)
    {
        LCD_Data((uint8_t)digits[--i]);
    }
}

/* Display the current object and inspection stage */
void LCD_ShowStatus(const char *status)
{
    LCD_Clear();

    LCD_SetCursor(0U, 0U);
    LCD_Print("Obj: ");
    LCD_PrintNumber(objectCount);

    LCD_SetCursor(1U, 0U);
    LCD_Print(status);
}

/* =============== BUZZER =============== */

void Buzzer_Beep(void)
{
    GPIOB->BSRR = (1U << 5);
    buzzerState = 1U;

    delay_ms(80U);

    GPIOB->BSRR = (1U << 21);
    buzzerState = 0U;
}

/* =============== METAL SENSOR: PA1 =============== */

void Update_Metal_State(void)
{
    uint8_t level;

    level = (GPIOA->IDR & (1U << 1)) ? 1U : 0U;

    metalDetected =
        (level == METAL_ACTIVE_LEVEL) ? 1U : 0U;

    if ((level == METAL_ACTIVE_LEVEL) &&
        (metalPreviousState != METAL_ACTIVE_LEVEL))
    {
        metalCount++;
    }

    metalPreviousState = level;
}

uint8_t Check_Metal_Window(void)
{
    uint32_t elapsed;
    uint8_t found = 0U;

    for (elapsed = 0U;
         elapsed < METAL_CHECK_TIME_MS;
         elapsed++)
    {
        Update_Metal_State();

        if (metalDetected == 1U)
        {
            found = 1U;
        }

        delay_ms(1U);
    }

    return found;
}

/* =============== ULTRASONIC SENSOR =============== */

uint32_t Read_Distance(void)
{
    uint32_t start;
    uint32_t duration;

    distance_cm = 0U;
    echo_us = 0U;
    timeoutFlag = 0U;

    /* TRIG LOW briefly */
    GPIOA->BSRR = (1U << 24);
    delay_us(2U);

    /* TRIG HIGH for 10 us */
    GPIOA->BSRR = (1U << 8);
    delay_us(10U);
    GPIOA->BSRR = (1U << 24);

    /* Wait for ECHO HIGH */
    start = TIM2->CNT;

    while ((GPIOB->IDR & (1U << 1)) == 0U)
    {
        if ((uint32_t)(TIM2->CNT - start) >= 30000U)
        {
            timeoutFlag = 1U;
            return 0U;
        }
    }

    /* Measure ECHO HIGH pulse */
    start = TIM2->CNT;

    while ((GPIOB->IDR & (1U << 1)) != 0U)
    {
        duration = (uint32_t)(TIM2->CNT - start);

        if (duration >= 30000U)
        {
            timeoutFlag = 1U;
            return 0U;
        }
    }

    echo_us = (uint32_t)(TIM2->CNT - start);
    distance_cm = echo_us / 58U;

    return distance_cm;
}

uint32_t Calculate_Height(void)
{
    if (timeoutFlag != 0U)
    {
        return 0U;
    }

    if (distance_cm < BELT_DISTANCE_CM)
    {
        return BELT_DISTANCE_CM - distance_cm;
    }

    return 0U;
}

/* =============== TCS3200 COLOUR SENSOR =============== */

void Select_Filter(uint8_t s2, uint8_t s3)
{
    if (s2 != 0U)
        GPIOB->BSRR = (1U << 10);
    else
        GPIOB->BSRR = (1U << 26);

    if (s3 != 0U)
        GPIOB->BSRR = (1U << 12);
    else
        GPIOB->BSRR = (1U << 28);

    delay_us(2000U);
}

uint32_t Read_Color(void)
{
    uint32_t start;
    uint32_t count = 0U;
    uint8_t previous;
    uint8_t current;

    previous = (GPIOA->IDR & (1U << 6)) ? 1U : 0U;
    start = TIM2->CNT;

    while ((uint32_t)(TIM2->CNT - start) < 100000U)
    {
        current = (GPIOA->IDR & (1U << 6)) ? 1U : 0U;

        if ((previous == 0U) && (current == 1U))
        {
            count++;
        }

        previous = current;
    }

    return count;
}

/* =============== SERVO: PA10 / TIM1 CH3 =============== */

void Servo_Init(void)
{
    RCC->AHB1ENR |= (1U << 0);
    RCC->APB2ENR |= (1U << 0);

    (void)RCC->AHB1ENR;
    (void)RCC->APB2ENR;

    GPIOA->MODER &= ~(3U << 20);
    GPIOA->MODER |=  (2U << 20);

    GPIOA->OTYPER &= ~(1U << 10);
    GPIOA->OSPEEDR |= (3U << 20);

    GPIOA->AFR[1] &= ~(15U << 8);
    GPIOA->AFR[1] |=  (1U << 8);

    TIM1->CR1 = 0U;
    TIM1->CCER = 0U;

    /* Assumes TIM1 clock is 16 MHz */
    TIM1->PSC = 15U;
    TIM1->ARR = 19999U;

    /* PWM mode 1 on channel 3 */
    TIM1->CCMR2 &= ~((3U << 0) | (7U << 4));
    TIM1->CCMR2 |= (6U << 4);
    TIM1->CCMR2 |= (1U << 3);

    TIM1->CCR3 = 1500U;
    TIM1->CCER |= (1U << 8);
    TIM1->CR1 |= (1U << 7);

    TIM1->EGR = 1U;
    TIM1->BDTR |= (1U << 15);
    TIM1->CR1 |= (1U << 0);
}

void Servo_SetAngle(uint16_t angle)
{
    if (angle > 180U)
    {
        angle = 180U;
    }

    TIM1->CCR3 =
        1000U + ((uint32_t)angle * 1000U) / 180U;
}

/* =============== MAIN =============== */

int main(void)
{
    uint8_t irNow;
    uint8_t heightValid;

    GPIO_Init();
    TIM2_Init();
    USART2_Init();
    Servo_Init();
    EXTI4_Init();
    ADC1_Init();

    /* LCD initialization added */
    I2C1_Init();
    LCD_Init();

    Servo_SetAngle(REST_ANGLE);

    LCD_Clear();
    LCD_SetCursor(0U, 0U);
    LCD_Print("SWIS SYSTEM");
    LCD_SetCursor(1U, 0U);
    LCD_Print("SYSTEM READY");
    delay_ms(1000U);

    irPreviousState =
        (GPIOA->IDR & (1U << 0)) ? 1U : 0U;

    metalPreviousState =
        (GPIOA->IDR & (1U << 1)) ? 1U : 0U;

    Bluetooth_SendString("\r\nSWIS READY\r\n");
    Bluetooth_SendString("PA4 EXTI4 enabled\r\n");
    Bluetooth_SendString("PA5 LDR ADC enabled\r\n");

    while (1)
    {
        irNow = (GPIOA->IDR & (1U << 0)) ? 1U : 0U;
        irDetected = (irNow == 0U) ? 1U : 0U;

        Update_Metal_State();

        /* First IR detects a new object: active LOW */
        if ((irNow == 0U) && (irPreviousState == 1U))
        {
            objectCount++;

            objectIsMetal = 0U;
            objectHeight_cm = 0U;
            timeoutFlag = 0U;
            accepted = 0U;
            heightValid = 0U;

            /* Arm PA4 sorting interrupt for this object */
            sortingIRFlag = 0U;
            EXTI->PR = (1U << 4);
            EXTI->IMR |= (1U << 4);

            /* Status LEDs OFF */
            GPIOB->BSRR = (1U << 19) | (1U << 20);

            /* Servo stays at REST during inspection */
            Servo_SetAngle(REST_ANGLE);
            Buzzer_Beep();

            Bluetooth_SendString("\r\nOBJECT #");
            Bluetooth_SendNumber(objectCount);
            Bluetooth_SendString("\r\n");

            /* STEP 1: metal check */
            LCD_ShowStatus("Checking metal");

            Bluetooth_SendString("Checking metal...\r\n");
            objectIsMetal = Check_Metal_Window();

            Bluetooth_SendString("Metal: ");

            if (objectIsMetal != 0U)
            {
                Bluetooth_SendString("DETECTED\r\n");
            }
            else
            {
                Bluetooth_SendString("NOT DETECTED\r\n");
            }

            /* STEP 2: height check */
            LCD_ShowStatus("Checking height");

            Bluetooth_SendString("Checking height...\r\n");

            Read_Distance();

            if (timeoutFlag == 0U)
            {
                heightValid = 1U;
                objectHeight_cm = Calculate_Height();

                Bluetooth_SendString("Height: ");
                Bluetooth_SendNumber(objectHeight_cm);
                Bluetooth_SendString(" cm\r\n");
            }
            else
            {
                heightValid = 0U;
                objectHeight_cm = 0U;

                Bluetooth_SendString("Height: INVALID\r\n");
            }

            /* STEP 3: decision */
            if ((objectIsMetal == 1U) &&
                (heightValid == 1U) &&
                (objectHeight_cm >= MIN_HEIGHT_CM))
            {
                accepted = 1U;
                Bluetooth_SendString("Decision: ACCEPTED\r\n");
            }
            else
            {
                accepted = 0U;
                Bluetooth_SendString("Decision: REJECTED\r\n");
            }

            /* STEP 4: wait for PA4 interrupt */
            LCD_ShowStatus("Waiting for PA4");

            Bluetooth_SendString(
                "Waiting for PA4 sorting IR...\r\n"
            );

            while (sortingIRFlag == 0U)
            {
                /* EXTI4 ISR sets sortingIRFlag */
            }

            /* Disable EXTI4 during the current sorting action */
            EXTI->IMR &= ~(1U << 4);

            Bluetooth_SendString("PA4 detected object.\r\n");

            /* STEP 5: sort and display the final result */
            LCD_Clear();

            LCD_SetCursor(0U, 0U);
            LCD_Print("Obj: ");
            LCD_PrintNumber(objectCount);

            LCD_SetCursor(1U, 0U);

            if (accepted == 1U)
            {
                /* Green ON, red OFF */
                GPIOB->BSRR = (1U << 19) | (1U << 4);

                LCD_Print("ACCEPTED");

                Bluetooth_SendString("RESULT: ACCEPTED\r\n");
                Bluetooth_SendString(
                    "Servo pushing accepted object.\r\n"
                );

                Servo_SetAngle(ACCEPTED_ANGLE);
                delay_ms(PUSH_TIME_MS);
                Servo_SetAngle(REST_ANGLE);

                Bluetooth_SendString("Servo returned to REST.\r\n");
            }
            else
            {
                /* Red ON, green OFF */
                GPIOB->BSRR = (1U << 20) | (1U << 3);

                LCD_Print("REJECTED");

                Bluetooth_SendString("RESULT: REJECTED\r\n");
                Bluetooth_SendString(
                    "Servo remains at REST.\r\n"
                );

                Servo_SetAngle(REST_ANGLE);
            }

            delay_ms(500U);

            /* Re-arm PA4 for the next object */
            sortingIRFlag = 0U;
            EXTI->PR = (1U << 4);
            EXTI->IMR |= (1U << 4);
        }

        irPreviousState = irNow;

        /* Live metal monitoring */
        Update_Metal_State();

        /* Live distance and height monitoring */
        Read_Distance();

        if (timeoutFlag == 0U)
        {
            objectHeight_cm = Calculate_Height();
        }
        else
        {
            objectHeight_cm = 0U;
        }

        inRange = ((timeoutFlag == 0U) &&
                   (distance_cm >= 10U) &&
                   (distance_cm <= 15U)) ? 1U : 0U;

        /* LDR reading on PA5 */
        ldrADCValue = ADC1_Read();
        ldrMillivolts = (ldrADCValue * 3300U) / 4095U;

        /* Colour readings only when LDR brightness is sufficient */
        if (ldrADCValue >= LDR_MIN_ADC)
        {
            Select_Filter(0U, 0U);
            redCount = Read_Color();

            Select_Filter(1U, 1U);
            greenCount = Read_Color();

            Select_Filter(0U, 1U);
            blueCount = Read_Color();
        }
        else
        {
            redCount = 0U;
            greenCount = 0U;
            blueCount = 0U;
        }

        /* Bluetooth live status */
        Bluetooth_SendString("\r\n--- LIVE STATUS ---\r\n");

        Bluetooth_SendString("Objects: ");
        Bluetooth_SendNumber(objectCount);
        Bluetooth_SendString("\r\n");

        Bluetooth_SendString("Metal: ");

        if (metalDetected != 0U)
        {
            Bluetooth_SendString("DETECTED\r\n");
        }
        else
        {
            Bluetooth_SendString("NOT DETECTED\r\n");
        }

        Bluetooth_SendString("Metal count: ");
        Bluetooth_SendNumber(metalCount);
        Bluetooth_SendString("\r\n");

        Bluetooth_SendString("Height: ");

        if (timeoutFlag != 0U)
        {
            Bluetooth_SendString("INVALID\r\n");
        }
        else
        {
            Bluetooth_SendNumber(objectHeight_cm);
            Bluetooth_SendString(" cm\r\n");
        }

        Bluetooth_SendString("LDR ADC: ");
        Bluetooth_SendNumber(ldrADCValue);

        Bluetooth_SendString(" | LDR mV: ");
        Bluetooth_SendNumber(ldrMillivolts);
        Bluetooth_SendString("\r\n");

        if (ldrADCValue >= LDR_MIN_ADC)
        {
            Bluetooth_SendString("Colour R=");
            Bluetooth_SendNumber(redCount);

            Bluetooth_SendString(" G=");
            Bluetooth_SendNumber(greenCount);

            Bluetooth_SendString(" B=");
            Bluetooth_SendNumber(blueCount);
            Bluetooth_SendString("\r\n");
        }
        else
        {
            Bluetooth_SendString(
                "Colour: NOT READ - INSUFFICIENT LIGHT\r\n"
            );
        }

        Bluetooth_SendString("--------------------\r\n");

        delay_ms(2000U);
    }
}
