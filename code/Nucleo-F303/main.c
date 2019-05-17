#include "main.h"

void SysTick_Handler(void)
{
	msTicks++;
}

void setSysTick()
{
	// ---------- SysTick timer (1ms) -------- //
	if (SysTick_Config(SystemCoreClock / 1000))
	{
		while (1)
		{
		};
	}
}

static void Delay(__IO uint32_t dlyTicks)
{
	uint32_t curTicks = msTicks;
	while ((msTicks - curTicks) < dlyTicks);
}

void init_peripherals(void)
{
	// Initialize structures for GPIO, USART and NVIC for Interrupts
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_LEDInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	// Start the buses onto which the peripherals lie
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	// On the Nucleo-F303, 
	// USART 1 is on Pin A10 (Tx) and Pin A9 (Rx)
	// USART 2 is on Pin A
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// Configure USART1 Alternate Function pins
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_7);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_7);

	// Configure USART2 Alternate Function pins
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_7);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_7);

	// LED blink config that lies on B3
	GPIO_LEDInitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_LEDInitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_LEDInitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_LEDInitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_LEDInitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_LEDInitStructure);

	// USART 1 and USART 2 config
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

	//Activate both USART 1 and 2 with the same configuration settings
	USART_Init(USART1, &USART_InitStructure);

	// configure USART2 for only sending purposes
	USART_InitStructure.USART_Mode = USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

	// USART1 lies least in the NVIC so we except it to be serviced first
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// USART1 lies least in the NVIC so we except it to be serviced first
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// Initialize the peripherals
	USART_Cmd(USART1, ENABLE);
	USART_Cmd(USART2, ENABLE);
}

void USART_puts(USART_TypeDef* usar_tx, volatile char* str)
{
	while (*str)
	{
		while (USART_GetFlagStatus(usar_tx, USART_FLAG_TC) == RESET){}
		USART_SendData(usar_tx, *str);
		str++;
	}
}

void USART1_IRQHandler(void)
{
	if (USART1->ISR & USART_ISR_RXNE)
	{
		const char ch = USART_ReceiveData(USART1);

		if (ch != '\n' && ch != '\r') rcvd_str[cnt++] = ch;
		else
		{
			rcvd_str[cnt] = '\0';
			cnt = 0;
			str_built = true;
		}

		USART_ClearITPendingBit(USART1, USART_ISR_RXNE);
	}
}

void HardFault_Handler()
{
	NVIC_SystemReset();
}

void configureGPSDevice(void)
{
	for (int k = 0;k < 5; k++)
	{
		USART_puts(USART1, "LOG GPRMC ONTIME 1 \r\n");
		USART_puts(USART1, "SAVECONFIG \r\n");

		// wait for 50ms 
		Delay(300);
	}
}

int main()
{
	// Configure the system clock
	setSysTick();

	// Initialize all peripherals
	init_peripherals();

	// Delay for 2 seconds
	Delay(2000);

	// Reset the GPS device
	USART_puts(USART1, "RESET 1 \r\n");

	// Dely for 500ms to still wait for the GPS to initialize
	Delay(500);

	// Send initial config commands to the GPS
	configureGPSDevice();

	// Go into a forever loop
	while (1)
	{
		// Note the size of the character string array
		size_t len = sizeof rcvd_str;

		// If the string has been built and the string character has some characters
		if(str_built && len > 0)
		{
			// Take a pointer to where that text lies
			char* str = rcvd_str;

			// If the text is SystemReset, then reset the Nucleo-F303
			if (strcmp(str, "SystemReset") == 0) {
				NVIC_SystemReset();
			}

			// If the string begins with a P, then its parameters. Just send them forward to Jetson Xavier
			if (rcvd_str[0] == 'P')
			{
				GPIO_WriteBit(GPIOB, GPIO_Pin_3, Bit_SET);

				// Note that the leading character will be P to designate that they are Parameters
				USART_puts(USART2, rcvd_str);
				USART_puts(USART2, "\n");

				GPIO_WriteBit(GPIOB, GPIO_Pin_3, Bit_RESET);
			}
			else
			{
				// Otherwise, prepare to slice the string
				// Initialize a matrix to hold the sliced components
				char matrix[20][15] = { 0 };

				// Initialize the number of rows and columns
				size_t r = 0, c = 0;

				// While we are not reading the null character
				while (*str != '\0')
				{
					const char tmp = *str;

					// if the temp is a comma, just terminate the string that is at that column
					// move on to the next row and reset the column count
					if (tmp == ',')
					{
						matrix[r][c] = '\0';
						r++;
						c = 0;
					}
					else
					{
						// add the character to the matrix
						// increment the column count
						matrix[r][c] = tmp;
						c++;
					}

					// Increment to the next character of the string literal
					str++;
				}

				// If the slicing was done successfully the rows should be greater than 10
				// the first string should be equivalent to $GPRMC
				// and the string at [2] should be A for valid. Then we have a valid result
				if (r >= 10 && strcmp(matrix[0], "$GPRMC") == 0 && strcmp(matrix[2], "A") == 0)
				{
					GPIO_WriteBit(GPIOB, GPIO_Pin_3, Bit_SET);

					// leading character should be a C to designate Coordinates
					USART_puts(USART2, "C,");

					// UTC
					USART_puts(USART2, matrix[1]);
					USART_puts(USART2, ",");
				
					// Latitude
					USART_puts(USART2, matrix[3]);
					USART_puts(USART2, ",");
				
					// Longitude
					char* longitude = &matrix[5][0];
					longitude++;
					USART_puts(USART2, longitude);

					USART_puts(USART2, "\n");

					GPIO_WriteBit(GPIOB, GPIO_Pin_3, Bit_RESET);
				}
			}

			// Reset the string processor initializer
			str_built = false;
		}
	}
}
