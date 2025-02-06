/* USER CODE BEGIN Header */
/*
 +--------+---------+   +------+---------+   +--------+------+
 | RFID   | STM     |   | LCD  | STM     |   | PIEZO  | STM  |
 +--------+---------+   +------+---------+   +--------+------+
 | VCC    | 3.3V    |   | VCC  | 5.0V    |   | SIG    | A01  |
 | RST    | PB0(A03)|   | GND  | GND     |   | GND    | GND  |
 | GND    | GND     |   | SCL  | SCL/D15 |   +--------+------+
 | IRQ    | ---     |   | SDA  | SDA/D14 |
 | MISO   | PA6(D12)|   +------+---------+
 | MOSI   | PA7(D11)|	+-------+------+
 | SCK    | PA5(D13)|	| RELAY | STM  |
 | SDA    | PA4(A02)|	+-------+------+
 +--------+---------+	| SIG   | A08  |
 	 	 	 	 	 	| GND   | GND  |
						+-------+------+
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/

#include <stdio.h>
#include <main.h>
#include <string.h>
#include <stdlib.h>
#include <poruke.h>
/* ODOBRENE KARTICE define BEGIN----------------------------------------------*/
#define ALLOWED_UID_0 75
#define ALLOWED_UID_1 161
#define ALLOWED_UID_2 169
#define ALLOWED_UID_3 195
#define ALLOWED_UID_4 128
/* ODOBRENE KARTICE define END-------------------------------------------------*/

/* LCD includes BEGIN------------------------------------------------------*/

// Deklaracije LCD funkcija
void Delay_us(uint32_t us);				//prototip funkcije mikrosekunda
void DWT_Init(void);			//prototip funkcije inicijalizacije mikrosekundi
void I2C_LCD_Init(void); // Inicijalizacija LCD ekrana
void I2C_LCD_Clear(void); // Briše sadržaj LCD ekrana
void I2C_LCD_SetCursor(uint8_t Col, uint8_t Row); // Postavlja kursor na određenu poziciju
void I2C_LCD_WriteString(char *Str); // Ispisuje string na LCD
typedef struct {
	// I2C LCD Module Instance Index
	uint8_t I2C_LCD_Instance;

	// I2C Hardware Peripheral Handle
	I2C_HandleTypeDef *I2C_Handle;

	// I2C LCD Hardware Device Address
	uint8_t I2C_LCD_Address;

	// I2C LCD Columns Count
	uint8_t I2C_LCD_nCol;

	// I2C LCD Rows Count
	uint8_t I2C_LCD_nRow;

} I2C_LCD_CfgType;

/* LCD includes END------------------------------------------------------------------*/

/* RC522 includes BEGIN------------------------------------------------------------------*/
// Maksimalna dužina buffera za prijenos podataka s RFID modula
#define MAX_LEN 16
// Definicija SPI instance koja se koristi za komunikaciju s RC522
#define HSPI_INSTANCE &hspi1
// Definicije GPIO pinova za RC522 čip selekt (CS) i reset (RST)
#define MFRC522_CS_PORT GPIOA     // Port za čip selekt
#define MFRC522_CS_PIN GPIO_PIN_4 // Pin za čip selekt (SDA)
#define MFRC522_RST_PORT GPIOB    // Port za reset
#define MFRC522_RST_PIN GPIO_PIN_0 // Pin za reset

// Funkcije za rad s RC522 RFID modulom
void MFRC522_Init(void);		//Inicijalizira RC522 RFID modul

uint8_t MFRC522_Request(uint8_t reqMode, uint8_t *TagType);

uint8_t MFRC522_Anticoll(uint8_t *serNum);

/* RC522 includes END------------------------------------------------------------------*/
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Definicije GPIO pinova za upravljanje relejom
#define RELAY_PIN       	GPIO_PIN_8  // Pin koji kontrolira relej (uključivanje/isključivanje)
#define RELAY_PORT      	GPIOA       // GPIO port gdje je povezan relej
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;  // Struktura za rad s I2C perifernim modulom
SPI_HandleTypeDef hspi1;  // Struktura za rad s SPI komunikacijom
TIM_HandleTypeDef htim2; // Struktura za rad s tajmerom 2 (PWM za piezo zvučnik)

/* USER CODE BEGIN PV */
uint8_t status;  // Status RFID očitanja, pokazuje prisutnost kartice
uint8_t str[MAX_LEN]; // Buffer za pohranu očitanih podataka s RFID kartice (maksimalna dužina 16 bajtova)
uint8_t sNum[5];  // Polje za spremanje UID broja RFID kartice (5 bajtova)

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void); // Konfiguracija sistemskog takta mikrokontrolera
void MX_GPIO_Init(void);  // Inicijalizacija GPIO pinova
void MX_SPI1_Init(void); // Inicijalizacija SPI1 periferije za komunikaciju s RFID modulom
static void MX_I2C1_Init(void);  // Inicijalizacija I2C1 periferije za LCD ekran
static void MX_TIM2_Init(void); // Inicijalizacija TIM2 timera za generiranje PWM signala
void Reset_TIM2(void); // Reset timera TIM2 za potrebe ispravnog funkcioniranja PWM-a
/* USER CODE BEGIN PFP */

//prototipi funkcija
void RFID_Process(void); //Proces identifikacije
void RC522_Init(void);  // Inicijalizacija RFID modula RC522
void I2C_DENIED(void); // Prikazuje poruku 'DENIED' na LCD-u i generira zvučni signal za odbijen pristup
void I2C_GRANTED(void); // Prikazuje poruku 'GRANTED' na LCD-u, aktivira relej i generira zvučni signal
void I2C_WELCOME(void); // Prikazuje poruku dobrodošlice i upute za skeniranje kartice na LCD-u
void I2C_CARD(uint8_t *data);  // Prikazuje UID kartice na LCD ekranu u dva reda
#define TIM_CLOCK 16000000  // 16 MHz
void GenerateTone(int tone, int duration_ms); // Generira zvučni signal pomoću PWM-a na piezo zvučniku

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();  // Inicijalizacija HAL biblioteke

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();  // Konfiguracija sistemskog takta

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */

	MX_GPIO_Init();  // Inicijalizacija GPIO pinova
	MX_SPI1_Init();  // Inicijalizacija SPI-a
	MX_I2C1_Init();  // Inicijalizacija I2C-a
	MX_TIM2_Init();  // Inicijalizacija timera 2
	/* USER CODE BEGIN 2 */
	DWT_Init();		// Inicijalizacija microSeconds funkcije (izbjegavam util.h)
	MFRC522_Init();  // Inicijalizacija RFID modula
	I2C_LCD_Init();  // Inicijalizacija LCD ekrana
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // Pokretanje PWM-a za piezo zvučnik
	I2C_WELCOME();  // Prikaz poruke dobrodošlice

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		RFID_Process();
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
void MX_SPI1_Init(void) {

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;					//naziv SPI instance
	hspi1.Init.Mode = SPI_MODE_MASTER;				//STM radi kao master
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;	//dvosmjerna komunikacija
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;		//veličina podatka je 8 bita
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;		//SCK je u 0 kad miruje
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;			//podaci se uzorkuju na prvom bridu
	hspi1.Init.NSS = SPI_NSS_SOFT;					//software kontrolira chip select
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2; //Brzina komunikacij
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;			//prvi bit je MSB
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 7999;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4 | GPIO_PIN_8, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

	/*Configure GPIO pin : PA4 */
	GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PB0 */
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : PA8 */
	GPIO_InitStruct.Pin = GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void RFID_Process(void) {
	// Attempt to detect a card
	status = MFRC522_Request(0x26, str); // Provjera prisutnosti kartice
	if (status == 0) {
		status = MFRC522_Anticoll(str);  // Antikolidacija - dobivanje UID-a
		if (status == 0) {
			memcpy(sNum, str, 5); // Kopiranje UID-a u sNum
			HAL_Delay(100);
			if ((str[0] == ALLOWED_UID_0) && (str[1] == ALLOWED_UID_1)
					&& (str[2] == ALLOWED_UID_2) && (str[3] == ALLOWED_UID_3)
					&& (str[4] == ALLOWED_UID_4)) {
				//I2C_CARD((uint8_t*) str);  // Prikaz broja kartice na LCD-u
				I2C_GRANTED();  // Dozvoljen pristup
				I2C_WELCOME();
			} else {
				//I2C_CARD((uint8_t*) str);  // Prikaz broja kartice na LCD-u
				I2C_DENIED();  // Odbijen pristup
				I2C_WELCOME();
			}
		}
	}
};
void I2C_DENIED(void) {
	// Generate a random index
	int random_index = rand() % denied_messages_count;
	// Display the message
	I2C_LCD_Clear();
	I2C_LCD_SetCursor(0, 0);
	I2C_LCD_WriteString((char*) denied_messages[random_index][0]); // First line
	I2C_LCD_SetCursor(0, 1);
	I2C_LCD_WriteString((char*) denied_messages[random_index][1]); // Second line
	// Generiranje zvučnog signala
	GenerateTone(800, 300);

	// Mala pauza da korisnik vidi poruku
	HAL_Delay(2500);

}

void I2C_GRANTED(void) {
	// Generate a random index
	int random_index = rand() % granted_messages_count;
	// Display the message
	I2C_LCD_Clear();
	I2C_LCD_SetCursor(0, 0);
	I2C_LCD_WriteString((char*) granted_messages[random_index][0]); // First line
	I2C_LCD_SetCursor(0, 1);
	I2C_LCD_WriteString((char*) granted_messages[random_index][1]); // Second line
	// Aktivacija releja
	HAL_GPIO_WritePin(RELAY_PORT, RELAY_PIN, GPIO_PIN_SET);

	// Zvuk odobrenja
	GenerateTone(2000, 100);

	// Drži relej uključen 5 sekundi
	HAL_Delay(2500);

	// Isključivanje releja
	HAL_GPIO_WritePin(RELAY_PORT, RELAY_PIN, GPIO_PIN_RESET);
}

void I2C_WELCOME(void) {
	// Lista šaljivih poruka za skeniranje kartice (prvi red | drugi red)
	// Generate a random index
	int random_index = rand() % welcome_messages_count;
	// Display the message
	I2C_LCD_Clear();
	I2C_LCD_SetCursor(0, 0);
	I2C_LCD_WriteString((char*) welcome_messages[random_index][0]); // First line
	I2C_LCD_SetCursor(0, 1);
	I2C_LCD_WriteString((char*) welcome_messages[random_index][1]); // Second line
};

void I2C_CARD(uint8_t *data) {
	char buffer[64] = { 0 }; // Glavni buffer za pohranu ASCII vrijednosti UID brojeva
	char temp[6];  // Privremeni buffer za pojedinačne brojeve

	I2C_LCD_Clear();  // Brisanje ekrana LCD-a prije prikaza podataka

	// Postavljanje kursora na prvi red LCD-a
	I2C_LCD_SetCursor(0, 0);
	for (int i = 0; i < 3; i++) {  // Petlja za ispis prva tri bajta UID-a
		snprintf(temp, sizeof(temp), "%d ", data[i]); // Pretvaranje broja u string
		strcat(buffer, temp);  // Dodavanje konvertiranog broja u buffer
	}
	I2C_LCD_WriteString(buffer); // Prikaz prvog dijela UID-a na LCD-u

	// Resetiranje buffera za prikaz drugog reda
	memset(buffer, 0, sizeof(buffer));

	// Postavljanje kursora na drugi red LCD-a
	I2C_LCD_SetCursor(0, 1);
	for (int i = 3; i < 5; i++) {  // Petlja za ispis preostala dva bajta UID-a
		snprintf(temp, sizeof(temp), "%d ", data[i]); // Pretvaranje broja u string
		strcat(buffer, temp);  // Dodavanje u buffer
	}
	I2C_LCD_WriteString(buffer); // Prikaz drugog dijela UID-a na LCD-u
	HAL_Delay(3000);  // Zadržavanje prikaza na ekranu 3 sekunde
}

void GenerateTone(int tone, int duration_ms) {
	Reset_TIM2();
    uint32_t arr = (TIM_CLOCK / tone) - 1; // Izračun ARR za željenu frekvenciju

    __HAL_TIM_SET_AUTORELOAD(&htim2, arr); // Postavljanje perioda PWM-a
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, arr / 2); // 50% duty cycle

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // Pokretanje PWM-a
    HAL_Delay(duration_ms); // Trajanje tona
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2); // Isključivanje PWM-a
}

void Reset_TIM2(void) {
	// Zaustavi PWM na kanalu 2 kako bi se osiguralo da se timer može sigurno resetirati
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
	// Deinicijaliziraj osnovni timer kako bi se oslobodili resursi i omogućila nova inicijalizacija
	HAL_TIM_Base_DeInit(&htim2);
	// Ponovno inicijaliziraj timer TIM2 s prethodno definiranim postavkama
	MX_TIM2_Init();
}

/* RC522 functions BEGIN------------------------------------------------------------------*/
uint8_t RC522_SPI_Transfer(uint8_t data) {
	uint8_t rx_data;
	HAL_SPI_TransmitReceive(HSPI_INSTANCE, &data, &rx_data, 1, 100);
	//HSPI_instance je gdje štalje, &data - što šalje, &rx_data - gdje prima, 1-bajt, 100ms timeout
	return rx_data;
}

void Write_MFRC522(uint8_t addr, uint8_t val) {
	HAL_GPIO_WritePin(MFRC522_CS_PORT, MFRC522_CS_PIN, GPIO_PIN_RESET); //aktivira RC522
	RC522_SPI_Transfer((addr << 1) & 0x7E); //adresa je ovisna o registru,   prvi bajt
	//Adresu shiftamo u lijevo za jedan bit, a operator & dodaje bajt 01111110
	//da osigura da su svi bitovi ispravno postavljeni
	//kod pisanja moramo osigurati da je najviši bit 0
	RC522_SPI_Transfer(val);	//drugi bajt je vrijednost za slanje
	HAL_GPIO_WritePin(MFRC522_CS_PORT, MFRC522_CS_PIN, GPIO_PIN_SET);  //deaktivira RC522
}

uint8_t Read_MFRC522(uint8_t addr) {
	//generička funkcija za čitanje vrijednosti registara
	//Za pisanje u registar: najniži bit adrese je 0.
	//Za čitanje iz registra: najniži bit adrese mora biti 1.
	uint8_t val; //definiramo varijablu za return vrijednost
	//select modula
	HAL_GPIO_WritePin(MFRC522_CS_PORT, MFRC522_CS_PIN, GPIO_PIN_RESET);
	//shiftamo adresu u lijevo za jedan bit, osiguravamo 6 srednjih bitova
	//operatorom OR stavljamo bit 1 na najviše mjesto jer označava čitanje
	RC522_SPI_Transfer(((addr << 1) & 0x7E) | 0x80);
	//šaljemo dummy byte da bi RC522 mogao vratitit vrijednost iz registra
	val = RC522_SPI_Transfer(0x00);
	//unselect modula
	HAL_GPIO_WritePin(MFRC522_CS_PORT, MFRC522_CS_PIN, GPIO_PIN_SET);
	return val;
}

void SetBitMask(uint8_t reg, uint8_t mask) {
	uint8_t tmp = Read_MFRC522(reg);
	Write_MFRC522(reg, tmp | mask);
}

void ClearBitMask(uint8_t reg, uint8_t mask) {
	uint8_t tmp = Read_MFRC522(reg);
	Write_MFRC522(reg, tmp & (~mask));
}

void AntennaOn(void) {
	Read_MFRC522(0x14);
	SetBitMask(0x14, 0x03);
}

void MFRC522_Reset(void) {
	Write_MFRC522(0x01, 0x0F);
}

void MFRC522_Init(void) {
	HAL_GPIO_WritePin(MFRC522_CS_PORT, MFRC522_CS_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MFRC522_RST_PORT, MFRC522_RST_PIN, GPIO_PIN_SET);
	MFRC522_Reset();
	Write_MFRC522(0x2A, 0x8D);
	Write_MFRC522(0x2B, 0x3E);
	Write_MFRC522(0x2D, 30);
	Write_MFRC522(0x2C, 0);
	Write_MFRC522(0x15, 0x40);
	Write_MFRC522(0x11, 0x3D);
	AntennaOn();
}

uint8_t MFRC522_ToCard(uint8_t command, uint8_t *sendData, uint8_t sendLen,
		uint8_t *backData, uint16_t *backLen) {
	uint8_t status = 2;
	uint8_t irqEn, waitIRq, n;
	int i;
	if (command == 0x0E) {
		irqEn = 0x12;
		waitIRq = 0x10;
	} else {
		irqEn = 0x77;
		waitIRq = 0x30;
	}
	Write_MFRC522(0x02, irqEn | 0x80);
	ClearBitMask(0x04, 0x80);
	SetBitMask(0x0A, 0x80);
	Write_MFRC522(0x01, 0x00);
	for (i = 0; i < sendLen; i++)
		Write_MFRC522(0x09, sendData[i]);
	Write_MFRC522(0x01, command);
	if (command == 0x0C)
		SetBitMask(0x0D, 0x80);
	i = 2000;
	do {
		n = Read_MFRC522(0x04);
		i--;
	} while ((i != 0) && !(n & 0x01) && !(n & waitIRq));
	ClearBitMask(0x0D, 0x80);
	if (i != 0) {
		if (!(Read_MFRC522(0x06) & 0x1B)) {
			status = 0;
			if (n & irqEn & 0x01)
				status = 1;
			if (command == 0x0C) {
				n = Read_MFRC522(0x0A);
				*backLen = n * 8;
				for (i = 0; i < n; i++)
					backData[i] = Read_MFRC522(0x09);
			}
		}
	}
	return status;
}

uint8_t MFRC522_Request(uint8_t reqMode, uint8_t *TagType) {
	uint8_t status;
	uint16_t backBits;  // Promijenjeno sa int na uint16_t

	Write_MFRC522(0x0D, 0x07);
	TagType[0] = reqMode;

	status = MFRC522_ToCard(0x0C, TagType, 1, TagType, &backBits);
	// if true, else
	return (status == 0 && backBits == 0x10) ? 0 : 2;
}

uint8_t MFRC522_Anticoll(uint8_t *serNum) {
	uint8_t status, i, serNumCheck = 0;
	uint16_t unLen;
	Write_MFRC522(0x0D, 0x00);
	serNum[0] = 0x93;
	serNum[1] = 0x20;
	status = MFRC522_ToCard(0x0C, serNum, 2, serNum, &unLen);
	if (status == 0) {
		for (i = 0; i < 4; i++)
			serNumCheck ^= serNum[i];
		if (serNumCheck != serNum[i])
			status = 2;
	}
	return status;
}
/* RC522 functions END------------------------------------------------------------------*/

/* LCD functions BEGIN------------------------------------------------------------------*/

//extern I2C_HandleTypeDef hi2c1;


/*-----------------------[INTERNAL DEFINITIONS]-----------------------*/
// CMD


/*-----------------------[INTERNAL VARIABLES]-----------------------*/

typedef struct I2C_LCD_InfoParam_s {}
I2C_LCD_InfoParam_t;


/*---------------------[STATIC INTERNAL FUNCTIONS]-----------------------*/

static void I2C_LCD_ExpanderWrite(uint8_t DATA) {
	uint8_t TxData = DATA | 0x08;
	HAL_I2C_Master_Transmit(&hi2c1,(0x27 << 1),
			&TxData, sizeof(TxData), 100);
}

static void I2C_LCD_EnPulse(uint8_t DATA) {
	I2C_LCD_ExpanderWrite(DATA | 0b00000100); // En high
	Delay_us(2);

	I2C_LCD_ExpanderWrite(DATA & ~0b00000100); // En low
	Delay_us(50);

}

static void I2C_LCD_Write4Bits(uint8_t Val) {
	I2C_LCD_ExpanderWrite(Val);
	I2C_LCD_EnPulse(Val);
}
//funkcija za slanje putem I2C
static void I2C_LCD_Send(uint8_t Val, uint8_t Mode) {
	//vrijednost koju je primio putem val varijable šalje putem 4 bita koje dijelimo
	//maskiramo 4 MSB bita i spremamo u varijablu HighNib
	uint8_t HighNib = Val & 0xF0;
	//shiftamo vrijednost 4 LSB bita u gornje bitove 4567
	uint8_t LowNib = (Val << 4) & 0xF0;
	// mode može biti 0x00 ili 0x01 ovisno šaljemo li komandu ili podatak
	// Mode 0x00 je komanda, mode 0x01 je podatak
	I2C_LCD_Write4Bits((HighNib) | Mode);
	I2C_LCD_Write4Bits((LowNib) | Mode);
}

static void I2C_LCD_Cmd(uint8_t CMD) {
	I2C_LCD_Send(CMD, 0);
}

static void I2C_LCD_Data(uint8_t DATA) {
	I2C_LCD_Send(DATA, 1);
}

/*-----------------------[USER EXTERNAL FUNCTIONS]-----------------------*/

void I2C_LCD_Init() {
	HAL_Delay(50);
	I2C_LCD_Cmd(0x30); //probudi LCD, ali on možda još nije stabilan
	HAL_Delay(5);
	I2C_LCD_Cmd(0x30);//osigurava da LCD shvati da radimo u 8-bitnom modu
	HAL_Delay(5);
	I2C_LCD_Cmd(0x30);//potvrđuje i zaključava 8-bitni način rada
	Delay_us(150);
	I2C_LCD_Cmd(0x02);//postavlja 4-bitni način rada

	//(LCD_FUNCTIONSET | LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS);
	I2C_LCD_Cmd(0x28);
	//I2C_LCD_Cmd(LCD_DISPLAYCONTROL | LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF);
	I2C_LCD_Cmd(0x0C);

	//I2C_LCD_Cmd(LCD_ENTRYMODESET | LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT);
	I2C_LCD_Cmd(0x06);
	I2C_LCD_Clear();
}

void I2C_LCD_Clear() {
	I2C_LCD_Cmd(0x01);
	HAL_Delay(2);
}

void I2C_LCD_SetCursor(uint8_t Col, uint8_t Row) {
	int Row_Offsets[] = { 0x00, 0x40, 0x14, 0x54 };
	if (Row > 2) {
		Row = 1;
	}
	I2C_LCD_Cmd(0x80 | (Col + Row_Offsets[Row]));
}

void I2C_LCD_WriteString(char *Str) {
	while (*Str) {
		I2C_LCD_Data(*Str++);
	}
}

/* LCD functions END-------------------------------------------------------*/

/* MicroSeconds functions END--------------------------------------------------*/
void Delay_us(uint32_t us) {
	uint32_t start = DWT->CYCCNT;
	uint32_t ticks = (SystemCoreClock / 1000000) * us; // Pretvara µs u CPU cikluse
	while ((DWT->CYCCNT - start) < ticks)
		;
}

void DWT_Init(void) {
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  // Omogući DWT
	DWT->CYCCNT = 0;  // Reset counter
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;  // Omogući counter
}

/* MicroSeconds functions END--------------------------------------------------*/

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
