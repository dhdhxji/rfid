/*
 * --------------------------------------------------------------------------------------------------------------------
 * Example sketch/program showing how to read data from a PICC to serial.
 * --------------------------------------------------------------------------------------------------------------------
 * This is a MFRC522 library example; for further details and other examples see: https://github.com/miguelbalboa/rfid
 * 
 * Example sketch/program showing how to read data from a PICC (that is: a RFID Tag or Card) using a MFRC522 based RFID
 * Reader on the Arduino SPI interface.
 * 
 * When the Arduino and the MFRC522 module are connected (see the pin layout below), load this sketch into Arduino IDE
 * then verify/compile and upload it. To see the output: use Tools, Serial Monitor of the IDE (hit Ctrl+Shft+M). When
 * you present a PICC (that is: a RFID Tag or Card) at reading distance of the MFRC522 Reader/PCD, the serial output
 * will show the ID/UID, type and any data blocks it can read. Note: you may see "Timeout in communication" messages
 * when removing the PICC from reading distance too early.
 * 
 * If your reader supports it, this sketch/program will read all the PICCs presented (that is: multiple tag reading).
 * So if you stack two or more PICCs on top of each other and present them to the reader, it will first output all
 * details of the first and then the next PICC. Note that this may take some time as all data blocks are dumped, so
 * keep the PICCs at reading distance until complete.
 * 
 * @license Released into the public domain.
 * 
 * Typical pin layout used:
 * -----------------------------------------------------------------------------------------
 *             MFRC522      Arduino       Arduino   Arduino    Arduino          Arduino
 *             Reader/PCD   Uno/101       Mega      Nano v3    Leonardo/Micro   Pro Micro
 * Signal      Pin          Pin           Pin       Pin        Pin              Pin
 * -----------------------------------------------------------------------------------------
 * RST/Reset   RST          9             5         D9         RESET/ICSP-5     RST
 * SPI SS      SDA(SS)      10            53        D10        10               10
 * SPI MOSI    MOSI         11 / ICSP-4   51        D11        ICSP-4           16
 * SPI MISO    MISO         12 / ICSP-1   50        D12        ICSP-1           14
 * SPI SCK     SCK          13 / ICSP-3   52        D13        ICSP-3           15
 *
 * More pin layouts for other boards can be found here: https://github.com/miguelbalboa/rfid#pin-layout
 */

#include <SPI.h>
#include <MFRC522.h>

#define RST_PIN         22          // Configurable, see typical pin layout above
#define SS_PIN          21         // Configurable, see typical pin layout above

uint8_t gpio_get_level(uint8_t pin, void* ctx) {
	return digitalRead(pin);
}

void gpio_set_level(uint8_t pin, uint8_t level, void* ctx) {
	digitalWrite(pin, level);
}

size_t log_write(const char* msg, size_t len, void* ctx) {
	Serial.write((const uint8_t*)msg, len);
}

void spi_exchange(const uint8_t* send, uint8_t* rcv, size_t len, void* ctx) {
	const uint8_t cs = (const uint32_t)ctx;
	
	SPI.beginTransaction(SPISettings(MFRC522_SPICLOCK, MSBFIRST, SPI_MODE0));
	digitalWrite(cs, LOW);

	for(size_t i = 0; i < len; ++i) {
		uint8_t read = SPI.transfer(send[i]);

		if(rcv) {
			rcv[i] = read;
		}
	}
	digitalWrite(cs, HIGH);
	SPI.endTransaction();
}

void delay_ms(uint32_t time, void* ctx) {
	delay(time);
}

uint32_t time_ms(void* ctx) {
	return millis();
}

MFRC522_cfg_t mfrc522cfg = {
	.chipSelectPin = SS_PIN,
	.resetPowerDownPin = RST_PIN,
	.gpio_cfg = {
		.set_level = gpio_set_level,
		.get_level = gpio_get_level,
		.ctx = NULL
	},
	.spi_cfg = {
		.exchange = spi_exchange,
		.ctx = (void*)SS_PIN
	},
	.log_cfg = {
		.write = log_write,
		.ctx = NULL
	},
	.time_cfg = {
		.delay_ms = delay_ms,
		.time_ms = time_ms,
		.ctx = NULL
	}
};


MFRC522_t mfrc;
//(SS_PIN, RST_PIN);  // Create MFRC522 instance

void setup() {
	Serial.begin(115200);		// Initialize serial communications with the PC
	while (!Serial);		// Do nothing if no serial port is opened (added for Arduinos based on ATMEGA32U4)

	Serial.println(MISO);

	pinMode(SS_PIN, OUTPUT);
	pinMode(RST_PIN, OUTPUT);
	pinMode(MISO, INPUT);
	pinMode(MOSI, OUTPUT);

	SPI.begin();			// Init SPI bus
	MFRC522_init(&mfrc522cfg, &mfrc);
	//mfrc522.PCD_Init();		// Init MFRC522
	delay(4);				// Optional delay. Some board do need more time after init to be ready, see Readme
	//mfrc522.PCD_DumpVersionToSerial();	// Show details of PCD - MFRC522 Card Reader details
	PCD_DumpVersionToSerial(&mfrc);
	Serial.println(F("Scan PICC to see UID, SAK, type, and data blocks..."));
}

void loop() {
	// Reset the loop if no new card present on the sensor/reader. This saves the entire process when idle.
	/* if ( ! mfrc522.PICC_IsNewCardPresent()) {
		return;
	} */
	if( ! PICC_IsNewCardPresent(&mfrc)) {
		return;
	}

	// Select one of the cards
	/* if ( ! mfrc522.PICC_ReadCardSerial()) {
		return;
	} */
	if( ! PICC_ReadCardSerial(&mfrc)) {
		return;
	} 

	// Dump debug info about the card; PICC_HaltA() is automatically called
	//mfrc522.PICC_DumpToSerial(&(mfrc522.uid));
	PICC_DumpToSerial(&mfrc, &mfrc.uid);
}
