// #include <driver/spi_slave.h>
// #include <esp32cam.h>
// #include <esp_system.h>
// #include <stdio.h>
// #include <string.h>
// #include "driver/gpio.h"

// // Pin Definitions
// #define SPI_CLK_PIN 14    // Serial Clock (SCK)
// #define SPI_MISO_PIN 12   // Master In Slave Out
// #define SPI_MOSI_PIN 13   // Master Out Slave In
// #define SPI_CS_PIN 15     // Chip Select (Slave Select)

// // Resolution Setting
// static auto hiRes = esp32cam::Resolution::find(350, 530);

// // SPI Transaction Buffer
// spi_slave_transaction_t trans;

// // SPI Slave Initialization
// void initSPI() 
// {
//   // SPI Bus Configuration
//   spi_bus_config_t buscfg = {
//     .mosi_io_num = SPI_MOSI_PIN,
//     .miso_io_num = SPI_MISO_PIN,
//     .sclk_io_num = SPI_CLK_PIN,
//     .quadwp_io_num = -1, // Not used
//     .quadhd_io_num = -1, // Not used
//     .max_transfer_sz = 4096, // Maximum transfer size
//   };

//   // SPI Slave Interface Configuration
//   spi_slave_interface_config_t slvcfg = {
//     .spics_io_num = SPI_CS_PIN, // CS pin
//     .flags = 0,                 // No special flags
//     .queue_size = 3,            // Transaction queue size
//     .mode = 0,                  // SPI Mode 0
//     .post_setup_cb = NULL,      // No post-setup callback
//     .post_trans_cb = NULL,      // No post-transaction callback
//   };

//   // Initialize SPI Slave
//   ESP_ERROR_CHECK(spi_slave_initialize(HSPI_HOST, &buscfg, &slvcfg, SPI_DMA_DISABLED));
//   Serial.println("SPI Slave Initialized");
// }

// // Send Image Over SPI
// void sendImageOverSPI() 
// {
//   // Capture Image
//   auto frame = esp32cam::capture();
//   if (frame == nullptr) {
//       Serial.println("Image capture failed.");
//       return;
//   }

//   Serial.println("Image captured. Sending...");

//   // Send Image Size
//   uint32_t imgSize = frame->size();
//   memset(&trans, 0, sizeof(trans));
//   trans.length = sizeof(imgSize) * 8;  // Data length in bits
//   trans.tx_buffer = &imgSize;
//   ESP_ERROR_CHECK(spi_slave_transmit(HSPI_HOST, &trans, portMAX_DELAY)); // Transmit image size

//   // Send Image Data in Chunks
//   const size_t CHUNK_SIZE = 512;
//   for (size_t i = 0; i < imgSize; i += CHUNK_SIZE) {
//       size_t len = (imgSize - i) < CHUNK_SIZE ? (imgSize - i) : CHUNK_SIZE;
//       memset(&trans, 0, sizeof(trans));
//       trans.length = len * 8; // Data length in bits
//       trans.tx_buffer = frame->data() + i;
//       ESP_ERROR_CHECK(spi_slave_transmit(HSPI_HOST, &trans, portMAX_DELAY)); // Transmit image chunk
//   }

//   Serial.println("Image sent.");
// }

// void setup() 
// {
//   // Initialize Serial for Debugging
//   Serial.begin(115200);

//   // Initialize SPI Slave
//   initSPI();

//   // Initialize Camera
//   using namespace esp32cam;
//   Config config;
//   config.setPins(pins::AiThinker);
//   config.setResolution(hiRes);
//   config.setBufferCount(2);
//   config.setJpeg(80);

//   bool ok = Camera.begin(config);
//   Serial.println(ok ? "CAMERA STARTED" : "CAMERA FAILED");
// }

// void loop() 
// {
//   sendImageOverSPI();
//   delay(5000); // Wait before sending the next image
// }

#include <Wire.h>
#include <esp32cam.h>
#include <esp_system.h>
#include <stdio.h>
#include <string.h>

// Pin Definitions for I2C
#define I2C_SDA_PIN 15  // SDA Line
#define I2C_SCL_PIN 14  // SCL Line

// Resolution Setting
static auto hiRes = esp32cam::Resolution::find(350, 530);

void initI2C() {
  // Initialize I2C with custom pins
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Serial.println("Software I2C initialized on GPIO 15 (SDA) and GPIO 14 (SCL).");
}

void sendImageOverI2C() {
  // Capture Image
  auto frame = esp32cam::capture();
  if (frame == nullptr) {
    Serial.println("Image capture failed.");
    return;
  }

  Serial.println("Image captured. Sending...");

  // Send Image Size
  uint32_t imgSize = frame->size();
  Wire.beginTransmission(0x08);  // Replace 0x08 with the I2C address of the Jetson
  Wire.write((uint8_t*)&imgSize, sizeof(imgSize));
  Wire.endTransmission();

  // Send Image Data in Chunks
  const size_t CHUNK_SIZE = 32; // I2C buffer size limit (32 bytes max per transmission)
  for (size_t i = 0; i < imgSize; i += CHUNK_SIZE) {
    size_t len = (imgSize - i) < CHUNK_SIZE ? (imgSize - i) : CHUNK_SIZE;
    Wire.beginTransmission(0x08);
    Wire.write(frame->data() + i, len); // Write chunk data
    Wire.endTransmission();
    delay(10); // Small delay for receiver to process
  }

  Serial.println("Image sent.");
}

void setup() {
  // Initialize Serial for Debugging
  Serial.begin(115200);

  // Initialize I2C
  initI2C();

  // Initialize Camera
  using namespace esp32cam;
  Config config;
  config.setPins(pins::AiThinker);
  config.setResolution(hiRes);
  config.setBufferCount(2);
  config.setJpeg(80);

  bool ok = Camera.begin(config);
  Serial.println(ok ? "CAMERA STARTED" : "CAMERA FAILED");
}

void loop() {
  sendImageOverI2C();
  delay(5000); // Wait before sending the next image
}

