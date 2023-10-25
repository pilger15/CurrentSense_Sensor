

#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "esp_log.h"
// #include "esp_sleep.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_private/wifi.h"
#include "esp_private/esp_wifi_types_private.h"

#include "driver/usb_serial_jtag.h"

#include "nvs_flash.h"

#include "sdkconfig.h"

#include "driver/gpio.h"
#include "soc/gpio_reg.h"
// #include "driver/timer.h"
#include "esp_timer.h"
#include "driver/spi_master.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
// #include "freertos/ringbuf.h"

esp_timer_handle_t periodic_timer;
#define TIMERPERIOD_US 125

#define BLUE_LED GPIO_NUM_3
// #define BIT_BLUE_LED BIT3
#define ORANGE_LED GPIO_NUM_2
// #define BIT_ORANGE_LED BIT2

#define PIN_NUM_CS GPIO_NUM_5
#define PIN_NUM_CLK GPIO_NUM_8
#define PIN_NUM_MISO GPIO_NUM_9
#define PIN_NUM_MOSI GPIO_NUM_6

#define SPI_DMA_CH 1

#define SPI_FREQUENCY 20000000

#define CONFIG_ESPNOW_CHANNEL 1
// a0:76:4e:43:9b:88
// static const uint8_t sensor_mac[ESP_NOW_ETH_ALEN] = {0xA0, 0x76, 0x4E, 0x43, 0x9B, 0x88};
// Define the structure for the data to be sent over ESP-NOW

esp_now_peer_info_t *peer_dongle;
bool dongleRegistered = false;
// Number of ADC data to be sent (16-bit)
#define ADC_BUFFER_LEN (ESP_NOW_MAX_DATA_LEN / 2) // Length of the ADC buffer
uint16_t buffer_idx = 0;
bool buffer_sel = false;
bool buffer_sel_prev = false;

spi_device_handle_t spi;
uint8_t adc_data[2] = {0x00, 0x00}; // Data to be sent over SPI
spi_transaction_t adc_trans;

#define RINGBUFFER_READ_LEN_16 (ESP_NOW_MAX_DATA_LEN / 2 - 1) // one uint16_t for packet counter
#define RINGBUFFER_SIZE (RINGBUFFER_READ_LEN_16 * 32)
uint16_t adc_ringbuffer[RINGBUFFER_SIZE];
uint16_t buffer_write_idx = 0;
uint16_t buffer_read_idx = 0;
uint16_t transfer_buffer[RINGBUFFER_READ_LEN_16 + 1];
uint8_t receive_buffer[RINGBUFFER_READ_LEN_16 + 1];
uint16_t packet_cnt = 0;

typedef enum
{
    TRANSFER_USB,
    TRANSFER_WIFI
} transfertype_t;
const transfertype_t transfertype = TRANSFER_WIFI;
typedef enum
{
    espcommand_empty,
    espcommand_start,
    espcommand_stop,
    espcommand_wifiTx_power
} esp_command_t;

// static const uint64_t espcommand_channel = 0x74797380ec3cce13ULL;
static const uint64_t espcommand_channel_bigendian = 0x13CE3CEC80737974ULL; // reverse due to little endian when casting

bool measure = false;

void spi_init(void)
{

    // gpio_set_direction(GPIO_NUM_10, GPIO_MODE_OUTPUT); // conversion

    // SPI
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = -1, // not used
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 16,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO));

    // SPI device configuration
    spi_device_interface_config_t dev_cfg = {
        .command_bits = 0, ///< Default amount of bits in command phase (0-16), used when ``SPI_TRANS_VARIABLE_CMD`` is not used, otherwise ignored.
        .address_bits = 0, ///< Default amount of bits in address phase (0-64), used when ``SPI_TRANS_VARIABLE_ADDR`` is not used, otherwise ignored.
        .dummy_bits = 0,   ///< Amount of dummy bits to insert between address and data phase
        .mode = 0,
        .cs_ena_pretrans = 4, // CS is initialisation conversion and transmission
        .cs_ena_posttrans = 16,
        .spics_io_num = GPIO_NUM_10,
        .clock_speed_hz = SPI_FREQUENCY,
        .queue_size = 1,
        .flags = SPI_DEVICE_HALFDUPLEX,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev_cfg, &spi));

    adc_trans.tx_buffer = NULL; // not used

    // adc_trans.length = 16;
    adc_trans.rxlength = 16;
    adc_trans.flags = SPI_TRANS_USE_RXDATA;

    // memset(&adc_trans, 0, sizeof(adc_trans));

    // adc_trans.length = 16; // length in bits

    //   memset(&buffer0, 0, sizeof(buffer0));
    //  memset(&buffer1, 0, sizeof(buffer1));

    ESP_ERROR_CHECK(spi_device_acquire_bus(spi, portMAX_DELAY));
}
static void IRAM_ATTR periodic_timer_callback(void *arg)
{
    if (measure)
    {
        // REG_WRITE(GPIO_OUT_W1TC_REG, BIT10); // Set conversion to 0 (initialize transfer)

        esp_err_t ret = spi_device_polling_transmit(spi, &adc_trans); // Read last conversion
        if (ret != ESP_OK)
        {
            ESP_LOGE("SPI", "SPI transaction failed");
        }
        adc_ringbuffer[buffer_write_idx] = ((uint16_t)adc_trans.rx_data[0] << 8) | (adc_trans.rx_data[1]);
        // SPI_TRANS_USE_RXDATA;

        buffer_write_idx++;
        buffer_write_idx %= RINGBUFFER_SIZE;
        if (buffer_write_idx == buffer_read_idx)
        { // since incrementing in single steps this should catch an overflow
          // ESP_LOGE("SPI", "RINGBUFFER BUFFER OVERFLOW, w=%d r=%d,buffersize=%d", buffer_write_idx, buffer_read_idx, RINGBUFFER_SIZE);
        }
        /*if (++buffer_idx >= ADC_BUFFER_LEN)
        {

            buffer_sel = !buffer_sel;                    // Toggle buffer selection using logical NOT operator
            buffer_idx = 0;                              // Reset buffer index

            buffer_store = buffer_sel ? buffer0 : buffer1;   // Set buffer_p to the selected buffer
        }
        buffer_store[buffer_idx] = ((uint16_t)adc_trans.rx_data[1]|(uint16_t)adc_trans.rx_data[0])+buffer_idx;        // Store data in the selected buffer #TODO: #DEBUG remove bufferIDX
        */
        //  REG_WRITE(GPIO_OUT_W1TS_REG, BIT10); // Set conversion to 1 (start conversion)
    }
    else
    {
        // REG_WRITE(GPIO_OUT_W1TC_REG, BIT2); // clear
        ESP_ERROR_CHECK(esp_timer_stop(periodic_timer));
        buffer_read_idx = 0;
        buffer_write_idx = 0;
    }
}
static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    if (status == ESP_NOW_SEND_SUCCESS)
    {
        // Data sent successfully
    }
    else
    {
        ESP_LOGE("ESP-NOW", "Error sending data: %s", esp_err_to_name(status));
        REG_WRITE(GPIO_OUT_W1TC_REG, 1 << BLUE_LED); // Clear BLUE LED;
    }
}
static void espnow_recv_cb(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len)
{
    //
    if (dongleRegistered && esp_now_is_peer_exist(esp_now_info->src_addr)) // check if command is from registed Dongle
    {
        ESP_LOGD("ESP-NOW", "Received command %d- entering Switch", data[0]);
        switch ((esp_command_t)data[0])
        {
        case espcommand_start:
            if (!measure) // start measure
            {
                measure = true;
                packet_cnt = 0;
                buffer_read_idx = 0;
                buffer_write_idx = 0;
                ESP_LOGD("ESP-NOW", "Start measurement");
                ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, TIMERPERIOD_US)); // in us
                                                                                           // garbage transmit
                ESP_LOGD("TIMER", "Measurement started");

                REG_WRITE(GPIO_OUT_W1TS_REG, 1 << ORANGE_LED); // High
            }
            break;
        case espcommand_stop:
            if (measure)
            {
                measure = false;
                ESP_LOGD("ESP-NOW", "Stop measurement"); // stop
                // timer stopped in ISR
                REG_WRITE(GPIO_OUT_W1TC_REG, 1 << ORANGE_LED); // LOW
                                                               //  REG_WRITE(GPIO_OUT_W1TC_REG, BIT10); // LOW
                ESP_LOGD("TIMER", "Measurement stopped");
                REG_WRITE(GPIO_OUT_W1TS_REG, 1 << BLUE_LED); // Set blue in case it was turned off as error indicator
            }

            break;
        case espcommand_wifiTx_power:
        {
            ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(data[1]));
        }
        break;
        default:
            break;
        }
    }
    // register dongle
    else if (!dongleRegistered && (data_len == sizeof(uint64_t)) && *(uint64_t *)data == espcommand_channel_bigendian) // cast command an receive reverse endian
    {
        uint8_t espcommand_channel[8];
        memcpy(espcommand_channel, data, 8);
        ESP_LOGD("ESP-NOW", "Registration of channel %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
                 espcommand_channel[0],
                 espcommand_channel[1],
                 espcommand_channel[2],
                 espcommand_channel[3],
                 espcommand_channel[4],
                 espcommand_channel[5],
                 espcommand_channel[6],
                 espcommand_channel[7]);

        /* Add Dongle information to peer list. */

        memcpy(peer_dongle->peer_addr, esp_now_info->src_addr, ESP_NOW_ETH_ALEN);
        ESP_ERROR_CHECK(esp_now_add_peer(peer_dongle));
        ESP_ERROR_CHECK(esp_now_set_peer_rate_config(peer_dongle->peer_addr, WIFI_PHY_RATE_54M));
        ESP_LOGD("ESP-NOW", "Dongle-MAC: %02x:%02x:%02x:%02x:%02x:%02x",
                 peer_dongle->peer_addr[0],
                 peer_dongle->peer_addr[1],
                 peer_dongle->peer_addr[2],
                 peer_dongle->peer_addr[3],
                 peer_dongle->peer_addr[4],
                 peer_dongle->peer_addr[5]);
        dongleRegistered = true;

        REG_WRITE(GPIO_OUT_W1TS_REG, 1 << BLUE_LED); // Set blue
    }
    else
    {
        ESP_LOGD("ESP-NOW", "Received message but not addressed to this channel");
    }
}

void IRAM_ATTR usb_transfer_task(void *pvParameters)
{
    usb_serial_jtag_driver_config_t usb_serial_config = {
        .tx_buffer_size = sizeof(transfer_buffer),
        .rx_buffer_size = sizeof(transfer_buffer)};
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_serial_config));

    /* Configure a bridge buffer for the incoming data */

    // memset(buffer_espnow_rx, 0x55, sizeof(buffer_espnow_rx) * sizeof(uint8_t));
    ESP_LOGI("USB", "USB initialised");

    while (true)
    {

        // check ringbuffer wrap

        if (usb_serial_jtag_read_bytes(receive_buffer, sizeof(receive_buffer), pdMS_TO_TICKS(1)))
        {
            if (!measure) // start measure
            {
                ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, TIMERPERIOD_US)); // in us
                                                                                           // garbage transmit
                ESP_LOGD("TIMER", "Measurement started");

                REG_WRITE(GPIO_OUT_W1TS_REG, 1 << ORANGE_LED); // High
                packet_cnt = 0;
            }
            else
            {                                                  // stop
                REG_WRITE(GPIO_OUT_W1TC_REG, 1 << ORANGE_LED); // LOW
                                                               //  REG_WRITE(GPIO_OUT_W1TC_REG, BIT10); // LOW
                ESP_LOGD("TIMER", "Measurement stopped");
                REG_WRITE(GPIO_OUT_W1TS_REG, 1 << BLUE_LED); // Set blue in case it was turned off as error indicator
            }

            measure = !measure;
            buffer_read_idx = 0;
            buffer_write_idx = 0;
        }
        else if (((buffer_read_idx + RINGBUFFER_READ_LEN_16) < buffer_write_idx) // this assumes that read+read_len will not wrap around
                 || (buffer_write_idx < buffer_read_idx))
        { // this assumes the write has wrapped around so the read is bigger

            // Send data over ESP-NOW
            // REG_WRITE(GPIO_OUT_W1TS_REG, 1 <<BLUE_LED); // initialise conversion
            memcpy(&transfer_buffer[1], &adc_ringbuffer[buffer_read_idx], RINGBUFFER_READ_LEN_16 * sizeof(uint16_t));
            transfer_buffer[0] = (packet_cnt++); // will overflow to 0
            usb_serial_jtag_write_bytes((uint8_t *)transfer_buffer, sizeof(transfer_buffer), portMAX_DELAY);
            buffer_read_idx = (buffer_read_idx + RINGBUFFER_READ_LEN_16) % RINGBUFFER_SIZE;
            // ESP_LOGE("CNT", "CNT = %d", espnow_buffer_tx[0]);
            // REG_WRITE(GPIO_OUT_W1TC_REG, 1 <<BLUE_LED); // initialise conversion
        }
        taskYIELD();
    }
}

void IRAM_ATTR espnow_transfer_task(void *pvParameters)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    // esp_wifi_internal_set_fix_rate(CONFIG_ESPNOW_CHANNEL, 1, WIFI_PHY_RATE_MCS7_SGI);

    ESP_ERROR_CHECK(esp_wifi_start());
    // ESP_ERROR_CHECK(esp_wifi_internal_set_fix_rate(WIFI_IF_STA, true, WIFI_PHY_RATE_MCS7_SGI));
    ESP_ERROR_CHECK(esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

    // Initialize ESPNOW and register sending and receiving callback function.

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_wifi_config_espnow_rate(WIFI_MODE_STA, WIFI_PHY_RATE_MCS7_SGI));
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));

    uint8_t mac_address[6];
    esp_wifi_get_mac(ESP_IF_WIFI_STA, mac_address);
    ESP_LOGI("MAC", "%02x:%02x:%02x:%02x:%02x:%02x\n",
             mac_address[0], mac_address[1], mac_address[2],
             mac_address[3], mac_address[4], mac_address[5]);
    // esp_log_level_set("ESP-NOW", ESP_LOG_DEBUG);

    while (true)
    {

        // check ringbuffer wrap
        if (dongleRegistered && (((buffer_read_idx + RINGBUFFER_READ_LEN_16) < buffer_write_idx) // this assumes that read+read_len will not wrap around
                                 || (buffer_write_idx < buffer_read_idx)))
        { // this assumes the write has wrapped around so the read is bigger

            // Send data over ESP-NOW
            // REG_WRITE(GPIO_OUT_W1TS_REG, 1 <<BLUE_LED); // initialise conversion
            memcpy(&transfer_buffer[1], &adc_ringbuffer[buffer_read_idx], RINGBUFFER_READ_LEN_16 * sizeof(uint16_t));
            transfer_buffer[0] = (packet_cnt++); // will overflow to 0
            esp_err_t ret = esp_now_send(peer_dongle->peer_addr, (uint8_t *)transfer_buffer, sizeof(transfer_buffer));
            buffer_read_idx = (buffer_read_idx + RINGBUFFER_READ_LEN_16) % RINGBUFFER_SIZE;
            // ESP_LOGE("CNT", "CNT = %d", espnow_buffer_tx[0]);
            // REG_WRITE(GPIO_OUT_W1TC_REG, 1 <<BLUE_LED); // initialise conversion
            if (ret != ESP_OK)
            {
                ESP_LOGE("ESP-NOW", "Error sending data: %s", esp_err_to_name(ret));
                REG_WRITE(GPIO_OUT_W1TC_REG, 1 << BLUE_LED); // Clear BLUE LED;
            }
        }
        taskYIELD();
    }
}

void app_main(void)
{
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // Initialize NVS
    esp_err_t ret;
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI("NVS", "NVS Initialised");

    // a0:76:4e:44:51:38
    // a0:76:4e:43:9b:88
    // INIT WIFI

    gpio_set_direction(BLUE_LED, GPIO_MODE_OUTPUT);
    gpio_set_direction(ORANGE_LED, GPIO_MODE_OUTPUT);

    REG_WRITE(GPIO_OUT_W1TS_REG, 1 << BLUE_LED);   // initialise High
    REG_WRITE(GPIO_OUT_W1TC_REG, 1 << ORANGE_LED); // initialise low

    spi_init();
    // Initialise timer----------------------------------------------------------------------

    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &periodic_timer_callback,
        /* name is optional, but may help identify the timer when debugging */
        .name = "periodic"};

    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));

    peer_dongle = malloc(sizeof(esp_now_peer_info_t));

    memset(peer_dongle, 0, sizeof(esp_now_peer_info_t));
    peer_dongle->channel = CONFIG_ESPNOW_CHANNEL;
    peer_dongle->ifidx = ESP_IF_WIFI_STA;
    peer_dongle->encrypt = false;

    //  ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, TIMERPERIOD_US)); // in us
    // Create data to send
    //

    // Send data over ESP-NOW
    // Create the task for sending ESP-NOW data
    switch (transfertype)
    {
    case TRANSFER_WIFI:
        xTaskCreate(espnow_transfer_task, "espnow_send_task", 4096, NULL, tskIDLE_PRIORITY, NULL);
        break;

    case TRANSFER_USB:
        xTaskCreate(usb_transfer_task, "usb_transfer_task", 4096, NULL, tskIDLE_PRIORITY, NULL);
        break;
    }

    /*    while (false)
        {
            REG_WRITE(GPIO_OUT_W1TS_REG, BIT2); //  High
            vTaskDelay(pdMS_TO_TICKS(300));
            REG_WRITE(GPIO_OUT_W1TC_REG, BIT2); // LOW
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    */
}
