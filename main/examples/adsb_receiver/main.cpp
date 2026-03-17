/*
 * @Description: lvgl_9_ui
 * @Author: LILYGO_L
 * @Date: 2025-06-13 13:34:16
 * @LastEditTime: 2026-03-10 (jstockdale: non-blocking USB, ADSB table on live screen)
 * @License: GPL 3.0
 */
#include <stdio.h>
#include <unistd.h>
#include <sys/lock.h>
#include <sys/time.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_timer.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_ldo_regulator.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_intr_alloc.h"
#include "usb/usb_host.h"
#include "driver/gpio.h"
#include "lvgl.h"
#include "t_display_p4_driver.h"
#include "cpp_bus_driver_library.h"
#if defined CONFIG_BOARD_TYPE_T_DISPLAY_P4
#include "t_display_p4_config.h"
#elif defined CONFIG_BOARD_TYPE_T_DISPLAY_P4_KEYBOARD
#include "t_display_p4_keyboard_config.h"
#include "st25r3916_driver.h"
#include "RadioLib.h"
#include "radiolib_bridge_driver.h"
#include "kode_bq25896.h"
#endif
#include "lvgl_ui.h"
#include "sd_pwr_ctrl_by_on_chip_ldo.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "New Notification 010_c2_b16_s44100.h"
#include "ICM20948_WE.h"
#include "esp_netif.h"
#include "esp_eth.h"
#include "esp_event.h"
#include "ethernet_init.h"
#if CONFIG_ENABLE_USB_DISPLAY == true
#include "esp_lcd_usb_display.h"
#else
//#include "tinyusb.h"
//#include "tusb_cdc_acm.h"
#endif
#include "app_video.h"
#include "driver/ppa.h"
#include "esp_private/esp_cache_private.h"
#include <fstream>

#include "class_driver.h"
#include "serial_console.h"
// WiFi disabled — Espressif Issue #17889 (SDMMC controller DMA conflict)
// WiFi disabled — esp_hosted and esp_wifi_remote removed from build
// to prevent SDIO auto-init that fragments internal heap (Issue #17889)
// #include "sd_http_server.h"
// #include "esp_wifi.h"
// #include "esp_sntp.h"

#define CONFIG_APP_QUIT_PIN 0
#define HOST_LIB_TASK_PRIORITY 2
#define CLASS_TASK_PRIORITY 3
#define APP_QUIT_PIN CONFIG_APP_QUIT_PIN

#ifdef CONFIG_USB_HOST_ENABLE_ENUM_FILTER_CALLBACK
#define ENABLE_ENUM_FILTER_CALLBACK
#endif // CONFIG_USB_HOST_ENABLE_ENUM_FILTER_CALLBACK

// extern void class_driver_task(void *arg);
// extern void class_driver_client_deregister(void);
// extern void adsb_update_display(lv_obj_t *table);

//static const char *TAG = "USB host lib";

QueueHandle_t app_event_queue = NULL;

#define SD_FILE_PATH_MUSIC "/sdcard/t_display_p4_lvgl_9_ui_resource/music/Erik Satie-Gymnopedie 1-Chase Coleman (piano).wav"

#define LVGL_TICK_PERIOD_MS 1

#define MCLK_MULTIPLE i2s_mclk_multiple_t::I2S_MCLK_MULTIPLE_256
#define SAMPLE_RATE 44100
#define BITS_PER_SAMPLE 16
#define NUM_CHANNEL 2

#define PREPEND_STRING "esp32p4 hardware usb cdc receive: "
#define PREPEND_LENGTH 34

#define ALIGN_UP(num, align) (((num) + ((align) - 1)) & ~((align) - 1))

enum class Es8311_Mode
{
    TEST = 0,
    PLAY_MUSIC,
};

enum class Imu_Mode
{
    TEST = 0,
};

enum class Battery_Health_Mode
{
    TEST = 0,
};

enum class Gps_Mode
{
    RUN = 0,   // Normal operation — feed ADS-B receiver pos, no CIT UI
    TEST = 1,  // CIT test screen — update GPS data label
};

enum class Ethernet_Mode
{
    TEST = 0,
};

struct Ethernet_Info
{
    bool link_up_flag = false;

    struct
    {
        std::string data;

        bool update_flag = false;
    } status;

    struct
    {
        std::string data;

        bool update_flag = false;
    } connect_ip_status;
};

enum class Rtc_Mode
{
    TEST = 0,
    GET_TIME,
};

enum class At_Mode
{
    TEST = 0,
};

enum class Music_File_Read_Speed_Enum
{
    LOW_SPEED,
    HIGH_SPEED,
};

// WAV file header
struct Wav_Header
{
    char riff_header[4];
    uint32_t riff_size;
    char wave_header[4];
    char fmt_header[4];
    uint32_t fmt_chunk_size;
    uint16_t audio_format;
    uint16_t num_channel;
    uint32_t sample_rate;
    uint32_t byte_rate;
    uint16_t block_align;
    uint16_t bits_per_sample;
    char data_header[4];
    uint32_t data_size;
};

struct System_Status
{
    struct { bool init_flag = false; } sgm38121;
    struct { bool init_flag = false; } sx1262;
    struct { bool init_flag = false; } camera;

#if defined CONFIG_BOARD_TYPE_T_DISPLAY_P4_KEYBOARD
    struct { bool init_flag = false; } xl9555;
    struct { bool init_flag = false; } tca8418;
    struct { bool init_flag = false; } st25r3916;
    struct { bool init_flag = false; } cc1101;
    struct { bool init_flag = false; } nrf24l01;
    struct { bool init_flag = false; } bq25896;
#endif

    struct { bool init_flag = false; } pcf8563;
    struct { bool init_flag = false; } bq27220;
    struct { bool init_flag = false; } aw86224;
    struct { bool init_flag = false; } es8311;
    struct { bool init_flag = false; } icm20948;
    struct { bool init_flag = false; } l76k;
    struct
    {
        bool init_flag = false;
        bool wifi_connect_status = false;
    } esp32c6;
};

Ethernet_Info Eth_Info;
System_Status Sys_Status;

_lock_t lvgl_api_lock;

lv_obj_t *Lvgl_Startup_Progress_Bar;

size_t Cycle_Time = 0;

TaskHandle_t Vibration_Task_Handle = NULL;
TaskHandle_t Speaker_Task_Handle = NULL;
TaskHandle_t Microphone_Task_Handle = NULL;
TaskHandle_t Imu_Task_Handle = NULL;
TaskHandle_t Gps_Task_Handle = NULL;
TaskHandle_t Ethernet_Task_Handle = NULL;
TaskHandle_t At_Task_Handle = NULL;
TaskHandle_t Sleep_Task_Handle = NULL;
TaskHandle_t Rf_Task_Handle = NULL;
TaskHandle_t Adsb_App_Task_Handle = NULL;
TaskHandle_t Iis_Transmission_Data_Stream_Task = NULL;

// USB host task handles — kept so app_main can clean up if needed
static TaskHandle_t s_host_lib_task_hdl = NULL;
static TaskHandle_t s_class_driver_task_hdl = NULL;

uint8_t AW86224_Vibration_Play_Count = 0;

Es8311_Mode ES8311_Speaker_Mode = Es8311_Mode::TEST;
Es8311_Mode ES8311_Microphone_Mode = Es8311_Mode::TEST;

bool Music_Play_End_Flag = false;
bool Set_Music_Current_Time_S_Flag = false;
double Set_Music_Current_Time_S = 0;
std::vector<char> Iis_Transmission_Data_Stream;
size_t Iis_Read_Data_Size_Index = 0;
std::ifstream Music_File;
Music_File_Read_Speed_Enum Music_File_Read_Speed = Music_File_Read_Speed_Enum::HIGH_SPEED;

Imu_Mode ICM20948_Imu_Mode = Imu_Mode::TEST;
Gps_Mode L76k_Gps_Mode = Gps_Mode::RUN;

bool L76k_Gps_Positioning_Flag = false;
size_t L76k_Gps_Positioning_Time = 0;

Ethernet_Mode Ip101gri_Ethernet_Mode = Ethernet_Mode::TEST;

At_Mode Esp32c6_At_Mode = At_Mode::TEST;

ppa_client_handle_t ppa_srm_handle = NULL;
size_t data_cache_line_size = 0;
ppa_client_handle_t ppa_srm_handle_2 = NULL;
size_t data_cache_line_size_2 = 0;
void *lcd_buffer[CONFIG_EXAMPLE_CAM_BUF_COUNT];
int32_t fps_count;
int64_t start_time;
int32_t video_cam_fd0;

bool Rf_Send_Flag = false;
uint8_t Rf_Send_Package[255] = {0};

bool Device_Rf_Task_Stop_Flag = false;

QueueHandle_t app_queue;

esp_lcd_panel_handle_t Screen_Mipi_Dpi_Panel = NULL;

// IIC 1
auto XL9535_IIC_Bus = std::make_shared<Cpp_Bus_Driver::Hardware_Iic_1>(XL9535_SDA, XL9535_SCL, I2C_NUM_0);
auto BQ27220_IIC_Bus = std::make_shared<Cpp_Bus_Driver::Hardware_Iic_1>(BQ27220_SDA, BQ27220_SCL, I2C_NUM_0);
auto PCF8563_IIC_Bus = std::make_shared<Cpp_Bus_Driver::Hardware_Iic_1>(PCF8563_SDA, PCF8563_SCL, I2C_NUM_0);

// IIC 2
auto SGM38121_IIC_Bus = std::make_shared<Cpp_Bus_Driver::Hardware_Iic_1>(SGM38121_SDA, SGM38121_SCL, I2C_NUM_1);
auto AW86224_IIC_Bus = std::make_shared<Cpp_Bus_Driver::Hardware_Iic_1>(AW86224_SDA, AW86224_SCL, I2C_NUM_1);
auto ES8311_IIC_Bus = std::make_shared<Cpp_Bus_Driver::Hardware_Iic_1>(ES8311_SDA, ES8311_SCL, I2C_NUM_1);

// IIS
auto ES8311_IIS_Bus = std::make_shared<Cpp_Bus_Driver::Hardware_Iis>(ES8311_ADC_DATA, ES8311_DAC_DATA, ES8311_WS_LRCK, ES8311_BCLK, ES8311_MCLK, I2S_NUM_0);

// UART
auto L76K_Uart_Bus = std::make_shared<Cpp_Bus_Driver::Hardware_Uart>(GPS_RX, GPS_TX, UART_NUM_1);

// SDIO
auto ESP32C6_AT_SDIO_Bus = std::make_shared<Cpp_Bus_Driver::Hardware_Sdio>(ESP32C6_SDIO_CLK, ESP32C6_SDIO_CMD,
                                                                           ESP32C6_SDIO_D0, ESP32C6_SDIO_D1, ESP32C6_SDIO_D2, ESP32C6_SDIO_D3, DEFAULT_CPP_BUS_DRIVER_VALUE,
                                                                           DEFAULT_CPP_BUS_DRIVER_VALUE, DEFAULT_CPP_BUS_DRIVER_VALUE, DEFAULT_CPP_BUS_DRIVER_VALUE,
                                                                           Cpp_Bus_Driver::Hardware_Sdio::Sdio_Port::SLOT_1);

// SPI
auto SX1262_SPI_Bus = std::make_shared<Cpp_Bus_Driver::Hardware_Spi>(SX1262_MOSI, SX1262_SCLK, SX1262_MISO, SPI2_HOST, 0);

// IIC 1
auto XL9535 = std::make_unique<Cpp_Bus_Driver::Xl95x5>(XL9535_IIC_Bus, XL9535_IIC_ADDRESS, DEFAULT_CPP_BUS_DRIVER_VALUE);
auto BQ27220 = std::make_unique<Cpp_Bus_Driver::Bq27220xxxx>(BQ27220_IIC_Bus, BQ27220_IIC_ADDRESS);
auto PCF8563 = std::make_unique<Cpp_Bus_Driver::Pcf8563x>(PCF8563_IIC_Bus, PCF8563_IIC_ADDRESS, DEFAULT_CPP_BUS_DRIVER_VALUE);

// IIC 2
auto SGM38121 = std::make_unique<Cpp_Bus_Driver::Sgm38121>(SGM38121_IIC_Bus, SGM38121_IIC_ADDRESS, DEFAULT_CPP_BUS_DRIVER_VALUE);
auto AW86224 = std::make_unique<Cpp_Bus_Driver::Aw862xx>(AW86224_IIC_Bus, AW86224_IIC_ADDRESS, DEFAULT_CPP_BUS_DRIVER_VALUE);
auto ES8311 = std::make_unique<Cpp_Bus_Driver::Es8311>(ES8311_IIC_Bus, ES8311_IIS_Bus, ES8311_IIC_ADDRESS, DEFAULT_CPP_BUS_DRIVER_VALUE);
auto ICM20948 = std::make_unique<ICM20948_WE>(&Wire1, ICM20948_IIC_ADDRESS);

// UART
auto L76K = std::make_unique<Cpp_Bus_Driver::L76k>(L76K_Uart_Bus, [](bool Value) -> IRAM_ATTR bool
                                                   { return XL9535->pin_write(XL9535_GPS_WAKE_UP, static_cast<Cpp_Bus_Driver::Xl95x5::Value>(Value)); }, DEFAULT_CPP_BUS_DRIVER_VALUE);

// SDIO
auto ESP32C6_AT = std::make_unique<Cpp_Bus_Driver::Esp_At>(ESP32C6_AT_SDIO_Bus,
                                                           [](bool value) -> IRAM_ATTR void
                                                           {
                                                               XL9535->pin_write(XL9535_ESP32C6_EN, static_cast<Cpp_Bus_Driver::Xl95x5::Value>(value));
                                                           });

// SPI
auto SX1262 = std::make_unique<Cpp_Bus_Driver::Sx126x>(SX1262_SPI_Bus, Cpp_Bus_Driver::Sx126x::Chip_Type::SX1262, SX1262_BUSY,
                                                       SX1262_CS, DEFAULT_CPP_BUS_DRIVER_VALUE);

// Screen detection globals — set by detect_screen_type() before display init
#include "screen_detect.h"
screen_type_t g_screen_type = SCREEN_TYPE_UNKNOWN;
uint32_t g_screen_width = HI8561_SCREEN_WIDTH;   // default until detected
uint32_t g_screen_height = HI8561_SCREEN_HEIGHT;

#if defined SCREEN_ROTATION_DIRECTION_0
auto System_Ui = std::make_unique<Lvgl_Ui::System>(SCREEN_WIDTH_MAX, SCREEN_HEIGHT_MAX);
#elif defined SCREEN_ROTATION_DIRECTION_90
auto System_Ui = std::make_unique<Lvgl_Ui::System>(SCREEN_HEIGHT_MAX, SCREEN_WIDTH_MAX);
#else
#error "unknown macro definition, please select the correct macro definition."
#endif

// Both touch drivers always constructed — only the detected one gets begin()
auto HI8561_T_IIC_Bus = std::make_shared<Cpp_Bus_Driver::Hardware_Iic_1>(HI8561_TOUCH_SDA, HI8561_TOUCH_SCL, I2C_NUM_0);
auto HI8561_T = std::make_unique<Cpp_Bus_Driver::Hi8561_Touch>(HI8561_T_IIC_Bus, HI8561_TOUCH_IIC_ADDRESS, DEFAULT_CPP_BUS_DRIVER_VALUE);

auto GT9895_IIC_Bus = std::make_shared<Cpp_Bus_Driver::Hardware_Iic_1>(GT9895_TOUCH_SDA, GT9895_TOUCH_SCL, I2C_NUM_0);
auto GT9895 = std::make_unique<Cpp_Bus_Driver::Gt9895>(GT9895_IIC_Bus, GT9895_IIC_ADDRESS, GT9895_X_SCALE_FACTOR, GT9895_Y_SCALE_FACTOR,
                                                       DEFAULT_CPP_BUS_DRIVER_VALUE);

#if defined CONFIG_BOARD_TYPE_T_DISPLAY_P4_KEYBOARD

enum class Nfc_Mode { TEST = 0, };
enum class Cc1101_Rf_Switch { RF_SWITCH_315MHZ, RF_SWITCH_434MHZ, RF_SWITCH_868_915MHZ, };

volatile bool TCA8418_Interrupt_Flag = false;
volatile bool Cc1101_Interrupt_Flag = false;
volatile bool Nrf24l01_Interrupt_Flag = false;

bool Device_Nfc_Task_Stop_Flag = false;
TaskHandle_t Nfc_Task_Handle = NULL;
Nfc_Mode St25r3916_Nfc_Mode = Nfc_Mode::TEST;

auto Bq25896_Dev = std::make_shared<Kode_Bq25896::bq25896_dev_t>();
Kode_Bq25896::bq25896_handle_t Bq25896_Handle = Bq25896_Dev.get();

auto XL9555_IIC_Bus = std::make_shared<Cpp_Bus_Driver::Software_Iic>(XL9555_SDA, XL9555_SCL);
auto TCA8418_IIC_Bus = std::make_shared<Cpp_Bus_Driver::Software_Iic>(TCA8418_SDA, TCA8418_SCL);
auto Bq25896_Iic_Bus = std::make_shared<Cpp_Bus_Driver::Software_Iic>(BQ25896_SDA, BQ25896_SCL);

auto Cc1101_SPI_Bus = std::make_shared<Cpp_Bus_Driver::Hardware_Spi>(T_MIXRF_CC1101_MOSI, T_MIXRF_CC1101_SCLK, T_MIXRF_CC1101_MISO, SPI2_HOST, 0);
auto Nrf24l01_SPI_Bus = std::make_shared<Cpp_Bus_Driver::Hardware_Spi>(T_MIXRF_NRF24L01_MOSI, T_MIXRF_NRF24L01_SCLK, T_MIXRF_NRF24L01_MISO, SPI2_HOST, 0);
RadioLibHal *Cc1101_Radiolib_Hal = new Radiolib_Cpp_Bus_Driver_Hal(Cc1101_SPI_Bus, 10000000, T_MIXRF_CC1101_CS);
RadioLibHal *Nrf24l01_Radiolib_Hal = new Radiolib_Cpp_Bus_Driver_Hal(Nrf24l01_SPI_Bus, 10000000, T_MIXRF_NRF24L01_CS);

auto XL9555 = std::make_unique<Cpp_Bus_Driver::Xl95x5>(XL9555_IIC_Bus, XL9555_IIC_ADDRESS, DEFAULT_CPP_BUS_DRIVER_VALUE);
auto TCA8418 = std::make_unique<Cpp_Bus_Driver::Tca8418>(TCA8418_IIC_Bus, TCA8418_IIC_ADDRESS, DEFAULT_CPP_BUS_DRIVER_VALUE);

CC1101 Cc1101 = new Module(Cc1101_Radiolib_Hal, static_cast<uint32_t>(RADIOLIB_NC),
                           static_cast<uint32_t>(RADIOLIB_NC), static_cast<uint32_t>(RADIOLIB_NC), T_MIXRF_CC1101_BUSY);
nRF24 Nrf24l01 = new Module(Nrf24l01_Radiolib_Hal, static_cast<uint32_t>(RADIOLIB_NC),
                            static_cast<uint32_t>(RADIOLIB_NC), static_cast<uint32_t>(T_MIXRF_NRF24L01_CE), static_cast<uint32_t>(RADIOLIB_NC));

auto ESP32P4 = std::make_unique<Cpp_Bus_Driver::Tool>();

#endif

#if CONFIG_ENABLE_USB_DISPLAY == true
#else
typedef struct
{
    size_t buf_len;
    uint8_t itf;
} app_message_t;
#endif

// Get local timezone offset in minutes from GPS position and UTC date.
// Uses the full tz_lookup library with 0.1° resolution grid and DST rules.
// Returns 0 (UTC) if no GPS fix or system clock not set.
#include "tz_lookup.h"

static int16_t get_tz_offset_minutes(void) {
    if (tz_is_manual()) {
        // Manual override — tz_lookup handles this internally but we
        // need the value here for direct RTC calculations.
        receiver_pos_t rx = {0};  // dummy
        struct timeval tv;
        gettimeofday(&tv, NULL);
        struct tm tm_utc;
        gmtime_r(&tv.tv_sec, &tm_utc);
        tz_result_t r = tz_lookup(0, 0, tm_utc.tm_year + 1900, tm_utc.tm_mon + 1, tm_utc.tm_mday);
        return r.total_offset_min;
    }
    receiver_pos_t rx = adsb_get_receiver_pos();
    if (!rx.fix_valid) return 0;
    struct timeval tv;
    gettimeofday(&tv, NULL);
    struct tm tm_utc;
    gmtime_r(&tv.tv_sec, &tm_utc);
    return tz_get_offset_minutes(rx.lat, rx.lon,
        tm_utc.tm_year + 1900, tm_utc.tm_mon + 1, tm_utc.tm_mday);
}

void Save_Real_Time(Cpp_Bus_Driver::Esp_At::Real_Time time)
{
    int16_t tz_off_min = get_tz_offset_minutes();

    // Compute local time from UTC for RTC and UI display
    time_t utc_epoch = 0;
    {
        struct tm tm_utc = {};
        tm_utc.tm_year = time.year - 1900;
        tm_utc.tm_mon  = time.month - 1;
        tm_utc.tm_mday = time.day;
        tm_utc.tm_hour = time.hour;
        tm_utc.tm_min  = time.minute;
        tm_utc.tm_sec  = time.second;
        utc_epoch = mktime(&tm_utc);
    }
    time_t local_epoch = utc_epoch + tz_off_min * 60;
    struct tm tm_local;
    gmtime_r(&local_epoch, &tm_local);

    Cpp_Bus_Driver::Pcf8563x::Time t =
        {
            .second = static_cast<uint8_t>(tm_local.tm_sec),
            .minute = static_cast<uint8_t>(tm_local.tm_min),
            .hour = static_cast<uint8_t>(tm_local.tm_hour),
            .day = static_cast<uint8_t>(tm_local.tm_mday),
            .week = Cpp_Bus_Driver::Pcf8563x::Week::SUNDAY,
            .month = time.month,
            .year = static_cast<uint8_t>(time.year - 2000),
        };

    if (time.week == "Sun")       t.week = Cpp_Bus_Driver::Pcf8563x::Week::SUNDAY;
    else if (time.week == "Mon")  t.week = Cpp_Bus_Driver::Pcf8563x::Week::MONDAY;
    else if (time.week == "Tue")  t.week = Cpp_Bus_Driver::Pcf8563x::Week::TUESDAY;
    else if (time.week == "Wed")  t.week = Cpp_Bus_Driver::Pcf8563x::Week::WEDNESDAY;
    else if (time.week == "Thu")  t.week = Cpp_Bus_Driver::Pcf8563x::Week::THURSDAY;
    else if (time.week == "Fri")  t.week = Cpp_Bus_Driver::Pcf8563x::Week::FRIDAY;
    else if (time.week == "Sat")  t.week = Cpp_Bus_Driver::Pcf8563x::Week::SATURDAY;

    PCF8563->set_time(t);

    // Sync the POSIX system clock so gettimeofday() returns real UTC time.
    if (utc_epoch > 0) {
        struct timeval tv_now = { .tv_sec = utc_epoch, .tv_usec = 0 };
        settimeofday(&tv_now, NULL);
    }

    // UI displays local time
    static const char *wday_names[] = {"Sun","Mon","Tue","Wed","Thu","Fri","Sat"};
    System_Ui->_time.week = wday_names[tm_local.tm_wday];
    System_Ui->_time.year = static_cast<uint16_t>(tm_local.tm_year + 1900);
    System_Ui->_time.month = static_cast<uint8_t>(tm_local.tm_mon + 1);
    System_Ui->_time.day = static_cast<uint8_t>(tm_local.tm_mday);
    System_Ui->_time.hour = static_cast<uint8_t>(tm_local.tm_hour);
    System_Ui->_time.minute = static_cast<uint8_t>(tm_local.tm_min);
    System_Ui->_time.second = static_cast<uint8_t>(tm_local.tm_sec);
    System_Ui->_time.time_zone = time.time_zone;
}

bool Play_Wav_File(const char *file_path)
{
    Music_File.open(file_path, std::ios::binary);

    if (Music_File.is_open() == false)
    {
        printf("failed to open wav file: %s\n", file_path);
        return false;
    }

    Wav_Header wav_header;
    if (!Music_File.read(reinterpret_cast<char *>(&wav_header), sizeof(wav_header)))
    {
        printf("failed to read wav header\n");
        Music_File.close();
        return false;
    }

    if (strncmp(wav_header.riff_header, "RIFF", 4) != 0)
        printf("invalid wav file format: riff_header is not 'RIFF'\n");
    else if (strncmp(wav_header.wave_header, "WAVE", 4) != 0)
        printf("invalid wav file format: wave_header is not 'WAVE'\n");
    else if (strncmp(wav_header.fmt_header, "fmt ", 4) != 0)
        printf("invalid wav file format: fmt_header is not 'fmt '\n");
    else if (strncmp(wav_header.data_header, "data", 4) != 0)
        printf("invalid wav file format: data_header is not 'data'\n");

    printf("sample rate: %ld\n", wav_header.sample_rate);
    printf("channels: %d\n", wav_header.num_channel);
    printf("bits per sample: %d\n", wav_header.bits_per_sample);
    printf("data_size: %ld\n", wav_header.data_size);

    if (wav_header.sample_rate != SAMPLE_RATE ||
        wav_header.num_channel != NUM_CHANNEL ||
        wav_header.bits_per_sample != BITS_PER_SAMPLE)
    {
        printf("wav file parameters do not match i2s configuration\n");
        Music_File.close();
        return false;
    }

    double duration = 0.0;
    if (wav_header.sample_rate > 0 && wav_header.num_channel > 0 && wav_header.bits_per_sample > 0)
        duration = static_cast<double>(wav_header.data_size) / (wav_header.sample_rate * wav_header.num_channel * (wav_header.bits_per_sample / 8.0));

    printf("duration: %.2f s\n", duration);

    _lock_acquire(&lvgl_api_lock);
    System_Ui->set_win_music_current_total_time(0, duration);
    _lock_release(&lvgl_api_lock);

    size_t cycle_time = 0;

    Iis_Transmission_Data_Stream.clear();
    Iis_Read_Data_Size_Index = 0;

    vTaskResume(Iis_Transmission_Data_Stream_Task);

    while (Music_File.good())
    {
        if (Music_Play_End_Flag == true)
            break;

        if (ES8311_Speaker_Mode == Es8311_Mode::TEST)
        {
            ES8311->write_data(c2_b16_s44100, sizeof(c2_b16_s44100));
            ES8311_Speaker_Mode = Es8311_Mode::PLAY_MUSIC;
        }

        if (Set_Music_Current_Time_S_Flag == true)
        {
            printf("music play set current time: %.2f s\n", Set_Music_Current_Time_S);
            size_t bytes_per_frame = wav_header.num_channel * (wav_header.bits_per_sample / 8);
            std::streamoff seek_offset = static_cast<std::streamoff>(Set_Music_Current_Time_S * wav_header.sample_rate) * bytes_per_frame;
            Music_File.seekg(sizeof(wav_header) + seek_offset, std::ios::beg);
            Iis_Transmission_Data_Stream.clear();
            Iis_Read_Data_Size_Index = 0;
            Set_Music_Current_Time_S_Flag = false;
        }

        if (System_Ui->_registry.win.music.play_flag == true)
        {
            if (System_Ui->_current_win == Lvgl_Ui::System::Current_Win::MUSIC)
            {
                if (esp_log_timestamp() > cycle_time)
                {
                    std::streamoff current_pos = Music_File.tellg();
                    double current_time = 0.0;
                    if (current_pos > 0)
                    {
                        std::streamoff data_offset = current_pos - sizeof(wav_header);
                        current_time = static_cast<double>(data_offset) / (wav_header.sample_rate * wav_header.num_channel * (wav_header.bits_per_sample / 8.0));
                        _lock_acquire(&lvgl_api_lock);
                        System_Ui->set_win_music_current_total_time(current_time, duration);
                        _lock_release(&lvgl_api_lock);
                    }
                    printf("music play current time: %.2f s\n", current_time);
                    cycle_time = esp_log_timestamp() + 1000;
                }
            }

            if (Iis_Transmission_Data_Stream.size() > 1024 * 10)
            {
                size_t bytes_read = ES8311->write_data(Iis_Transmission_Data_Stream.data() + Iis_Read_Data_Size_Index, 1024 * 10);
                Iis_Read_Data_Size_Index += bytes_read;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    vTaskSuspend(Iis_Transmission_Data_Stream_Task);
    Iis_Transmission_Data_Stream.clear();
    Iis_Transmission_Data_Stream.shrink_to_fit();

    Music_File.close();

    System_Ui->_registry.win.music.play_flag = false;

    if (Music_Play_End_Flag == false)
    {
        printf("music play finish\n");
        _lock_acquire(&lvgl_api_lock);
        System_Ui->set_win_music_play_imagebutton_status(System_Ui->_registry.win.music.play_flag);
        System_Ui->set_win_music_current_total_time(0, duration);
        _lock_release(&lvgl_api_lock);
    }
    else
    {
        printf("music play end\n");
        Music_Play_End_Flag = false;
    }

    return true;
}

void lvgl_ui_task(void *arg)
{
    printf("lvgl_ui_task start\n");
    uint32_t time_till_next_ms = 0;
    uint32_t tick_count = 0;

    while (1)
    {
        _lock_acquire(&lvgl_api_lock);
        time_till_next_ms = lv_timer_handler();
        _lock_release(&lvgl_api_lock);

        if (time_till_next_ms < 10)
            time_till_next_ms = 10;

        // Temporary: print every 5 seconds to confirm handler is running
        if (++tick_count % 500 == 0)
            printf("lvgl tick %lu, next_ms=%lu\n", tick_count, time_till_next_ms);

        usleep(1000 * time_till_next_ms);
    }
}

void device_vibration_task(void *arg)
{
    printf("device_vibration_task start\n");
    vTaskSuspend(Vibration_Task_Handle);

    while (1)
    {
        if (AW86224_Vibration_Play_Count == static_cast<uint8_t>(-1))
        {
            uint8_t timeout_count = 0;
            uint32_t f0_value = 0;
            bool f0_detection_result = false;

            while (1)
            {
                f0_value = AW86224->get_f0_detection();
                printf("AW86224 get f0 detection value: %ld\n", f0_value);

                if (AW86224->set_f0_calibrate(f0_value) == true)
                {
                    f0_detection_result = true;
                    break;
                }
                else
                {
                    if (f0_value > AW86224->_f0_value)
                    {
                        if ((f0_value - AW86224->_f0_value) <= 1500)
                        {
                            f0_detection_result = true;
                            AW86224->_f0_value = f0_value;
                            break;
                        }
                    }
                    else
                    {
                        if ((AW86224->_f0_value - f0_value) <= 1500)
                        {
                            f0_detection_result = true;
                            AW86224->_f0_value = f0_value;
                            break;
                        }
                    }
                }

                timeout_count++;
                if (timeout_count > 5)
                {
                    printf("AW86224 get f0 detection fail\n");
                    f0_detection_result = false;
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(10));
            }

            std::string vibration_data_str = "vibration data:\n";
            vibration_data_str += "f0 value: " + std::to_string(f0_value) + "\n";

            if (System_Ui->get_current_win() == Lvgl_Ui::System::Current_Win::CIT_VIBRATION_TEST)
            {
                _lock_acquire(&lvgl_api_lock);
                if (f0_detection_result == false)
                {
                    vibration_data_str += "result: fail\n";
                    lv_obj_set_style_text_color(System_Ui->_registry.win.cit.vibration_test.data_label, lv_color_hex(0xEE2C2C), LV_PART_MAIN);
                }
                else
                {
                    vibration_data_str += "result: success\n";
                    lv_obj_set_style_text_color(System_Ui->_registry.win.cit.vibration_test.data_label, lv_color_hex(0x008B45), LV_PART_MAIN);
                }
                lv_label_set_text(System_Ui->_registry.win.cit.vibration_test.data_label, vibration_data_str.c_str());
                _lock_release(&lvgl_api_lock);
            }

            AW86224_Vibration_Play_Count = 0;
        }
        else if (AW86224_Vibration_Play_Count > 0)
        {
            AW86224->run_ram_playback_waveform(1, 15, 255);
            vTaskDelay(pdMS_TO_TICKS(50));
            AW86224->stop_ram_playback_waveform();
            AW86224_Vibration_Play_Count--;
        }
        else
        {
            vTaskSuspend(Vibration_Task_Handle);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void device_speaker_task(void *arg)
{
    printf("device_speaker_task start\n");
    vTaskSuspend(Speaker_Task_Handle);

    while (1)
    {
        switch (ES8311_Speaker_Mode)
        {
        case Es8311_Mode::TEST:
            ES8311->write_data(c2_b16_s44100, sizeof(c2_b16_s44100));
            break;
        case Es8311_Mode::PLAY_MUSIC:
            Play_Wav_File(SD_FILE_PATH_MUSIC);
            break;
        default:
            break;
        }

        vTaskSuspend(Speaker_Task_Handle);
    }
}

void device_microphone_task(void *arg)
{
    printf("device_microphone_task start\n");
    vTaskSuspend(Microphone_Task_Handle);

    size_t cycle_time = 0;

    while (1)
    {
        switch (ES8311_Microphone_Mode)
        {
        case Es8311_Mode::TEST:
        {
            if (esp_log_timestamp() > cycle_time)
            {
                int16_t microphone_data[1] = {0};
                ES8311->read_data(microphone_data, 1 * sizeof(int16_t));

                if (microphone_data[0] < 0)
                    continue;

                int16_t max_microphone_data = microphone_data[0];
                int16_t max_microphone_data_2 = microphone_data[0];

                if (max_microphone_data >= 1000)
                    max_microphone_data_2 = 1000;
                uint8_t max_microphone_data_percentage = (static_cast<float>(max_microphone_data_2) / static_cast<float>(1000)) * 100;

                std::string microphone_data_str = "microphone data: " + std::to_string(max_microphone_data);

                _lock_acquire(&lvgl_api_lock);
                lv_anim_t anim;
                lv_anim_init(&anim);
                lv_anim_set_var(&anim, System_Ui->_registry.win.cit.microphone_test.needle_line);
                lv_anim_set_values(&anim, System_Ui->_registry.win.cit.microphone_test.data.value_percentage, max_microphone_data_percentage);
                lv_anim_set_time(&anim, 300);
                lv_anim_set_exec_cb(&anim, [](void *needle, int32_t value)
                                    { lv_scale_set_line_needle_value(System_Ui->_registry.win.cit.microphone_test.scale_line,
                                                                     (lv_obj_t *)needle, 150, value); });
                lv_anim_start(&anim);
                lv_label_set_text(System_Ui->_registry.win.cit.microphone_test.data.label, microphone_data_str.c_str());
                _lock_release(&lvgl_api_lock);

                System_Ui->_registry.win.cit.microphone_test.data.value_percentage = max_microphone_data_percentage;
                cycle_time = esp_log_timestamp() + 300;
            }
        }
        break;

        default:
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void device_imu_task(void *arg)
{
    printf("device_imu_task start\n");
    vTaskSuspend(Imu_Task_Handle);

    size_t cycle_time = 0;

    while (1)
    {
        switch (ICM20948_Imu_Mode)
        {
        case Imu_Mode::TEST:
        {
            if (esp_log_timestamp() > cycle_time)
            {
                ICM20948->readSensor();
                xyzFloat gValue;
                ICM20948->getGValues(&gValue);
                xyzFloat angle;
                ICM20948->getAngles(&angle);
                float pitch = ICM20948->getPitch();
                float roll = ICM20948->getRoll();

                xyzFloat magValues;
                ICM20948->getMagValues(&magValues);
                float yaw = atan2(magValues.y, magValues.x) * (180.0 / M_PI);

                std::string imu_data_str = "imu data:\n";
                imu_data_str += "gyroscope:\nx: " + std::to_string(gValue.x) + "\ny: " + std::to_string(gValue.y) + "\nz:  " + std::to_string(gValue.z) + "\n\n";
                imu_data_str += "accelerometer:\nx: " + std::to_string(angle.x) + "\ny: " + std::to_string(angle.y) + "\nz: " + std::to_string(angle.z) + "\n\n";
                imu_data_str += "magnetometer:\nx: " + std::to_string(magValues.x) + "\ny: " + std::to_string(magValues.y) + "\nz: " + std::to_string(magValues.z) + "\n\n";
                imu_data_str += "euler angles:\npitch: " + std::to_string(pitch) + "\nroll: " + std::to_string(roll) + "\nyaw: " + std::to_string(yaw);

                _lock_acquire(&lvgl_api_lock);
                lv_label_set_text(System_Ui->_registry.win.cit.imu_test.data_label, imu_data_str.c_str());
                _lock_release(&lvgl_api_lock);

                cycle_time = esp_log_timestamp() + 100;
            }
        }
        break;

        default:
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void device_battery_health_task(void *arg)
{
    printf("device_battery_health_task start\n");

    size_t cycle_time = 0;

    while (1)
    {
        if (esp_log_timestamp() > cycle_time)
        {
            uint16_t battery_level = BQ27220->get_status_of_charge();

            System_Ui->set_battery_level(battery_level);

            // Update all status bar indicators (runs every 1s)
            {
                // GPS status
                receiver_pos_t rx = adsb_get_receiver_pos();
                System_Ui->set_gps_status(rx.fix_valid, rx.sats);

                // ADS-B status
                adsb_stats_t stats = adsb_get_stats();
                System_Ui->set_adsb_status(stats.rtlsdr_connected, stats.active_aircraft);

                // SD card status
                extern bool sd_is_mounted(void);
                extern bool sd_is_logging(void);
                System_Ui->set_sd_status(sd_is_mounted(), sd_is_logging());
            }

            _lock_acquire(&lvgl_api_lock);
            System_Ui->status_bar_battery_level_update();
            System_Ui->status_bar_gps_update();
            System_Ui->status_bar_adsb_update();
            System_Ui->status_bar_sd_update();
            _lock_release(&lvgl_api_lock);

            switch (System_Ui->get_current_win())
            {
            case Lvgl_Ui::System::Current_Win::CIT_BATTERY_HEALTH_TEST:
            {
                std::string battery_health_data_str = "battery health data:\n\n";
                battery_health_data_str += "bq27220 data:\n";
                battery_health_data_str += "device id: " + std::to_string(BQ27220->get_device_id()) + "\n\n";
                battery_health_data_str += "design capacity: " + std::to_string(BQ27220->get_design_capacity()) + " mah\n";
                battery_health_data_str += "remaining capacity: " + std::to_string(BQ27220->get_remaining_capacity()) + " mah\n";
                battery_health_data_str += "full charge capacity: " + std::to_string(BQ27220->get_full_charge_capacity()) + " mah\n\n";
                battery_health_data_str += "battery level: " + std::to_string(battery_level) + "%\n";
                battery_health_data_str += "battery health: " + std::to_string(BQ27220->get_status_of_charge()) + "%\n\n";
                battery_health_data_str += "voltage: " + std::to_string(BQ27220->get_voltage()) + " mv\n";
                battery_health_data_str += "current: " + std::to_string(BQ27220->get_current()) + " ma\n";
                battery_health_data_str += "chip temperature: " + std::to_string(BQ27220->get_chip_temperature_celsius()) + " °c\n\n";

                Cpp_Bus_Driver::Bq27220xxxx::Battery_Status bs;
                if (BQ27220->get_battery_status(bs) == true)
                {
                    battery_health_data_str += "sleep flag: " + std::to_string(bs.flag.sleep) + "\n";
                    battery_health_data_str += "discharge flag: " + std::to_string(bs.flag.dsg) + "\n";
                }

#if defined CONFIG_BOARD_TYPE_T_DISPLAY_P4_KEYBOARD
                battery_health_data_str += "\nbq25896 data:\n";
                uint8_t part_number = 0;
                Kode_Bq25896::bq25896_get_part_number(Bq25896_Handle, &part_number);
                battery_health_data_str += "device id: " + std::to_string(part_number) + "\n\n";

                Kode_Bq25896::bq25896_vbus_stat_t vbus_stat;
                Kode_Bq25896::bq25896_get_vbus_status(Bq25896_Handle, &vbus_stat);
                switch (vbus_stat)
                {
                case Kode_Bq25896::BQ25896_VBUS_STAT_NO_INPUT:    battery_health_data_str += "vbus status: no input\n"; break;
                case Kode_Bq25896::BQ25896_VBUS_STAT_USB_HOST:    battery_health_data_str += "vbus status: usb host sdp\n"; break;
                case Kode_Bq25896::BQ25896_VBUS_STAT_ADAPTER:     battery_health_data_str += "vbus status: adapter (3.25a)\n"; break;
                case Kode_Bq25896::BQ25896_VBUS_STAT_OTG:         battery_health_data_str += "vbus status: otg\n"; break;
                default:                                           battery_health_data_str += "vbus status: unknown\n"; break;
                }

                Kode_Bq25896::bq25896_chrg_stat_t chrg_stat;
                Kode_Bq25896::bq25896_get_charging_status(Bq25896_Handle, &chrg_stat);
                switch (chrg_stat)
                {
                case Kode_Bq25896::BQ25896_CHRG_STAT_NOT_CHARGING:  battery_health_data_str += "charging status: not charging\n"; break;
                case Kode_Bq25896::BQ25896_CHRG_STAT_PRE_CHARGE:    battery_health_data_str += "charging status: pre charge\n"; break;
                case Kode_Bq25896::BQ25896_CHRG_STAT_FAST_CHARGING: battery_health_data_str += "charging status: fast charging\n"; break;
                case Kode_Bq25896::BQ25896_CHRG_STAT_TERM_DONE:     battery_health_data_str += "charging status: done charging\n"; break;
                default:                                             battery_health_data_str += "charging status: unknown\n"; break;
                }

                battery_health_data_str += "\n";

                uint16_t bat_voltage = 0, sys_voltage = 0, vbus_voltage = 0;
                Kode_Bq25896::bq25896_get_battery_voltage(Bq25896_Handle, &bat_voltage);
                Kode_Bq25896::bq25896_get_system_voltage(Bq25896_Handle, &sys_voltage);
                Kode_Bq25896::bq25896_get_vbus_voltage(Bq25896_Handle, &vbus_voltage);

                battery_health_data_str += "battery voltage: " + std::to_string(bat_voltage) + "mv\n";
                battery_health_data_str += "system voltage: " + std::to_string(sys_voltage) + "mv\n";
                battery_health_data_str += "vbus voltage: " + std::to_string(vbus_voltage) + "mv\n\n";

                uint16_t charge_current = 0, ico_current_limit = 0;
                Kode_Bq25896::bq25896_get_charge_current(Bq25896_Handle, &charge_current);
                Kode_Bq25896::bq25896_get_ico_current_limit(Bq25896_Handle, &ico_current_limit);

                battery_health_data_str += "charge current: " + std::to_string(charge_current) + "ma\n";
                battery_health_data_str += "ico current limit: " + std::to_string(ico_current_limit) + "ma\n";
#endif

                _lock_acquire(&lvgl_api_lock);
                lv_label_set_text(System_Ui->_registry.win.cit.battery_health_test.data_label, battery_health_data_str.c_str());
                lv_obj_align(System_Ui->_registry.win.cit.battery_health_test.data_label, LV_ALIGN_TOP_MID, 0, 10);
#if defined CONFIG_BOARD_TYPE_T_DISPLAY_P4_KEYBOARD
                if ((vbus_stat == Kode_Bq25896::BQ25896_VBUS_STAT_ADAPTER) || (vbus_stat == Kode_Bq25896::BQ25896_VBUS_STAT_USB_HOST))
                {
                    lv_obj_add_flag(System_Ui->_registry.win.cit.battery_health_test.otg_label, LV_OBJ_FLAG_HIDDEN);
                    lv_obj_add_flag(System_Ui->_registry.win.cit.battery_health_test.otg_switch, LV_OBJ_FLAG_HIDDEN);
                }
                else
                {
                    lv_obj_remove_flag(System_Ui->_registry.win.cit.battery_health_test.otg_label, LV_OBJ_FLAG_HIDDEN);
                    lv_obj_remove_flag(System_Ui->_registry.win.cit.battery_health_test.otg_switch, LV_OBJ_FLAG_HIDDEN);
                    lv_obj_align_to(System_Ui->_registry.win.cit.battery_health_test.otg_label,
                                    System_Ui->_registry.win.cit.battery_health_test.data_label, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);
                    lv_obj_align_to(System_Ui->_registry.win.cit.battery_health_test.otg_switch,
                                    System_Ui->_registry.win.cit.battery_health_test.otg_label, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);
                }
#endif
                _lock_release(&lvgl_api_lock);
            }
            break;

            default:
                break;
            }

            cycle_time = esp_log_timestamp() + 1000;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void device_gps_task(void *arg)
{
    printf("device_gps_task start\n");
    // Task runs continuously — no vTaskSuspend.
    // GPS is kept awake at boot to always feed ADS-B receiver position.

    size_t cycle_time = 0;
    size_t last_summary_time = 0;  // for cooked output every ~5s

    while (1)
    {
        if (esp_log_timestamp() > cycle_time)
        {
            std::unique_ptr<uint8_t[]> buffer;
            uint32_t buffer_length = 0;

            if (L76K->get_info_data(buffer, &buffer_length) == true)
            {
                Cpp_Bus_Driver::L76k::Rmc rmc;

                if (L76K->parse_rmc_info(buffer.get(), buffer_length, rmc) == true)
                {
                    // Parse GGA every cycle for sat count, HDOP, altitude
                    Cpp_Bus_Driver::L76k::Gga gga;
                    bool gga_ok = L76K->parse_gga_info(buffer.get(), buffer_length, gga);

                    // ------------------------------------------------
                    // Always feed position to ADS-B when we have a fix
                    // ------------------------------------------------
                    bool has_pos = rmc.location.lat.update_flag &&
                                   rmc.location.lat.direction_update_flag &&
                                   rmc.location.lon.update_flag &&
                                   rmc.location.lon.direction_update_flag;

                    if (has_pos)
                    {
                        L76k_Gps_Positioning_Flag = true;

                        double lat = rmc.location.lat.degrees_minutes;
                        if (rmc.location.lat.direction == "S") lat = -lat;

                        double lon = rmc.location.lon.degrees_minutes;
                        if (rmc.location.lon.direction == "W") lon = -lon;

                        adsb_set_receiver_pos(lat, lon, 0.0,
                            gga_ok ? gga.online_satellite_count : 0,
                            gga_ok ? (double)gga.hdop : 99.9,
                            gga_ok ? gga.gps_mode_status : 0);
                    }

                    // ------------------------------------------------
                    // GPS → system clock + RTC sync
                    // Priority: RTC < GPS serial << WiFi NTP
                    // GPS serial has only whole-second resolution (no PPS
                    // on this board — L76K pin 5 is RESERVED / not routed),
                    // plus ~500ms serial latency.  Only correct when drift
                    // exceeds 1s to avoid oscillation from sub-second jitter.
                    // NTP (when available) keeps <50ms so GPS won't interfere.
                    // ------------------------------------------------
                    static size_t last_rtc_write = 0;

                    if (rmc.location_status == "A" &&
                        rmc.utc.update_flag &&
                        rmc.data.update_flag)
                    {
                        int utc_year = rmc.data.year + 2000;
                        int utc_mon  = rmc.data.month;
                        int utc_day  = rmc.data.day;
                        int utc_hour = rmc.utc.hour;
                        int utc_min  = rmc.utc.minute;
                        int utc_sec  = static_cast<int>(rmc.utc.second);

                        struct tm tm_gps = {};
                        tm_gps.tm_year = utc_year - 1900;
                        tm_gps.tm_mon  = utc_mon - 1;
                        tm_gps.tm_mday = utc_day;
                        tm_gps.tm_hour = utc_hour;
                        tm_gps.tm_min  = utc_min;
                        tm_gps.tm_sec  = utc_sec;
                        time_t gps_epoch = mktime(&tm_gps);  // ESP32 default TZ is UTC

                        if (gps_epoch > 1704067200) {  // sanity: after 2024-01-01
                            // The RMC timestamp is the actual UTC fix time, but
                            // arrives ~650ms later over serial (UART TX + library
                            // buffering).  Compensate so the system clock is set
                            // to (gps_epoch + 650ms) rather than exactly on the
                            // second boundary.  Measured from observed jitter:
                            // serial delay ranges 530–780ms, median ~650ms.
                            static const long GPS_SERIAL_DELAY_US = 650000;  // 650ms

                            struct timeval tv_sys;
                            gettimeofday(&tv_sys, NULL);
                            long long drift_s = (long long)(gps_epoch - tv_sys.tv_sec);

                            // Only step if off by more than 1 second
                            if (drift_s > 1 || drift_s < -1) {
                                struct timeval tv_now = {
                                    .tv_sec = gps_epoch,
                                    .tv_usec = GPS_SERIAL_DELAY_US
                                };
                                settimeofday(&tv_now, NULL);
                                serial_console_print("[GNSS] Clock corrected by %+llds → %04d-%02d-%02d %02d:%02d:%02d.%03ld UTC\n",
                                    drift_s, utc_year, utc_mon, utc_day,
                                    utc_hour, utc_min, utc_sec,
                                    GPS_SERIAL_DELAY_US / 1000);

                                // Rename boot-numbered SD log to UTC timestamp
                                static bool log_renamed = false;
                                if (!log_renamed) {
                                    sd_log_rename_with_time();
                                    log_renamed = true;
                                }
                            }

                            // --- Set PCF8563 RTC at first fix, then every 60s ---
                            // RTC stores local time (offset from tz_lookup)
                            size_t now_ms_clk = esp_log_timestamp();
                            if (!last_rtc_write || (now_ms_clk > last_rtc_write + 60000)) {
                                int16_t tz_off_min = get_tz_offset_minutes();
                                time_t epoch_local = gps_epoch + tz_off_min * 60;
                                struct tm tm_local;
                                gmtime_r(&epoch_local, &tm_local);

                                static const Cpp_Bus_Driver::Pcf8563x::Week wday_map[] = {
                                    Cpp_Bus_Driver::Pcf8563x::Week::SUNDAY,
                                    Cpp_Bus_Driver::Pcf8563x::Week::MONDAY,
                                    Cpp_Bus_Driver::Pcf8563x::Week::TUESDAY,
                                    Cpp_Bus_Driver::Pcf8563x::Week::WEDNESDAY,
                                    Cpp_Bus_Driver::Pcf8563x::Week::THURSDAY,
                                    Cpp_Bus_Driver::Pcf8563x::Week::FRIDAY,
                                    Cpp_Bus_Driver::Pcf8563x::Week::SATURDAY,
                                };

                                Cpp_Bus_Driver::Pcf8563x::Time t = {
                                    .second = static_cast<uint8_t>(tm_local.tm_sec),
                                    .minute = static_cast<uint8_t>(tm_local.tm_min),
                                    .hour   = static_cast<uint8_t>(tm_local.tm_hour),
                                    .day    = static_cast<uint8_t>(tm_local.tm_mday),
                                    .week   = wday_map[tm_local.tm_wday],
                                    .month  = static_cast<uint8_t>(tm_local.tm_mon + 1),
                                    .year   = static_cast<uint8_t>(tm_local.tm_year + 1900 - 2000),
                                };
                                PCF8563->set_time(t);

                                serial_console_print("[GNSS] RTC set: %04d-%02d-%02d %02d:%02d:%02d UTC%+d:%02d\n",
                                    tm_local.tm_year + 1900, tm_local.tm_mon + 1, tm_local.tm_mday,
                                    tm_local.tm_hour, tm_local.tm_min, tm_local.tm_sec,
                                    tz_off_min / 60, abs(tz_off_min) % 60);

                                last_rtc_write = now_ms_clk;
                            }
                        }
                    }

                    // ------------------------------------------------
                    // Periodic cooked GPS summary (every 5s)
                    // ------------------------------------------------
                    size_t now_ms = esp_log_timestamp();
                    if (now_ms > last_summary_time) {
                        double lat = 0.0, lon = 0.0;
                        if (has_pos) {
                            lat = rmc.location.lat.degrees_minutes;
                            if (rmc.location.lat.direction == "S") lat = -lat;
                            lon = rmc.location.lon.degrees_minutes;
                            if (rmc.location.lon.direction == "W") lon = -lon;
                        }

                        int sats   = gga_ok ? gga.online_satellite_count : 0;
                        double hdop = gga_ok ? (double)gga.hdop : 99.9;
                        int fix_q  = gga_ok ? gga.gps_mode_status : -1;

                        serial_console_print("[GNSS] fix=%s lat=%.6f lon=%.6f sats=%d hdop=%.1f q=%d\n",
                            rmc.location_status.c_str(), lat, lon, sats, hdop, fix_q);

                        last_summary_time = now_ms + 5000;
                    }

                    // ------------------------------------------------
                    // Update CIT GPS test label when that screen is up
                    // ------------------------------------------------
                    switch (L76k_Gps_Mode)
                    {
                    case Gps_Mode::TEST:
                    {
                        std::string rmc_data_str = "";
                        if (L76k_Gps_Positioning_Flag == false)
                        {
                            L76k_Gps_Positioning_Time++;
                            rmc_data_str = "getting location time: " + std::to_string(L76k_Gps_Positioning_Time) + " s\n\n";
                        }
                        else
                        {
                            rmc_data_str = "location found time: " + std::to_string(L76k_Gps_Positioning_Time) + " s\n\n";
                        }

                        rmc_data_str += "gps data:\nrmc data:\nlocation status: " + rmc.location_status + "\n\n";

                        if (rmc.data.update_flag == true)
                        {
                            rmc_data_str += "utc data: " + std::to_string(rmc.data.year + 2000) + "/" + std::to_string(rmc.data.month) + "/" + std::to_string(rmc.data.day) + "\n";
                            rmc.data.update_flag = false;
                        }
                        if (rmc.utc.update_flag == true)
                        {
                            rmc_data_str += "utc time: " + std::to_string(rmc.utc.hour) + ":" + std::to_string(rmc.utc.minute) + ":" + std::to_string(static_cast<uint8_t>(rmc.utc.second)) + "\n";
                            {
                                int16_t tz_m = get_tz_offset_minutes();
                                int local_h = ((int)rmc.utc.hour * 60 + (int)rmc.utc.minute + tz_m + 1440) / 60 % 24;
                                int local_min = ((int)rmc.utc.hour * 60 + (int)rmc.utc.minute + tz_m + 1440) % 60;
                                rmc_data_str += "local time: " + std::to_string(local_h) + ":" + std::to_string(local_min) + ":" + std::to_string(static_cast<uint8_t>(rmc.utc.second)) + "\n";
                            }
                            rmc.utc.update_flag = false;
                        }

                        rmc_data_str += "\n";

                        if ((rmc.location.lat.update_flag == true) && (rmc.location.lat.direction_update_flag == true))
                        {
                            rmc_data_str += "lat degrees: " + std::to_string(rmc.location.lat.degrees) + "\n";
                            rmc_data_str += "lat minutes: " + std::to_string(rmc.location.lat.minutes) + "\n";
                            rmc_data_str += "lat degrees_minutes: " + std::to_string(rmc.location.lat.degrees_minutes) + "\n";
                            rmc_data_str += "lat direction: " + rmc.location.lat.direction + "\n";
                            rmc.location.lat.update_flag = false;
                            rmc.location.lat.direction_update_flag = false;
                        }

                        rmc_data_str += "\n";

                        if ((rmc.location.lon.update_flag == true) && (rmc.location.lon.direction_update_flag == true))
                        {
                            rmc_data_str += "lon degrees: " + std::to_string(rmc.location.lon.degrees) + "\n";
                            rmc_data_str += "lon minutes: " + std::to_string(rmc.location.lon.minutes) + "\n";
                            rmc_data_str += "lon degrees_minutes: " + std::to_string(rmc.location.lon.degrees_minutes) + "\n";
                            rmc_data_str += "lon direction: " + rmc.location.lon.direction + "\n";
                            rmc.location.lon.update_flag = false;
                            rmc.location.lon.direction_update_flag = false;
                        }

                        _lock_acquire(&lvgl_api_lock);
                        if (System_Ui->_registry.win.cit.gps_test.data_label)
                            lv_label_set_text(System_Ui->_registry.win.cit.gps_test.data_label, rmc_data_str.c_str());
                        _lock_release(&lvgl_api_lock);
                    }
                    break;

                    default:
                        break;
                    }
                }
            }

            cycle_time = esp_log_timestamp() + 1000;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void device_ethernet_task(void *arg)
{
    printf("device_ethernet_task start\n");
    vTaskSuspend(Ethernet_Task_Handle);

    size_t cycle_time = 0;

    while (1)
    {
        switch (Ip101gri_Ethernet_Mode)
        {
        case Ethernet_Mode::TEST:
        {
            if (esp_log_timestamp() > cycle_time)
            {
                if (Eth_Info.status.update_flag == true)
                {
                    std::string ethernet_data_str = "ethernet data:\n" + Eth_Info.status.data + "\n";
                    _lock_acquire(&lvgl_api_lock);
                    lv_label_set_text(System_Ui->_registry.win.cit.ethernet_test.data_label, ethernet_data_str.c_str());
                    _lock_release(&lvgl_api_lock);
                    Eth_Info.status.update_flag = false;
                }

                if (Eth_Info.connect_ip_status.update_flag == true)
                {
                    if (Eth_Info.link_up_flag == true)
                    {
                        std::string ethernet_data_str = "ethernet data:\n" + Eth_Info.status.data + "\n" + Eth_Info.connect_ip_status.data;
                        _lock_acquire(&lvgl_api_lock);
                        lv_label_set_text(System_Ui->_registry.win.cit.ethernet_test.data_label, ethernet_data_str.c_str());
                        _lock_release(&lvgl_api_lock);
                    }
                    Eth_Info.connect_ip_status.update_flag = false;
                }

                cycle_time = esp_log_timestamp() + 1000;
            }
        }
        break;

        default:
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void device_rtc_task(void *arg)
{
    printf("device_rtc_task start\n");

    size_t cycle_time = 0;

    while (1)
    {
        if (esp_log_timestamp() > cycle_time)
        {
            Cpp_Bus_Driver::Pcf8563x::Time t;
            if (PCF8563->get_time(t) == true)
            {
                printf("pcf8563 year:[%d] month:[%d] day:[%d] time:[%d:%d:%d] week:[%d]\n", t.year, t.month, t.day,
                       t.hour, t.minute, t.second, static_cast<uint8_t>(t.week));

                System_Ui->set_time(t);

                _lock_acquire(&lvgl_api_lock);
                System_Ui->status_bar_time_update();
                _lock_release(&lvgl_api_lock);

                switch (System_Ui->get_current_win())
                {
                case Lvgl_Ui::System::Current_Win::CIT_RTC_TEST:
                {
                    std::string rtc_data_str = "rtc data:\n";
                    char buffer[100];
                    snprintf(buffer, sizeof(buffer), "week:[%s]\ndata: [%d/%d/%d]\ntime: [%02d:%02d:%02d]\n",
                             System_Ui->_time.week.c_str(), System_Ui->_time.year, System_Ui->_time.month, System_Ui->_time.day,
                             System_Ui->_time.hour, System_Ui->_time.minute, System_Ui->_time.second);
                    rtc_data_str += buffer;

                    _lock_acquire(&lvgl_api_lock);
                    lv_label_set_text(System_Ui->_registry.win.cit.rtc_test.data_label, rtc_data_str.c_str());
                    _lock_release(&lvgl_api_lock);
                }
                break;
                case Lvgl_Ui::System::Current_Win::HOME:
                    _lock_acquire(&lvgl_api_lock);
                    System_Ui->win_home_time_update();
                    _lock_release(&lvgl_api_lock);
                    break;

                default:
                    break;
                }
            }
            else
            {
                printf("pcf8563 integrity of the clock information is not guaranteed\n");

                if (System_Ui->get_current_win() == Lvgl_Ui::System::Current_Win::CIT_RTC_TEST)
                {
                    _lock_acquire(&lvgl_api_lock);
                    lv_label_set_text(System_Ui->_registry.win.cit.rtc_test.data_label,
                                      "rtc data:\npcf8563 integrity of the clock\ninformation is not guaranteed\n");
                    _lock_release(&lvgl_api_lock);
                }

                PCF8563->clear_clock_integrity_flag();
            }

            cycle_time = esp_log_timestamp() + 1000;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void device_at_task(void *arg)
{
    printf("device_at_task start\n");
    vTaskSuspend(At_Task_Handle);

    size_t cycle_time = 0;

    while (1)
    {
        switch (Esp32c6_At_Mode)
        {
        case At_Mode::TEST:
        {
            if (esp_log_timestamp() > cycle_time)
            {
                // ADS-B status display (repurposed from AT test)
                adsb_stats_t stats = adsb_get_stats();
                receiver_pos_t rx = adsb_get_receiver_pos();

                std::string status_str = "ADS-B Receiver Status\n\n";

                // RTL-SDR connection
                status_str += "RTL-SDR: ";
                status_str += stats.rtlsdr_connected ? "[OK] connected\n" : "[--] not connected\n";

                // Message stats
                char buf[128];
                snprintf(buf, sizeof(buf), "\nMessages: %lu\nRate: %.1f msg/s\n",
                    (unsigned long)stats.total_messages, stats.msg_rate);
                status_str += buf;

                // Aircraft count
                snprintf(buf, sizeof(buf), "\nAircraft tracked: %d\n", stats.active_aircraft);
                status_str += buf;

                // Nearest aircraft
                if (stats.nearest_icao) {
                    snprintf(buf, sizeof(buf), "\nNearest aircraft:\n  %06lX %s\n  %.1f nm, %d ft\n",
                        (unsigned long)stats.nearest_icao,
                        stats.nearest_callsign[0] ? stats.nearest_callsign : "----",
                        stats.nearest_dist_nm,
                        stats.nearest_alt);
                    status_str += buf;
                }

                // GPS status
                status_str += "\nReceiver GPS: ";
                if (rx.fix_valid) {
                    snprintf(buf, sizeof(buf), "fix (%d sats)\n  %.4f, %.4f\n  HDOP: %.1f\n",
                        rx.sats, rx.lat, rx.lon, rx.hdop);
                    status_str += buf;
                } else {
                    status_str += "no fix\n";
                }

                // UTC time
                struct timeval tv;
                gettimeofday(&tv, NULL);
                struct tm tm_info;
                gmtime_r(&tv.tv_sec, &tm_info);
                if (tm_info.tm_year + 1900 >= 2024) {
                    snprintf(buf, sizeof(buf), "\nUTC: %04d-%02d-%02d %02d:%02d:%02d\n",
                        tm_info.tm_year + 1900, tm_info.tm_mon + 1, tm_info.tm_mday,
                        tm_info.tm_hour, tm_info.tm_min, tm_info.tm_sec);
                    status_str += buf;
                }

                _lock_acquire(&lvgl_api_lock);
                if (System_Ui->_registry.win.cit.esp32c6_at_test.data_label)
                    lv_label_set_text(System_Ui->_registry.win.cit.esp32c6_at_test.data_label, status_str.c_str());
                _lock_release(&lvgl_api_lock);

                cycle_time = esp_log_timestamp() + 1000;
            }
        }
        break;

        default:
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void device_adsb_app_task(void *arg)
{
    printf("device_adsb_app_task start\n");
    vTaskSuspend(Adsb_App_Task_Handle);

    // Buffers for formatting — allocated once on task stack
    static char stats_buf[512];
    static char list_buf[4096];

    size_t cycle_time = 0;

    while (1)
    {
        if (esp_log_timestamp() > cycle_time)
        {
            if (System_Ui->get_current_win() == Lvgl_Ui::System::Current_Win::ADSB)
            {
                adsb_stats_t stats = adsb_get_stats();
                receiver_pos_t rx = adsb_get_receiver_pos();

                // Format stats panel
                int pos = 0;
                pos += snprintf(stats_buf + pos, sizeof(stats_buf) - pos,
                    "RTL-SDR: %s", stats.rtlsdr_connected ? "connected" : "disconnected");

                if (stats.rtlsdr_connected) {
                    pos += snprintf(stats_buf + pos, sizeof(stats_buf) - pos,
                        "   %.1f msg/s", stats.msg_rate);
                }

                pos += snprintf(stats_buf + pos, sizeof(stats_buf) - pos,
                    "\nAircraft: %d   Messages: %lu",
                    stats.active_aircraft, (unsigned long)stats.total_messages);

                // GPS line
                if (rx.fix_valid) {
                    pos += snprintf(stats_buf + pos, sizeof(stats_buf) - pos,
                        "\nGPS: %.4f, %.4f (%d sats)",
                        rx.lat, rx.lon, rx.sats);
                } else {
                    pos += snprintf(stats_buf + pos, sizeof(stats_buf) - pos,
                        "\nGPS: searching...");
                }

                // Nearest aircraft
                if (stats.nearest_icao) {
                    pos += snprintf(stats_buf + pos, sizeof(stats_buf) - pos,
                        "\nNearest: %06lX %s  %.1fnm  %dft",
                        (unsigned long)stats.nearest_icao,
                        stats.nearest_callsign[0] ? stats.nearest_callsign : "----",
                        stats.nearest_dist_nm, stats.nearest_alt);
                }

                // UTC time
                struct timeval tv;
                gettimeofday(&tv, NULL);
                struct tm tm_info;
                gmtime_r(&tv.tv_sec, &tm_info);
                if (tm_info.tm_year + 1900 >= 2024) {
                    pos += snprintf(stats_buf + pos, sizeof(stats_buf) - pos,
                        "\nUTC: %02d:%02d:%02d",
                        tm_info.tm_hour, tm_info.tm_min, tm_info.tm_sec);
                }

                // Format aircraft list
                adsb_format_aircraft_list(list_buf, sizeof(list_buf));

                _lock_acquire(&lvgl_api_lock);
                System_Ui->win_adsb_update(stats_buf, list_buf);
                _lock_release(&lvgl_api_lock);
            }

            cycle_time = esp_log_timestamp() + 500;  // update every 500ms for responsive feel
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void device_rf_task(void *arg)
{
    printf("device_rf_task start\n");

    size_t cycle_time = 0;
    size_t auto_send_cycle_time = 0;

    while (1)
    {
        switch (System_Ui->_rf_chip_type)
        {
        case Lvgl_Ui::System::Rf_Chip_Type::SX1262:
        {
            if (System_Ui->_device_sx1262.auto_send.flag == true)
            {
                if (Rf_Send_Flag == false)
                {
                    if (esp_log_timestamp() > auto_send_cycle_time)
                    {
                        memset(Rf_Send_Package, '\0', sizeof(Rf_Send_Package));

                        if (System_Ui->_device_sx1262.auto_send.text.size() <= 255)
                            memcpy(Rf_Send_Package, System_Ui->_device_sx1262.auto_send.text.data(), System_Ui->_device_sx1262.auto_send.text.size());
                        else
                        {
                            memcpy(Rf_Send_Package, System_Ui->_device_sx1262.auto_send.text.data(), 254);
                            Rf_Send_Package[254] = '\0';
                            printf("sx1262 send out of bounds(data > Rf_Send_Package)\n");
                        }

                        char buffer_time[15];
                        snprintf(buffer_time, sizeof(buffer_time), "%02d:%02d:%02d", System_Ui->_time.hour, System_Ui->_time.minute, System_Ui->_time.second);

                        Lvgl_Ui::System::Win_Rf_Chat_Message wlcm =
                            {
                                .direction = Lvgl_Ui::System::Chat_Message_Direction::SEND,
                                .time = buffer_time,
                                .data = System_Ui->_device_sx1262.auto_send.text,
                            };
                        System_Ui->_registry.win.rf.chat_message_data.push_back(wlcm);

                        if (System_Ui->_current_win == Lvgl_Ui::System::Current_Win::RF)
                        {
                            _lock_acquire(&lvgl_api_lock);
                            System_Ui->win_rf_chat_message_data_update(System_Ui->_registry.win.rf.chat_message_data);
                            _lock_release(&lvgl_api_lock);
                        }

                        Rf_Send_Flag = true;
                        auto_send_cycle_time = esp_log_timestamp() + System_Ui->_device_sx1262.auto_send.interval;
                    }
                }
            }

            if (Rf_Send_Flag == true)
            {
                SX1262->start_lora_transmit(Cpp_Bus_Driver::Sx126x::Chip_Mode::TX, 0, Cpp_Bus_Driver::Sx126x::Fallback_Mode::FS);
                SX1262->set_irq_pin_mode(Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::TX_DONE);
                SX1262->clear_irq_flag(Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::TX_DONE);

                printf("sx1262 send start\n");
                uint16_t timeout_count = 0;
                if (SX1262->send_data(Rf_Send_Package, strlen(reinterpret_cast<const char *>(Rf_Send_Package))) == true)
                {
                    while (1)
                    {
                        if (XL9535->pin_read(XL9535_SX1262_DIO1) == 1)
                        {
                            Cpp_Bus_Driver::Sx126x::Irq_Status is;
                            if (SX1262->parse_irq_status(SX1262->get_irq_flag(), is) == false)
                                printf("parse_Iqr_status fail\n");
                            else if (is.all_flag.tx_done == true)
                            {
                                printf("sx1262 send success\n");
                                break;
                            }
                        }
                        timeout_count++;
                        if (timeout_count > 1000)
                        {
                            printf("sx1262 send timeout\n");
                            break;
                        }
                        vTaskDelay(pdMS_TO_TICKS(10));
                    }
                }
                else
                    printf("sx1262 send fail\n");

                SX1262->start_lora_transmit(Cpp_Bus_Driver::Sx126x::Chip_Mode::RX);
                SX1262->set_irq_pin_mode(Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::RX_DONE);
                SX1262->clear_irq_flag(Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::RX_DONE);

                Rf_Send_Flag = false;
            }

            if (XL9535->pin_read(XL9535_SX1262_DIO1) == 1)
            {
                Cpp_Bus_Driver::Sx126x::Irq_Status is;
                if (SX1262->parse_irq_status(SX1262->get_irq_flag(), is) == false)
                    printf("parse_irq_status fail\n");
                else
                {
                    if (is.all_flag.tx_rx_timeout == true)
                    {
                        printf("receive timeout\n");
                        SX1262->clear_irq_flag(Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::TIMEOUT);
                    }
                    else if (is.all_flag.crc_error == true)
                    {
                        printf("receive crc error\n");
                        SX1262->clear_irq_flag(Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::CRC_ERROR);
                    }
                    else if (is.lora_reg_flag.header_error == true)
                    {
                        printf("receive header error\n");
                        SX1262->clear_irq_flag(Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::HEADER_ERROR);
                    }
                    else
                    {
                        uint8_t receive_package[255] = {0};
                        uint8_t length_buffer = SX1262->receive_data(receive_package);
                        if (length_buffer == 0)
                            printf("sx1262 receive fail (error assert: %d)\n", SX1262->_assert);
                        else
                        {
                            Cpp_Bus_Driver::Sx126x::Packet_Metrics pm;
                            if (SX1262->get_lora_packet_metrics(pm) == true)
                                printf("sx1262 receive rssi_average: %.01f rssi_instantaneous: %.01f snr: %.01f\n", pm.lora.rssi_average, pm.lora.rssi_instantaneous, pm.lora.snr);

                            char buffer_time[15];
                            snprintf(buffer_time, sizeof(buffer_time), "%02d:%02d:%02d", System_Ui->_time.hour, System_Ui->_time.minute, System_Ui->_time.second);

                            std::vector<uint8_t> buffer_vector(receive_package, receive_package + length_buffer);
                            buffer_vector.erase(std::remove(buffer_vector.begin(), buffer_vector.end(), 0), buffer_vector.end());
                            std::string message_str(buffer_vector.begin(), buffer_vector.end());
                            message_str += '\0';

                            char buffer_data_info[30];
                            snprintf(buffer_data_info, sizeof(buffer_data_info), "rssi[%.01f] snr[%.01f]", pm.lora.rssi_instantaneous, pm.lora.snr);

                            Lvgl_Ui::System::Win_Rf_Chat_Message wlcm =
                                {
                                    .direction = Lvgl_Ui::System::Chat_Message_Direction::RECEIVE,
                                    .time = buffer_time,
                                    .data = message_str,
                                    .data_info = buffer_data_info,
                                };
                            System_Ui->_registry.win.rf.chat_message_data.push_back(wlcm);

                            if (System_Ui->_current_win == Lvgl_Ui::System::Current_Win::RF)
                            {
                                _lock_acquire(&lvgl_api_lock);
                                System_Ui->win_rf_chat_message_data_update(System_Ui->_registry.win.rf.chat_message_data);
                                _lock_release(&lvgl_api_lock);
                            }
                        }
                    }
                }

                SX1262->clear_irq_flag(Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::RX_DONE);
            }
        }
        break;

#if defined CONFIG_BOARD_TYPE_T_DISPLAY_P4_KEYBOARD
        case Lvgl_Ui::System::Rf_Chip_Type::CC1101:
        {
            if (System_Ui->_device_cc1101.auto_send.flag == true)
            {
                if (Rf_Send_Flag == false)
                {
                    if (esp_log_timestamp() > auto_send_cycle_time)
                    {
                        memset(Rf_Send_Package, '\0', sizeof(Rf_Send_Package));
                        if (System_Ui->_device_cc1101.auto_send.text.size() <= 255)
                            memcpy(Rf_Send_Package, System_Ui->_device_cc1101.auto_send.text.data(), System_Ui->_device_cc1101.auto_send.text.size());
                        else
                        {
                            memcpy(Rf_Send_Package, System_Ui->_device_cc1101.auto_send.text.data(), 254);
                            Rf_Send_Package[254] = '\0';
                        }

                        char buffer_time[15];
                        snprintf(buffer_time, sizeof(buffer_time), "%02d:%02d:%02d", System_Ui->_time.hour, System_Ui->_time.minute, System_Ui->_time.second);

                        Lvgl_Ui::System::Win_Rf_Chat_Message wlcm =
                            {
                                .direction = Lvgl_Ui::System::Chat_Message_Direction::SEND,
                                .time = buffer_time,
                                .data = System_Ui->_device_cc1101.auto_send.text,
                            };
                        System_Ui->_registry.win.rf.chat_message_data.push_back(wlcm);

                        if (System_Ui->_current_win == Lvgl_Ui::System::Current_Win::RF)
                        {
                            _lock_acquire(&lvgl_api_lock);
                            System_Ui->win_rf_chat_message_data_update(System_Ui->_registry.win.rf.chat_message_data);
                            _lock_release(&lvgl_api_lock);
                        }

                        Rf_Send_Flag = true;
                        auto_send_cycle_time = esp_log_timestamp() + System_Ui->_device_cc1101.auto_send.interval;
                    }
                }
            }

            if (Rf_Send_Flag == true)
            {
                printf("cc1101 send start\n");
                Cc1101.finishTransmit();
                int16_t assert = Cc1101.transmit(Rf_Send_Package, strlen(reinterpret_cast<const char *>(Rf_Send_Package)));
                if (assert != RADIOLIB_ERR_NONE)
                    printf("cc1101 transmit fail (error code: %d)\n", assert);

                assert = Cc1101.startReceive();
                if (assert != RADIOLIB_ERR_NONE)
                    printf("cc1101 startReceive fail (error code: %d)\n", assert);

                Cc1101_Interrupt_Flag = false;
                Rf_Send_Flag = false;
            }

            if (Cc1101_Interrupt_Flag == true)
            {
                uint8_t receive_package[255] = {0};
                uint8_t length_buffer = Cc1101.getPacketLength();
                int16_t assert = Cc1101.readData(receive_package, length_buffer);
                if (assert != RADIOLIB_ERR_NONE)
                    printf("cc1101 receive fail (error assert: %d)\n", assert);
                else
                {
                    float buffer_rssi = Cc1101.getRSSI();
                    uint8_t buffer_lqi = Cc1101.getLQI();

                    char buffer_time[15];
                    snprintf(buffer_time, sizeof(buffer_time), "%02d:%02d:%02d", System_Ui->_time.hour, System_Ui->_time.minute, System_Ui->_time.second);

                    std::vector<uint8_t> buffer_vector(receive_package, receive_package + length_buffer);
                    buffer_vector.erase(std::remove(buffer_vector.begin(), buffer_vector.end(), 0), buffer_vector.end());
                    std::string message_str(buffer_vector.begin(), buffer_vector.end());
                    message_str += '\0';

                    char buffer_data_info[30];
                    snprintf(buffer_data_info, sizeof(buffer_data_info), "rssi[%.01f] lqi[%d]", buffer_rssi, buffer_lqi);

                    Lvgl_Ui::System::Win_Rf_Chat_Message wlcm =
                        {
                            .direction = Lvgl_Ui::System::Chat_Message_Direction::RECEIVE,
                            .time = buffer_time,
                            .data = message_str,
                            .data_info = buffer_data_info,
                        };
                    System_Ui->_registry.win.rf.chat_message_data.push_back(wlcm);

                    if (System_Ui->_current_win == Lvgl_Ui::System::Current_Win::RF)
                    {
                        _lock_acquire(&lvgl_api_lock);
                        System_Ui->win_rf_chat_message_data_update(System_Ui->_registry.win.rf.chat_message_data);
                        _lock_release(&lvgl_api_lock);
                    }
                }

                Cc1101_Interrupt_Flag = false;
            }
        }
        break;

        case Lvgl_Ui::System::Rf_Chip_Type::NRF24L01:
        {
            if (System_Ui->_device_nrf24l01.auto_send.flag == true)
            {
                if (Rf_Send_Flag == false)
                {
                    if (esp_log_timestamp() > auto_send_cycle_time)
                    {
                        memset(Rf_Send_Package, '\0', sizeof(Rf_Send_Package));
                        if (System_Ui->_device_nrf24l01.auto_send.text.size() <= 255)
                            memcpy(Rf_Send_Package, System_Ui->_device_nrf24l01.auto_send.text.data(), System_Ui->_device_nrf24l01.auto_send.text.size());
                        else
                        {
                            memcpy(Rf_Send_Package, System_Ui->_device_nrf24l01.auto_send.text.data(), 254);
                            Rf_Send_Package[254] = '\0';
                        }

                        char buffer_time[15];
                        snprintf(buffer_time, sizeof(buffer_time), "%02d:%02d:%02d", System_Ui->_time.hour, System_Ui->_time.minute, System_Ui->_time.second);

                        Lvgl_Ui::System::Win_Rf_Chat_Message wlcm =
                            {
                                .direction = Lvgl_Ui::System::Chat_Message_Direction::SEND,
                                .time = buffer_time,
                                .data = System_Ui->_device_nrf24l01.auto_send.text,
                            };
                        System_Ui->_registry.win.rf.chat_message_data.push_back(wlcm);

                        if (System_Ui->_current_win == Lvgl_Ui::System::Current_Win::RF)
                        {
                            _lock_acquire(&lvgl_api_lock);
                            System_Ui->win_rf_chat_message_data_update(System_Ui->_registry.win.rf.chat_message_data);
                            _lock_release(&lvgl_api_lock);
                        }

                        Rf_Send_Flag = true;
                        auto_send_cycle_time = esp_log_timestamp() + System_Ui->_device_nrf24l01.auto_send.interval;
                    }
                }
            }

            if (Rf_Send_Flag == true)
            {
                printf("nrf24l01 send start\n");
                Nrf24l01.finishTransmit();
                int16_t assert = Nrf24l01.transmit(Rf_Send_Package, strlen(reinterpret_cast<const char *>(Rf_Send_Package)), 0);
                if (assert != RADIOLIB_ERR_NONE)
                    printf("nrf24l01 transmit fail (error code: %d)\n", assert);

                assert = Nrf24l01.startReceive();
                if (assert != RADIOLIB_ERR_NONE)
                    printf("nrf24l01 startReceive fail (error code: %d)\n", assert);

                Nrf24l01_Interrupt_Flag = false;
                Rf_Send_Flag = false;
            }

            if (Nrf24l01_Interrupt_Flag == true)
            {
                uint8_t receive_package[255] = {0};
                uint8_t length_buffer = Nrf24l01.getPacketLength();
                int16_t assert = Nrf24l01.readData(receive_package, length_buffer);
                if (assert != RADIOLIB_ERR_NONE)
                    printf("nrf24l01 receive fail (error assert: %d)\n", assert);
                else
                {
                    float buffer_rssi = Nrf24l01.getRSSI();
                    float buffer_lqi = Nrf24l01.getSNR();

                    char buffer_time[15];
                    snprintf(buffer_time, sizeof(buffer_time), "%02d:%02d:%02d", System_Ui->_time.hour, System_Ui->_time.minute, System_Ui->_time.second);

                    std::vector<uint8_t> buffer_vector(receive_package, receive_package + length_buffer);
                    buffer_vector.erase(std::remove(buffer_vector.begin(), buffer_vector.end(), 0), buffer_vector.end());
                    std::string message_str(buffer_vector.begin(), buffer_vector.end());
                    message_str += '\0';

                    char buffer_data_info[30];
                    snprintf(buffer_data_info, sizeof(buffer_data_info), "rssi[%.01f] snr[%.01f]", buffer_rssi, buffer_lqi);

                    Lvgl_Ui::System::Win_Rf_Chat_Message wlcm =
                        {
                            .direction = Lvgl_Ui::System::Chat_Message_Direction::RECEIVE,
                            .time = buffer_time,
                            .data = message_str,
                            .data_info = buffer_data_info,
                        };
                    System_Ui->_registry.win.rf.chat_message_data.push_back(wlcm);

                    if (System_Ui->_current_win == Lvgl_Ui::System::Current_Win::RF)
                    {
                        _lock_acquire(&lvgl_api_lock);
                        System_Ui->win_rf_chat_message_data_update(System_Ui->_registry.win.rf.chat_message_data);
                        _lock_release(&lvgl_api_lock);
                    }
                }

                Nrf24l01_Interrupt_Flag = false;
            }
        }
        break;
#endif

        default:
            break;
        }

        if (Device_Rf_Task_Stop_Flag == true)
            vTaskSuspend(Rf_Task_Handle);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void iis_transmission_data_stream_task(void *arg)
{
    printf("iis_transmission_data_stream_task start\n");

    size_t cycle_time = 0;

    vTaskSuspend(Iis_Transmission_Data_Stream_Task);

    while (1)
    {
        if (esp_log_timestamp() > cycle_time)
        {
            if (Music_File.good())
            {
                const auto current_buf_size = Iis_Transmission_Data_Stream.size();
                if (current_buf_size < 1024 * 300)
                {
                    Iis_Transmission_Data_Stream.resize(current_buf_size + 1024 * 20);
                    Music_File.read(Iis_Transmission_Data_Stream.data() + current_buf_size, 1024 * 20);
                    std::streamsize bytes_read = Music_File.gcount();
                    if (bytes_read < 1024 * 20)
                        Iis_Transmission_Data_Stream.erase(Iis_Transmission_Data_Stream.end() - (1024 * 20 - bytes_read), Iis_Transmission_Data_Stream.end());
                }
            }

            cycle_time = esp_log_timestamp() + 30;
        }

        if (Music_File.good())
        {
            if (Iis_Read_Data_Size_Index > 1024 * 250)
            {
                Iis_Transmission_Data_Stream.erase(Iis_Transmission_Data_Stream.begin(), Iis_Transmission_Data_Stream.begin() + 1024 * 250);
                Iis_Read_Data_Size_Index -= 1024 * 250;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void my_touchpad_read(lv_indev_t *indev, lv_indev_data_t *data)
{
    static size_t edge_touch_scheduled_shutdown_time = 0;
    static size_t edge_touch_scheduled_shutdown_lock = false;

    if (edge_touch_scheduled_shutdown_lock == true)
    {
        if (esp_log_timestamp() > edge_touch_scheduled_shutdown_time)
        {
            System_Ui->_edge_touch_flag = false;
            edge_touch_scheduled_shutdown_lock = false;
        }
    }

    // --- Unified touch input: runtime screen type detection ---
    bool got_touch = false;
    Lvgl_Ui::System::TouchPoint tp;

    if (screen_is_rm69a10()) {
        Cpp_Bus_Driver::Gt9895::Touch_Point raw;
        if (GT9895->get_multiple_touch_point(raw)) {
            tp.finger_count = raw.finger_count;
            tp.edge_touch_flag = raw.edge_touch_flag;
            for (auto &pt : raw.info)
                tp.info.push_back({pt.x, pt.y, pt.pressure_value});
            got_touch = true;
            raw.info.clear();
        }
    } else {
        Cpp_Bus_Driver::Hi8561_Touch::Touch_Point raw;
        if (HI8561_T->get_multiple_touch_point(raw)) {
            tp.finger_count = raw.finger_count;
            tp.edge_touch_flag = raw.edge_touch_flag;
            for (auto &pt : raw.info)
                tp.info.push_back({pt.x, pt.y, pt.pressure_value});
            got_touch = true;
            raw.info.clear();
        }
    }

    if (got_touch)
    {
        if (System_Ui->get_current_win() == Lvgl_Ui::System::Current_Win::CIT_TOUCH_TEST)
        {
            data->point.x = tp.info[0].x;
            data->point.y = tp.info[0].y;
            data->state = LV_INDEV_STATE_PR;
        }
        else
        {
            if ((tp.finger_count == 1) && (tp.info[0].x != static_cast<uint16_t>(-1)) && (tp.info[0].y != static_cast<uint16_t>(-1)) && (tp.info[0].pressure_value != 0))
            {
                data->point.x = tp.info[0].x;
                data->point.y = tp.info[0].y;
                data->state = LV_INDEV_STATE_PR;
            }
            else
                data->state = LV_INDEV_STATE_REL;
        }

#if defined CONFIG_BOARD_TYPE_T_DISPLAY_P4
        if (tp.edge_touch_flag == true)
        {
            System_Ui->_edge_touch_flag = true;
            edge_touch_scheduled_shutdown_time = esp_log_timestamp() + 100;
            edge_touch_scheduled_shutdown_lock = true;
        }
#elif defined CONFIG_BOARD_TYPE_T_DISPLAY_P4_KEYBOARD
        if ((tp.info[0].y < 20) || ((tp.info[0].y > g_screen_height - 20) && (tp.info[0].y <= g_screen_height)))
        {
            tp.edge_touch_flag = true;
            System_Ui->_edge_touch_flag = true;
            edge_touch_scheduled_shutdown_time = esp_log_timestamp() + 200;
            edge_touch_scheduled_shutdown_lock = true;
        }
#endif

        System_Ui->_touch_point = tp;
    }
    else
        data->state = LV_INDEV_STATE_REL;
}

#if defined CONFIG_BOARD_TYPE_T_DISPLAY_P4_KEYBOARD
void my_keyboard_read(lv_indev_t *indev, lv_indev_data_t *data)
{
    static uint32_t last_key = 0;
    static bool pressed_state_flag = false;
    static bool caps_lock_flag = false;
    static bool shift_press_flag = false;

    if (TCA8418_Interrupt_Flag == true)
    {
        Cpp_Bus_Driver::Tca8418::Irq_Status is;

        if (TCA8418->parse_irq_status(TCA8418->get_irq_flag(), is) == false)
            printf("parse_irq_status fail\n");
        else
        {
            if (is.key_events_flag == true)
            {
                Cpp_Bus_Driver::Tca8418::Touch_Point tp;
                if (TCA8418->get_multiple_touch_point(tp) == true)
                {
                    for (uint8_t i = 0; i < tp.info.size(); i++)
                    {
                        switch (tp.info[i].event_type)
                        {
                        case Cpp_Bus_Driver::Tca8418::Event_Type::KEYPAD:
                        {
                            Cpp_Bus_Driver::Tca8418::Touch_Position tp_2;
                            if (TCA8418->parse_touch_num(tp.info[i].num, tp_2) == true)
                            {
                                if (tp.info[i].num <= (sizeof(Tca8418_Map) / sizeof(std::string)))
                                {
                                    if (System_Ui->get_current_win() == Lvgl_Ui::System::Current_Win::CIT_KEYBOARD_TEST)
                                        lv_label_set_text(System_Ui->_registry.win.cit.keyboard_test.data_label, Tca8418_Map[tp.info[i].num - 1].c_str());
                                }

                                if (tp.info[i].press_flag == 1)
                                {
                                    pressed_state_flag = true;
                                    if (Tca8418_Map[tp.info[i].num - 1] == "Caps")
                                    {
                                        caps_lock_flag = !caps_lock_flag;
                                        XL9555->pin_write(XL9555_LED_1, static_cast<Cpp_Bus_Driver::Xl95x5::Value>(!caps_lock_flag));
                                        XL9555->pin_write(XL9555_LED_2, static_cast<Cpp_Bus_Driver::Xl95x5::Value>(!caps_lock_flag));
                                        XL9555->pin_write(XL9555_LED_3, static_cast<Cpp_Bus_Driver::Xl95x5::Value>(!caps_lock_flag));
                                    }

                                    if (Tca8418_Map[tp.info[i].num - 1] == "Shift")
                                        shift_press_flag = true;

                                    if (shift_press_flag == false)
                                    {
                                        last_key = Tca8418_Map_Lvgl[tp.info[i].num - 1];
                                        if (caps_lock_flag == true && last_key >= 'a' && last_key <= 'z')
                                            last_key = last_key - 'a' + 'A';
                                    }
                                    else
                                        last_key = Tca8418_Map_Lvgl_Shift[tp.info[i].num - 1];
                                }
                                else
                                {
                                    pressed_state_flag = false;
                                    if (Tca8418_Map[tp.info[i].num - 1] == "Shift")
                                        shift_press_flag = false;
                                }
                            }
                            break;
                        }
                        case Cpp_Bus_Driver::Tca8418::Event_Type::GPIO:
                            break;
                        default:
                            break;
                        }
                    }
                }

                TCA8418->clear_irq_flag(Cpp_Bus_Driver::Tca8418::Irq_Flag::KEY_EVENTS);
            }
        }

        TCA8418_Interrupt_Flag = false;
    }

    if (pressed_state_flag == false)
        data->state = LV_INDEV_STATE_RELEASED;
    else
    {
        data->state = LV_INDEV_STATE_PRESSED;
        data->key = last_key;
    }
}

void device_nfc_task(void *arg)
{
    printf("device_nfc_task start\n");
    vTaskSuspend(Nfc_Task_Handle);

    while (1)
    {
        switch (St25r3916_Nfc_Mode)
        {
        case Nfc_Mode::TEST:
            St25r3916_Loop();
            break;
        default:
            break;
        }

        if (Device_Nfc_Task_Stop_Flag == true)
            vTaskSuspend(Nfc_Task_Handle);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void Cc1101_Rf_Switch_Control(Cc1101_Rf_Switch rf_switch)
{
    switch (rf_switch)
    {
    case Cc1101_Rf_Switch::RF_SWITCH_315MHZ:
        XL9555->pin_write(XL9555_T_MIXRF_CC1101_RF_SWITCH_0, Cpp_Bus_Driver::Xl95x5::Value::LOW);
        XL9555->pin_write(XL9555_T_MIXRF_CC1101_RF_SWITCH_1, Cpp_Bus_Driver::Xl95x5::Value::HIGH);
        break;
    case Cc1101_Rf_Switch::RF_SWITCH_434MHZ:
        XL9555->pin_write(XL9555_T_MIXRF_CC1101_RF_SWITCH_0, Cpp_Bus_Driver::Xl95x5::Value::HIGH);
        XL9555->pin_write(XL9555_T_MIXRF_CC1101_RF_SWITCH_1, Cpp_Bus_Driver::Xl95x5::Value::HIGH);
        break;
    case Cc1101_Rf_Switch::RF_SWITCH_868_915MHZ:
        XL9555->pin_write(XL9555_T_MIXRF_CC1101_RF_SWITCH_0, Cpp_Bus_Driver::Xl95x5::Value::HIGH);
        XL9555->pin_write(XL9555_T_MIXRF_CC1101_RF_SWITCH_1, Cpp_Bus_Driver::Xl95x5::Value::LOW);
        break;
    default:
        printf("unknown rf switch\n");
        break;
    }
}

bool Set_T_Mixrf_Lr1121_Sleep()
{
    XL9555->pin_mode(XL9555_T_MIXRF_LR1121_RST, Cpp_Bus_Driver::Xl95x5::Mode::OUTPUT);
    XL9555->pin_write(XL9555_T_MIXRF_LR1121_RST, Cpp_Bus_Driver::Xl95x5::Value::LOW);
    return true;
}

#endif

static sdmmc_card_t *sd_card_handle = NULL;

bool Sdmmc_Init(const char *base_path, int max_retries = 1)
{
    esp_vfs_fat_sdmmc_mount_config_t mount_config =
        {
            .format_if_mount_failed = false,
            .max_files = 5,
            .allocation_unit_size = 16 * 1024,
        };

    sdmmc_card_t *card;

    printf("initializing sd card\n");

    sd_pwr_ctrl_ldo_config_t ldo_config = { .ldo_chan_id = 4 };
    sd_pwr_ctrl_handle_t pwr_ctrl_handle = NULL;

    int32_t assert = sd_pwr_ctrl_new_on_chip_ldo(&ldo_config, &pwr_ctrl_handle);
    if (assert != ESP_OK)
        printf("failed to create a new on-chip ldo power control driver\n");

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.slot = SDMMC_HOST_SLOT_0;
    host.max_freq_khz = SDMMC_FREQ_52M;
    host.pwr_ctrl_handle = pwr_ctrl_handle;

    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 4;
    slot_config.clk = static_cast<gpio_num_t>(SD_SDIO_CLK);
    slot_config.cmd = static_cast<gpio_num_t>(SD_SDIO_CMD);
    slot_config.d0 = static_cast<gpio_num_t>(SD_SDIO_D0);
    slot_config.d1 = static_cast<gpio_num_t>(SD_SDIO_D1);
    slot_config.d2 = static_cast<gpio_num_t>(SD_SDIO_D2);
    slot_config.d3 = static_cast<gpio_num_t>(SD_SDIO_D3);
    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    printf("mounting filesystem\n");

    // Retry mount — SD card may need extra time to come online
    for (int attempt = 0; attempt < max_retries; attempt++) {
        assert = esp_vfs_fat_sdmmc_mount(base_path, &host, &slot_config, &mount_config, &card);
        if (assert == ESP_OK) break;
        if (attempt < max_retries - 1) {
            printf("SD mount attempt %d failed (0x%lx), retrying in 1s...\n", attempt + 1, assert);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    if (assert != ESP_OK)
    {
        printf("failed to mount filesystem\n");
        return false;
    }

    printf("filesystem mounted\n");
    printf("[MEM] after SD mount: internal=%u\n", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    sdmmc_card_print_info(stdout, card);
    sd_card_handle = card;

    // Clear FAT32 dirty flag after mount
    sd_clear_dirty_flag();

    return true;
}

// Clear the FAT32 dirty flag (ClnShutBit + HrdErrBit in FAT[1]).
// Called after mount and periodically during sd_log_sync so that
// a hard crash always leaves the flag clean for macOS.
extern "C" void sd_clear_dirty_flag(void) {
    if (!sd_card_handle) return;
    uint8_t bpb[512];
    if (sdmmc_read_sectors(sd_card_handle, bpb, 0, 1) != ESP_OK) return;
    if (bpb[510] != 0x55 || bpb[511] != 0xAA) return;
    uint16_t reserved = *(uint16_t *)(bpb + 14);
    uint8_t n_fats = bpb[16];
    uint32_t fat_size = *(uint32_t *)(bpb + 36);
    if (reserved == 0) return;
    uint8_t fat_sec[512];
    if (sdmmc_read_sectors(sd_card_handle, fat_sec, reserved, 1) != ESP_OK) return;
    uint32_t fat1 = *(uint32_t *)(fat_sec + 4);
    if ((fat1 & 0x0C000000) == 0x0C000000) return; // already clean
    fat1 |= 0x0C000000;
    *(uint32_t *)(fat_sec + 4) = fat1;
    sdmmc_write_sectors(sd_card_handle, fat_sec, reserved, 1);
    if (n_fats > 1 && fat_size > 0) {
        sdmmc_write_sectors(sd_card_handle, fat_sec, reserved + fat_size, 1);
    }
}

// Clean SD shutdown: close log file, unmount FATFS so dirty bit is cleared.
// Called from shutdown handler and reboot command.
extern "C" void sd_safe_shutdown(void) {
    sd_log_close();
    if (sd_card_handle) {
        esp_vfs_fat_sdcard_unmount("/sdcard", sd_card_handle);
        sd_card_handle = NULL;
        printf("[SD] Filesystem unmounted cleanly\n");
    }
}

extern "C" bool sd_remount(void) {
    if (sd_card_handle) {
        printf("[SD] Already mounted\n");
        return true;
    }
    bool ok = Sdmmc_Init("/sdcard", 1);  // single attempt, no retries
    if (ok) {
        printf("[SD] Filesystem mounted\n");
    } else {
        printf("[SD] Mount failed\n");
    }
    return ok;
}

bool sd_is_mounted(void) {
    return sd_card_handle != NULL;
}

bool sd_is_logging(void) {
    // We're logging if SD is mounted and the RTL-SDR reader task is running
    // (sd_log_create is called when the reader task starts)
    adsb_stats_t stats = adsb_get_stats();
    return sd_card_handle != NULL && stats.rtlsdr_connected;
}

void System_Ui_Callback_Init(void)
{
    System_Ui->_device_vibration_callback = [](uint8_t vibration_count)
    {
        AW86224_Vibration_Play_Count = vibration_count;
        vTaskResume(Vibration_Task_Handle);
    };

    System_Ui->_win_cit_speaker_test_callback = [](void)
    {
        ES8311_Speaker_Mode = Es8311_Mode::TEST;
        vTaskResume(Speaker_Task_Handle);
    };

    System_Ui->_win_cit_microphone_test_callback = [](bool status)
    {
        if (status == true)
        {
            ES8311_Microphone_Mode = Es8311_Mode::TEST;
            vTaskResume(Microphone_Task_Handle);
        }
        else
            vTaskSuspend(Microphone_Task_Handle);
    };

    System_Ui->_win_cit_adc_to_dac_switch_callback = [](bool status)
    {
        ES8311->set_adc_data_to_dac(status);
    };

    System_Ui->_win_cit_imu_test_callback = [](bool status)
    {
        if (status == true)
        {
            ICM20948_Imu_Mode = Imu_Mode::TEST;
            vTaskResume(Imu_Task_Handle);
        }
        else
            vTaskSuspend(Imu_Task_Handle);
    };

    System_Ui->_win_cit_gps_test_callback = [](bool status)
    {
        if (status == true)
        {
            L76k_Gps_Mode = Gps_Mode::TEST;
            L76K->clear_rx_buffer_data();
            L76k_Gps_Positioning_Time = 0;
            L76k_Gps_Positioning_Flag = false;
            // Task runs continuously for ADS-B — no resume needed
        }
        else
        {
            // Don't suspend task or sleep GPS — ADS-B needs continuous position
            L76k_Gps_Mode = Gps_Mode::RUN;
        }
    };

    System_Ui->_win_cit_ethernet_test_callback = [](bool status)
    {
        if (status == true)
        {
            Ip101gri_Ethernet_Mode = Ethernet_Mode::TEST;
            Eth_Info.status.update_flag = true;
            Eth_Info.connect_ip_status.update_flag = true;
            vTaskResume(Ethernet_Task_Handle);
        }
        else
            vTaskSuspend(Ethernet_Task_Handle);
    };

    System_Ui->_win_cit_esp32c6_at_test_callback = [](bool status)
    {
        // Repurposed: shows ADS-B receiver status instead of AT test
        if (status == true)
        {
            Esp32c6_At_Mode = At_Mode::TEST;
            vTaskResume(At_Task_Handle);
        }
        else
        {
            vTaskSuspend(At_Task_Handle);
        }
    };

    System_Ui->_win_camera_status_callback = [](bool status)
    {
        if (Sys_Status.camera.init_flag == true)
        {
#if defined CONFIG_BOARD_TYPE_T_DISPLAY_P4_KEYBOARD
            vTaskDelay(pdMS_TO_TICKS(1000));
#endif
            if (status == true)
            {
                esp_err_t assert = app_video_stream_task_restart(video_cam_fd0);
                if (assert != ESP_OK)
                    printf("app_video_stream_task_restart fail (error code: %#X)\n", assert);
                else
                    start_time = esp_timer_get_time();
            }
            else
            {
                esp_err_t assert = app_video_stream_task_stop(video_cam_fd0);
                if (assert != ESP_OK)
                    printf("app_video_stream_task_stop fail (error code: %#X)\n", assert);
            }
        }
    };

    System_Ui->_win_adsb_status_callback = [](bool status)
    {
        if (status == true)
            vTaskResume(Adsb_App_Task_Handle);
        else
            vTaskSuspend(Adsb_App_Task_Handle);
    };

    System_Ui->_win_rf_config_sx1262_params_callback = [](Lvgl_Ui::System::Device_Sx1262 device_sx1262) -> bool
    {
        XL9535->pin_write(XL9535_SKY13453_VCTL,
            device_sx1262.params.rf_switch == 0 ? Cpp_Bus_Driver::Xl95x5::Value::HIGH : Cpp_Bus_Driver::Xl95x5::Value::LOW);

        if (SX1262->config_lora_params(device_sx1262.params.freq, device_sx1262.params.bandwidth, device_sx1262.params.current_limit,
                                       device_sx1262.params.power, device_sx1262.params.sf, device_sx1262.params.cr, device_sx1262.params.crc_type,
                                       device_sx1262.params.preamble_length, device_sx1262.params.sync_word) == false)
        {
            printf("config_lora_params fail\n");
            return false;
        }
        SX1262->clear_buffer();
        SX1262->start_lora_transmit(Cpp_Bus_Driver::Sx126x::Chip_Mode::RX);
        SX1262->set_irq_pin_mode(Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::RX_DONE);
        SX1262->clear_irq_flag(Cpp_Bus_Driver::Sx126x::Irq_Mask_Flag::RX_DONE);
        printf("config_lora_params finish start sx1262 transmit\n");
        return true;
    };

    System_Ui->_win_rf_send_data_callback = [](std::string data)
    {
        memset(Rf_Send_Package, '\0', sizeof(Rf_Send_Package));
        if (data.size() <= 255)
            memcpy(Rf_Send_Package, data.data(), data.size());
        else
        {
            memcpy(Rf_Send_Package, data.data(), 254);
            Rf_Send_Package[254] = '\0';
            printf("lora send out of bounds(data > Rf_Send_Package)\n");
        }
        Rf_Send_Flag = true;
    };

    System_Ui->_win_rf_status_callback = [](bool status)
    {
        if (status == true)
        {
            Device_Rf_Task_Stop_Flag = false;
            vTaskResume(Rf_Task_Handle);
        }
        else
            Device_Rf_Task_Stop_Flag = true;
    };

    System_Ui->_win_music_start_end_callback = [](bool status)
    {
        if (status == true)
        {
            ES8311_Speaker_Mode = Es8311_Mode::PLAY_MUSIC;
            vTaskResume(Speaker_Task_Handle);
        }
        else
            Music_Play_End_Flag = true;
    };

    System_Ui->_set_music_current_time_s_callback = [](double current_time_s)
    {
        Set_Music_Current_Time_S = current_time_s;
        Set_Music_Current_Time_S_Flag = true;
    };

#if defined CONFIG_BOARD_TYPE_T_DISPLAY_P4_KEYBOARD
    System_Ui->_win_cit_nfc_test_callback = [](bool status)
    {
        if (status == true)
        {
            St25r3916_Nfc_Mode = Nfc_Mode::TEST;
            Device_Nfc_Task_Stop_Flag = false;
            vTaskResume(Nfc_Task_Handle);
        }
        else
            Device_Nfc_Task_Stop_Flag = true;
    };

    System_Ui->_win_rf_config_cc1101_params_callback = [](Lvgl_Ui::System::Device_Cc1101 device_cc1101) -> bool
    {
        Cc1101_Rf_Switch_Control(static_cast<Cc1101_Rf_Switch>(device_cc1101.params.rf_switch));

        float buffer_bandwidth = 0;
        switch (device_cc1101.params.bandwidth)
        {
        case Lvgl_Ui::System::Cc1101_Bw::BW_58KHZ:   buffer_bandwidth = 58.0f;  break;
        case Lvgl_Ui::System::Cc1101_Bw::BW_68KHZ:   buffer_bandwidth = 68.0f;  break;
        case Lvgl_Ui::System::Cc1101_Bw::BW_81KHZ:   buffer_bandwidth = 81.0f;  break;
        case Lvgl_Ui::System::Cc1101_Bw::BW_102KHZ:  buffer_bandwidth = 102.0f; break;
        case Lvgl_Ui::System::Cc1101_Bw::BW_116KHZ:  buffer_bandwidth = 116.0f; break;
        case Lvgl_Ui::System::Cc1101_Bw::BW_135KHZ:  buffer_bandwidth = 135.0f; break;
        case Lvgl_Ui::System::Cc1101_Bw::BW_162KHZ:  buffer_bandwidth = 162.0f; break;
        case Lvgl_Ui::System::Cc1101_Bw::BW_203KHZ:  buffer_bandwidth = 203.0f; break;
        case Lvgl_Ui::System::Cc1101_Bw::BW_232KHZ:  buffer_bandwidth = 232.0f; break;
        case Lvgl_Ui::System::Cc1101_Bw::BW_270KHZ:  buffer_bandwidth = 270.0f; break;
        case Lvgl_Ui::System::Cc1101_Bw::BW_325KHZ:  buffer_bandwidth = 325.0f; break;
        case Lvgl_Ui::System::Cc1101_Bw::BW_406KHZ:  buffer_bandwidth = 406.0f; break;
        case Lvgl_Ui::System::Cc1101_Bw::BW_464KHZ:  buffer_bandwidth = 464.0f; break;
        case Lvgl_Ui::System::Cc1101_Bw::BW_541KHZ:  buffer_bandwidth = 541.0f; break;
        case Lvgl_Ui::System::Cc1101_Bw::BW_650KHZ:  buffer_bandwidth = 650.0f; break;
        case Lvgl_Ui::System::Cc1101_Bw::BW_812KHZ:  buffer_bandwidth = 812.0f; break;
        default: break;
        }

        int16_t assert = Cc1101.begin(device_cc1101.params.freq, device_cc1101.params.bit_rate, device_cc1101.params.freq_deviation_khz,
                                      buffer_bandwidth, device_cc1101.params.power, device_cc1101.params.preamble_length);
        if (assert != RADIOLIB_ERR_NONE)
        {
            printf("cc1101 begin fail (error code: %d)\n", assert);
            return false;
        }

        assert = Cc1101.setSyncWord(device_cc1101.params.sync_word >> 8, device_cc1101.params.sync_word);
        if (assert != RADIOLIB_ERR_NONE)
        {
            printf("cc1101 setSyncWord fail (error code: %d)\n", assert);
            return false;
        }

        assert = Cc1101.startReceive();
        if (assert != RADIOLIB_ERR_NONE)
            printf("cc1101 startReceive fail (error code: %d)\n", assert);

        Cc1101_Interrupt_Flag = false;
        printf("config_cc1101_params finish start cc1101 transmit\n");
        return true;
    };

    System_Ui->_win_rf_config_nrf24l01_params_callback = [](Lvgl_Ui::System::Device_Nrf24l01 device_nrf24l01) -> bool
    {
        int16_t assert = Nrf24l01.begin(device_nrf24l01.params.freq, device_nrf24l01.params.bit_rate, device_nrf24l01.params.power,
                                        device_nrf24l01.params.address_width);
        if (assert != RADIOLIB_ERR_NONE)
        {
            printf("nrf24l01 begin fail (error code: %d)\n", assert);
            return false;
        }

        uint8_t address[] = {
            static_cast<uint8_t>(device_nrf24l01.params.address >> 32),
            static_cast<uint8_t>(device_nrf24l01.params.address >> 24),
            static_cast<uint8_t>(device_nrf24l01.params.address >> 16),
            static_cast<uint8_t>(device_nrf24l01.params.address >> 8),
            static_cast<uint8_t>(device_nrf24l01.params.address),
        };
        assert = Nrf24l01.setTransmitPipe(address);
        if (assert != RADIOLIB_ERR_NONE)
        {
            printf("nrf24l01 setTransmitPipe fail (error code: %d)\n", assert);
            return false;
        }

        assert = Nrf24l01.startReceive();
        if (assert != RADIOLIB_ERR_NONE)
            printf("nrf24l01 startReceive fail (error code: %d)\n", assert);

        Nrf24l01_Interrupt_Flag = false;
        printf("config_nrf24l01_params finish start nrf24l01 transmit\n");
        return true;
    };

    System_Ui->_win_cit_otg_switch_callback = [](bool status)
    {
        Kode_Bq25896::bq25896_set_otg(Bq25896_Handle,
            status ? Kode_Bq25896::bq25896_otg_state_t::BQ25896_OTG_ENABLE
                   : Kode_Bq25896::bq25896_otg_state_t::BQ25896_OTG_DISABLE);
    };

#endif
}

void Lvgl_Init(void)
{
    printf("initialize lvgl\n");

    lv_init();

    lv_display_t *display = lv_display_create(g_screen_width, g_screen_height);
    lv_display_set_user_data(display, Screen_Mipi_Dpi_Panel);
    lv_display_set_color_format(display, LVGL_COLOR_FORMAT);

    printf("allocate separate lvgl draw buffers\n");
    // Allocate for max screen size so buffer works for either variant
    size_t draw_buffer_sz = SCREEN_WIDTH_MAX * SCREEN_HEIGHT_MAX * sizeof(lv_color_t);
    void *buf1 = heap_caps_malloc(draw_buffer_sz, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT | MALLOC_CAP_DMA);
    assert(buf1);
    lv_display_set_buffers(display, buf1, NULL, draw_buffer_sz, LV_DISPLAY_RENDER_MODE_PARTIAL);

    lv_display_set_flush_cb(display, [](lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
                            {
                                lv_display_rotation_t rotation = lv_display_get_rotation(disp);
                                esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)lv_display_get_user_data(disp);

                                int32_t offsetx1 = area->x1;
                                int32_t offsetx2 = area->x2;
                                int32_t offsety1 = area->y1;
                                int32_t offsety2 = area->y2;

                                if (rotation != LV_DISPLAY_ROTATION_0)
                                {
#if CONFIG_ENABLE_PPA_SCREEN_ROTATION == true
                                    uint32_t input_img_width = area->x2 - area->x1 + 1;
                                    uint32_t input_img_height = area->y2 - area->y1 + 1;
                                    uint32_t output_img_width = input_img_width;
                                    uint32_t output_img_height = input_img_height;

                                    if (rotation == LV_DISPLAY_ROTATION_90 || rotation == LV_DISPLAY_ROTATION_270)
                                    {
                                        output_img_width = input_img_height;
                                        output_img_height = input_img_width;
                                    }

                                    size_t output_buffer_size = output_img_width * output_img_height * (SCREEN_BITS_PER_PIXEL / 8);
                                    uint8_t *output_buffer = (uint8_t *)heap_caps_malloc(output_buffer_size, MALLOC_CAP_DMA | MALLOC_CAP_SPIRAM);
                                    if (output_buffer == NULL) { printf("failed to allocate rotated buffer\n"); return; }

                                    ppa_srm_oper_config_t srm_config = {
                                        .in = { .buffer = px_map, .pic_w = input_img_width, .pic_h = input_img_height,
                                                .block_w = input_img_width, .block_h = input_img_height, .block_offset_x = 0, .block_offset_y = 0,
#if defined CONFIG_SCREEN_PIXEL_FORMAT_RGB565
                                                .srm_cm = ppa_srm_color_mode_t::PPA_SRM_COLOR_MODE_RGB565,
#elif defined CONFIG_SCREEN_PIXEL_FORMAT_RGB888
                                                .srm_cm = ppa_srm_color_mode_t::PPA_SRM_COLOR_MODE_RGB888,
#endif
                                        },
                                        .out = { .buffer = output_buffer, .buffer_size = ALIGN_UP(output_buffer_size, data_cache_line_size_2),
                                                 .pic_w = output_img_width, .pic_h = output_img_height, .block_offset_x = 0, .block_offset_y = 0,
#if defined CONFIG_SCREEN_PIXEL_FORMAT_RGB565
                                                 .srm_cm = ppa_srm_color_mode_t::PPA_SRM_COLOR_MODE_RGB565,
#elif defined CONFIG_SCREEN_PIXEL_FORMAT_RGB888
                                                 .srm_cm = ppa_srm_color_mode_t::PPA_SRM_COLOR_MODE_RGB888,
#endif
                                        },
                                        .rotation_angle = PPA_SRM_ROTATION_ANGLE_0,
                                        .scale_x = 1, .scale_y = 1, .mirror_x = false, .mirror_y = false,
                                        .rgb_swap = false, .byte_swap = false, .mode = PPA_TRANS_MODE_BLOCKING,
                                    };

                                    switch (rotation)
                                    {
                                    case LV_DISPLAY_ROTATION_90:  srm_config.rotation_angle = PPA_SRM_ROTATION_ANGLE_90;  break;
                                    case LV_DISPLAY_ROTATION_180: srm_config.rotation_angle = PPA_SRM_ROTATION_ANGLE_180; break;
                                    case LV_DISPLAY_ROTATION_270: srm_config.rotation_angle = PPA_SRM_ROTATION_ANGLE_270; break;
                                    default: break;
                                    }

                                    esp_err_t ret = ppa_do_scale_rotate_mirror(ppa_srm_handle_2, &srm_config);
                                    if (ret != ESP_OK) { printf("ppa_do_scale_rotate_mirror fail\n"); heap_caps_free(output_buffer); return; }

                                    int32_t rx1 = offsetx1, ry1 = offsety1, rx2 = offsetx2, ry2 = offsety2;
                                    switch (rotation)
                                    {
                                    case LV_DISPLAY_ROTATION_90:
                                        rx1 = offsety1; ry1 = (int32_t)g_screen_height - offsetx2 - 1;
                                        rx2 = offsety2; ry2 = (int32_t)g_screen_height - offsetx1 - 1;
                                        break;
                                    case LV_DISPLAY_ROTATION_180:
                                        rx1 = (int32_t)g_screen_width - offsetx2 - 1; ry1 = (int32_t)g_screen_height - offsety2 - 1;
                                        rx2 = (int32_t)g_screen_width - offsetx1 - 1; ry2 = (int32_t)g_screen_height - offsety1 - 1;
                                        break;
                                    case LV_DISPLAY_ROTATION_270:
                                        rx1 = (int32_t)g_screen_width - offsety2 - 1; ry1 = offsetx1;
                                        rx2 = (int32_t)g_screen_width - offsety1 - 1; ry2 = offsetx2;
                                        break;
                                    default: break;
                                    }

                                    rx1 = (rx1 < 0) ? 0 : rx1;
                                    ry1 = (ry1 < 0) ? 0 : ry1;
                                    rx2 = (rx2 >= (int32_t)g_screen_width)  ? (int32_t)g_screen_width  - 1 : rx2;
                                    ry2 = (ry2 >= (int32_t)g_screen_height) ? (int32_t)g_screen_height - 1 : ry2;
                                    if (rx1 > rx2) { int32_t t = rx1; rx1 = rx2; rx2 = t; }
                                    if (ry1 > ry2) { int32_t t = ry1; ry1 = ry2; ry2 = t; }

                                    esp_lcd_panel_draw_bitmap(panel_handle, rx1, ry1, rx2 + 1, ry2 + 1, output_buffer);
                                    heap_caps_free(output_buffer);

#else
                                    lv_area_t rotated_area;
                                    lv_color_format_t cf = lv_display_get_color_format(disp);
                                    rotated_area = *area;
                                    lv_display_rotate_area(disp, &rotated_area);
                                    uint32_t src_stride = lv_draw_buf_width_to_stride(lv_area_get_width(area), cf);
                                    uint32_t dest_stride = lv_draw_buf_width_to_stride(lv_area_get_width(&rotated_area), cf);
                                    int32_t src_w = lv_area_get_width(area);
                                    int32_t src_h = lv_area_get_height(area);
                                    auto rotated_buf = std::make_unique<uint8_t[]>(g_screen_width * g_screen_height * (SCREEN_BITS_PER_PIXEL / 8));
                                    lv_draw_sw_rotate(px_map, rotated_buf.get(), src_w, src_h, src_stride, dest_stride, rotation, cf);
                                    area = &rotated_area;
                                    px_map = rotated_buf.get();
                                    offsetx1 = area->x1; offsetx2 = area->x2;
                                    offsety1 = area->y1; offsety2 = area->y2;
                                    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, px_map);
#endif
                                }
                                else
                                    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, px_map);

#if CONFIG_ENABLE_USB_DISPLAY == true
                                lv_display_flush_ready(disp);
#endif
                            });

    lv_indev_t *indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev, my_touchpad_read);

#if defined CONFIG_BOARD_TYPE_T_DISPLAY_P4_KEYBOARD
    lv_indev_t *indev_2 = lv_indev_create();
    lv_indev_set_type(indev_2, LV_INDEV_TYPE_KEYPAD);
    lv_indev_set_read_cb(indev_2, my_keyboard_read);
#endif

#if CONFIG_ENABLE_USB_DISPLAY == true
#else
    printf("register dpi panel event callback for lvgl flush ready notification\n");
    esp_lcd_dpi_panel_event_callbacks_t cbs = {
        .on_color_trans_done = [](esp_lcd_panel_handle_t panel, esp_lcd_dpi_panel_event_data_t *edata, void *user_ctx) -> bool
        {
            lv_display_t *disp = (lv_display_t *)user_ctx;
            lv_display_flush_ready(disp);
            return false;
        },
        .on_refresh_done = [](esp_lcd_panel_handle_t panel, esp_lcd_dpi_panel_event_data_t *edata, void *user_ctx) -> bool
        {
            return false;
        },
    };
    ESP_ERROR_CHECK(esp_lcd_dpi_panel_register_event_callbacks(Screen_Mipi_Dpi_Panel, &cbs, display));
#endif

    printf("use esp_timer as lvgl tick timer\n");
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = [](void *arg) { lv_tick_inc(LVGL_TICK_PERIOD_MS); },
        .name = "lvgl_tick"
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, LVGL_TICK_PERIOD_MS * 1000));

    lv_display_set_rotation(display, LV_DISPLAY_ROTATION);

    System_Ui_Callback_Init();
}

void Lvgl_Startup(void)
{
    lv_obj_t *bg = lv_obj_create(NULL);
    lv_obj_set_size(bg, lv_display_get_horizontal_resolution(lv_display_get_default()), lv_display_get_vertical_resolution(lv_display_get_default()));
    lv_obj_set_style_bg_color(bg, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_border_width(bg, 0, LV_PART_MAIN);

    Lvgl_Startup_Progress_Bar = lv_bar_create(bg);
    lv_obj_set_size(Lvgl_Startup_Progress_Bar, lv_pct(70), 10);
    lv_bar_set_range(Lvgl_Startup_Progress_Bar, 0, 100);
    lv_bar_set_value(Lvgl_Startup_Progress_Bar, 10, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(Lvgl_Startup_Progress_Bar, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_bg_color(Lvgl_Startup_Progress_Bar, lv_color_white(), LV_PART_INDICATOR);
    lv_obj_align(Lvgl_Startup_Progress_Bar, LV_ALIGN_CENTER, 0, 15);

    lv_obj_t *logo_label = lv_label_create(bg);
    lv_label_set_text(logo_label, "LILYGO");
    lv_obj_set_style_text_color(logo_label, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_text_font(logo_label, &lv_font_montserrat_48, LV_PART_MAIN);
    lv_obj_align_to(logo_label, Lvgl_Startup_Progress_Bar, LV_ALIGN_OUT_TOP_MID, 0, -30);

    lv_obj_update_layout(bg);
    lv_screen_load(bg);
}

void Set_Lvgl_Startup_Progress_Bar(uint8_t percentage)
{
    if (Lvgl_Startup_Progress_Bar != nullptr)
        lv_bar_set_value(Lvgl_Startup_Progress_Bar, percentage, LV_ANIM_OFF);
}

void ES8311_Init(void)
{
    ES8311->begin(MCLK_MULTIPLE, SAMPLE_RATE, i2s_data_bit_width_t::I2S_DATA_BIT_WIDTH_16BIT);

    if (ES8311->begin(50000) == true)
    {
        printf("es8311 initialization success\n");
        Sys_Status.es8311.init_flag = true;
    }
    else
    {
        printf("es8311 initialization fail\n");
        Sys_Status.es8311.init_flag = false;
    }

    ES8311->set_master_clock_source(Cpp_Bus_Driver::Es8311::Clock_Source::ADC_DAC_MCLK);
    ES8311->set_clock(Cpp_Bus_Driver::Es8311::Clock_Source::ADC_DAC_MCLK, true);
    ES8311->set_clock(Cpp_Bus_Driver::Es8311::Clock_Source::ADC_DAC_BCLK, true);
    ES8311->set_clock_coeff(MCLK_MULTIPLE, SAMPLE_RATE);
    ES8311->set_serial_port_mode(Cpp_Bus_Driver::Es8311::Serial_Port_Mode::SLAVE);
    ES8311->set_sdp_data_bit_length(Cpp_Bus_Driver::Es8311::Sdp::ADC, Cpp_Bus_Driver::Es8311::Bits_Per_Sample::DATA_16BIT);
    ES8311->set_sdp_data_bit_length(Cpp_Bus_Driver::Es8311::Sdp::DAC, Cpp_Bus_Driver::Es8311::Bits_Per_Sample::DATA_16BIT);

    Cpp_Bus_Driver::Es8311::Power_Status ps =
        {
            .contorl =
                {
                    .analog_circuits = true,
                    .analog_bias_circuits = true,
                    .analog_adc_bias_circuits = true,
                    .analog_adc_reference_circuits = true,
                    .analog_dac_reference_circuit = true,
                    .internal_reference_circuits = false,
                },
            .vmid = Cpp_Bus_Driver::Es8311::Vmid::START_UP_VMID_NORMAL_SPEED_CHARGE,
        };
    ES8311->set_power_status(ps);
    ES8311->set_pga_power(true);
    ES8311->set_adc_power(true);
    ES8311->set_dac_power(true);
    ES8311->set_output_to_hp_drive(true);
    ES8311->set_adc_offset_freeze(Cpp_Bus_Driver::Es8311::Adc_Offset_Freeze::DYNAMIC_HPF);
    ES8311->set_adc_hpf_stage2_coeff(10);
    ES8311->set_dac_equalizer(false);
    ES8311->set_mic(Cpp_Bus_Driver::Es8311::Mic_Type::ANALOG_MIC, Cpp_Bus_Driver::Es8311::Mic_Input::MIC1P_1N);
    ES8311->set_adc_auto_volume_control(false);
    ES8311->set_adc_gain(Cpp_Bus_Driver::Es8311::Adc_Gain::GAIN_18DB);
    ES8311->set_adc_pga_gain(Cpp_Bus_Driver::Es8311::Adc_Pga_Gain::GAIN_30DB);
    ES8311->set_adc_volume(191);
    ES8311->set_dac_volume(200);
}

bool ICM20948_Init(void)
{
    Wire1.begin(ICM20948_SDA, ICM20948_SCL);
    if (ICM20948->init() == false) { printf("icm20948 ag init fail\n"); return false; }
    if (ICM20948->initMagnetometer() == false) { printf("icm20948 m init fail\n"); return false; }

    printf("Position your ICM20948 flat and don't move it - calibrating...\n");
    ICM20948->autoOffsets();
    printf("Done!\n");

    ICM20948->setAccRange(ICM20948_ACC_RANGE_2G);
    ICM20948->setAccDLPF(ICM20948_DLPF_6);
    ICM20948->setMagOpMode(AK09916_CONT_MODE_20HZ);
    return true;
}

void eth_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    uint8_t mac_addr[6] = {0};
    esp_eth_handle_t eth_handle = *(esp_eth_handle_t *)event_data;

    switch (event_id)
    {
    case ETHERNET_EVENT_CONNECTED:
        esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
        printf("ethernet link up\n");
        Eth_Info.status.data = "status: link up\nhw addr: " +
            std::to_string(mac_addr[0]) + ":" + std::to_string(mac_addr[1]) + ":" +
            std::to_string(mac_addr[2]) + ":" + std::to_string(mac_addr[3]) + ":" +
            std::to_string(mac_addr[4]) + ":" + std::to_string(mac_addr[5]) + "\n";
        Eth_Info.status.update_flag = true;
        Eth_Info.link_up_flag = true;
        break;
    case ETHERNET_EVENT_DISCONNECTED:
        printf("ethernet link down\n");
        Eth_Info.status.data = "status: link down\n";
        Eth_Info.status.update_flag = true;
        Eth_Info.link_up_flag = false;
        break;
    case ETHERNET_EVENT_START:
        printf("ethernet started\n");
        Eth_Info.status.data = "status: started\n";
        Eth_Info.status.update_flag = true;
        Eth_Info.link_up_flag = false;
        break;
    case ETHERNET_EVENT_STOP:
        printf("ethernet stopped\n");
        Eth_Info.status.data = "status: stopped\n";
        Eth_Info.status.update_flag = true;
        Eth_Info.link_up_flag = false;
        break;
    default:
        break;
    }
}

void got_ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    const esp_netif_ip_info_t *ip_info = &event->ip_info;

    printf("ethernet get ip address\n~~~~~~~~~~~\n");
    printf("eth ip: %d.%d.%d.%d\neth mask: %d.%d.%d.%d\neth gw: %d.%d.%d.%d\n~~~~~~~~~~~\n",
           IP2STR(&ip_info->ip), IP2STR(&ip_info->netmask), IP2STR(&ip_info->gw));

    char ip_status_data[256];
    snprintf(ip_status_data, sizeof(ip_status_data),
             "ethernet get ip address\neth ip: %d.%d.%d.%d\neth mask: %d.%d.%d.%d\neth gw: %d.%d.%d.%d\n",
             IP2STR(&ip_info->ip), IP2STR(&ip_info->netmask), IP2STR(&ip_info->gw));

    Eth_Info.connect_ip_status.data = ip_status_data;
    Eth_Info.connect_ip_status.update_flag = true;
}

void Ethernet_Init(void)
{
    uint8_t eth_port_cnt = 0;
    esp_eth_handle_t *eth_handles;
    ESP_ERROR_CHECK(example_eth_init(&eth_handles, &eth_port_cnt));

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_t *eth_netifs[eth_port_cnt];
    esp_eth_netif_glue_handle_t eth_netif_glues[eth_port_cnt];

    if (eth_port_cnt == 1)
    {
        esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
        eth_netifs[0] = esp_netif_new(&cfg);
        eth_netif_glues[0] = esp_eth_new_netif_glue(eth_handles[0]);
        ESP_ERROR_CHECK(esp_netif_attach(eth_netifs[0], eth_netif_glues[0]));
    }
    else
    {
        esp_netif_inherent_config_t esp_netif_config = ESP_NETIF_INHERENT_DEFAULT_ETH();
        esp_netif_config_t cfg_spi = { .base = &esp_netif_config, .stack = ESP_NETIF_NETSTACK_DEFAULT_ETH };
        char if_key_str[10], if_desc_str[10], num_str[3];
        for (int i = 0; i < eth_port_cnt; i++)
        {
            itoa(i, num_str, 10);
            strcat(strcpy(if_key_str, "ETH_"), num_str);
            strcat(strcpy(if_desc_str, "eth"), num_str);
            esp_netif_config.if_key = if_key_str;
            esp_netif_config.if_desc = if_desc_str;
            esp_netif_config.route_prio -= i * 5;
            eth_netifs[i] = esp_netif_new(&cfg_spi);
            eth_netif_glues[i] = esp_eth_new_netif_glue(eth_handles[0]);
            ESP_ERROR_CHECK(esp_netif_attach(eth_netifs[i], eth_netif_glues[i]));
        }
    }

    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL));

    for (int i = 0; i < eth_port_cnt; i++)
        ESP_ERROR_CHECK(esp_eth_start(eth_handles[i]));
}

// ============================================================
// WiFi disabled — Espressif Issue #17889
// ESP-Hosted SDIO DMA corrupts heap when sharing SDMMC controller
// with SD card. WiFi functions removed until fix is available.
// TODO: Implement AT firmware TCP bridge for WiFi connectivity.
// ============================================================

#if CONFIG_ENABLE_USB_DISPLAY == true
bool Usb_Screen_Init(esp_lcd_panel_handle_t *mipi_dpi_panel)
{
    usb_display_vendor_config_t vendor_config_usb = DEFAULT_USB_DISPLAY_VENDOR_CONFIG(SCREEN_WIDTH, SCREEN_HEIGHT,
                                                                                      SCREEN_BITS_PER_PIXEL, *mipi_dpi_panel);

    if (esp_lcd_new_panel_usb_display(&vendor_config_usb, mipi_dpi_panel) != ESP_OK)
    {
        printf("esp_lcd_new_panel_usb_display fail\n");
        return false;
    }
    return true;
}
#else
void hardware_usb_cdc_task(void *arg)
{
    printf("hardware_usb_cdc_task start\n");
    while (1)
    {
        app_message_t msg;
        if (xQueueReceive(app_queue, &msg, portMAX_DELAY))
        {
            if (msg.buf_len)
                printf("data from channel %d: ", msg.itf);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
#endif

void camera_video_frame_operation(uint8_t *camera_buf, uint8_t camera_buf_index, uint32_t camera_buf_hes, uint32_t camera_buf_ves,
                                  size_t camera_buf_len, void *user_data)
{
    fps_count++;
    if (fps_count == 50)
    {
        int64_t end_time = esp_timer_get_time();
        printf("fps: %f\n", 1000000.0 / ((end_time - start_time) / 50.0));
        start_time = end_time;
        fps_count = 0;
        printf("camera_buf_hes: %lu, camera_buf_ves: %lu, camera_buf_len: %d KB\n", camera_buf_hes, camera_buf_ves, camera_buf_len / 1024);
    }

    uint32_t input_img_block_width = (camera_buf_hes - g_screen_width) / 2;
    uint32_t input_img_width = g_screen_width;
    uint32_t input_img_height = camera_buf_ves;
    uint32_t output_img_width = input_img_width;
    uint32_t output_img_height = input_img_height;

    size_t output_buffer_size = output_img_width * output_img_height * (SCREEN_BITS_PER_PIXEL / 8);
    uint8_t *output_buffer = (uint8_t *)heap_caps_malloc(output_buffer_size, MALLOC_CAP_DMA | MALLOC_CAP_SPIRAM);
    if (output_buffer == NULL) { printf("heap_caps_malloc fail\n"); return; }

    ppa_srm_oper_config_t srm_config =
        {
            .in =
                {
                    .buffer = camera_buf, .pic_w = camera_buf_hes, .pic_h = camera_buf_ves,
                    .block_w = input_img_width, .block_h = input_img_height,
                    .block_offset_x = input_img_block_width, .block_offset_y = 0,
#if (defined CONFIG_CAMERA_TYPE_SC2336) || (defined CONFIG_CAMERA_TYPE_OV2710)
#if defined CONFIG_SCREEN_PIXEL_FORMAT_RGB565
                    .srm_cm = ppa_srm_color_mode_t::PPA_SRM_COLOR_MODE_RGB565,
#elif defined CONFIG_SCREEN_PIXEL_FORMAT_RGB888
                    .srm_cm = ppa_srm_color_mode_t::PPA_SRM_COLOR_MODE_RGB888,
#endif
#elif defined CONFIG_CAMERA_TYPE_OV5645
                    .srm_cm = ppa_srm_color_mode_t::PPA_SRM_COLOR_MODE_RGB565,
#endif
                },
            .out =
                {
                    .buffer = output_buffer, .buffer_size = ALIGN_UP(output_buffer_size, data_cache_line_size),
                    .pic_w = output_img_width, .pic_h = output_img_height, .block_offset_x = 0, .block_offset_y = 0,
#if defined CONFIG_SCREEN_PIXEL_FORMAT_RGB565
                    .srm_cm = ppa_srm_color_mode_t::PPA_SRM_COLOR_MODE_RGB565,
#elif defined CONFIG_SCREEN_PIXEL_FORMAT_RGB888
                    .srm_cm = ppa_srm_color_mode_t::PPA_SRM_COLOR_MODE_RGB888,
#endif
                },
            .rotation_angle = PPA_SRM_ROTATION_ANGLE_0,
            .scale_x = 1, .scale_y = 1, .mirror_x = false,
#if defined SCREEN_ROTATION_DIRECTION_0
            .mirror_y = screen_is_hi8561(),  // HI8561 needs Y mirror in portrait
#elif defined SCREEN_ROTATION_DIRECTION_90
            .mirror_y = false,
#endif
            .rgb_swap = false, .byte_swap = false, .mode = PPA_TRANS_MODE_BLOCKING,
        };

    esp_err_t assert = ppa_do_scale_rotate_mirror(ppa_srm_handle, &srm_config);
    if (assert != ESP_OK)
    {
        printf("ppa_do_scale_rotate_mirror fail (error code: %#X)\n", assert);
        heap_caps_free(output_buffer);
        return;
    }

    if (System_Ui->get_current_win() == Lvgl_Ui::System::Current_Win::CAMERA)
    {
        assert = esp_lcd_panel_draw_bitmap(Screen_Mipi_Dpi_Panel, 0, (g_screen_height - output_img_height) / 2,
                                           output_img_width, output_img_height + ((g_screen_height - output_img_height) / 2),
                                           output_buffer);
        if (assert != ESP_OK)
        {
            printf("esp_lcd_panel_draw_bitmap fail (error code: %#X)\n", assert);
            heap_caps_free(output_buffer);
            return;
        }
    }

    heap_caps_free(output_buffer);
}

bool App_Video_Init(void)
{
    esp_lcd_panel_handle_t mipi_dpi_panel = NULL;

    if (Camera_Init(&mipi_dpi_panel) == false) { printf("Camera_Init fail\n"); return false; }

    ppa_client_config_t ppa_srm_config = { .oper_type = PPA_OPERATION_SRM };
    esp_err_t assert = ppa_register_client(&ppa_srm_config, &ppa_srm_handle);
    if (assert != ESP_OK) { printf("ppa_register_client fail\n"); return false; }

    assert = esp_cache_get_alignment(MALLOC_CAP_SPIRAM, &data_cache_line_size);
    if (assert != ESP_OK) { printf("esp_cache_get_alignment fail\n"); return false; }

    assert = app_video_main(SGM38121_IIC_Bus->get_bus_handle());
    if (assert != ESP_OK) { printf("video_init fail\n"); return false; }

#if (defined CONFIG_CAMERA_TYPE_SC2336) || (defined CONFIG_CAMERA_TYPE_OV2710)
#if defined CONFIG_SCREEN_PIXEL_FORMAT_RGB565
    video_cam_fd0 = app_video_open(EXAMPLE_CAM_DEV_PATH, video_fmt_t::APP_VIDEO_FMT_RGB565);
#elif defined CONFIG_SCREEN_PIXEL_FORMAT_RGB888
    video_cam_fd0 = app_video_open(EXAMPLE_CAM_DEV_PATH, video_fmt_t::APP_VIDEO_FMT_RGB888);
#endif
#elif defined CONFIG_CAMERA_TYPE_OV5645
    video_cam_fd0 = app_video_open(EXAMPLE_CAM_DEV_PATH, video_fmt_t::APP_VIDEO_FMT_RGB565);
#endif
    if (video_cam_fd0 < 0) { printf("video cam open fail\n"); return false; }

#if CONFIG_EXAMPLE_CAM_BUF_COUNT == 2
    assert = esp_lcd_dpi_panel_get_frame_buffer(mipi_dpi_panel, 2, &lcd_buffer[0], &lcd_buffer[1]);
#else
    assert = esp_lcd_dpi_panel_get_frame_buffer(mipi_dpi_panel, 3, &lcd_buffer[0], &lcd_buffer[1], &lcd_buffer[2]);
#endif
    if (assert != ESP_OK) { printf("esp_lcd_dpi_panel_get_frame_buffer fail\n"); return false; }

    assert = app_video_set_bufs(video_cam_fd0, CONFIG_EXAMPLE_CAM_BUF_COUNT, (const void **)lcd_buffer);
    if (assert != ESP_OK) { printf("app_video_set_bufs fail\n"); return false; }

    assert = app_video_register_frame_operation_cb(camera_video_frame_operation);
    if (assert != ESP_OK) { printf("app_video_register_frame_operation_cb fail\n"); return false; }

    assert = app_video_stream_task_start(video_cam_fd0, 0, NULL);
    if (assert != ESP_OK) { printf("app_video_stream_task_start fail\n"); return false; }

    app_video_stream_task_stop(video_cam_fd0);
    return true;
}

// PPA rotation client — always initialized so runtime rotation toggle works
bool Ppa_Screen_Rotation_Init(void)
{
    ppa_client_config_t ppa_srm_config = { .oper_type = PPA_OPERATION_SRM };
    esp_err_t assert = ppa_register_client(&ppa_srm_config, &ppa_srm_handle_2);
    if (assert != ESP_OK) { printf("ppa_register_client fail\n"); return false; }

    assert = esp_cache_get_alignment(MALLOC_CAP_SPIRAM, &data_cache_line_size_2);
    if (assert != ESP_OK) { printf("esp_cache_get_alignment fail\n"); return false; }

    return true;
}

void System_Startup_Message_Init(void)
{
    auto show_msg = [](const char *msg)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
        _lock_acquire(&lvgl_api_lock);
        System_Ui->create_system_message_box(lv_screen_active(), "device massage", msg);
        _lock_release(&lvgl_api_lock);
        while (System_Ui->_registry.system_message_box.occupancy_flag == true)
            vTaskDelay(pdMS_TO_TICKS(10));
    };

    if (!Sys_Status.sgm38121.init_flag)  show_msg("sgm38121 init fail");
    if (!Sys_Status.camera.init_flag)    show_msg("camera init fail");
    // esp32c6 and wifi checks skipped — WiFi is initialized AFTER this
    // function returns, so these would always block waiting for dismiss.

#if defined CONFIG_BOARD_TYPE_T_DISPLAY_P4_KEYBOARD
    if (!Sys_Status.xl9555.init_flag)    show_msg("xl9555 init fail");
    if (!Sys_Status.tca8418.init_flag)   show_msg("tca8418 init fail");
    if (!Sys_Status.st25r3916.init_flag) show_msg("st25r3916 init fail");
    if (!Sys_Status.cc1101.init_flag)    show_msg("cc1101 init fail");
    if (!Sys_Status.nrf24l01.init_flag)  show_msg("nrf24l01 init fail");
    if (!Sys_Status.bq25896.init_flag)   show_msg("bq25896 init fail");
#endif

    if (!Sys_Status.pcf8563.init_flag)   show_msg("pcf8563 init fail");
    if (!Sys_Status.bq27220.init_flag)   show_msg("bq27220 init fail");
    if (!Sys_Status.aw86224.init_flag)   show_msg("aw86224 init fail");
    if (!Sys_Status.es8311.init_flag)    show_msg("es8311 init fail");
    if (!Sys_Status.icm20948.init_flag)  show_msg("icm20948 init fail");
    if (!Sys_Status.l76k.init_flag)      show_msg("l76k init fail");
    if (!Sys_Status.sx1262.init_flag)    show_msg("sx1262 init fail");
}

typedef enum { APP_EVENT = 0, } app_event_group_t;
typedef struct { app_event_group_t event_group; } app_event_queue_t;

static void gpio_cb(void *arg)
{
    const app_event_queue_t evt_queue = { .event_group = APP_EVENT };
    BaseType_t xTaskWoken = pdFALSE;
    if (app_event_queue)
        xQueueSendFromISR(app_event_queue, &evt_queue, &xTaskWoken);
    if (xTaskWoken == pdTRUE)
        portYIELD_FROM_ISR();
}

#ifdef ENABLE_ENUM_FILTER_CALLBACK
static bool set_config_cb(const usb_device_desc_t *dev_desc, uint8_t *bConfigurationValue)
{
    *bConfigurationValue = (dev_desc->bNumConfigurations > 1) ? 2 : 1;
    return true;
}
#endif

static void usb_host_lib_task(void *arg)
{
    ESP_LOGI(TAG, "Installing USB Host Library");
    usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
#ifdef ENABLE_ENUM_FILTER_CALLBACK
        .enum_filter_cb = set_config_cb,
#endif
    };
    // Retry USB host install — may fail initially if WiFi SDIO init
    // hasn't released transient DMA buffers yet.
    esp_err_t err;
    for (int attempt = 0; attempt < 10; attempt++) {
        err = usb_host_install(&host_config);
        if (err == ESP_OK) break;
        ESP_LOGW(TAG, "usb_host_install attempt %d failed (0x%x), free internal: %u, retrying...",
                 attempt + 1, err, heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "usb_host_install failed after retries — ADS-B unavailable");
        xTaskNotifyGive(arg);
        vTaskDelete(NULL);
        return;
    }

    xTaskNotifyGive(arg);

    bool has_clients = true;
    bool has_devices = false;
    while (has_clients)
    {
        uint32_t event_flags;
        ESP_ERROR_CHECK(usb_host_lib_handle_events(portMAX_DELAY, &event_flags));
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS)
        {
            ESP_LOGI(TAG, "Get FLAGS_NO_CLIENTS");
            if (ESP_OK == usb_host_device_free_all())
                has_clients = false;
            else
                has_devices = true;
        }
        if (has_devices && event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE)
        {
            ESP_LOGI(TAG, "Get FLAGS_ALL_FREE");
            has_clients = false;
        }
    }
    ESP_LOGI(TAG, "No more clients and devices, uninstall USB Host library");
    ESP_ERROR_CHECK(usb_host_uninstall());
    vTaskSuspend(NULL);
}

// ============================================================
// rtlsdr_adsb_start — NON-BLOCKING
// Spawns USB host and class driver tasks, then returns.
// The blocking event-loop from the original code is gone;
// BOOT button shutdown is handled by a lightweight watcher task.
// ============================================================

static void usb_quit_watcher_task(void *arg)
{
    app_event_queue_t evt_queue;
    while (1)
    {
        if (xQueueReceive(app_event_queue, &evt_queue, portMAX_DELAY))
        {
            if (APP_EVENT == evt_queue.event_group)
            {
                usb_host_lib_info_t lib_info;
                ESP_ERROR_CHECK(usb_host_lib_info(&lib_info));
                if (lib_info.num_devices != 0)
                    ESP_LOGW(TAG, "Shutdown with attached devices.");

                class_driver_client_deregister();
                vTaskDelay(10);

                if (s_class_driver_task_hdl) vTaskDelete(s_class_driver_task_hdl);
                if (s_host_lib_task_hdl)     vTaskDelete(s_host_lib_task_hdl);

                gpio_isr_handler_remove(APP_QUIT_PIN);
                xQueueReset(app_event_queue);
                ESP_LOGI(TAG, "USB host shut down");
                vTaskDelete(NULL);
                return;
            }
        }
    }
}

void rtlsdr_adsb_start(void)
{
    ESP_LOGI(TAG, "Starting RTL-SDR ADS-B USB host");

    const gpio_config_t input_pin = {
        .pin_bit_mask = BIT64(APP_QUIT_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&input_pin));
    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1));
    ESP_ERROR_CHECK(gpio_isr_handler_add(APP_QUIT_PIN, gpio_cb, NULL));

    app_event_queue = xQueueCreate(10, sizeof(app_event_queue_t));

    BaseType_t task_created;
    task_created = xTaskCreatePinnedToCore(usb_host_lib_task,
                                           "usb_host",
                                           8192,
                                           xTaskGetCurrentTaskHandle(),
                                           HOST_LIB_TASK_PRIORITY,
                                           &s_host_lib_task_hdl,
                                           0);
    assert(task_created == pdTRUE);

    // Wait until the USB host library is installed
    ulTaskNotifyTake(false, 1000);

    task_created = xTaskCreatePinnedToCore(class_driver_task,
                                           "class",
                                           16 * 1024,
                                           NULL,
                                           CLASS_TASK_PRIORITY,
                                           &s_class_driver_task_hdl,
                                           0);
    assert(task_created == pdTRUE);
    vTaskDelay(10);

    // Lightweight watcher — handles BOOT button press to cleanly shut down USB
    xTaskCreate(usb_quit_watcher_task, "usb_quit", 2048, NULL, 1, NULL);

    ESP_LOGI(TAG, "RTL-SDR ADS-B tasks started");
}

extern "C" void app_main(void)
{
    printf("Hello world!\n");
    printf("[MEM] boot start: internal=%u PSRAM=%u\n",
           heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
           heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    XL9535->begin();

    XL9535->pin_mode(XL9535_SCREEN_RST, Cpp_Bus_Driver::Xl95x5::Mode::OUTPUT);
    XL9535->pin_mode(XL9535_TOUCH_RST, Cpp_Bus_Driver::Xl95x5::Mode::OUTPUT);

    XL9535->pin_write(XL9535_SCREEN_RST, Cpp_Bus_Driver::Xl95x5::Value::LOW);
    XL9535->pin_write(XL9535_TOUCH_RST, Cpp_Bus_Driver::Xl95x5::Value::LOW);

    XL9535->pin_mode(XL9535_ESP32P4_VCCA_POWER_EN, Cpp_Bus_Driver::Xl95x5::Mode::OUTPUT);
    XL9535->pin_mode(XL9535_5_0_V_POWER_EN, Cpp_Bus_Driver::Xl95x5::Mode::OUTPUT);
    XL9535->pin_mode(XL9535_3_3_V_POWER_EN, Cpp_Bus_Driver::Xl95x5::Mode::OUTPUT);

    XL9535->pin_mode(XL9535_GPS_WAKE_UP, Cpp_Bus_Driver::Xl95x5::Mode::OUTPUT);
    XL9535->pin_write(XL9535_GPS_WAKE_UP, Cpp_Bus_Driver::Xl95x5::Value::LOW);
    XL9535->pin_mode(XL9535_ESP32C6_EN, Cpp_Bus_Driver::Xl95x5::Mode::OUTPUT);
    XL9535->pin_write(XL9535_ESP32C6_EN, Cpp_Bus_Driver::Xl95x5::Value::LOW);

    XL9535->pin_write(XL9535_ESP32P4_VCCA_POWER_EN, Cpp_Bus_Driver::Xl95x5::Value::LOW);
    XL9535->pin_write(XL9535_5_0_V_POWER_EN, Cpp_Bus_Driver::Xl95x5::Value::HIGH);
    XL9535->pin_write(XL9535_3_3_V_POWER_EN, Cpp_Bus_Driver::Xl95x5::Value::LOW);
    vTaskDelay(pdMS_TO_TICKS(200));
    XL9535->pin_write(XL9535_5_0_V_POWER_EN, Cpp_Bus_Driver::Xl95x5::Value::LOW);
    XL9535->pin_write(XL9535_3_3_V_POWER_EN, Cpp_Bus_Driver::Xl95x5::Value::HIGH);
    vTaskDelay(pdMS_TO_TICKS(200));
    XL9535->pin_write(XL9535_5_0_V_POWER_EN, Cpp_Bus_Driver::Xl95x5::Value::HIGH);
    XL9535->pin_write(XL9535_3_3_V_POWER_EN, Cpp_Bus_Driver::Xl95x5::Value::LOW);
    vTaskDelay(pdMS_TO_TICKS(200));

    XL9535->pin_write(XL9535_SCREEN_RST, Cpp_Bus_Driver::Xl95x5::Value::HIGH);
    XL9535->pin_write(XL9535_TOUCH_RST, Cpp_Bus_Driver::Xl95x5::Value::HIGH);
    vTaskDelay(pdMS_TO_TICKS(200));
    XL9535->pin_write(XL9535_SCREEN_RST, Cpp_Bus_Driver::Xl95x5::Value::LOW);
    XL9535->pin_write(XL9535_TOUCH_RST, Cpp_Bus_Driver::Xl95x5::Value::LOW);
    vTaskDelay(pdMS_TO_TICKS(200));
    XL9535->pin_write(XL9535_SCREEN_RST, Cpp_Bus_Driver::Xl95x5::Value::HIGH);
    XL9535->pin_write(XL9535_TOUCH_RST, Cpp_Bus_Driver::Xl95x5::Value::HIGH);
    vTaskDelay(pdMS_TO_TICKS(200));

    XL9535->pin_mode(XL9535_ETHERNET_RST, Cpp_Bus_Driver::Xl95x5::Mode::OUTPUT);
    XL9535->pin_write(XL9535_ETHERNET_RST, Cpp_Bus_Driver::Xl95x5::Value::HIGH);

    // --- Runtime screen detection via I2C probe ---
    // Both touch ICs share I2C_NUM_0 (SDA=7, SCL=8) with XL9535.
    // GT9895 (AMOLED) is at 0x5D; HI8561 (LCD) touch is at 0x68.
    // Probe GT9895 first — if begin() succeeds, this is the AMOLED variant.
    // Detection MUST happen before Screen_Init_Runtime() and backlight PWM.
    {
        GT9895_IIC_Bus->set_bus_handle(XL9535_IIC_Bus->get_bus_handle());
        HI8561_T_IIC_Bus->set_bus_handle(XL9535_IIC_Bus->get_bus_handle());

        bool gt9895_found = GT9895->begin();
        if (gt9895_found) {
            g_screen_type = SCREEN_TYPE_RM69A10;
            g_screen_width = RM69A10_SCREEN_WIDTH;
            g_screen_height = RM69A10_SCREEN_HEIGHT;
        } else {
            HI8561_T->begin();
            g_screen_type = SCREEN_TYPE_HI8561;
            g_screen_width = HI8561_SCREEN_WIDTH;
            g_screen_height = HI8561_SCREEN_HEIGHT;
        }
        printf("[SCREEN] Detected: %s\n", screen_type_name());

        // Update UI layout dimensions to match detected screen
#if defined SCREEN_ROTATION_DIRECTION_0
        System_Ui->set_screen_size(g_screen_width, g_screen_height);
#elif defined SCREEN_ROTATION_DIRECTION_90
        System_Ui->set_screen_size(g_screen_height, g_screen_width);
#endif
    }

    // Kill SD card power immediately — resets the card's internal state
    // machine if a previous hard reset left it stuck mid-transaction.
    // Power stays off through the rest of peripheral init (~5 seconds),
    // giving caps plenty of time to drain.  Re-enabled just before mount.
    XL9535->pin_mode(XL9535_SD_EN, Cpp_Bus_Driver::Xl95x5::Mode::OUTPUT);
    XL9535->pin_write(XL9535_SD_EN, Cpp_Bus_Driver::Xl95x5::Value::HIGH);  // power OFF

    Ethernet_Init();

    // HI8561 uses PWM for backlight; RM69A10 uses MIPI DSI brightness command
    if (screen_is_hi8561()) {
        HI8561_T->create_pwm(HI8561_SCREEN_BL, ledc_channel_t::LEDC_CHANNEL_0, 2000);
    }

    if (SGM38121->begin() == false)
    {
        printf("sgm38121 init fail\n");
        Sys_Status.sgm38121.init_flag = false;
    }
    else
    {
        printf("sgm38121 init success\n");
        Sys_Status.sgm38121.init_flag = true;
    }

#if defined CONFIG_CAMERA_TYPE_SC2336
    SGM38121->set_output_voltage(Cpp_Bus_Driver::Sgm38121::Channel::AVDD_1, 1800);
    SGM38121->set_output_voltage(Cpp_Bus_Driver::Sgm38121::Channel::AVDD_2, 2800);
    SGM38121->set_channel_status(Cpp_Bus_Driver::Sgm38121::Channel::AVDD_1, Cpp_Bus_Driver::Sgm38121::Status::ON);
    SGM38121->set_channel_status(Cpp_Bus_Driver::Sgm38121::Channel::AVDD_2, Cpp_Bus_Driver::Sgm38121::Status::ON);
#elif defined CONFIG_CAMERA_TYPE_OV2710
    SGM38121->set_output_voltage(Cpp_Bus_Driver::Sgm38121::Channel::DVDD_1, 1500);
    SGM38121->set_output_voltage(Cpp_Bus_Driver::Sgm38121::Channel::AVDD_1, 1700);
    SGM38121->set_output_voltage(Cpp_Bus_Driver::Sgm38121::Channel::AVDD_2, 3000);
    SGM38121->set_channel_status(Cpp_Bus_Driver::Sgm38121::Channel::DVDD_1, Cpp_Bus_Driver::Sgm38121::Status::ON);
    SGM38121->set_channel_status(Cpp_Bus_Driver::Sgm38121::Channel::AVDD_1, Cpp_Bus_Driver::Sgm38121::Status::ON);
    SGM38121->set_channel_status(Cpp_Bus_Driver::Sgm38121::Channel::AVDD_2, Cpp_Bus_Driver::Sgm38121::Status::ON);
#elif defined CONFIG_CAMERA_TYPE_OV5645
    SGM38121->set_output_voltage(Cpp_Bus_Driver::Sgm38121::Channel::DVDD_1, 1500);
    SGM38121->set_output_voltage(Cpp_Bus_Driver::Sgm38121::Channel::AVDD_1, 1800);
    SGM38121->set_output_voltage(Cpp_Bus_Driver::Sgm38121::Channel::AVDD_2, 2800);
    SGM38121->set_channel_status(Cpp_Bus_Driver::Sgm38121::Channel::DVDD_1, Cpp_Bus_Driver::Sgm38121::Status::ON);
    SGM38121->set_channel_status(Cpp_Bus_Driver::Sgm38121::Channel::AVDD_1, Cpp_Bus_Driver::Sgm38121::Status::ON);
    SGM38121->set_channel_status(Cpp_Bus_Driver::Sgm38121::Channel::AVDD_2, Cpp_Bus_Driver::Sgm38121::Status::ON);
#else
#error "unknown macro definition"
#endif

    Init_Ldo_Channel_Power(3, 1830);
    vTaskDelay(pdMS_TO_TICKS(100));

    // App_Video_Init must run BEFORE Screen_Init — camera creates its own
    // MIPI-CSI panel and must complete before the DSI panel is configured.
    // This matches the stock LILYGO init order.
    if (App_Video_Init() == false)
    {
        printf("App_Video_Init fail\n");
        Sys_Status.camera.init_flag = false;
    }
    else
    {
        printf("App_Video_Init success\n");
        Sys_Status.camera.init_flag = true;
    }

    // Always init PPA rotation engine — needed for runtime rotation toggle
    if (Ppa_Screen_Rotation_Init() == false)
        printf("Ppa_Screen_Rotation_init fail\n");
    else
        printf("Ppa_Screen_Rotation_init success\n");

#if CONFIG_ENABLE_USB_DISPLAY == true
    Usb_Screen_Init(&Screen_Mipi_Dpi_Panel);
#else
    Screen_Init_Runtime(&Screen_Mipi_Dpi_Panel);
#endif

    // Stock LilyGO init sequence: Screen_Init() → esp_lcd_panel_init() only.
    // Do NOT call esp_lcd_panel_reset() or esp_lcd_panel_disp_on_off() here —
    // reset wipes the DSI lane config and panel commands that Screen_Init() sent.
    esp_err_t assert = esp_lcd_panel_init(Screen_Mipi_Dpi_Panel);
    if (assert != ESP_OK) printf("esp_lcd_panel_init fail (error code: %#X)\n", assert);

    // Touch driver already initialized in screen detection above

    // ESP32C6_AT->begin() is intentionally skipped — we use ESP-Hosted
    // protocol instead of AT commands for WiFi. Calling AT begin() would
    // put the C6 into AT mode and conflict with ESP-Hosted transport.
    Sys_Status.esp32c6.init_flag = true;  // ESP-Hosted inits automatically

    // Re-enable SD card power — card has been off since early in boot,
    // so its state machine is fully reset.
    XL9535->pin_write(XL9535_SD_EN, Cpp_Bus_Driver::Xl95x5::Value::LOW);   // power ON
    vTaskDelay(pdMS_TO_TICKS(100));  // let card power up and stabilize

    bool sd_mounted = Sdmmc_Init(SD_BASE_PATH, 3);
    if (!sd_mounted)
        printf("Sdmmc_Init fail -- wallpaper resources unavailable\n");
    else
        esp_register_shutdown_handler(sd_safe_shutdown);

    // ---------------------------------------------------------------
    // RTL-SDR / ADS-B USB host moved to after WiFi init — both need
    // internal DMA RAM, WiFi SDIO must claim its share first.
    // ---------------------------------------------------------------

    // ---------------------------------------------------------------
    // Now init LVGL and the startup screen.
    // ---------------------------------------------------------------
    Lvgl_Init();
    Lvgl_Startup();
    // Allocate lvgl_ui_task stack from PSRAM to avoid exhausting internal RAM.
    // Internal RAM is needed by USB host, DMA, and the DSI framebuffer.
    printf("[MEM] before LVGL task: internal=%u\n", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    BaseType_t task_ret = xTaskCreateWithCaps(lvgl_ui_task, "lvgl_ui_task",
        64 * 1024, NULL, 1, NULL, MALLOC_CAP_SPIRAM);
    if (task_ret != pdPASS) {
        printf("ERROR: lvgl_ui_task create failed (%d) -- falling back to internal RAM\n", task_ret);
        xTaskCreate(lvgl_ui_task, "lvgl_ui_task", 32 * 1024, NULL, 1, NULL);
    }

#if defined CONFIG_BOARD_TYPE_T_DISPLAY_P4_KEYBOARD
    if (XL9555->begin() == false)
    {
        printf("xl9555 init fail\n");
        Sys_Status.xl9555.init_flag = false;
    }
    else
    {
        printf("xl9555 init success\n");
        Sys_Status.xl9555.init_flag = true;
    }

    XL9555->pin_mode(XL9555_LED_1, Cpp_Bus_Driver::Xl95x5::Mode::OUTPUT);
    XL9555->pin_mode(XL9555_LED_2, Cpp_Bus_Driver::Xl95x5::Mode::OUTPUT);
    XL9555->pin_mode(XL9555_LED_3, Cpp_Bus_Driver::Xl95x5::Mode::OUTPUT);
    XL9555->pin_write(XL9555_LED_1, Cpp_Bus_Driver::Xl95x5::Value::HIGH);
    XL9555->pin_write(XL9555_LED_2, Cpp_Bus_Driver::Xl95x5::Value::HIGH);
    XL9555->pin_write(XL9555_LED_3, Cpp_Bus_Driver::Xl95x5::Value::HIGH);

    XL9555->pin_mode(XL9555_TCA8418_RST, Cpp_Bus_Driver::Xl95x5::Mode::OUTPUT);
    XL9555->pin_write(XL9555_TCA8418_RST, Cpp_Bus_Driver::Xl95x5::Value::HIGH);
    vTaskDelay(pdMS_TO_TICKS(10));
    XL9555->pin_write(XL9555_TCA8418_RST, Cpp_Bus_Driver::Xl95x5::Value::LOW);
    vTaskDelay(pdMS_TO_TICKS(10));
    XL9555->pin_write(XL9555_TCA8418_RST, Cpp_Bus_Driver::Xl95x5::Value::HIGH);
    vTaskDelay(pdMS_TO_TICKS(10));

    TCA8418->create_gpio_interrupt(TCA8418_INT, Cpp_Bus_Driver::Tool::Interrupt_Mode::FALLING,
                                   [](void *arg) -> IRAM_ATTR void { TCA8418_Interrupt_Flag = true; });

    if (TCA8418->begin() == false)
    {
        printf("tca8418 init fail\n");
        Sys_Status.tca8418.init_flag = false;
    }
    else
    {
        printf("tca8418 init success\n");
        Sys_Status.tca8418.init_flag = true;
    }
    TCA8418->set_keypad_scan_window(0, 0, TCA8418_KEYPAD_SCAN_WIDTH, TCA8418_KEYPAD_SCAN_HEIGHT);
    TCA8418->set_irq_pin_mode(Cpp_Bus_Driver::Tca8418::Irq_Mask::KEY_EVENTS);
    TCA8418->clear_irq_flag(Cpp_Bus_Driver::Tca8418::Irq_Flag::KEY_EVENTS);
    TCA8418->create_pwm(KEYBOARD_BL, ledc_channel_t::LEDC_CHANNEL_1, 20000);
    TCA8418->start_pwm_gradient_time(30, 1000);

    XL9555->pin_mode(XL9555_T_MIXRF_EN, Cpp_Bus_Driver::Xl95x5::Mode::OUTPUT);
    XL9555->pin_write(XL9555_T_MIXRF_EN, Cpp_Bus_Driver::Xl95x5::Value::HIGH);

    if (St25r3916_Init() == false)
    {
        printf("st25r3916 init fail\n");
        Sys_Status.st25r3916.init_flag = false;
    }
    else
    {
        printf("st25r3916 init success\n");
        Sys_Status.st25r3916.init_flag = true;
    }

    Set_T_Mixrf_Lr1121_Sleep();

    XL9555->pin_mode(XL9555_T_MIXRF_CC1101_RF_SWITCH_0, Cpp_Bus_Driver::Xl95x5::Mode::OUTPUT);
    XL9555->pin_mode(XL9555_T_MIXRF_CC1101_RF_SWITCH_1, Cpp_Bus_Driver::Xl95x5::Mode::OUTPUT);
    ESP32P4->pin_mode(T_MIXRF_CC1101_BUSY, Cpp_Bus_Driver::Tool::Pin_Mode::INPUT, Cpp_Bus_Driver::Tool::Pin_Status::PULLDOWN);
    ESP32P4->create_gpio_interrupt(T_MIXRF_CC1101_INT, Cpp_Bus_Driver::Tool::Interrupt_Mode::RISING,
                                   [](void *arg) -> IRAM_ATTR void { Cc1101_Interrupt_Flag = true; });

    Cc1101_SPI_Bus->_bus_init_flag = true;
    int16_t assert_2 = Cc1101.begin();
    Sys_Status.cc1101.init_flag = (assert_2 == RADIOLIB_ERR_NONE);
    printf("cc1101 init %s\n", Sys_Status.cc1101.init_flag ? "success" : "fail");

    System_Ui->set_config_rf_params(System_Ui->_device_cc1101);

    ESP32P4->create_gpio_interrupt(T_MIXRF_NRF24L01_INT, Cpp_Bus_Driver::Tool::Interrupt_Mode::FALLING,
                                   [](void *arg) -> IRAM_ATTR void { Nrf24l01_Interrupt_Flag = true; });

    Nrf24l01_SPI_Bus->_bus_init_flag = true;
    assert_2 = Nrf24l01.begin();
    Sys_Status.nrf24l01.init_flag = (assert_2 == RADIOLIB_ERR_NONE);
    printf("nrf24l01 init %s\n", Sys_Status.nrf24l01.init_flag ? "success" : "fail");

    System_Ui->set_config_rf_params(System_Ui->_device_nrf24l01);

    assert = Kode_Bq25896::bq25896_init(Bq25896_Iic_Bus, Bq25896_Handle);
    if (assert != ESP_OK)
    {
        Sys_Status.bq25896.init_flag = false;
        printf("bq25896 init fail (error code: %#X)\n", assert);
    }
    else
    {
        Sys_Status.bq25896.init_flag = true;
        printf("bq25896 init success\n");
        Kode_Bq25896::bq25896_set_watchdog_timer(Bq25896_Handle, Kode_Bq25896::bq25896_watchdog_t::BQ25896_WATCHDOG_DISABLE);
        Kode_Bq25896::bq25896_set_adc_conversion(Bq25896_Handle, Kode_Bq25896::bq25896_adc_conv_state_t::BQ25896_ADC_CONV_START);
        Kode_Bq25896::bq25896_set_adc_conversion_rate(Bq25896_Handle, Kode_Bq25896::bq25896_adc_conv_rate_t::BQ25896_ADC_CONV_RATE_CONTINUOUS);
    }
#endif

#if CONFIG_ENABLE_USB_DISPLAY == true
#else
    if (screen_is_hi8561()) {
        HI8561_T->start_pwm_gradient_time(100, 500);
    } else {
        for (uint8_t i = 0; i < 255; i += 5)
        {
            set_rm69a10_brightness(Screen_Mipi_Dpi_Panel, i);
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
#endif

    PCF8563_IIC_Bus->set_bus_handle(XL9535_IIC_Bus->get_bus_handle());

    if (PCF8563->begin() == false)
    {
        printf("pcf8563 init fail\n");
        Sys_Status.pcf8563.init_flag = false;
    }
    else
    {
        printf("pcf8563 init success\n");
        Sys_Status.pcf8563.init_flag = true;
    }

    _lock_acquire(&lvgl_api_lock);
    Set_Lvgl_Startup_Progress_Bar(20);
    _lock_release(&lvgl_api_lock);

    // ICM20948 init runs here before other I2C peripherals
    Wire1._bus->set_bus_handle(SGM38121_IIC_Bus->get_bus_handle());
    if (ICM20948_Init() == false)
    {
        printf("icm20948 init fail\n");
        Sys_Status.icm20948.init_flag = false;
    }
    else
    {
        printf("icm20948 init success\n");
        printf("[MEM] after ICM20948: internal=%u\n", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
        Sys_Status.icm20948.init_flag = true;
    }

    // ESP-Hosted WiFi + SNTP time sync — moved to end of boot
    // to avoid SDIO DMA heap corruption during peripheral init.

    // Start HTTP file server — task waits for STA netif to get an IP
    // (also moved to end of boot, after WiFi init)

    _lock_acquire(&lvgl_api_lock);
    Set_Lvgl_Startup_Progress_Bar(40);
    _lock_release(&lvgl_api_lock);

    BQ27220_IIC_Bus->set_bus_handle(XL9535_IIC_Bus->get_bus_handle());

    if (BQ27220->begin() == false)
    {
        printf("bq27220 init fail\n");
        Sys_Status.bq27220.init_flag = false;
    }
    else
    {
        printf("bq27220 init success\n");
        Sys_Status.bq27220.init_flag = true;
    }

    BQ27220->set_design_capacity(1000);
    BQ27220->set_temperature_mode(Cpp_Bus_Driver::Bq27220xxxx::Temperature_Mode::EXTERNAL_NTC);
    BQ27220->set_sleep_current_threshold(50);

    _lock_acquire(&lvgl_api_lock);
    Set_Lvgl_Startup_Progress_Bar(50);
    _lock_release(&lvgl_api_lock);

    AW86224_IIC_Bus->set_bus_handle(SGM38121_IIC_Bus->get_bus_handle());

    if (AW86224->begin(500000) == false)
    {
        printf("aw86224 init fail\n");
        Sys_Status.aw86224.init_flag = false;
    }
    else
    {
        printf("aw86224 init success\n");
        Sys_Status.aw86224.init_flag = true;
    }
    AW86224->init_ram_mode(Cpp_Bus_Driver::aw862xx_haptic_ram_12k_0809_170, sizeof(Cpp_Bus_Driver::aw862xx_haptic_ram_12k_0809_170));

    _lock_acquire(&lvgl_api_lock);
    Set_Lvgl_Startup_Progress_Bar(60);
    _lock_release(&lvgl_api_lock);

    ES8311_IIC_Bus->set_bus_handle(SGM38121_IIC_Bus->get_bus_handle());
    ES8311_Init();

    _lock_acquire(&lvgl_api_lock);
    Set_Lvgl_Startup_Progress_Bar(70);
    _lock_release(&lvgl_api_lock);

    // ICM20948 init already done above

    _lock_acquire(&lvgl_api_lock);
    Set_Lvgl_Startup_Progress_Bar(80);
    _lock_release(&lvgl_api_lock);

    XL9535->pin_write(XL9535_GPS_WAKE_UP, Cpp_Bus_Driver::Xl95x5::Value::HIGH);
    if (L76K->begin() == false)
    {
        L76K_Uart_Bus->set_baud_rate(115200);
        Sys_Status.l76k.init_flag = (L76K->begin() == true);
    }
    else
    {
        Sys_Status.l76k.init_flag = true;
        L76K->set_baud_rate(Cpp_Bus_Driver::L76k::Baud_Rate::BR_115200_BPS);
    }
    printf("l76k init %s (baud: %ld)\n", Sys_Status.l76k.init_flag ? "success" : "fail", L76K->get_baud_rate());
    L76K->set_update_frequency(Cpp_Bus_Driver::L76k::Update_Freq::FREQ_5HZ);
    L76K->clear_rx_buffer_data();
    L76K->sleep(false);  // Keep GPS awake — ADS-B needs continuous receiver position

    _lock_acquire(&lvgl_api_lock);
    Set_Lvgl_Startup_Progress_Bar(90);
    _lock_release(&lvgl_api_lock);

    XL9535->pin_mode(XL9535_SX1262_DIO1, Cpp_Bus_Driver::Xl95x5::Mode::INPUT);
    XL9535->pin_mode(XL9535_SX1262_RST, Cpp_Bus_Driver::Xl95x5::Mode::OUTPUT);
    XL9535->pin_write(XL9535_SX1262_RST, Cpp_Bus_Driver::Xl95x5::Value::HIGH);
    vTaskDelay(pdMS_TO_TICKS(10));
    XL9535->pin_write(XL9535_SX1262_RST, Cpp_Bus_Driver::Xl95x5::Value::LOW);
    vTaskDelay(pdMS_TO_TICKS(10));
    XL9535->pin_write(XL9535_SX1262_RST, Cpp_Bus_Driver::Xl95x5::Value::HIGH);
    vTaskDelay(pdMS_TO_TICKS(10));

    XL9535->pin_mode(XL9535_SKY13453_VCTL, Cpp_Bus_Driver::Xl95x5::Mode::OUTPUT);
    XL9535->pin_mode(XL9535_SX1262_DIO1, Cpp_Bus_Driver::Xl95x5::Mode::INPUT);

#if defined CONFIG_BOARD_TYPE_T_DISPLAY_P4_KEYBOARD
    SX1262_SPI_Bus->_bus_init_flag = true;
#endif

    if (SX1262->begin(10000000) == false)
    {
        printf("sx1262 begin fail\n");
        Sys_Status.sx1262.init_flag = false;
    }
    else
    {
        printf("sx1262 begin success\n");
        Sys_Status.sx1262.init_flag = true;
    }

    System_Ui->set_config_rf_params(System_Ui->_device_sx1262);

    _lock_acquire(&lvgl_api_lock);
    Set_Lvgl_Startup_Progress_Bar(100);
    _lock_release(&lvgl_api_lock);

    _lock_acquire(&lvgl_api_lock);
    System_Ui->begin(false);  // wallpaper disabled — no images on SD card
    _lock_release(&lvgl_api_lock);

    // ---------------------------------------------------------------
    // ADSB table — created on the live screen after System_Ui->begin()
    // Column structure set here at creation time so LVGL never needs
    // to free/realloc the column array later (avoids crash if USB DMA
    // corrupts adjacent heap metadata during RTL-SDR init).
    // ---------------------------------------------------------------
    _lock_acquire(&lvgl_api_lock);
    lv_obj_t *adsb_table = lv_table_create(lv_scr_act());
    lv_table_set_column_count(adsb_table, 7);
    lv_table_set_column_width(adsb_table, 0, 65);   // ICAO
    lv_table_set_column_width(adsb_table, 1, 65);   // Callsign
    lv_table_set_column_width(adsb_table, 2, 55);   // Alt
    lv_table_set_column_width(adsb_table, 3, 50);   // Speed
    lv_table_set_column_width(adsb_table, 4, 50);   // Heading
    lv_table_set_column_width(adsb_table, 5, 55);   // Distance
    lv_table_set_column_width(adsb_table, 6, 50);   // Bearing
    lv_table_set_cell_value(adsb_table, 0, 0, "ICAO");
    lv_table_set_cell_value(adsb_table, 0, 1, "Call");
    lv_table_set_cell_value(adsb_table, 0, 2, "Alt");
    lv_table_set_cell_value(adsb_table, 0, 3, "Spd");
    lv_table_set_cell_value(adsb_table, 0, 4, "Hdg");
    lv_table_set_cell_value(adsb_table, 0, 5, "Dist");
    lv_table_set_cell_value(adsb_table, 0, 6, "Brg");
    lv_obj_set_size(adsb_table, 480, 400);
    lv_obj_align(adsb_table, LV_ALIGN_TOP_MID, 0, 0);
    lv_timer_create([](lv_timer_t *t) {
        adsb_update_display((lv_obj_t *)lv_timer_get_user_data(t));
    }, 1000, adsb_table);
    _lock_release(&lvgl_api_lock);

    xTaskCreate(device_vibration_task,   "device_vibration_task",   4 * 1024, NULL, 2, &Vibration_Task_Handle);
    xTaskCreate(device_speaker_task,     "device_speaker_task",     4 * 1024, NULL, 3, &Speaker_Task_Handle);
    xTaskCreate(device_microphone_task,  "device_microphone_task",  4 * 1024, NULL, 3, &Microphone_Task_Handle);
    xTaskCreate(device_imu_task,         "device_imu_task",         4 * 1024, NULL, 3, &Imu_Task_Handle);
    xTaskCreate(device_battery_health_task, "device_battery_health_task", 8 * 1024, NULL, 3, NULL);
    xTaskCreate(device_gps_task,         "device_gps_task",         8 * 1024, NULL, 3, &Gps_Task_Handle);
    xTaskCreate(device_ethernet_task,    "device_ethernet_task",    4 * 1024, NULL, 3, &Ethernet_Task_Handle);
    xTaskCreate(device_rtc_task,         "device_rtc_task",         4 * 1024, NULL, 3, NULL);
    xTaskCreate(device_at_task,          "device_at_task",          4 * 1024, NULL, 3, &At_Task_Handle);
    xTaskCreate(device_rf_task,          "device_rf_task",          4 * 1024, NULL, 3, &Rf_Task_Handle);
    xTaskCreate(device_adsb_app_task,    "adsb_app_task",           8 * 1024, NULL, 3, &Adsb_App_Task_Handle);
    xTaskCreate(iis_transmission_data_stream_task, "iis_transmission_data_stream_task", 4 * 1024, NULL, 4, &Iis_Transmission_Data_Stream_Task);

#if defined CONFIG_BOARD_TYPE_T_DISPLAY_P4_KEYBOARD
    xTaskCreate(device_nfc_task, "device_nfc_task", 8 * 1024, NULL, 3, &Nfc_Task_Handle);
#endif

    while (lv_display_flush_is_last(lv_display_get_default()) == false)
        vTaskDelay(pdMS_TO_TICKS(10));

    printf("system ui init finish\n");
    printf("[MEM] after UI + tasks: internal=%u\n", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    System_Ui->set_vibration();

    System_Startup_Message_Init();

    // Start serial console (Ctrl+C for command mode)
    serial_console_init();

    // ---------------------------------------------------------------
    // WiFi disabled — Espressif Issue #17889 (SDMMC controller
    // shared between ESP-Hosted SDIO Slot 1 and SD card Slot 0
    // causes heap corruption from concurrent DMA).
    // TODO: Implement AT firmware TCP bridge, or re-enable when
    // Espressif ships the fix.
    // ---------------------------------------------------------------

    // ---------------------------------------------------------------
    // RTL-SDR / ADS-B USB host starts after all peripherals
    // ---------------------------------------------------------------
    printf("[MEM] before USB host: internal=%u PSRAM=%u\n",
           heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
           heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    rtlsdr_adsb_start();
}
