/*
 * @Description: None
 * @Author: LILYGO_L
 * @Date: 2024-11-28 17:07:50
 * @LastEditTime: 2026-01-21 14:05:06
 * @License: GPL 3.0
 */
#include "lvgl_ui.h"
#include "class_driver.h"
#include <math.h>
#include <string.h>
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "emoji_sprites.h"
#include "emoji_draw.h"
#include "device_settings.h"
#include "screen_detect.h"
#include "esp_app_desc.h"
#include "esp_system.h"
#include "esp_chip_info.h"
#include "esp_mac.h"
#include "music_player.h"
#include "meshtastic_task.h"
#include "meshy_channels.h"
#include <sys/time.h>
#include "esp_flash.h"
#include "t_display_p4_driver.h"
#include "screen_detect.h"
#include "driver/jpeg_decode.h"

extern "C" {
    extern void adsb_set_sort(int col, bool ascending);
}

// Format clock time respecting g_settings.time_24h and optional seconds
static void format_clock_time(int hour, int minute, char *buf, size_t bufsize,
                               int second = -1) {
    if (g_settings.time_24h) {
        if (second >= 0)
            snprintf(buf, bufsize, "%02d:%02d:%02d", hour, minute, second);
        else
            snprintf(buf, bufsize, "%02d:%02d", hour, minute);
    } else {
        int h12 = hour % 12;
        if (h12 == 0) h12 = 12;
        const char *ap = hour >= 12 ? "p" : "a";
        if (second >= 0)
            snprintf(buf, bufsize, "%d:%02d:%02d%s", h12, minute, second, ap);
        else
            snprintf(buf, bufsize, "%d:%02d%s", h12, minute, ap);
    }
}

// ═══════════════════════════════════════════════════════
// Persistent scope trail recording — runs from boot via esp_timer
// Independent of scope UI — records aircraft positions at 1Hz
// so trails are available immediately when the user opens Scope.
// ═══════════════════════════════════════════════════════

#define SCOPE_TRAIL_DEPTH 600   // 10 minutes at 1 position/second
#define SCOPE_TRAIL_MAX   64

struct ScopeTrailPt { double lat, lon; };
struct ScopeTrailAC {
    uint32_t icao;
    int head;       // next write position (ring buffer)
    int count;      // valid entries (up to SCOPE_TRAIL_DEPTH)
    ScopeTrailPt pts[SCOPE_TRAIL_DEPTH];
};

static ScopeTrailAC *g_trails = nullptr;
static int g_trail_count = 0;

// Timer callback — records aircraft positions at 1Hz
static void scope_trail_timer_cb(void *arg) {
    if (!g_trails) return;

    static scope_aircraft_t ac_buf[64];
    int ac_count = adsb_get_aircraft_for_scope(ac_buf, 64);

    for (int i = 0; i < ac_count; i++) {
        if (!ac_buf[i].has_position) continue;
        if (ac_buf[i].lat == 0.0 && ac_buf[i].lon == 0.0) continue;

        int slot = -1;
        for (int t = 0; t < g_trail_count; t++) {
            if (g_trails[t].icao == ac_buf[i].icao) { slot = t; break; }
        }
        if (slot < 0 && g_trail_count < SCOPE_TRAIL_MAX) {
            slot = g_trail_count++;
            g_trails[slot].icao = ac_buf[i].icao;
            g_trails[slot].head = 0;
            g_trails[slot].count = 0;
        }
        if (slot >= 0) {
            g_trails[slot].pts[g_trails[slot].head] = { ac_buf[i].lat, ac_buf[i].lon };
            g_trails[slot].head = (g_trails[slot].head + 1) % SCOPE_TRAIL_DEPTH;
            if (g_trails[slot].count < SCOPE_TRAIL_DEPTH) g_trails[slot].count++;
        }
    }
}

// Call from app_main after USB host and ADS-B are initialized
extern "C" void scope_trail_init(void) {
    if (g_trails) return; // already initialized
    g_trails = (ScopeTrailAC *)heap_caps_calloc(SCOPE_TRAIL_MAX, sizeof(ScopeTrailAC), MALLOC_CAP_SPIRAM);
    if (!g_trails) {
        printf("[TRAIL] Failed to allocate trail buffer in PSRAM\n");
        return;
    }
    printf("[TRAIL] Allocated %d trail slots (%d KB PSRAM)\n",
           SCOPE_TRAIL_MAX, (int)(SCOPE_TRAIL_MAX * sizeof(ScopeTrailAC) / 1024));

    // Start 1Hz periodic timer
    esp_timer_handle_t timer;
    esp_timer_create_args_t args = {};
    args.callback = scope_trail_timer_cb;
    args.name = "scope_trail";
    esp_timer_create(&args, &timer);
    esp_timer_start_periodic(timer, 1000000); // 1 second
    printf("[TRAIL] 1Hz trail recording started\n");
}

namespace Lvgl_Ui
{
    const System::Win_Home_App_Icon System::_win_home_app_icon_list[] =
        {
            {"Cit", &win_home_app_icon_cit_110x110px_rgb565a8},
    };

    const System::Win_Home_App_Icon System::_win_home_app_icon_fixed_list[] =
        {
            {"Camera", &win_home_app_icon_camera_110x110px_rgb565a8},
            {"Setings", &win_home_app_icon_setings_110x110px_rgb565a8},
    };

    System::Win_Cit_Test_Item System::_win_cit_test_item_list[] =
        {
            {"version information test", LV_SYMBOL_WARNING, 0xFFA500},
            {"touch test", LV_SYMBOL_REFRESH, 0x000000},
            {"screen color test", LV_SYMBOL_WARNING, 0xFFA500},
            {"vibration test", LV_SYMBOL_WARNING, 0xFFA500},
            {"speaker test", LV_SYMBOL_WARNING, 0xFFA500},
            {"microphone test", LV_SYMBOL_WARNING, 0xFFA500},
            {"imu test", LV_SYMBOL_WARNING, 0xFFA500},
            {"battery health test", LV_SYMBOL_WARNING, 0xFFA500},
            {"gps test", LV_SYMBOL_WARNING, 0xFFA500},
            {"ethernet test", LV_SYMBOL_WARNING, 0xFFA500},
            {"rtc test", LV_SYMBOL_WARNING, 0xFFA500},
            {"adsb test", LV_SYMBOL_WARNING, 0xFFA500},
// {"sleep test", LV_SYMBOL_WARNING, 0xFFA500},
#if defined CONFIG_BOARD_TYPE_T_DISPLAY_P4_KEYBOARD
            {"keyboard test", LV_SYMBOL_WARNING, 0xFFA500},
            {"nfc test", LV_SYMBOL_WARNING, 0xFFA500},
#endif
    };

    System::Device_Information System::_device_information_list[] =
        {
            {"chip model: ", ""},
            {"chip efuse mac:\n     ", ""},
            {"chip revision: ", ""},
            {"chip cores: ", ""},
            {"chip flash size: ", ""},
            {"chip flash features: ", ""},
            {"chip free heap size:\n     ", ""},
            {"espidf version:\n     ", ""},
            {"company: ", "lilygo"},
#if defined CONFIG_BOARD_TYPE_T_DISPLAY_P4
            {"board name: ", "t-display-p4"},
#elif defined CONFIG_BOARD_TYPE_T_DISPLAY_P4_KEYBOARD
            {"board name: ", "t-display-p4-keyboard"},
#else
#error "unknown macro definition, please select the correct macro definition."
#endif
            {"software name: ", "lvgl_9_ui"},

            {"screen type: ", "detecting..."},  // overridden at runtime in begin()

#if defined CONFIG_SCREEN_PIXEL_FORMAT_RGB565
            {"screen pixel format: ", "rgb565"},
#elif defined CONFIG_SCREEN_PIXEL_FORMAT_RGB888
            {"screen pixel format: ", "rgb888"},
#else
#error "unknown macro definition, please select the correct macro definition."
#endif

#if defined CONFIG_CAMERA_TYPE_SC2336
            {"camera type: ", "sc2333"},
#elif defined CONFIG_CAMERA_TYPE_OV2710
            {"camera type: ", "ov2710"},
#elif defined CONFIG_CAMERA_TYPE_OV5645
            {"camera type: ", "ov5645"},
#else
#error "unknown macro definition, please select the correct macro definition."
#endif

            {"firmware build date:\n     ", "202601211405"},
    };

    void System::begin(bool has_sd)
    {
        _has_sd = has_sd;
        _home_rotation = lv_display_get_rotation(lv_display_get_default());
#if defined SCREEN_ROTATION_DIRECTION_0
        _app_style.icon.edge_distance.height = std::min(_width, _height) / 5;
        _app_style.icon.edge_distance.width = _app_style.icon.edge_distance.height / 5;
        _app_style.icon.icon_distance.width = (_width - (_app_style.icon.edge_distance.width * 2) - (4 * APP_STYLE_ICON_WIDTH_HEIGHT)) / 3;
        _app_style.icon.icon_distance.height = APP_STYLE_ICON_WIDTH_HEIGHT + (APP_STYLE_ICON_WIDTH_HEIGHT / 1.5);
        _app_style.label.width = APP_STYLE_ICON_WIDTH_HEIGHT;
        _app_style.label.height = APP_STYLE_ICON_WIDTH_HEIGHT / 3;
        _app_style.icon.edge_distance_fixed.width = _app_style.icon.edge_distance.width + 40;
        _app_style.icon.edge_distance_fixed.height = 10;
        _app_style.icon.icon_distance.fixed_width = (_width - (_app_style.icon.edge_distance_fixed.width * 2) - (3 * APP_STYLE_ICON_WIDTH_HEIGHT)) / 2;
#elif defined SCREEN_ROTATION_DIRECTION_90
        _app_style.icon.edge_distance.height = std::min(_width, _height) / 6;
        _app_style.icon.edge_distance.width = _app_style.icon.edge_distance.height / 5;
        _app_style.icon.icon_distance.width = (_width - (_app_style.icon.edge_distance.width * 2) - (9 * APP_STYLE_ICON_WIDTH_HEIGHT)) / 3;
        _app_style.icon.icon_distance.height = APP_STYLE_ICON_WIDTH_HEIGHT + (APP_STYLE_ICON_WIDTH_HEIGHT / 1.5);
        _app_style.label.width = APP_STYLE_ICON_WIDTH_HEIGHT;
        _app_style.label.height = APP_STYLE_ICON_WIDTH_HEIGHT / 3;
        _app_style.icon.edge_distance_fixed.width = _app_style.icon.edge_distance.width + 40;
        _app_style.icon.edge_distance_fixed.height = 10;
        _app_style.icon.icon_distance.fixed_width = (_width - (_app_style.icon.edge_distance_fixed.width * 2) - (8 * APP_STYLE_ICON_WIDTH_HEIGHT)) / 2;
#else
#error "unknown macro definition, please select the correct macro definition."
#endif

        _device_information_list[0].info = "esp32p4";

        uint8_t mac[6];
        esp_efuse_mac_get_default(mac);
        char mac_str[18];

        snprintf(mac_str, sizeof(mac_str), "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        _device_information_list[1].info = mac_str;

        esp_chip_info_t chip_info;
        esp_chip_info(&chip_info);
        char revision_str[10];
        snprintf(revision_str, sizeof(revision_str), "v%d.%d", chip_info.revision / 100, chip_info.revision % 100);
        _device_information_list[2].info = revision_str;

        char cores_str[10];
        snprintf(cores_str, sizeof(cores_str), "%d", chip_info.cores);
        _device_information_list[3].info = cores_str;

        uint32_t flash_size;
        ESP_ERROR_CHECK(esp_flash_get_size(NULL, &flash_size));
        char flash_size_str[20];
        snprintf(flash_size_str, sizeof(flash_size_str), "%lu bytes", flash_size);
        _device_information_list[4].info = flash_size_str;

        _device_information_list[5].info = (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external";

        char free_heap_size_str[20];
        snprintf(free_heap_size_str, sizeof(free_heap_size_str), "%lu bytes", esp_get_free_heap_size());
        _device_information_list[6].info = free_heap_size_str;

        _device_information_list[7].info = esp_get_idf_version();

        // Runtime screen type (detected by I2C probe in main.cpp)
        _device_information_list[11].info = screen_is_rm69a10() ? "rm69a10 (amoled)" : "hi8561 (lcd)";

#if defined CONFIG_BOARD_TYPE_T_DISPLAY_P4_KEYBOARD
        _registry.keyboard_group = lv_group_create();
        set_keyboard_group(_registry.keyboard_group);
#endif

        init_win_home();
        // lv_screen_load(_registry.win.home.root);
        lv_screen_load_anim(_registry.win.home.root, LV_SCR_LOAD_ANIM_FADE_OUT, 500, 0, true);
    }

    System::Current_Win System::get_current_win(void)
    {
        return _current_win;
    }

    void System::set_time(Pcf8563x::Time time)
    {
        std::string week_str;
        switch (time.week)
        {
        case Pcf8563x::Week::SUNDAY:
            week_str = "Sun";
            break;
        case Pcf8563x::Week::MONDAY:
            week_str = "Mon";
            break;
        case Pcf8563x::Week::TUESDAY:
            week_str = "Tue";
            break;
        case Pcf8563x::Week::WEDNESDAY:
            week_str = "Wed";
            break;
        case Pcf8563x::Week::THURSDAY:
            week_str = "Thu";
            break;
        case Pcf8563x::Week::FRIDAY:
            week_str = "Fri";
            break;
        case Pcf8563x::Week::SATURDAY:
            week_str = "Sat";
            break;

        default:
            break;
        }

        _time.week = week_str;
        _time.year = static_cast<uint16_t>(time.year + 2000);
        _time.month = time.month;
        _time.day = time.day;
        _time.hour = time.hour;
        _time.minute = time.minute;
        _time.second = time.second;
    }

    void System::set_battery_level(uint16_t battery_level)
    {
        _battery_level = battery_level;
    }

    void System::set_wifi_connect_status(bool status)
    {
        _wifi_connect_status = status;
    }

    void System::add_event_cb_win_return_to_cit(lv_obj_t *obj)
    {
        lv_obj_add_event_cb(obj, [](lv_event_t *e)
                            {
                                System *self = static_cast<System *>(lv_event_get_user_data(e));
                                lv_event_code_t code = lv_event_get_code(e);

                                if (code == LV_EVENT_GESTURE)
                                {
                                    lv_dir_t gesture_dir = lv_indev_get_gesture_dir(lv_indev_active());

                                    // 边缘检测以及左右滑动
                                    if ((gesture_dir == LV_DIR_LEFT || gesture_dir == LV_DIR_RIGHT)&&(self->_edge_touch_flag == true))
                                    {
                                        self->set_vibration();
                                        self->init_win_cit();
                                        
                                        lv_screen_load_anim(self->_registry.win.cit.root, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 100, 0, true);

                                        self->_edge_touch_flag = false;
                                    }
                                } }, LV_EVENT_ALL, this);
    }

    void System::add_win_cit_test_item_pass_fail_button(lv_obj_t *parent)
    {
        // 创建一个容器来存放两个按键
        lv_obj_t *button_container = lv_obj_create(parent);
        lv_obj_set_size(button_container, _width, 140);
        lv_obj_align(button_container, LV_ALIGN_BOTTOM_MID, 0, 0);
        lv_obj_set_style_bg_color(button_container, lv_color_white(), (lv_style_selector_t)LV_PART_MAIN); // 设置背景颜色为白色
        lv_obj_set_style_radius(button_container, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_border_width(button_container, 0, (lv_style_selector_t)LV_PART_MAIN); // 移除边框

        // 创建PASS按键
        lv_obj_t *pass_button = lv_button_create(button_container);
        lv_obj_set_size(pass_button, 200, 60);
#if defined SCREEN_ROTATION_DIRECTION_0
        lv_obj_align(pass_button, LV_ALIGN_LEFT_MID, 10, 0);
#elif defined SCREEN_ROTATION_DIRECTION_90
        lv_obj_align(pass_button, LV_ALIGN_LEFT_MID, 150, 0);
#else
#error "unknown macro definition, please select the correct macro definition."
#endif
        lv_obj_set_style_radius(pass_button, 10, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_width(pass_button, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT); // 移除阴影

        lv_obj_t *pass_label = lv_label_create(pass_button);
        lv_obj_set_style_text_font(pass_label, &lv_font_montserrat_24, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_label_set_text(pass_label, "PASS");
        lv_obj_center(pass_label);

        lv_obj_add_event_cb(pass_button, [](lv_event_t *e)
                            {
                                System *self = static_cast<System *>(lv_event_get_user_data(e));
                                lv_event_code_t code = lv_event_get_code(e);

                                switch (code)
                                {
                                case LV_EVENT_CLICKED:
                                    _win_cit_test_item_list[self->_registry.win.cit.current_test_item_index].symbol = LV_SYMBOL_OK;
                                    _win_cit_test_item_list[self->_registry.win.cit.current_test_item_index].color = 0x008B45;

                                    self->init_win_cit();

                                    lv_screen_load_anim(self->_registry.win.cit.root, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 100, 0, true);
                                    break;
                                default:
                                    break;
                                } }, LV_EVENT_ALL, this);

        // 创建FAIL按键
        lv_obj_t *fail_button = lv_button_create(button_container);
        lv_obj_set_size(fail_button, 200, 60);
#if defined SCREEN_ROTATION_DIRECTION_0
        lv_obj_align(fail_button, LV_ALIGN_RIGHT_MID, -10, 0);
#elif defined SCREEN_ROTATION_DIRECTION_90
        lv_obj_align(fail_button, LV_ALIGN_RIGHT_MID, -150, 0);
#else
#error "unknown macro definition, please select the correct macro definition."
#endif
        lv_obj_set_style_radius(fail_button, 10, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_width(fail_button, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT); // 移除阴影

        lv_obj_t *fail_label = lv_label_create(fail_button);
        lv_obj_set_style_text_font(fail_label, &lv_font_montserrat_24, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_label_set_text(fail_label, "FAIL");
        lv_obj_center(fail_label);

        lv_obj_add_event_cb(fail_button, [](lv_event_t *e)
                            {
                                System *self = static_cast<System *>(lv_event_get_user_data(e));
                                lv_event_code_t code = lv_event_get_code(e);

                                switch (code)
                                {
                                case LV_EVENT_CLICKED:
                                    _win_cit_test_item_list[self->_registry.win.cit.current_test_item_index].symbol = LV_SYMBOL_CLOSE;
                                    _win_cit_test_item_list[self->_registry.win.cit.current_test_item_index].color = 0xEE2C2C;

                                    self->init_win_cit();

                                    lv_screen_load_anim(self->_registry.win.cit.root, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 100, 0, true);
                                    break;
                                default:
                                    break;
                                } }, LV_EVENT_ALL, this);
    }

    void System::set_vibration(uint8_t vibration_count)
    {
        if (_device_vibration_callback != nullptr)
        {
            _device_vibration_callback(vibration_count);
        }
    }

    void System::set_speaker_test(void)
    {
        if (_win_cit_speaker_test_callback != nullptr)
        {
            _win_cit_speaker_test_callback();
        }
    }

    void System::set_microphone_test(bool status)
    {
        if (_win_cit_microphone_test_callback != nullptr)
        {
            _win_cit_microphone_test_callback(status);
        }
    }

    void System::set_adc_to_dac_switch_status(bool status)
    {
        if (_win_cit_adc_to_dac_switch_callback != nullptr)
        {
            _win_cit_adc_to_dac_switch_callback(status);
        }
    }

    void System::set_imu_test(bool status)
    {
        if (_win_cit_imu_test_callback != nullptr)
        {
            _win_cit_imu_test_callback(status);
        }
    }

    void System::set_gps_test(bool status)
    {
        if (_win_cit_gps_test_callback != nullptr)
        {
            _win_cit_gps_test_callback(status);
        }
    }

    void System::set_ethernet_test(bool status)
    {
        if (_win_cit_ethernet_test_callback != nullptr)
        {
            _win_cit_ethernet_test_callback(status);
        }
    }

    void System::set_esp32c6_at_test(bool status)
    {
        if (_win_cit_esp32c6_at_test_callback != nullptr)
        {
            _win_cit_esp32c6_at_test_callback(status);
        }
    }

    // void System::start_sleep_test(Sleep_Mode mode)
    // {
    //     if (_device_start_sleep_test_callback != nullptr)
    //     {
    //         _device_start_sleep_test_callback(mode);
    //     }
    // }

    void System::set_camera_status(bool status)
    {
        if (_win_camera_status_callback != nullptr)
        {
            _win_camera_status_callback(status);
        }
    }

    void System::init_win_home(void)
    {
        // 主界面
        _registry.win.home.root = lv_obj_create(NULL);
        lv_obj_set_style_bg_color(_registry.win.home.root, lv_color_black(), (lv_style_selector_t)LV_PART_MAIN);

#if defined SCREEN_ROTATION_DIRECTION_0
        if (_has_sd) {
            if (screen_is_rm69a10())
                lv_obj_set_style_bg_image_src(_registry.win.home.root, GET_WALLPAPER_PATH("wallpaper_1_568x1232px.png"), (lv_style_selector_t)LV_PART_MAIN);
            else
                lv_obj_set_style_bg_image_src(_registry.win.home.root, GET_WALLPAPER_PATH("wallpaper_1_540x1168px.png"), (lv_style_selector_t)LV_PART_MAIN);
        }
#elif defined SCREEN_ROTATION_DIRECTION_90
        if (_has_sd) {
            if (screen_is_rm69a10())
                lv_obj_set_style_bg_image_src(_registry.win.home.root, GET_WALLPAPER_PATH("wallpaper_1_1232x568px.png"), (lv_style_selector_t)LV_PART_MAIN);
            else
                lv_obj_set_style_bg_image_src(_registry.win.home.root, GET_WALLPAPER_PATH("wallpaper_1_1168x540px.png"), (lv_style_selector_t)LV_PART_MAIN);
        }
#else
#error "unknown macro definition, please select the correct macro definition."
#endif

        lv_obj_set_size(_registry.win.home.root, _width, _height);
        lv_obj_set_scrollbar_mode(_registry.win.home.root, LV_SCROLLBAR_MODE_OFF);

        // 页
        lv_obj_t *tileview = lv_tileview_create(_registry.win.home.root);
        lv_obj_set_size(tileview, _width, _height - (APP_STYLE_ICON_WIDTH_HEIGHT + 40));
        lv_obj_set_style_bg_opa(tileview, LV_OPA_0, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_scrollbar_mode(tileview, LV_SCROLLBAR_MODE_ACTIVE);

        // 页1
        lv_obj_t *tileview_tile_1 = lv_tileview_add_tile(tileview, 0, 0, (lv_dir_t)(LV_DIR_LEFT | LV_DIR_RIGHT));

        lv_obj_t *image_button[sizeof(_win_home_app_icon_list) / sizeof(Win_Home_App_Icon)];
        for (uint16_t i = 0; i < sizeof(_win_home_app_icon_list) / sizeof(Win_Home_App_Icon); i++)
        {
            // 定义需要过渡的属性：缩放 X 和缩放 Y
            static const lv_style_prop_t tr_prop[] = {LV_STYLE_TRANSFORM_SCALE_X, LV_STYLE_TRANSFORM_SCALE_Y, 0};
            // 创建过渡描述符
            static lv_style_transition_dsc_t tr;
            lv_style_transition_dsc_init(&tr, tr_prop, lv_anim_path_ease_out, 100, 0, NULL);
            // 创建默认样式
            static lv_style_t style_def;
            lv_style_init(&style_def);
            // 设置过渡效果
            lv_style_set_transition(&style_def, &tr);
            // 设置缩放的中心点为对象的中心
            lv_style_set_transform_pivot_x(&style_def, LV_PCT(50)); // X 方向的中心点
            lv_style_set_transform_pivot_y(&style_def, LV_PCT(50)); // Y 方向的中心点
            // 创建按下时的样式
            static lv_style_t style_pr;
            lv_style_init(&style_pr);
            // 设置按下时的缩放比例
            lv_style_set_transform_scale_x(&style_pr, 256 * 0.9);
            lv_style_set_transform_scale_y(&style_pr, 256 * 0.9);
            // 设置缩放的中心点为对象的中心
            lv_style_set_transform_pivot_x(&style_pr, LV_PCT(50)); // X 方向的中心点
            lv_style_set_transform_pivot_y(&style_pr, LV_PCT(50)); // Y 方向的中心点

            // 创建图像按钮
            image_button[i] = lv_imagebutton_create(tileview_tile_1);
            lv_imagebutton_set_src(image_button[i], LV_IMAGEBUTTON_STATE_RELEASED, NULL, _win_home_app_icon_list[i].image, NULL);
            lv_imagebutton_set_src(image_button[i], LV_IMAGEBUTTON_STATE_PRESSED, NULL, _win_home_app_icon_list[i].image, NULL);
            lv_imagebutton_set_src(image_button[i], LV_IMAGEBUTTON_STATE_DISABLED, NULL, _win_home_app_icon_list[i].image, NULL);
            // 应用样式
            lv_obj_add_style(image_button[i], &style_def, (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_add_style(image_button[i], &style_pr, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_PRESSED);
#if defined SCREEN_ROTATION_DIRECTION_0
            // 设置按钮位置
            lv_obj_align(image_button[i], LV_ALIGN_TOP_LEFT,
                         _app_style.icon.edge_distance.width + (APP_STYLE_ICON_WIDTH_HEIGHT * i) + (_app_style.icon.icon_distance.width * i),
                         _app_style.icon.edge_distance.height + 300);

#elif defined SCREEN_ROTATION_DIRECTION_90
            // 设置按钮位置
            lv_obj_align(image_button[i], LV_ALIGN_TOP_LEFT,
                         _app_style.icon.edge_distance.width + (APP_STYLE_ICON_WIDTH_HEIGHT * i) + (_app_style.icon.icon_distance.width * i) + 400, _app_style.icon.edge_distance.height);
#else
#error "unknown macro definition, please select the correct macro definition."
#endif
            lv_obj_t *image_button_label = lv_label_create(tileview_tile_1);
            lv_label_set_text(image_button_label, _win_home_app_icon_list[i].name.c_str());
            lv_obj_set_style_text_align(image_button_label, LV_TEXT_ALIGN_CENTER, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
            lv_obj_set_style_text_font(image_button_label, &lv_font_montserrat_22, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
            lv_obj_set_size(image_button_label, _app_style.label.width, _app_style.label.height);
            lv_obj_set_style_text_color(image_button_label, lv_color_white(), (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
            lv_obj_align_to(image_button_label, image_button[i], LV_ALIGN_OUT_BOTTOM_MID, 0, 0);
        }

        lv_obj_add_event_cb(image_button[0], [](lv_event_t *e)
                            {
                                System *self = static_cast<System *>(lv_event_get_user_data(e));
                                lv_event_code_t code = lv_event_get_code(e);

                                switch (code)
                                {
                                case LV_EVENT_CLICKED:
                                self->init_win_cit();

                                lv_screen_load_anim(self->_registry.win.cit.root, LV_SCR_LOAD_ANIM_FADE_OUT, 500, 0, true);
                                break;
                                default:
                                break;
                                } }, LV_EVENT_ALL, this);

        // Music app button (2nd position — styled button, warm gradient with music note)
        {
            lv_obj_t *music_btn = lv_button_create(tileview_tile_1);
            lv_obj_set_size(music_btn, APP_STYLE_ICON_WIDTH_HEIGHT, APP_STYLE_ICON_WIDTH_HEIGHT);
            lv_obj_set_style_radius(music_btn, 20, (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_set_style_bg_color(music_btn, lv_color_hex(0xC44B2D), (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_set_style_bg_grad_color(music_btn, lv_color_hex(0x8B2FC9), (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_set_style_bg_grad_dir(music_btn, LV_GRAD_DIR_VER, (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_set_style_bg_opa(music_btn, LV_OPA_COVER, (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_set_style_shadow_width(music_btn, 0, (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_set_style_border_width(music_btn, 0, (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_set_style_bg_color(music_btn, lv_color_hex(0x9A3522), (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_PRESSED);
#if defined SCREEN_ROTATION_DIRECTION_0
            lv_obj_align(music_btn, LV_ALIGN_TOP_LEFT,
                         _app_style.icon.edge_distance.width + APP_STYLE_ICON_WIDTH_HEIGHT + _app_style.icon.icon_distance.width,
                         _app_style.icon.edge_distance.height + 300);
#elif defined SCREEN_ROTATION_DIRECTION_90
            lv_obj_align(music_btn, LV_ALIGN_TOP_LEFT,
                         _app_style.icon.edge_distance.width + APP_STYLE_ICON_WIDTH_HEIGHT + _app_style.icon.icon_distance.width + 400,
                         _app_style.icon.edge_distance.height);
#endif
            lv_obj_t *icon_lbl = lv_label_create(music_btn);
            lv_label_set_text(icon_lbl, LV_SYMBOL_AUDIO);
            lv_obj_set_style_text_font(icon_lbl, &lv_font_montserrat_48, (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_set_style_text_color(icon_lbl, lv_color_white(), (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_center(icon_lbl);

            lv_obj_t *music_label = lv_label_create(tileview_tile_1);
            lv_label_set_text(music_label, "Music");
            lv_obj_set_style_text_align(music_label, LV_TEXT_ALIGN_CENTER, (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_set_style_text_font(music_label, &lv_font_montserrat_22, (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_set_size(music_label, _app_style.label.width, _app_style.label.height);
            lv_obj_set_style_text_color(music_label, lv_color_white(), (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_align_to(music_label, music_btn, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);

            lv_obj_add_event_cb(music_btn, [](lv_event_t *e)
                                {
                                    System *self = static_cast<System *>(lv_event_get_user_data(e));
                                    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
                                    self->init_win_music();
                                    lv_screen_load_anim(self->_registry.win.music.root, LV_SCR_LOAD_ANIM_FADE_OUT, 500, 0, true);
                                }, LV_EVENT_ALL, this);
        }

        // Meshy app button (3rd position — styled button, green mesh icon)
        {
            lv_obj_t *meshy_btn = lv_button_create(tileview_tile_1);
            lv_obj_set_size(meshy_btn, APP_STYLE_ICON_WIDTH_HEIGHT, APP_STYLE_ICON_WIDTH_HEIGHT);
            lv_obj_set_style_radius(meshy_btn, 20, (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_set_style_bg_color(meshy_btn, lv_color_hex(0x2D8C3A), (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_set_style_bg_opa(meshy_btn, LV_OPA_COVER, (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_set_style_shadow_width(meshy_btn, 0, (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_set_style_border_width(meshy_btn, 0, (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_set_style_bg_color(meshy_btn, lv_color_hex(0x1A6628), (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_PRESSED);
#if defined SCREEN_ROTATION_DIRECTION_0
            lv_obj_align(meshy_btn, LV_ALIGN_TOP_LEFT,
                         _app_style.icon.edge_distance.width + (APP_STYLE_ICON_WIDTH_HEIGHT * 2) + (_app_style.icon.icon_distance.width * 2),
                         _app_style.icon.edge_distance.height + 300);
#elif defined SCREEN_ROTATION_DIRECTION_90
            lv_obj_align(meshy_btn, LV_ALIGN_TOP_LEFT,
                         _app_style.icon.edge_distance.width + (APP_STYLE_ICON_WIDTH_HEIGHT * 2) + (_app_style.icon.icon_distance.width * 2) + 400,
                         _app_style.icon.edge_distance.height);
#endif
            lv_obj_t *icon_lbl = lv_label_create(meshy_btn);
            lv_label_set_text(icon_lbl, LV_SYMBOL_WIFI);
            lv_obj_set_style_text_font(icon_lbl, &lv_font_montserrat_48, (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_set_style_text_color(icon_lbl, lv_color_white(), (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_center(icon_lbl);

            lv_obj_t *meshy_label = lv_label_create(tileview_tile_1);
            lv_label_set_text(meshy_label, "Meshy");
            lv_obj_set_style_text_align(meshy_label, LV_TEXT_ALIGN_CENTER, (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_set_style_text_font(meshy_label, &lv_font_montserrat_22, (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_set_size(meshy_label, _app_style.label.width, _app_style.label.height);
            lv_obj_set_style_text_color(meshy_label, lv_color_white(), (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_align_to(meshy_label, meshy_btn, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);

            lv_obj_add_event_cb(meshy_btn, [](lv_event_t *e)
                                {
                                    System *self = static_cast<System *>(lv_event_get_user_data(e));
                                    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
                                    self->init_win_meshy();
                                    lv_screen_load_anim(self->_registry.win.meshy.root, LV_SCR_LOAD_ANIM_FADE_OUT, 500, 0, true);
                                }, LV_EVENT_ALL, this);
        }

        // ADS-B app button (4th position — no icon image, uses styled button)
        {
            lv_obj_t *adsb_btn = lv_button_create(tileview_tile_1);
            lv_obj_set_size(adsb_btn, APP_STYLE_ICON_WIDTH_HEIGHT, APP_STYLE_ICON_WIDTH_HEIGHT);
            lv_obj_set_style_radius(adsb_btn, 20, (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_set_style_bg_color(adsb_btn, lv_color_hex(0x1A3A5C), (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_set_style_bg_opa(adsb_btn, LV_OPA_COVER, (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_set_style_shadow_width(adsb_btn, 0, (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_set_style_border_width(adsb_btn, 0, (lv_style_selector_t)LV_PART_MAIN);
            // Press animation
            lv_obj_set_style_bg_color(adsb_btn, lv_color_hex(0x0E2640), (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_PRESSED);
#if defined SCREEN_ROTATION_DIRECTION_0
            lv_obj_align(adsb_btn, LV_ALIGN_TOP_LEFT,
                         _app_style.icon.edge_distance.width + (APP_STYLE_ICON_WIDTH_HEIGHT * 3) + (_app_style.icon.icon_distance.width * 3),
                         _app_style.icon.edge_distance.height + 300);
#elif defined SCREEN_ROTATION_DIRECTION_90
            lv_obj_align(adsb_btn, LV_ALIGN_TOP_LEFT,
                         _app_style.icon.edge_distance.width + (APP_STYLE_ICON_WIDTH_HEIGHT * 3) + (_app_style.icon.icon_distance.width * 3) + 400,
                         _app_style.icon.edge_distance.height);
#endif
            // Radar icon inside the button
            lv_obj_t *icon_lbl = lv_label_create(adsb_btn);
            lv_label_set_text(icon_lbl, LV_SYMBOL_GPS);
            lv_obj_set_style_text_font(icon_lbl, &lv_font_montserrat_48, (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_set_style_text_color(icon_lbl, lv_color_hex(0x00DD00), (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_center(icon_lbl);

            // Label below
            lv_obj_t *adsb_label = lv_label_create(tileview_tile_1);
            lv_label_set_text(adsb_label, "ADS-B");
            lv_obj_set_style_text_align(adsb_label, LV_TEXT_ALIGN_CENTER, (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_set_style_text_font(adsb_label, &lv_font_montserrat_22, (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_set_size(adsb_label, _app_style.label.width, _app_style.label.height);
            lv_obj_set_style_text_color(adsb_label, lv_color_white(), (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_align_to(adsb_label, adsb_btn, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);

            lv_obj_add_event_cb(adsb_btn, [](lv_event_t *e)
                                {
                                    System *self = static_cast<System *>(lv_event_get_user_data(e));
                                    lv_event_code_t code = lv_event_get_code(e);
                                    switch (code)
                                    {
                                    case LV_EVENT_CLICKED:
                                        self->init_win_adsb();
                                        if (self->_registry.win.adsb.has_user_rotation) {
                                            // Load instantly, then apply rotation after a short
                                            // delay so the home screen is fully gone first.
                                            lv_screen_load(self->_registry.win.adsb.root);
                                            lv_timer_create([](lv_timer_t *t) {
                                                System *s = static_cast<System *>(lv_timer_get_user_data(t));
                                                lv_display_set_rotation(lv_display_get_default(), s->_registry.win.adsb.user_rotation);
                                                s->_registry.win.adsb.rotated = true;
                                                s->_registry.win.adsb.divider_y = 0;
                                                s->init_win_adsb();
                                                lv_screen_load(s->_registry.win.adsb.root);
                                                lv_timer_delete(t);
                                            }, 100, self);
                                        } else {
                                            lv_screen_load_anim(self->_registry.win.adsb.root, LV_SCR_LOAD_ANIM_FADE_OUT, 500, 0, true);
                                        }
                                        break;
                                    default:
                                        break;
                                    } }, LV_EVENT_ALL, this);
        }

        // Scope app button (row 2, position 0 — under CIT)
        {
            lv_obj_t *scope_btn = lv_button_create(tileview_tile_1);
            lv_obj_set_size(scope_btn, APP_STYLE_ICON_WIDTH_HEIGHT, APP_STYLE_ICON_WIDTH_HEIGHT);
            lv_obj_set_style_radius(scope_btn, 20, (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_set_style_bg_color(scope_btn, lv_color_hex(0x0E2640), (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_set_style_bg_opa(scope_btn, LV_OPA_COVER, (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_set_style_shadow_width(scope_btn, 0, (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_set_style_border_width(scope_btn, 0, (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_set_style_bg_color(scope_btn, lv_color_hex(0x091A30), (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_PRESSED);
#if defined SCREEN_ROTATION_DIRECTION_0
            lv_obj_align(scope_btn, LV_ALIGN_TOP_LEFT,
                         _app_style.icon.edge_distance.width,
                         _app_style.icon.edge_distance.height + 300 + _app_style.icon.icon_distance.height);
#elif defined SCREEN_ROTATION_DIRECTION_90
            lv_obj_align(scope_btn, LV_ALIGN_TOP_LEFT,
                         _app_style.icon.edge_distance.width + 400,
                         _app_style.icon.edge_distance.height + _app_style.icon.icon_distance.height);
#endif
            lv_obj_t *icon_lbl = lv_label_create(scope_btn);
            lv_label_set_text(icon_lbl, LV_SYMBOL_EYE_OPEN);
            lv_obj_set_style_text_font(icon_lbl, &lv_font_montserrat_48, (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_set_style_text_color(icon_lbl, lv_color_hex(0x00e5a0), (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_center(icon_lbl);

            lv_obj_t *scope_label = lv_label_create(tileview_tile_1);
            lv_label_set_text(scope_label, "Scope");
            lv_obj_set_style_text_align(scope_label, LV_TEXT_ALIGN_CENTER, (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_set_style_text_font(scope_label, &lv_font_montserrat_22, (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_set_size(scope_label, _app_style.label.width, _app_style.label.height);
            lv_obj_set_style_text_color(scope_label, lv_color_white(), (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_align_to(scope_label, scope_btn, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);

            lv_obj_add_event_cb(scope_btn, [](lv_event_t *e)
                                {
                                    System *self = static_cast<System *>(lv_event_get_user_data(e));
                                    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
                                    self->init_win_scope();
                                    lv_screen_load_anim(self->_registry.win.scope.root, LV_SCR_LOAD_ANIM_FADE_OUT, 500, 0, true);
                                }, LV_EVENT_ALL, this);
        }

        // 时钟
        _registry.win.home.clock.time_label = lv_label_create(tileview_tile_1);
        char buffer_time[16];
        format_clock_time(_time.hour, _time.minute, buffer_time, sizeof(buffer_time), g_settings.show_seconds ? _time.second : -1);
        lv_label_set_text(_registry.win.home.clock.time_label, buffer_time);
        lv_obj_set_style_text_align(_registry.win.home.clock.time_label, LV_TEXT_ALIGN_LEFT, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(_registry.win.home.clock.time_label, &lvgl_font_lineseedkr_rg_120, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_size(_registry.win.home.clock.time_label, 400, 110);
        lv_obj_set_style_text_color(_registry.win.home.clock.time_label, lv_color_white(), (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_align_to(_registry.win.home.clock.time_label, tileview_tile_1, LV_ALIGN_TOP_LEFT, 10, 90);

        _registry.win.home.clock.month_label = lv_label_create(tileview_tile_1);

        std::string month_str = "null";
        switch (_time.month)
        {
        case 1:
            month_str = "January";
            break;
        case 2:
            month_str = "February";
            break;
        case 3:
            month_str = "March";
            break;
        case 4:
            month_str = "April";
            break;
        case 5:
            month_str = "May";
            break;
        case 6:
            month_str = "June";
            break;
        case 7:
            month_str = "July";
            break;
        case 8:
            month_str = "August";
            break;
        case 9:
            month_str = "September";
            break;
        case 10:
            month_str = "October";
            break;
        case 11:
            month_str = "November";
            break;
        case 12:
            month_str = "December";
            break;

        default:
            break;
        }
        char buffer_month[20];
        snprintf(buffer_month, sizeof(buffer_month), "%s %dth", month_str.c_str(), _time.day);
        lv_label_set_text(_registry.win.home.clock.month_label, buffer_month);
        lv_obj_set_style_text_align(_registry.win.home.clock.month_label, LV_TEXT_ALIGN_LEFT, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(_registry.win.home.clock.month_label, &lvgl_font_lineseedkr_th_60, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_size(_registry.win.home.clock.month_label, 400, 70);
        lv_obj_set_style_text_color(_registry.win.home.clock.month_label, lv_color_white(), (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_align_to(_registry.win.home.clock.month_label, _registry.win.home.clock.time_label, LV_ALIGN_OUT_BOTTOM_LEFT, 10, 0);

        _registry.win.home.clock.week_label = lv_label_create(tileview_tile_1);
        lv_label_set_text(_registry.win.home.clock.week_label, _time.week.c_str());
        lv_obj_set_style_text_align(_registry.win.home.clock.week_label, LV_TEXT_ALIGN_LEFT, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(_registry.win.home.clock.week_label, &lvgl_font_lineseedkr_th_60, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_size(_registry.win.home.clock.week_label, 400, 50);
        lv_obj_set_style_text_color(_registry.win.home.clock.week_label, lv_color_white(), (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_align_to(_registry.win.home.clock.week_label, _registry.win.home.clock.month_label, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);

        // 页2
        lv_obj_t *tileview_tile_2 = lv_tileview_add_tile(tileview, 1, 0, (lv_dir_t)(LV_DIR_LEFT | LV_DIR_RIGHT));

        // 固定页
        lv_obj_t *tileview_fixed = lv_tileview_create(_registry.win.home.root);
        lv_obj_set_size(tileview_fixed, _width, APP_STYLE_ICON_WIDTH_HEIGHT + 50);
        lv_obj_set_style_bg_color(tileview_fixed, lv_color_white(), (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_bg_opa(tileview_fixed, LV_OPA_30, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_align(tileview_fixed, LV_ALIGN_BOTTOM_MID, 0, 0);

        // 页1
        lv_obj_t *tileview_fixed_tile_1 = lv_tileview_add_tile(tileview_fixed, 0, 0, (lv_dir_t)(LV_DIR_LEFT | LV_DIR_RIGHT));

        lv_obj_t *image_button_fixed[sizeof(_win_home_app_icon_fixed_list) / sizeof(Win_Home_App_Icon)];
        for (uint16_t i = 0; i < sizeof(_win_home_app_icon_fixed_list) / sizeof(Win_Home_App_Icon); i++)
        {
            // 定义需要过渡的属性：缩放 X 和缩放 Y
            static const lv_style_prop_t tr_prop[] = {LV_STYLE_TRANSFORM_SCALE_X, LV_STYLE_TRANSFORM_SCALE_Y, 0};
            // 创建过渡描述符
            static lv_style_transition_dsc_t tr;
            lv_style_transition_dsc_init(&tr, tr_prop, lv_anim_path_ease_out, 100, 0, NULL);
            // 创建默认样式
            static lv_style_t style_def;
            lv_style_init(&style_def);
            // 设置过渡效果
            lv_style_set_transition(&style_def, &tr);
            // 设置缩放的中心点为对象的中心
            lv_style_set_transform_pivot_x(&style_def, LV_PCT(50)); // X 方向的中心点
            lv_style_set_transform_pivot_y(&style_def, LV_PCT(50)); // Y 方向的中心点
            // 创建按下时的样式
            static lv_style_t style_pr;
            lv_style_init(&style_pr);
            // 设置按下时的缩放比例
            lv_style_set_transform_scale_x(&style_pr, 256 * 0.9);
            lv_style_set_transform_scale_y(&style_pr, 256 * 0.9);
            // 设置缩放的中心点为对象的中心
            lv_style_set_transform_pivot_x(&style_pr, LV_PCT(50)); // X 方向的中心点
            lv_style_set_transform_pivot_y(&style_pr, LV_PCT(50)); // Y 方向的中心点

            // 创建图像按钮
            image_button_fixed[i] = lv_imagebutton_create(tileview_fixed_tile_1);
            lv_imagebutton_set_src(image_button_fixed[i], LV_IMAGEBUTTON_STATE_RELEASED, NULL, _win_home_app_icon_fixed_list[i].image, NULL);
            lv_imagebutton_set_src(image_button_fixed[i], LV_IMAGEBUTTON_STATE_PRESSED, NULL, _win_home_app_icon_fixed_list[i].image, NULL);
            lv_imagebutton_set_src(image_button_fixed[i], LV_IMAGEBUTTON_STATE_DISABLED, NULL, _win_home_app_icon_fixed_list[i].image, NULL);
            // 应用样式
            lv_obj_add_style(image_button_fixed[i], &style_def, (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_add_style(image_button_fixed[i], &style_pr, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_PRESSED);
            // 设置按钮位置
            lv_obj_align(image_button_fixed[i], LV_ALIGN_TOP_LEFT, _app_style.icon.edge_distance_fixed.width + (APP_STYLE_ICON_WIDTH_HEIGHT * i) + (_app_style.icon.icon_distance.fixed_width * i),
                         _app_style.icon.edge_distance_fixed.height);

            lv_obj_t *image_button_label = lv_label_create(tileview_fixed_tile_1);
            lv_label_set_text(image_button_label, _win_home_app_icon_fixed_list[i].name.c_str());
            lv_obj_set_style_text_align(image_button_label, LV_TEXT_ALIGN_CENTER, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
            lv_obj_set_style_text_font(image_button_label, &lv_font_montserrat_22, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
            lv_obj_set_size(image_button_label, _app_style.label.width, _app_style.label.height);
            lv_obj_set_style_text_color(image_button_label, lv_color_white(), (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
            lv_obj_align_to(image_button_label, image_button_fixed[i], LV_ALIGN_OUT_BOTTOM_MID, 0, 0);
        }

        lv_obj_add_event_cb(image_button_fixed[0], [](lv_event_t *e)
                            {
                                System *self = static_cast<System *>(lv_event_get_user_data(e));
                                lv_event_code_t code = lv_event_get_code(e);

                                switch (code)
                                {
                                case LV_EVENT_CLICKED:
                                self->init_win_camera();

                                // lv_screen_load_anim(self->_registry.win.camera.root, LV_SCR_LOAD_ANIM_FADE_OUT, 500, 0, true);
                                lv_screen_load(self->_registry.win.camera.root);
                                break;
                                default:
                                break;
                                } }, LV_EVENT_ALL, this);

        // Settings icon click handler
        lv_obj_add_event_cb(image_button_fixed[1], [](lv_event_t *e) {
            System *self = static_cast<System *>(lv_event_get_user_data(e));
            if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
            self->set_vibration();
            self->init_win_settings();
            lv_screen_load_anim(self->_registry.win.settings.root, LV_SCR_LOAD_ANIM_FADE_OUT, 100, 0, true);
        }, LV_EVENT_ALL, this);

        init_status_bar(_registry.win.home.root);

        lv_obj_update_layout(_registry.win.home.root);

        _current_win = Current_Win::HOME;
    }

    void System::status_bar_time_update(void)
    {
        char buffer_time[16];
        format_clock_time(_time.hour, _time.minute, buffer_time, sizeof(buffer_time), g_settings.show_seconds ? _time.second : -1);
        lv_label_set_text(_registry.status_bar.time_label, buffer_time);
    }

    void System::status_bar_battery_level_update(void)
    {
        if (_battery_level == 100)
        {
            lv_label_set_text(_registry.status_bar.battery_icon, LV_SYMBOL_BATTERY_FULL);
        }
        else if ((_battery_level >= 66) && (_battery_level < 100))
        {
            lv_label_set_text(_registry.status_bar.battery_icon, LV_SYMBOL_BATTERY_3);
        }
        else if ((_battery_level >= 33) && (_battery_level < 66))
        {
            lv_label_set_text(_registry.status_bar.battery_icon, LV_SYMBOL_BATTERY_2);
        }
        else if ((_battery_level > 0) && (_battery_level < 33))
        {
            lv_label_set_text(_registry.status_bar.battery_icon, LV_SYMBOL_BATTERY_1);
        }
        else
        {
            lv_label_set_text(_registry.status_bar.battery_icon, LV_SYMBOL_BATTERY_EMPTY);
        }
    }

    void System::status_bar_wifi_connect_status_update(void)
    {
        if (_wifi_connect_status == true)
        {
            // 显示wifi信号强度图标
            lv_obj_remove_flag(_registry.status_bar.wifi_signal_icon, LV_OBJ_FLAG_HIDDEN);
        }
        else
        {
            // 隐藏wifi信号强度图标
            lv_obj_add_flag(_registry.status_bar.wifi_signal_icon, LV_OBJ_FLAG_HIDDEN);
        }
    }

    void System::set_gps_status(bool fix_valid, int sats)
    {
        _gps_fix_valid = fix_valid;
        _gps_sats = sats;
    }

    void System::status_bar_gps_update(void)
    {
        if (!_registry.status_bar.gps_icon) return;  // not yet created
        char buf[16];
        if (_gps_fix_valid && _gps_sats > 0) {
            // Green — have GPS fix with satellite count
            snprintf(buf, sizeof(buf), "%d " LV_SYMBOL_GPS, _gps_sats);
            lv_obj_set_style_text_color(_registry.status_bar.gps_icon, lv_color_hex(0x00CC00), (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        } else if (_gps_sats > 0) {
            // Yellow — searching, can see satellites
            snprintf(buf, sizeof(buf), "%d " LV_SYMBOL_GPS, _gps_sats);
            lv_obj_set_style_text_color(_registry.status_bar.gps_icon, lv_color_hex(0xFFAA00), (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        } else {
            // Gray — no GPS data
            snprintf(buf, sizeof(buf), LV_SYMBOL_GPS);
            lv_obj_set_style_text_color(_registry.status_bar.gps_icon, lv_color_hex(0x666666), (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        }
        lv_label_set_text(_registry.status_bar.gps_icon, buf);

        // Reposition WiFi relative to GPS icon
        if (_registry.status_bar.wifi_signal_icon) {
            lv_obj_update_layout(_registry.status_bar.gps_icon);
            lv_obj_align_to(_registry.status_bar.wifi_signal_icon,
                            _registry.status_bar.gps_icon, LV_ALIGN_OUT_LEFT_MID, -5, 0);
        }
    }

    void System::set_adsb_status(bool connected, bool error, int aircraft_count)
    {
        _adsb_connected = connected;
        _adsb_error = error;
        _adsb_aircraft_count = aircraft_count;
    }

    void System::status_bar_adsb_update(void)
    {
        if (!_registry.status_bar.adsb_icon) return;  // not yet created
        char buf[16];
        if (_adsb_error) {
            // Red — device seen but can't read (e.g. buffer alloc failed)
            snprintf(buf, sizeof(buf), "! " LV_SYMBOL_UP);
            lv_obj_set_style_text_color(_registry.status_bar.adsb_icon, lv_color_hex(0xCC0000), (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        } else if (_adsb_connected && _adsb_aircraft_count > 0) {
            // Green — receiving aircraft
            snprintf(buf, sizeof(buf), "%d " LV_SYMBOL_UP, _adsb_aircraft_count);
            lv_obj_set_style_text_color(_registry.status_bar.adsb_icon, lv_color_hex(0x00CC00), (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        } else if (_adsb_connected) {
            // White — connected but no aircraft yet
            snprintf(buf, sizeof(buf), "0 " LV_SYMBOL_UP);
            lv_obj_set_style_text_color(_registry.status_bar.adsb_icon, lv_color_white(), (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        } else {
            // Gray — RTL-SDR not plugged in (icon only, no number)
            snprintf(buf, sizeof(buf), LV_SYMBOL_UP);
            lv_obj_set_style_text_color(_registry.status_bar.adsb_icon, lv_color_hex(0x666666), (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        }
        lv_label_set_text(_registry.status_bar.adsb_icon, buf);

        // Reposition GPS and WiFi relative to ADS-B icon (chains left from right edge).
        // Uses lv_obj_align_to so it works regardless of screen width or rotation.
        if (_registry.status_bar.gps_icon && _registry.status_bar.wifi_signal_icon) {
            lv_obj_update_layout(_registry.status_bar.adsb_icon);
            lv_obj_align_to(_registry.status_bar.gps_icon,
                            _registry.status_bar.adsb_icon, LV_ALIGN_OUT_LEFT_MID, -6, 0);

            lv_obj_update_layout(_registry.status_bar.gps_icon);
            lv_obj_align_to(_registry.status_bar.wifi_signal_icon,
                            _registry.status_bar.gps_icon, LV_ALIGN_OUT_LEFT_MID, -5, 0);
        }
    }

    void System::set_sd_status(bool mounted, bool logging)
    {
        _sd_mounted = mounted;
        _sd_logging = logging;
    }

    void System::status_bar_sd_update(void)
    {
        if (_sd_logging) {
            // Green — actively logging
            lv_label_set_text(_registry.status_bar.sd_icon, LV_SYMBOL_SD_CARD);
            lv_obj_set_style_text_color(_registry.status_bar.sd_icon, lv_color_hex(0x00CC00), (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        } else if (_sd_mounted) {
            // Yellow — mounted but not logging
            lv_label_set_text(_registry.status_bar.sd_icon, LV_SYMBOL_SD_CARD);
            lv_obj_set_style_text_color(_registry.status_bar.sd_icon, lv_color_hex(0xCC0000), (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        } else {
            // Gray — no SD card
            lv_label_set_text(_registry.status_bar.sd_icon, LV_SYMBOL_SD_CARD);
            lv_obj_set_style_text_color(_registry.status_bar.sd_icon, lv_color_hex(0x666666), (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        }
    }

    void System::win_home_time_update(void)
    {
        char buffer_time[16];
        format_clock_time(_time.hour, _time.minute, buffer_time, sizeof(buffer_time), g_settings.show_seconds ? _time.second : -1);
        lv_label_set_text(_registry.win.home.clock.time_label, buffer_time);

        std::string month_str = "null";
        switch (_time.month)
        {
        case 1:
            month_str = "January";
            break;
        case 2:
            month_str = "February";
            break;
        case 3:
            month_str = "March";
            break;
        case 4:
            month_str = "April";
            break;
        case 5:
            month_str = "May";
            break;
        case 6:
            month_str = "June";
            break;
        case 7:
            month_str = "July";
            break;
        case 8:
            month_str = "August";
            break;
        case 9:
            month_str = "September";
            break;
        case 10:
            month_str = "October";
            break;
        case 11:
            month_str = "November";
            break;
        case 12:
            month_str = "December";
            break;

        default:
            break;
        }
        char buffer_month[20];
        snprintf(buffer_month, sizeof(buffer_month), "%s %dth", month_str.c_str(), _time.day);
        lv_label_set_text(_registry.win.home.clock.month_label, buffer_month);

        lv_label_set_text(_registry.win.home.clock.week_label, _time.week.c_str());
    }

    void System::init_status_bar(lv_obj_t *parent)
    {
        // 创建一个容器来放置状态栏内容
        _registry.status_bar.root = lv_obj_create(parent);
        lv_obj_set_size(_registry.status_bar.root, LV_HOR_RES, 50);                                                // 设置状态栏的宽度和高度
        lv_obj_set_style_bg_opa(_registry.status_bar.root, LV_OPA_20, (lv_style_selector_t)LV_PART_MAIN);          // 设置背景透明度
        lv_obj_set_style_bg_color(_registry.status_bar.root, lv_color_black(), (lv_style_selector_t)LV_PART_MAIN); // 设置背景颜色
        lv_obj_set_style_border_width(_registry.status_bar.root, 0, (lv_style_selector_t)LV_PART_MAIN);            // 移除边框
        lv_obj_align(_registry.status_bar.root, LV_ALIGN_TOP_MID, 0, 0);                                           // 将状态栏对齐到顶部中间
        lv_obj_remove_flag(_registry.status_bar.root, LV_OBJ_FLAG_SCROLLABLE); // 禁止滚动
        lv_obj_remove_flag(_registry.status_bar.root, LV_OBJ_FLAG_CLICKABLE);  // 禁止触摸

        // 创建时间标签
        _registry.status_bar.time_label = lv_label_create(_registry.status_bar.root);
        lv_obj_set_style_text_color(_registry.status_bar.time_label, lv_color_white(), (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(_registry.status_bar.time_label, &lv_font_montserrat_22, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        char buffer_time[16];
        format_clock_time(_time.hour, _time.minute, buffer_time, sizeof(buffer_time), g_settings.show_seconds ? _time.second : -1);
        lv_label_set_text(_registry.status_bar.time_label, buffer_time);
        lv_obj_align(_registry.status_bar.time_label, LV_ALIGN_LEFT_MID, 0, 0);

        // 创建电池图标 (rightmost)
        _registry.status_bar.battery_icon = lv_label_create(_registry.status_bar.root);
        lv_obj_set_style_text_color(_registry.status_bar.battery_icon, lv_color_white(), (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(_registry.status_bar.battery_icon, &lv_font_montserrat_22, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        status_bar_battery_level_update();
        lv_obj_align(_registry.status_bar.battery_icon, LV_ALIGN_RIGHT_MID, 0, 0);

        // SD card status icon
        _registry.status_bar.sd_icon = lv_label_create(_registry.status_bar.root);
        lv_obj_set_style_text_font(_registry.status_bar.sd_icon, &lv_font_montserrat_22, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_align(_registry.status_bar.sd_icon, LV_ALIGN_RIGHT_MID, -39, 0);
        status_bar_sd_update();

        // ADS-B aircraft count icon
        _registry.status_bar.adsb_icon = lv_label_create(_registry.status_bar.root);
        lv_obj_set_style_text_font(_registry.status_bar.adsb_icon, &lv_font_montserrat_22, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_align(_registry.status_bar.adsb_icon, LV_ALIGN_RIGHT_MID, -68, 0);

        // GPS status icon — positioned dynamically by status_bar_adsb_update()
        _registry.status_bar.gps_icon = lv_label_create(_registry.status_bar.root);
        lv_obj_set_style_text_font(_registry.status_bar.gps_icon, &lv_font_montserrat_22, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);

        // 创建wifi信号强度图标 — positioned dynamically by status_bar_gps_update()
        _registry.status_bar.wifi_signal_icon = lv_label_create(_registry.status_bar.root);
        lv_obj_set_style_text_color(_registry.status_bar.wifi_signal_icon, lv_color_white(), (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(_registry.status_bar.wifi_signal_icon, &lv_font_montserrat_22, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_label_set_text(_registry.status_bar.wifi_signal_icon, LV_SYMBOL_WIFI);
        status_bar_wifi_connect_status_update();

        // Update dynamic icons AFTER all status bar objects exist.
        // adsb_update repositions gps_icon; gps_update repositions wifi_icon.
        status_bar_adsb_update();
        status_bar_gps_update();
    }

    void System::init_win_cit(void)
    {
        // 主界面
        _registry.win.cit.root = lv_obj_create(NULL);
        lv_obj_set_style_bg_color(_registry.win.cit.root, lv_color_hex(0xFF7F58), (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_size(_registry.win.cit.root, _width, _height);
        lv_obj_set_scrollbar_mode(_registry.win.cit.root, LV_SCROLLBAR_MODE_OFF);

        // 创建标题
        lv_obj_t *title_label = lv_label_create(_registry.win.cit.root);
        lv_label_set_text(title_label, "CIT");
        lv_obj_set_style_text_color(title_label, lv_color_white(), (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_text_align(title_label, LV_TEXT_ALIGN_LEFT, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(title_label, &lv_font_montserrat_48, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_size(title_label, _width - 100, 40);
        lv_obj_align(title_label, LV_ALIGN_TOP_LEFT, 20, 10 + 50);

        // 创建列表
        lv_obj_t *list = lv_list_create(_registry.win.cit.root);
        lv_obj_set_size(list, _width, _height - 50 - 80);
        lv_obj_align(list, LV_ALIGN_BOTTOM_MID, 0, 0);
        lv_obj_set_style_pad_left(list, 20, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_right(list, 20, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(list, &lv_font_montserrat_24, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_border_width(list, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_radius(list, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_scrollbar_mode(list, LV_SCROLLBAR_MODE_ACTIVE);

        lv_obj_t *list_button[sizeof(_win_cit_test_item_list) / sizeof(Win_Cit_Test_Item)];
        for (uint16_t i = 0; i < sizeof(_win_cit_test_item_list) / sizeof(Win_Cit_Test_Item); i++)
        {
            list_button[i] = lv_list_add_button(list, _win_cit_test_item_list[i].symbol.c_str(), _win_cit_test_item_list[i].name.c_str());
            lv_obj_set_style_text_color(list_button[i], lv_color_hex(_win_cit_test_item_list[i].color), (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
            lv_obj_set_style_text_font(list_button[i], &lv_font_montserrat_30, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        }

        lv_obj_add_event_cb(list_button[0], [](lv_event_t *e)
                            {
                                System *self = static_cast<System *>(lv_event_get_user_data(e));
                                lv_event_code_t code = lv_event_get_code(e);

                                switch (code)
                                {
                                case LV_EVENT_CLICKED:
                                self->_registry.win.cit.current_test_item_index = 0;

                                self->init_win_cit_version_information_test();

                                lv_screen_load_anim(self->_registry.win.cit.version_information_test, LV_SCR_LOAD_ANIM_MOVE_LEFT, 100, 0, true);
                                break;
                                default:
                                break;
                                } }, LV_EVENT_ALL, this);

        lv_obj_add_event_cb(list_button[1], [](lv_event_t *e)
                            {
                                System *self = static_cast<System *>(lv_event_get_user_data(e));
                                lv_event_code_t code = lv_event_get_code(e);

                                switch (code)
                                {
                                case LV_EVENT_CLICKED:
                                self->_registry.win.cit.current_test_item_index = 1;

                                self->init_win_cit_touch_test();

                                lv_screen_load_anim(self->_registry.win.cit.touch_test.root, LV_SCR_LOAD_ANIM_MOVE_LEFT, 100, 0, true);
                                break;
                                default:
                                break;
                                } }, LV_EVENT_ALL, this);
        lv_obj_add_event_cb(list_button[2], [](lv_event_t *e)
                            {
                                System *self = static_cast<System *>(lv_event_get_user_data(e));
                                lv_event_code_t code = lv_event_get_code(e);

                                switch (code)
                                {
                                case LV_EVENT_CLICKED:
                                self->_registry.win.cit.current_test_item_index = 2;

                                self->init_win_cit_screen_color_test();

                                lv_screen_load_anim(self->_registry.win.cit.screen_color_test.root, LV_SCR_LOAD_ANIM_MOVE_LEFT, 100, 0, true);
                                break;
                                default:
                                break;
                                } }, LV_EVENT_ALL, this);

        lv_obj_add_event_cb(list_button[3], [](lv_event_t *e)
                            {
                                System *self = static_cast<System *>(lv_event_get_user_data(e));
                                lv_event_code_t code = lv_event_get_code(e);

                                switch (code)
                                {
                                case LV_EVENT_CLICKED:
                                self->_registry.win.cit.current_test_item_index = 3;

                                self->init_win_cit_vibration_test();

                                lv_screen_load_anim(self->_registry.win.cit.vibration_test.root, LV_SCR_LOAD_ANIM_MOVE_LEFT, 100, 0, true);
                                break;
                                default:
                                break;
                                } }, LV_EVENT_ALL, this);

        lv_obj_add_event_cb(list_button[4], [](lv_event_t *e)
                            {
                                    System *self = static_cast<System *>(lv_event_get_user_data(e));
                                    lv_event_code_t code = lv_event_get_code(e);
    
                                    switch (code)
                                    {
                                    case LV_EVENT_CLICKED:
                                    self->_registry.win.cit.current_test_item_index = 4;
    
                                    self->init_win_cit_speaker_test();
    
                                    lv_screen_load_anim(self->_registry.win.cit.speaker_test, LV_SCR_LOAD_ANIM_MOVE_LEFT, 100, 0, true);
                                    break;
                                    default:
                                    break;
                                    } }, LV_EVENT_ALL, this);

        lv_obj_add_event_cb(list_button[5], [](lv_event_t *e)
                            {
                                System *self = static_cast<System *>(lv_event_get_user_data(e));
                                lv_event_code_t code = lv_event_get_code(e);

                                switch (code)
                                {
                                case LV_EVENT_CLICKED:
                                self->_registry.win.cit.current_test_item_index = 5;

                                self->init_win_cit_microphone_test();

                                lv_screen_load_anim(self->_registry.win.cit.microphone_test.root, LV_SCR_LOAD_ANIM_MOVE_LEFT, 100, 0, true);
                                break;
                                default:
                                break;
                                } }, LV_EVENT_ALL, this);

        lv_obj_add_event_cb(list_button[6], [](lv_event_t *e)
                            {
                                System *self = static_cast<System *>(lv_event_get_user_data(e));
                                lv_event_code_t code = lv_event_get_code(e);

                                switch (code)
                                {
                                case LV_EVENT_CLICKED:
                                self->_registry.win.cit.current_test_item_index = 6;

                                self->init_win_cit_imu_test();

                                lv_screen_load_anim(self->_registry.win.cit.imu_test.root, LV_SCR_LOAD_ANIM_MOVE_LEFT, 100, 0, true);
                                break;
                                default:
                                break;
                                } }, LV_EVENT_ALL, this);

        lv_obj_add_event_cb(list_button[7], [](lv_event_t *e)
                            {
                                System *self = static_cast<System *>(lv_event_get_user_data(e));
                                lv_event_code_t code = lv_event_get_code(e);

                                switch (code)
                                {
                                case LV_EVENT_CLICKED:
                                self->_registry.win.cit.current_test_item_index = 7;

                                self->init_win_cit_battery_health_test();

                                lv_screen_load_anim(self->_registry.win.cit.battery_health_test.root, LV_SCR_LOAD_ANIM_MOVE_LEFT, 100, 0, true);
                                break;
                                default:
                                break;
                                } }, LV_EVENT_ALL, this);

        lv_obj_add_event_cb(list_button[8], [](lv_event_t *e)
                            {
                                    System *self = static_cast<System *>(lv_event_get_user_data(e));
                                    lv_event_code_t code = lv_event_get_code(e);
    
                                    switch (code)
                                    {
                                    case LV_EVENT_CLICKED:
                                    self->_registry.win.cit.current_test_item_index = 8;
    
                                    self->init_win_cit_gps_test();
    
                                    lv_screen_load_anim(self->_registry.win.cit.gps_test.root, LV_SCR_LOAD_ANIM_MOVE_LEFT, 100, 0, true);
                                    break;
                                    default:
                                    break;
                                    } }, LV_EVENT_ALL, this);
        lv_obj_add_event_cb(list_button[9], [](lv_event_t *e)
                            {
                                System *self = static_cast<System *>(lv_event_get_user_data(e));
                                lv_event_code_t code = lv_event_get_code(e);

                                switch (code)
                                {
                                case LV_EVENT_CLICKED:
                                self->_registry.win.cit.current_test_item_index = 9;

                                self->init_win_cit_ethernet_test();

                                lv_screen_load_anim(self->_registry.win.cit.ethernet_test.root, LV_SCR_LOAD_ANIM_MOVE_LEFT, 100, 0, true);
                                break;
                                default:
                                break;
                                } }, LV_EVENT_ALL, this);

        lv_obj_add_event_cb(list_button[10], [](lv_event_t *e)
                            {
                                System *self = static_cast<System *>(lv_event_get_user_data(e));
                                lv_event_code_t code = lv_event_get_code(e);

                                switch (code)
                                {
                                case LV_EVENT_CLICKED:
                                self->_registry.win.cit.current_test_item_index = 10;

                                self->init_win_cit_rtc_test();

                                lv_screen_load_anim(self->_registry.win.cit.rtc_test.root, LV_SCR_LOAD_ANIM_MOVE_LEFT, 100, 0, true);
                                break;
                                default:
                                break;
                                } }, LV_EVENT_ALL, this);

        lv_obj_add_event_cb(list_button[11], [](lv_event_t *e)
                            {
                                System *self = static_cast<System *>(lv_event_get_user_data(e));
                                lv_event_code_t code = lv_event_get_code(e);

                                switch (code)
                                {
                                case LV_EVENT_CLICKED:
                                self->_registry.win.cit.current_test_item_index = 11;

                                self->init_win_cit_esp32c6_at_test();

                                lv_screen_load_anim(self->_registry.win.cit.esp32c6_at_test.root, LV_SCR_LOAD_ANIM_MOVE_LEFT, 100, 0, true);
                                break;
                                default:
                                break;
                                } }, LV_EVENT_ALL, this);

#if defined CONFIG_BOARD_TYPE_T_DISPLAY_P4_KEYBOARD
        lv_obj_add_event_cb(list_button[12], [](lv_event_t *e)
                            {
                                System *self = static_cast<System *>(lv_event_get_user_data(e));
                                lv_event_code_t code = lv_event_get_code(e);

                                switch (code)
                                {
                                case LV_EVENT_CLICKED:
                                self->_registry.win.cit.current_test_item_index = 12;

                                self->init_win_cit_keyboard_test();

                                lv_screen_load_anim(self->_registry.win.cit.keyboard_test.root, LV_SCR_LOAD_ANIM_MOVE_LEFT, 100, 0, true);
                                break;
                                default:
                                break;
                                } }, LV_EVENT_ALL, this);

        lv_obj_add_event_cb(list_button[13], [](lv_event_t *e)
                            {
                                System *self = static_cast<System *>(lv_event_get_user_data(e));
                                lv_event_code_t code = lv_event_get_code(e);

                                switch (code)
                                {
                                case LV_EVENT_CLICKED:
                                self->_registry.win.cit.current_test_item_index = 13;

                                self->init_win_cit_nfc_test();

                                lv_screen_load_anim(self->_registry.win.cit.nfc_test.root, LV_SCR_LOAD_ANIM_MOVE_LEFT, 100, 0, true);
                                break;
                                default:
                                break;
                                } }, LV_EVENT_ALL, this);
#endif

        // lv_obj_add_event_cb(list_button[12], [](lv_event_t *e)
        //                     {
        //                         System *self = static_cast<System *>(lv_event_get_user_data(e));
        //                         lv_event_code_t code = lv_event_get_code(e);

        //                         switch (code)
        //                         {
        //                         case LV_EVENT_CLICKED:
        //                         self->_registry.win.cit.current_test_item_index = 12;

        //                         self->init_win_cit_sleep_test();

        //                         lv_screen_load_anim(self->_registry.win.cit.sleep_test, LV_SCR_LOAD_ANIM_MOVE_LEFT, 100, 0, true);
        //                         break;
        //                         default:
        //                         break;
        //                         } }, LV_EVENT_ALL, this);

        lv_obj_add_event_cb(_registry.win.cit.root, [](lv_event_t *e)
                            {
                                System *self = static_cast<System *>(lv_event_get_user_data(e));
                                lv_event_code_t code = lv_event_get_code(e);

                                if (code == LV_EVENT_GESTURE)
                                {
                                    lv_dir_t gesture_dir = lv_indev_get_gesture_dir(lv_indev_active());

                                    // 边缘检测以及左右滑动
                                    if ((gesture_dir == LV_DIR_LEFT || gesture_dir == LV_DIR_RIGHT)&&(self->_edge_touch_flag == true))
                                    {
                                        self->set_vibration();
                                        self->init_win_home();
                                        
                                        lv_screen_load_anim(self->_registry.win.home.root, LV_SCR_LOAD_ANIM_FADE_OUT, 100, 0, true);

                                        self->_edge_touch_flag = false;
                                    }
                                } }, LV_EVENT_ALL, this);

        init_status_bar(_registry.win.cit.root);

        lv_obj_update_layout(_registry.win.cit.root);

        _current_win = Current_Win::CIT;
    }

    void System::init_win_cit_version_information_test(void)
    {
        _registry.win.cit.version_information_test = lv_obj_create(NULL);
        lv_obj_set_style_bg_color(_registry.win.cit.version_information_test, lv_color_hex(0xFF7F58), (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_size(_registry.win.cit.version_information_test, _width, _height);
        lv_obj_set_scrollbar_mode(_registry.win.cit.version_information_test, LV_SCROLLBAR_MODE_OFF);

        // 创建标题
        lv_obj_t *title_label = lv_label_create(_registry.win.cit.version_information_test);
        lv_label_set_text(title_label, "Version Information Test");
        lv_obj_set_style_text_color(title_label, lv_color_white(), (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_text_align(title_label, LV_TEXT_ALIGN_LEFT, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(title_label, &lv_font_montserrat_48, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_size(title_label, _width - 100, 40);
        lv_obj_align(title_label, LV_ALIGN_TOP_LEFT, 20, 10 + 50);

        // 创建列表
        lv_obj_t *list = lv_list_create(_registry.win.cit.version_information_test);
        lv_obj_set_size(list, _width, _height - 50 - 80 - 140);
        lv_obj_align(list, LV_ALIGN_TOP_MID, 0, 50 + 80);
        lv_obj_set_style_pad_left(list, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_right(list, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(list, &lv_font_montserrat_24, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_border_width(list, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_radius(list, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_scrollbar_mode(list, LV_SCROLLBAR_MODE_ACTIVE);

        for (uint16_t i = 0; i < sizeof(_device_information_list) / sizeof(Device_Information); i++)
        {
            lv_obj_t *list_button = lv_list_add_button(list, NULL, (_device_information_list[i].name + _device_information_list[i].info).c_str());

            lv_obj_set_style_text_font(list_button, &lv_font_montserrat_28, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        }

        add_win_cit_test_item_pass_fail_button(_registry.win.cit.version_information_test);

        add_event_cb_win_return_to_cit(_registry.win.cit.version_information_test);

        init_status_bar(_registry.win.cit.version_information_test);

        lv_obj_update_layout(_registry.win.cit.version_information_test);
    }

    void System::init_win_cit_touch_test(void)
    {
        _registry.win.cit.touch_test.root = lv_obj_create(NULL);
        lv_obj_set_style_bg_color(_registry.win.cit.touch_test.root, lv_color_hex(0xFF7F58), (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_size(_registry.win.cit.touch_test.root, _width, _height);
        lv_obj_set_scrollbar_mode(_registry.win.cit.touch_test.root, LV_SCROLLBAR_MODE_OFF);

        // 创建画布并初始化调色板
        _registry.win.cit.touch_test.canvas = lv_canvas_create(_registry.win.cit.touch_test.root);
        lv_canvas_set_buffer(_registry.win.cit.touch_test.canvas, _lv_color_win_draw_buf.get(), _width, _height, LVGL_COLOR_FORMAT);
        lv_canvas_fill_bg(_registry.win.cit.touch_test.canvas, lv_color_hex(0xCCCCCC), LV_OPA_COVER);
        lv_obj_center(_registry.win.cit.touch_test.canvas);

        lv_canvas_init_layer(_registry.win.cit.touch_test.canvas, &_registry.win.cit.touch_test.layer);

        _registry.win.cit.touch_test.draw_x.clear();
        _registry.win.cit.touch_test.draw_y.clear();

        lv_obj_add_event_cb(_registry.win.cit.touch_test.root, [](lv_event_t *e)
                            {
                                System *self = static_cast<System *>(lv_event_get_user_data(e));
                                lv_event_code_t code = lv_event_get_code(e);

                                switch (code)
                                {
                                case LV_EVENT_PRESSING:
                                {
                                    lv_point_t point;
                                    lv_indev_get_point(lv_indev_active(), &point);

                                    // printf("touch x: %ld y: %ld\n", point.x, point.y);

                                    // 在画布上绘制点
                                    // lv_canvas_set_px(canvas, point.x, point.y, lv_palette_main(LV_PALETTE_RED), LV_OPA_COVER);

                                    // printf("touch finger: %d edge touch flag: %d\n", self->_touch_point.finger_count, self->_touch_point.edge_touch_flag);
                                    // for (uint8_t i = 0; i < self->_touch_point.info.size(); i++)
                                    // {
                                    //     printf("touch num [%d] x: %d y: %d p: %d\n", i + 1, self->_touch_point.info[i].x, self->_touch_point.info[i].y, self->_touch_point.info[i].pressure_value);
                                    // }

                                    // 将触摸数据格式化为字符串
                                    std::string touch_data = "touch data:\n";
                                    touch_data += "finger count: " + std::to_string(self->_touch_point.finger_count) + "\n";
                                    touch_data += "edge touch flag: " + std::to_string(self->_touch_point.edge_touch_flag) + "\n";

                                    for (uint8_t i = 0; i < self->_touch_point.info.size(); i++)
                                    {
                                        touch_data += "touch [" + std::to_string(i + 1) + "] x: " + std::to_string(self->_touch_point.info[i].x) +
                                                      " y: " + std::to_string(self->_touch_point.info[i].y) +
                                                      " p: " + std::to_string(self->_touch_point.info[i].pressure_value) + "\n";
                                    }
                                    // 更新触摸数据的标签
                                    lv_label_set_text(self->_registry.win.cit.touch_test.touch_data_label, touch_data.c_str());
                                    lv_obj_align(self->_registry.win.cit.touch_test.touch_data_label, LV_ALIGN_CENTER, 0, 0);

                                    self->_registry.win.cit.touch_test.draw_x.push_back(point.x);
                                    self->_registry.win.cit.touch_test.draw_y.push_back(point.y);

                                    if ((self->_registry.win.cit.touch_test.draw_x.size() >= 2) && (self->_registry.win.cit.touch_test.draw_y.size() >= 2))
                                    {
                                        lv_draw_line_dsc_t dsc;
                                        lv_draw_line_dsc_init(&dsc);
                                        dsc.color = lv_palette_main(LV_PALETTE_RED);
                                        dsc.width = 4;
                                        dsc.round_end = 1;
                                        dsc.round_start = 1;
                                        dsc.p1.x = self->_registry.win.cit.touch_test.draw_x[0];
                                        dsc.p1.y = self->_registry.win.cit.touch_test.draw_y[0];
                                        dsc.p2.x = self->_registry.win.cit.touch_test.draw_x[1];
                                        dsc.p2.y = self->_registry.win.cit.touch_test.draw_y[1];
                                        lv_draw_line(&self->_registry.win.cit.touch_test.layer, &dsc);

                                        lv_canvas_finish_layer(self->_registry.win.cit.touch_test.canvas, &self->_registry.win.cit.touch_test.layer);

                                        self->_registry.win.cit.touch_test.draw_x.erase(self->_registry.win.cit.touch_test.draw_x.begin());
                                        self->_registry.win.cit.touch_test.draw_y.erase(self->_registry.win.cit.touch_test.draw_y.begin());
                                    }
                                }
                                    break;

                                case LV_EVENT_RELEASED:
                                    self->_registry.win.cit.touch_test.draw_x.clear();
                                    self->_registry.win.cit.touch_test.draw_y.clear();
                                break;
                                default:
                                    break;
                                } }, LV_EVENT_ALL, this);

        // 创建一个标签用于显示触摸点数据
        _registry.win.cit.touch_test.touch_data_label = lv_label_create(_registry.win.cit.touch_test.root);
        lv_obj_set_style_text_color(_registry.win.cit.touch_test.touch_data_label, lv_color_black(), (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(_registry.win.cit.touch_test.touch_data_label, &lv_font_montserrat_24, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_label_set_text(_registry.win.cit.touch_test.touch_data_label, "touch data:");
        lv_obj_align(_registry.win.cit.touch_test.touch_data_label, LV_ALIGN_CENTER, 0, 0);

        add_event_cb_win_return_to_cit(_registry.win.cit.touch_test.root);

        init_status_bar(_registry.win.cit.touch_test.root);

        lv_obj_update_layout(_registry.win.cit.touch_test.root);

        _current_win = Current_Win::CIT_TOUCH_TEST;
    }

    void System::init_win_cit_screen_color_test(void)
    {
        // 主界面
        _registry.win.cit.screen_color_test.root = lv_obj_create(NULL);
        lv_obj_set_style_bg_color(_registry.win.cit.screen_color_test.root, lv_color_hex(0xFF7F58), (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_size(_registry.win.cit.screen_color_test.root, _width, _height);
        lv_obj_set_scrollbar_mode(_registry.win.cit.screen_color_test.root, LV_SCROLLBAR_MODE_OFF);

        // 创建标题
        lv_obj_t *title_label = lv_label_create(_registry.win.cit.screen_color_test.root);
        lv_label_set_text(title_label, "Screen Color");
        lv_obj_set_style_text_color(title_label, lv_color_white(), (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_text_align(title_label, LV_TEXT_ALIGN_LEFT, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(title_label, &lv_font_montserrat_48, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_size(title_label, _width - 100, 40);
        lv_obj_align(title_label, LV_ALIGN_TOP_LEFT, 20, 10 + 50);

        // 创建容器
        lv_obj_t *container = lv_obj_create(_registry.win.cit.screen_color_test.root);
        lv_obj_set_size(container, _width, _height - 50 - 80 - 140);
        lv_obj_align(container, LV_ALIGN_TOP_MID, 0, 50 + 80);
        lv_obj_set_style_bg_color(container, lv_color_white(), (lv_style_selector_t)LV_PART_MAIN); // 设置背景颜色为白色
        lv_obj_set_style_radius(container, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_border_width(container, 0, (lv_style_selector_t)LV_PART_MAIN); // 移除边框

        // 创建START按键
        lv_obj_t *start_button = lv_button_create(container);
        lv_obj_set_size(start_button, 150, 80);
        lv_obj_align(start_button, LV_ALIGN_RIGHT_MID, -10, 0);
        lv_obj_set_style_radius(start_button, 10, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_width(start_button, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT); // 移除阴影
        lv_obj_set_style_bg_color(start_button, lv_color_hex(0xFF6A6A), (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_align(start_button, LV_ALIGN_CENTER, 0, 0);

        lv_obj_t *start_label = lv_label_create(start_button);
        lv_obj_set_style_text_font(start_label, &lv_font_montserrat_24, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_label_set_text(start_label, "START");
        lv_obj_center(start_label);

        lv_obj_add_event_cb(start_button, [](lv_event_t *e)
                            {
                                System *self = static_cast<System *>(lv_event_get_user_data(e));
                                lv_event_code_t code = lv_event_get_code(e);

                                switch (code)
                                {
                                case LV_EVENT_CLICKED:

                                    self->_registry.win.cit.screen_color_test.color_change_count = 0;

                                    self->init_win_cit_screen_color_test_start_color_test();

                                    lv_screen_load(self->_registry.win.cit.screen_color_test.start_color_test);
                                    break;
                                default:
                                    break;
                                } }, LV_EVENT_ALL, this);

        add_win_cit_test_item_pass_fail_button(_registry.win.cit.screen_color_test.root);

        add_event_cb_win_return_to_cit(_registry.win.cit.screen_color_test.root);

        init_status_bar(_registry.win.cit.screen_color_test.root);

        lv_obj_update_layout(_registry.win.cit.screen_color_test.root);
    }

    void System::init_win_cit_screen_color_test_start_color_test(void)
    {
        // 主界面
        _registry.win.cit.screen_color_test.start_color_test = lv_obj_create(NULL);
        lv_obj_set_style_bg_color(_registry.win.cit.screen_color_test.start_color_test, lv_color_hex(0xFF0000), (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_size(_registry.win.cit.screen_color_test.start_color_test, _width, _height);
        lv_obj_set_scrollbar_mode(_registry.win.cit.screen_color_test.start_color_test, LV_SCROLLBAR_MODE_OFF);
        const uint32_t color_list[4] = {0xFF0000, 0x00FF00, 0x0000FF, 0xFFFFFF};
        lv_obj_add_event_cb(_registry.win.cit.screen_color_test.start_color_test, [](lv_event_t *e)
                            {
                                System *self = static_cast<System *>(lv_event_get_user_data(e));
                                lv_event_code_t code = lv_event_get_code(e);

                                switch (code)
                                {
                                case LV_EVENT_CLICKED:
                                    self->_registry.win.cit.screen_color_test.color_change_count++;

                                    switch (self->_registry.win.cit.screen_color_test.color_change_count)
                                    {
                                    case 1:
                                        lv_obj_set_style_bg_color(self->_registry.win.cit.screen_color_test.start_color_test, lv_color_hex(0x00FF00), (lv_style_selector_t)LV_PART_MAIN);
                                        break;
                                    case 2:
                                        lv_obj_set_style_bg_color(self->_registry.win.cit.screen_color_test.start_color_test, lv_color_hex(0x00FF00), (lv_style_selector_t)LV_PART_MAIN);
                                        break;
                                    case 3:
                                        lv_obj_set_style_bg_color(self->_registry.win.cit.screen_color_test.start_color_test, lv_color_hex(0x0000FF), (lv_style_selector_t)LV_PART_MAIN);
                                        break;
                                    case 4:
                                        lv_obj_set_style_bg_color(self->_registry.win.cit.screen_color_test.start_color_test, lv_color_hex(0xFFFFFF), (lv_style_selector_t)LV_PART_MAIN);
                                        break;
                                    case 5:
                                        self->init_win_cit_screen_color_test();

                                        lv_screen_load_anim(self->_registry.win.cit.screen_color_test.root, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 100, 0, true);
                                    break;

                                    default:
                                        break;
                                    }


                                    break;
                                
                                default:
                                    break;
                                }

                                if (code == LV_EVENT_GESTURE)
                                {
                                lv_dir_t gesture_dir = lv_indev_get_gesture_dir(lv_indev_active());

                                // 边缘检测以及左右滑动
                                if ((gesture_dir == LV_DIR_LEFT || gesture_dir == LV_DIR_RIGHT)&&(self->_edge_touch_flag == true))
                                {
                                    self->set_vibration();
                                    self->init_win_cit_screen_color_test();

                                    lv_screen_load_anim(self->_registry.win.cit.screen_color_test.root, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 100, 0, true);

                                    self->_edge_touch_flag = false;
                                }
                                } }, LV_EVENT_ALL, this);

        lv_obj_update_layout(_registry.win.cit.screen_color_test.start_color_test);
    }

    void System::init_win_cit_vibration_test(void)
    {
        // 主界面
        _registry.win.cit.vibration_test.root = lv_obj_create(NULL);
        lv_obj_set_style_bg_color(_registry.win.cit.vibration_test.root, lv_color_hex(0xFF7F58), (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_size(_registry.win.cit.vibration_test.root, _width, _height);
        lv_obj_set_scrollbar_mode(_registry.win.cit.vibration_test.root, LV_SCROLLBAR_MODE_OFF);

        // 创建标题
        lv_obj_t *title_label = lv_label_create(_registry.win.cit.vibration_test.root);
        lv_label_set_text(title_label, "Vibration");
        lv_obj_set_style_text_color(title_label, lv_color_white(), (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_text_align(title_label, LV_TEXT_ALIGN_LEFT, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(title_label, &lv_font_montserrat_48, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_size(title_label, _width - 100, 40);
        lv_obj_align(title_label, LV_ALIGN_TOP_LEFT, 20, 10 + 50);

        // 创建容器
        lv_obj_t *container = lv_obj_create(_registry.win.cit.vibration_test.root);
        lv_obj_set_size(container, _width, _height - 50 - 80 - 140);
        lv_obj_align(container, LV_ALIGN_TOP_MID, 0, 50 + 80);
        lv_obj_set_style_bg_color(container, lv_color_white(), (lv_style_selector_t)LV_PART_MAIN); // 设置背景颜色为白色
        lv_obj_set_style_radius(container, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_border_width(container, 0, (lv_style_selector_t)LV_PART_MAIN); // 移除边框

        // 创建一个标签用于显示振动数据
        _registry.win.cit.vibration_test.data_label = lv_label_create(container);
        lv_obj_set_style_text_color(_registry.win.cit.vibration_test.data_label, lv_color_black(), (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(_registry.win.cit.vibration_test.data_label, &lv_font_montserrat_24, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_label_set_text(_registry.win.cit.vibration_test.data_label, "vibration data:");
        lv_obj_align(_registry.win.cit.vibration_test.data_label, LV_ALIGN_CENTER, 0, -50);

        // 创建START F0按键
        lv_obj_t *start_button = lv_button_create(container);
        lv_obj_set_size(start_button, 200, 80);
        lv_obj_align(start_button, LV_ALIGN_RIGHT_MID, -10, 0);
        lv_obj_set_style_radius(start_button, 10, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_width(start_button, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT); // 移除阴影
        lv_obj_set_style_bg_color(start_button, lv_color_hex(0xFF6A6A), (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_align_to(start_button, _registry.win.cit.vibration_test.data_label, LV_ALIGN_OUT_BOTTOM_MID, 0, 50);

        lv_obj_t *start_label = lv_label_create(start_button);
        lv_obj_set_style_text_font(start_label, &lv_font_montserrat_24, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_label_set_text(start_label, "START F0");
        lv_obj_center(start_label);

        lv_obj_add_event_cb(start_button, [](lv_event_t *e)
                            {
                                System *self = static_cast<System *>(lv_event_get_user_data(e));
                                lv_event_code_t code = lv_event_get_code(e);

                                switch (code)
                                {
                                case LV_EVENT_CLICKED:

                                    self->set_vibration(-1);//启动振动F0校验
                                    
                                    break;
                                default:
                                    break;
                                } }, LV_EVENT_ALL, this);

        add_win_cit_test_item_pass_fail_button(_registry.win.cit.vibration_test.root);

        add_event_cb_win_return_to_cit(_registry.win.cit.vibration_test.root);

        init_status_bar(_registry.win.cit.vibration_test.root);

        lv_obj_update_layout(_registry.win.cit.vibration_test.root);

        _current_win = Current_Win::CIT_VIBRATION_TEST;
    }

    void System::init_win_cit_speaker_test(void)
    {
        // 主界面
        _registry.win.cit.speaker_test = lv_obj_create(NULL);
        lv_obj_set_style_bg_color(_registry.win.cit.speaker_test, lv_color_hex(0xFF7F58), (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_size(_registry.win.cit.speaker_test, _width, _height);
        lv_obj_set_scrollbar_mode(_registry.win.cit.speaker_test, LV_SCROLLBAR_MODE_OFF);

        // 创建标题
        lv_obj_t *title_label = lv_label_create(_registry.win.cit.speaker_test);
        lv_label_set_text(title_label, "Speaker");
        lv_obj_set_style_text_color(title_label, lv_color_white(), (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_text_align(title_label, LV_TEXT_ALIGN_LEFT, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(title_label, &lv_font_montserrat_48, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_size(title_label, _width - 100, 40);
        lv_obj_align(title_label, LV_ALIGN_TOP_LEFT, 20, 10 + 50);

        // 创建容器
        lv_obj_t *container = lv_obj_create(_registry.win.cit.speaker_test);
        lv_obj_set_size(container, _width, _height - 50 - 80 - 140);
        lv_obj_align(container, LV_ALIGN_TOP_MID, 0, 50 + 80);
        lv_obj_set_style_bg_color(container, lv_color_white(), (lv_style_selector_t)LV_PART_MAIN); // 设置背景颜色为白色
        lv_obj_set_style_radius(container, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_border_width(container, 0, (lv_style_selector_t)LV_PART_MAIN); // 移除边框

        // 创建START按键
        lv_obj_t *start_button = lv_button_create(container);
        lv_obj_set_size(start_button, 250, 80);
        lv_obj_align(start_button, LV_ALIGN_RIGHT_MID, -10, 0);
        lv_obj_set_style_radius(start_button, 10, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_width(start_button, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT); // 移除阴影
        lv_obj_set_style_bg_color(start_button, lv_color_hex(0xFF6A6A), (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_align(start_button, LV_ALIGN_CENTER, 0, 0);

        lv_obj_t *start_label = lv_label_create(start_button);
        lv_obj_set_style_text_font(start_label, &lv_font_montserrat_24, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_label_set_text(start_label, "START PLAY");
        lv_obj_center(start_label);

        lv_obj_add_event_cb(start_button, [](lv_event_t *e)
                            {
                                System *self = static_cast<System *>(lv_event_get_user_data(e));
                                lv_event_code_t code = lv_event_get_code(e);

                                switch (code)
                                {
                                case LV_EVENT_CLICKED:
                                    self->set_speaker_test();

                                    break;
                                default:
                                    break;
                                } }, LV_EVENT_ALL, this);

        add_win_cit_test_item_pass_fail_button(_registry.win.cit.speaker_test);

        add_event_cb_win_return_to_cit(_registry.win.cit.speaker_test);

        init_status_bar(_registry.win.cit.speaker_test);

        lv_obj_update_layout(_registry.win.cit.speaker_test);
    }

    void System::init_win_cit_microphone_test(void)
    {
        // 主界面
        _registry.win.cit.microphone_test.root = lv_obj_create(NULL);
        lv_obj_set_style_bg_color(_registry.win.cit.microphone_test.root, lv_color_hex(0xFF7F58), (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_size(_registry.win.cit.microphone_test.root, _width, _height);
        lv_obj_set_scrollbar_mode(_registry.win.cit.microphone_test.root, LV_SCROLLBAR_MODE_OFF);

        // 创建标题
        lv_obj_t *title_label = lv_label_create(_registry.win.cit.microphone_test.root);
        lv_label_set_text(title_label, "Microphone");
        lv_obj_set_style_text_color(title_label, lv_color_white(), (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_text_align(title_label, LV_TEXT_ALIGN_LEFT, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(title_label, &lv_font_montserrat_48, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_size(title_label, _width - 100, 40);
        lv_obj_align(title_label, LV_ALIGN_TOP_LEFT, 20, 10 + 50);

        // 创建容器
        lv_obj_t *container = lv_obj_create(_registry.win.cit.microphone_test.root);
        lv_obj_set_size(container, _width, _height - 50 - 80 - 140);
        lv_obj_align(container, LV_ALIGN_TOP_MID, 0, 50 + 80);
        lv_obj_set_style_bg_color(container, lv_color_white(), (lv_style_selector_t)LV_PART_MAIN); // 设置背景颜色为白色
        lv_obj_set_style_radius(container, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_border_width(container, 0, (lv_style_selector_t)LV_PART_MAIN); // 移除边框

        // 创建音量圆盘
        _registry.win.cit.microphone_test.scale_line = lv_scale_create(container);
        lv_obj_set_size(_registry.win.cit.microphone_test.scale_line, 400, 400);
        lv_scale_set_mode(_registry.win.cit.microphone_test.scale_line, LV_SCALE_MODE_ROUND_INNER);
        lv_obj_set_style_bg_opa(_registry.win.cit.microphone_test.scale_line, LV_OPA_COVER, 0);
        lv_obj_set_style_bg_color(_registry.win.cit.microphone_test.scale_line, lv_color_white(), 0);
        lv_obj_set_style_radius(_registry.win.cit.microphone_test.scale_line, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_clip_corner(_registry.win.cit.microphone_test.scale_line, true, 0);
        lv_obj_align(_registry.win.cit.microphone_test.scale_line, LV_ALIGN_TOP_MID, 0, 100);

        lv_scale_set_label_show(_registry.win.cit.microphone_test.scale_line, true);
        lv_scale_set_total_tick_count(_registry.win.cit.microphone_test.scale_line, 51);
        lv_scale_set_major_tick_every(_registry.win.cit.microphone_test.scale_line, 5);

        lv_obj_set_style_length(_registry.win.cit.microphone_test.scale_line, 5, LV_PART_ITEMS);
        lv_obj_set_style_length(_registry.win.cit.microphone_test.scale_line, 10, LV_PART_INDICATOR);
        lv_scale_set_range(_registry.win.cit.microphone_test.scale_line, 0, 100);

        lv_scale_set_angle_range(_registry.win.cit.microphone_test.scale_line, 270);
        lv_scale_set_rotation(_registry.win.cit.microphone_test.scale_line, 135);

        _registry.win.cit.microphone_test.needle_line = lv_line_create(_registry.win.cit.microphone_test.scale_line);
        lv_obj_set_style_line_width(_registry.win.cit.microphone_test.needle_line, 3, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_line_rounded(_registry.win.cit.microphone_test.needle_line, true, (lv_style_selector_t)LV_PART_MAIN);

        lv_scale_set_line_needle_value(_registry.win.cit.microphone_test.scale_line, _registry.win.cit.microphone_test.needle_line, 150, 0);

        // 创建一个标签用于显示麦克风数据
        _registry.win.cit.microphone_test.data.label = lv_label_create(container);
        lv_obj_set_style_text_color(_registry.win.cit.microphone_test.data.label, lv_color_black(), (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(_registry.win.cit.microphone_test.data.label, &lv_font_montserrat_24, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_label_set_text(_registry.win.cit.microphone_test.data.label, "microphone data:");
        lv_obj_align(_registry.win.cit.microphone_test.data.label, LV_ALIGN_TOP_MID, 0, 500);

        lv_obj_t *adc_to_dac_label = lv_label_create(container);
        lv_label_set_text(adc_to_dac_label, "adc -> dac");
        lv_obj_set_style_text_font(adc_to_dac_label, &lv_font_montserrat_26, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_text_color(adc_to_dac_label, lv_color_black(), (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_align_to(adc_to_dac_label, _registry.win.cit.microphone_test.data.label, LV_ALIGN_OUT_BOTTOM_MID, 0, 50);

        _registry.win.cit.microphone_test.adc_to_dac_switch = lv_switch_create(container);
        lv_obj_set_size(_registry.win.cit.microphone_test.adc_to_dac_switch, 90, 50);
        lv_obj_align_to(_registry.win.cit.microphone_test.adc_to_dac_switch, adc_to_dac_label, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);

        if (_registry.win.cit.microphone_test.adc_to_dac_switch_status == true)
        {
            lv_obj_add_state(_registry.win.cit.microphone_test.adc_to_dac_switch, LV_STATE_CHECKED);

            set_adc_to_dac_switch_status(true);
        }
        else
        {
            lv_obj_remove_state(_registry.win.cit.microphone_test.adc_to_dac_switch, LV_STATE_CHECKED);

            set_adc_to_dac_switch_status(false);
        }

        lv_obj_add_event_cb(_registry.win.cit.microphone_test.adc_to_dac_switch, [](lv_event_t *e)
                            {
                                System *self = static_cast<System *>(lv_event_get_user_data(e));
                                self->_registry.win.cit.microphone_test.adc_to_dac_switch_status = lv_obj_has_state(self->_registry.win.cit.microphone_test.adc_to_dac_switch, LV_STATE_CHECKED);

                                if (self->_registry.win.cit.microphone_test.adc_to_dac_switch_status == true)
                                {
                                    self->set_adc_to_dac_switch_status(true);
                                }
                                else
                                {
                                    self->set_adc_to_dac_switch_status(false);
                                } }, LV_EVENT_VALUE_CHANGED, this);

        // 创建一个容器来存放两个按键
        lv_obj_t *button_container = lv_obj_create(_registry.win.cit.microphone_test.root);
        lv_obj_set_size(button_container, _width, 140);
        lv_obj_align(button_container, LV_ALIGN_BOTTOM_MID, 0, 0);
        lv_obj_set_style_bg_color(button_container, lv_color_white(), (lv_style_selector_t)LV_PART_MAIN); // 设置背景颜色为白色
        lv_obj_set_style_radius(button_container, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_border_width(button_container, 0, (lv_style_selector_t)LV_PART_MAIN); // 移除边框

        // 创建PASS按键
        lv_obj_t *pass_button = lv_button_create(button_container);
        lv_obj_set_size(pass_button, 200, 60);
        lv_obj_align(pass_button, LV_ALIGN_LEFT_MID, 10, 0);
        lv_obj_set_style_radius(pass_button, 10, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_width(pass_button, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT); // 移除阴影

        lv_obj_t *pass_label = lv_label_create(pass_button);
        lv_obj_set_style_text_font(pass_label, &lv_font_montserrat_24, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_label_set_text(pass_label, "PASS");
        lv_obj_center(pass_label);

        lv_obj_add_event_cb(pass_button, [](lv_event_t *e)
                            {
                                System *self = static_cast<System *>(lv_event_get_user_data(e));
                                lv_event_code_t code = lv_event_get_code(e);

                                switch (code)
                                {
                                case LV_EVENT_CLICKED:
                                    self->set_microphone_test(false);
                                    self->set_adc_to_dac_switch_status(false);

                                    _win_cit_test_item_list[self->_registry.win.cit.current_test_item_index].symbol = LV_SYMBOL_OK;
                                    _win_cit_test_item_list[self->_registry.win.cit.current_test_item_index].color = 0x008B45;

                                    self->init_win_cit();

                                    lv_screen_load_anim(self->_registry.win.cit.root, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 100, 0, true);
                                    break;
                                default:
                                    break;
                                } }, LV_EVENT_ALL, this);

        // 创建FAIL按键
        lv_obj_t *fail_button = lv_button_create(button_container);
        lv_obj_set_size(fail_button, 200, 60);
        lv_obj_align(fail_button, LV_ALIGN_RIGHT_MID, -10, 0);
        lv_obj_set_style_radius(fail_button, 10, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_width(fail_button, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT); // 移除阴影

        lv_obj_t *fail_label = lv_label_create(fail_button);
        lv_obj_set_style_text_font(fail_label, &lv_font_montserrat_24, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_label_set_text(fail_label, "FAIL");
        lv_obj_center(fail_label);

        lv_obj_add_event_cb(fail_button, [](lv_event_t *e)
                            {
                                System *self = static_cast<System *>(lv_event_get_user_data(e));
                                lv_event_code_t code = lv_event_get_code(e);

                                switch (code)
                                {
                                case LV_EVENT_CLICKED:
                                    self->set_microphone_test(false);
                                    self->set_adc_to_dac_switch_status(false);

                                    _win_cit_test_item_list[self->_registry.win.cit.current_test_item_index].symbol = LV_SYMBOL_CLOSE;
                                    _win_cit_test_item_list[self->_registry.win.cit.current_test_item_index].color = 0xEE2C2C;

                                    self->init_win_cit();

                                    lv_screen_load_anim(self->_registry.win.cit.root, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 100, 0, true);
                                    break;
                                default:
                                    break;
                                } }, LV_EVENT_ALL, this);

        lv_obj_add_event_cb(_registry.win.cit.microphone_test.root, [](lv_event_t *e)
                            {
                                    System *self = static_cast<System *>(lv_event_get_user_data(e));
                                    lv_event_code_t code = lv_event_get_code(e);
    
                                    if (code == LV_EVENT_GESTURE)
                                    {
                                        lv_dir_t gesture_dir = lv_indev_get_gesture_dir(lv_indev_active());
    
                                        // 边缘检测以及左右滑动
                                        if ((gesture_dir == LV_DIR_LEFT || gesture_dir == LV_DIR_RIGHT)&&(self->_edge_touch_flag == true))
                                        {
                                            self->set_microphone_test(false);
                                            self->set_adc_to_dac_switch_status(false);
                                            
                                            self->set_vibration();
                                            self->init_win_cit();
                                            
                                            lv_screen_load_anim(self->_registry.win.cit.root, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 100, 0, true);
    
                                            self->_edge_touch_flag = false;
                                        }
                                    } }, LV_EVENT_ALL, this);

        init_status_bar(_registry.win.cit.microphone_test.root);

        lv_obj_update_layout(_registry.win.cit.microphone_test.root);

        set_microphone_test(true);
    }

    void System::init_win_cit_imu_test(void)
    {
        // 主界面
        _registry.win.cit.imu_test.root = lv_obj_create(NULL);
        lv_obj_set_style_bg_color(_registry.win.cit.imu_test.root, lv_color_hex(0xFF7F58), (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_size(_registry.win.cit.imu_test.root, _width, _height);
        lv_obj_set_scrollbar_mode(_registry.win.cit.imu_test.root, LV_SCROLLBAR_MODE_OFF);

        // 创建标题
        lv_obj_t *title_label = lv_label_create(_registry.win.cit.imu_test.root);
        lv_label_set_text(title_label, "Imu");
        lv_obj_set_style_text_color(title_label, lv_color_white(), (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_text_align(title_label, LV_TEXT_ALIGN_LEFT, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(title_label, &lv_font_montserrat_48, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_size(title_label, _width - 100, 40);
        lv_obj_align(title_label, LV_ALIGN_TOP_LEFT, 20, 10 + 50);

        // 创建容器
        lv_obj_t *container = lv_obj_create(_registry.win.cit.imu_test.root);
        lv_obj_set_size(container, _width, _height - 50 - 80 - 140);
        lv_obj_align(container, LV_ALIGN_TOP_MID, 0, 50 + 80);
        lv_obj_set_style_bg_color(container, lv_color_white(), (lv_style_selector_t)LV_PART_MAIN); // 设置背景颜色为白色
        lv_obj_set_style_radius(container, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_border_width(container, 0, (lv_style_selector_t)LV_PART_MAIN); // 移除边框

        // 创建一个标签用于显示触摸点数据
        _registry.win.cit.imu_test.data_label = lv_label_create(container);
        lv_obj_set_style_text_color(_registry.win.cit.imu_test.data_label, lv_color_black(), (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(_registry.win.cit.imu_test.data_label, &lv_font_montserrat_24, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_label_set_text(_registry.win.cit.imu_test.data_label, "imu data:");
        lv_obj_align(_registry.win.cit.imu_test.data_label, LV_ALIGN_CENTER, 0, 0);

        // 创建一个容器来存放两个按键
        lv_obj_t *button_container = lv_obj_create(_registry.win.cit.imu_test.root);
        lv_obj_set_size(button_container, _width, 140);
        lv_obj_align(button_container, LV_ALIGN_BOTTOM_MID, 0, 0);
        lv_obj_set_style_bg_color(button_container, lv_color_white(), (lv_style_selector_t)LV_PART_MAIN); // 设置背景颜色为白色
        lv_obj_set_style_radius(button_container, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_border_width(button_container, 0, (lv_style_selector_t)LV_PART_MAIN); // 移除边框

        // 创建PASS按键
        lv_obj_t *pass_button = lv_button_create(button_container);
        lv_obj_set_size(pass_button, 200, 60);
        lv_obj_align(pass_button, LV_ALIGN_LEFT_MID, 10, 0);
        lv_obj_set_style_radius(pass_button, 10, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_width(pass_button, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT); // 移除阴影

        lv_obj_t *pass_label = lv_label_create(pass_button);
        lv_obj_set_style_text_font(pass_label, &lv_font_montserrat_24, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_label_set_text(pass_label, "PASS");
        lv_obj_center(pass_label);

        lv_obj_add_event_cb(pass_button, [](lv_event_t *e)
                            {
                                System *self = static_cast<System *>(lv_event_get_user_data(e));
                                lv_event_code_t code = lv_event_get_code(e);

                                switch (code)
                                {
                                case LV_EVENT_CLICKED:
                                    self->set_imu_test(false);

                                    _win_cit_test_item_list[self->_registry.win.cit.current_test_item_index].symbol = LV_SYMBOL_OK;
                                    _win_cit_test_item_list[self->_registry.win.cit.current_test_item_index].color = 0x008B45;

                                    self->init_win_cit();

                                    lv_screen_load_anim(self->_registry.win.cit.root, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 100, 0, true);
                                    break;
                                default:
                                    break;
                                } }, LV_EVENT_ALL, this);

        // 创建FAIL按键
        lv_obj_t *fail_button = lv_button_create(button_container);
        lv_obj_set_size(fail_button, 200, 60);
        lv_obj_align(fail_button, LV_ALIGN_RIGHT_MID, -10, 0);
        lv_obj_set_style_radius(fail_button, 10, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_width(fail_button, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT); // 移除阴影

        lv_obj_t *fail_label = lv_label_create(fail_button);
        lv_obj_set_style_text_font(fail_label, &lv_font_montserrat_24, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_label_set_text(fail_label, "FAIL");
        lv_obj_center(fail_label);

        lv_obj_add_event_cb(fail_button, [](lv_event_t *e)
                            {
                                System *self = static_cast<System *>(lv_event_get_user_data(e));
                                lv_event_code_t code = lv_event_get_code(e);

                                switch (code)
                                {
                                case LV_EVENT_CLICKED:
                                    self->set_imu_test(false);

                                    _win_cit_test_item_list[self->_registry.win.cit.current_test_item_index].symbol = LV_SYMBOL_CLOSE;
                                    _win_cit_test_item_list[self->_registry.win.cit.current_test_item_index].color = 0xEE2C2C;

                                    self->init_win_cit();

                                    lv_screen_load_anim(self->_registry.win.cit.root, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 100, 0, true);
                                    break;
                                default:
                                    break;
                                } }, LV_EVENT_ALL, this);

        lv_obj_add_event_cb(_registry.win.cit.imu_test.root, [](lv_event_t *e)
                            {
                                    System *self = static_cast<System *>(lv_event_get_user_data(e));
                                    lv_event_code_t code = lv_event_get_code(e);
    
                                    if (code == LV_EVENT_GESTURE)
                                    {
                                        lv_dir_t gesture_dir = lv_indev_get_gesture_dir(lv_indev_active());
    
                                        // 边缘检测以及左右滑动
                                        if ((gesture_dir == LV_DIR_LEFT || gesture_dir == LV_DIR_RIGHT)&&(self->_edge_touch_flag == true))
                                        {
                                            self->set_imu_test(false);
                                            
                                            self->set_vibration();
                                            self->init_win_cit();
                                            
                                            lv_screen_load_anim(self->_registry.win.cit.root, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 100, 0, true);
    
                                            self->_edge_touch_flag = false;
                                        }
                                    } }, LV_EVENT_ALL, this);

        init_status_bar(_registry.win.cit.imu_test.root);

        lv_obj_update_layout(_registry.win.cit.imu_test.root);

        set_imu_test(true);
    }

    void System::init_win_cit_battery_health_test(void)
    {
        // 主界面
        _registry.win.cit.battery_health_test.root = lv_obj_create(NULL);
        lv_obj_set_style_bg_color(_registry.win.cit.battery_health_test.root, lv_color_hex(0xFF7F58), (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_size(_registry.win.cit.battery_health_test.root, _width, _height);
        lv_obj_set_scrollbar_mode(_registry.win.cit.battery_health_test.root, LV_SCROLLBAR_MODE_OFF);

        // 创建标题
        lv_obj_t *title_label = lv_label_create(_registry.win.cit.battery_health_test.root);
        lv_label_set_text(title_label, "Battery Health");
        lv_obj_set_style_text_color(title_label, lv_color_white(), (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_text_align(title_label, LV_TEXT_ALIGN_LEFT, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(title_label, &lv_font_montserrat_48, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_size(title_label, _width - 100, 40);
        lv_obj_align(title_label, LV_ALIGN_TOP_LEFT, 20, 10 + 50);

        // 创建容器
        lv_obj_t *container = lv_obj_create(_registry.win.cit.battery_health_test.root);
        lv_obj_set_size(container, _width, _height - 50 - 80 - 140);
        lv_obj_align(container, LV_ALIGN_TOP_MID, 0, 50 + 80);
        lv_obj_set_style_bg_color(container, lv_color_white(), (lv_style_selector_t)LV_PART_MAIN); // 设置背景颜色为白色
        lv_obj_set_style_radius(container, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_border_width(container, 0, (lv_style_selector_t)LV_PART_MAIN); // 移除边框
        lv_obj_set_scrollbar_mode(container, LV_SCROLLBAR_MODE_ACTIVE);

        // 创建一个标签用于显示电池健康数据
        _registry.win.cit.battery_health_test.data_label = lv_label_create(container);
        lv_obj_set_style_text_color(_registry.win.cit.battery_health_test.data_label, lv_color_black(), (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(_registry.win.cit.battery_health_test.data_label, &lv_font_montserrat_24, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_label_set_text(_registry.win.cit.battery_health_test.data_label, "battery health data:");
        lv_obj_align(_registry.win.cit.battery_health_test.data_label, LV_ALIGN_CENTER, 0, 0);

#if defined CONFIG_BOARD_TYPE_T_DISPLAY_P4_KEYBOARD

        _registry.win.cit.battery_health_test.otg_label = lv_label_create(container);
        lv_label_set_text(_registry.win.cit.battery_health_test.otg_label, "OTG");
        lv_obj_set_style_text_font(_registry.win.cit.battery_health_test.otg_label, &lv_font_montserrat_26, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_text_color(_registry.win.cit.battery_health_test.otg_label, lv_color_black(), (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_align_to(_registry.win.cit.battery_health_test.otg_label, _registry.win.cit.battery_health_test.data_label, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);

        _registry.win.cit.battery_health_test.otg_switch = lv_switch_create(container);
        lv_obj_set_size(_registry.win.cit.battery_health_test.otg_switch, 90, 50);
        lv_obj_align_to(_registry.win.cit.battery_health_test.otg_switch, _registry.win.cit.battery_health_test.otg_label, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);

        if (_registry.win.cit.battery_health_test.otg_switch_status == true)
        {
            lv_obj_add_state(_registry.win.cit.battery_health_test.otg_switch, LV_STATE_CHECKED);

            set_otg_switch_status(true);
        }
        else
        {
            lv_obj_remove_state(_registry.win.cit.battery_health_test.otg_switch, LV_STATE_CHECKED);

            set_otg_switch_status(false);
        }

        lv_obj_add_flag(_registry.win.cit.battery_health_test.otg_label, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(_registry.win.cit.battery_health_test.otg_switch, LV_OBJ_FLAG_HIDDEN);

        lv_obj_add_event_cb(_registry.win.cit.battery_health_test.otg_switch, [](lv_event_t *e)
                            {
                                System *self = static_cast<System *>(lv_event_get_user_data(e));
                                self->_registry.win.cit.battery_health_test.otg_switch_status = lv_obj_has_state(self->_registry.win.cit.battery_health_test.otg_switch, LV_STATE_CHECKED);

                                if (self->_registry.win.cit.battery_health_test.otg_switch_status == true)
                                {
                                    self->set_otg_switch_status(true);
                                }
                                else
                                {
                                    self->set_otg_switch_status(false);
                                } }, LV_EVENT_VALUE_CHANGED, this);

#endif

        lv_obj_add_event_cb(_registry.win.cit.battery_health_test.root, [](lv_event_t *e)
                            {
                                    System *self = static_cast<System *>(lv_event_get_user_data(e));
                                    lv_event_code_t code = lv_event_get_code(e);
    
                                    if (code == LV_EVENT_GESTURE)
                                    {
                                        lv_dir_t gesture_dir = lv_indev_get_gesture_dir(lv_indev_active());
    
                                        // 边缘检测以及左右滑动
                                        if ((gesture_dir == LV_DIR_LEFT || gesture_dir == LV_DIR_RIGHT)&&(self->_edge_touch_flag == true))
                                        {
                                            self->set_vibration();
                                            self->init_win_cit();
                                            
                                            lv_screen_load_anim(self->_registry.win.cit.root, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 100, 0, true);
    
                                            self->_edge_touch_flag = false;
                                        }
                                    } }, LV_EVENT_ALL, this);

        add_win_cit_test_item_pass_fail_button(_registry.win.cit.battery_health_test.root);

        init_status_bar(_registry.win.cit.battery_health_test.root);

        lv_obj_update_layout(_registry.win.cit.battery_health_test.root);

        _current_win = Current_Win::CIT_BATTERY_HEALTH_TEST;
    }

    void System::init_win_cit_gps_test(void)
    {
        // 主界面
        _registry.win.cit.gps_test.root = lv_obj_create(NULL);
        lv_obj_set_style_bg_color(_registry.win.cit.gps_test.root, lv_color_hex(0xFF7F58), (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_size(_registry.win.cit.gps_test.root, _width, _height);
        lv_obj_set_scrollbar_mode(_registry.win.cit.gps_test.root, LV_SCROLLBAR_MODE_OFF);

        // 创建标题
        lv_obj_t *title_label = lv_label_create(_registry.win.cit.gps_test.root);
        lv_label_set_text(title_label, "Gps");
        lv_obj_set_style_text_color(title_label, lv_color_white(), (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_text_align(title_label, LV_TEXT_ALIGN_LEFT, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(title_label, &lv_font_montserrat_48, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_size(title_label, _width - 100, 40);
        lv_obj_align(title_label, LV_ALIGN_TOP_LEFT, 20, 10 + 50);

        // 创建容器
        lv_obj_t *container = lv_obj_create(_registry.win.cit.gps_test.root);
        lv_obj_set_size(container, _width, _height - 50 - 80 - 140);
        lv_obj_align(container, LV_ALIGN_TOP_MID, 0, 50 + 80);
        lv_obj_set_style_bg_color(container, lv_color_white(), (lv_style_selector_t)LV_PART_MAIN); // 设置背景颜色为白色
        lv_obj_set_style_radius(container, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_border_width(container, 0, (lv_style_selector_t)LV_PART_MAIN); // 移除边框
        lv_obj_set_scrollbar_mode(container, LV_SCROLLBAR_MODE_ACTIVE);

        // 创建一个标签用于显示电池健康数据
        _registry.win.cit.gps_test.data_label = lv_label_create(container);
        lv_obj_set_style_text_color(_registry.win.cit.gps_test.data_label, lv_color_black(), (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(_registry.win.cit.gps_test.data_label, &lv_font_montserrat_24, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_label_set_text(_registry.win.cit.gps_test.data_label, "gps data:");
        lv_obj_align(_registry.win.cit.gps_test.data_label, LV_ALIGN_CENTER, 0, 0);

        // 创建一个容器来存放两个按键
        lv_obj_t *button_container = lv_obj_create(_registry.win.cit.gps_test.root);
        lv_obj_set_size(button_container, _width, 140);
        lv_obj_align(button_container, LV_ALIGN_BOTTOM_MID, 0, 0);
        lv_obj_set_style_bg_color(button_container, lv_color_white(), (lv_style_selector_t)LV_PART_MAIN); // 设置背景颜色为白色
        lv_obj_set_style_radius(button_container, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_border_width(button_container, 0, (lv_style_selector_t)LV_PART_MAIN); // 移除边框

        // 创建PASS按键
        lv_obj_t *pass_button = lv_button_create(button_container);
        lv_obj_set_size(pass_button, 200, 60);
        lv_obj_align(pass_button, LV_ALIGN_LEFT_MID, 10, 0);
        lv_obj_set_style_radius(pass_button, 10, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_width(pass_button, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT); // 移除阴影

        lv_obj_t *pass_label = lv_label_create(pass_button);
        lv_obj_set_style_text_font(pass_label, &lv_font_montserrat_24, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_label_set_text(pass_label, "PASS");
        lv_obj_center(pass_label);

        lv_obj_add_event_cb(pass_button, [](lv_event_t *e)
                            {
                                System *self = static_cast<System *>(lv_event_get_user_data(e));
                                lv_event_code_t code = lv_event_get_code(e);

                                switch (code)
                                {
                                case LV_EVENT_CLICKED:
                                    self->set_gps_test(false);

                                    _win_cit_test_item_list[self->_registry.win.cit.current_test_item_index].symbol = LV_SYMBOL_OK;
                                    _win_cit_test_item_list[self->_registry.win.cit.current_test_item_index].color = 0x008B45;

                                    self->init_win_cit();

                                    lv_screen_load_anim(self->_registry.win.cit.root, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 100, 0, true);
                                    break;
                                default:
                                    break;
                                } }, LV_EVENT_ALL, this);

        // 创建FAIL按键
        lv_obj_t *fail_button = lv_button_create(button_container);
        lv_obj_set_size(fail_button, 200, 60);
        lv_obj_align(fail_button, LV_ALIGN_RIGHT_MID, -10, 0);
        lv_obj_set_style_radius(fail_button, 10, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_width(fail_button, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT); // 移除阴影

        lv_obj_t *fail_label = lv_label_create(fail_button);
        lv_obj_set_style_text_font(fail_label, &lv_font_montserrat_24, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_label_set_text(fail_label, "FAIL");
        lv_obj_center(fail_label);

        lv_obj_add_event_cb(fail_button, [](lv_event_t *e)
                            {
                                System *self = static_cast<System *>(lv_event_get_user_data(e));
                                lv_event_code_t code = lv_event_get_code(e);

                                switch (code)
                                {
                                case LV_EVENT_CLICKED:
                                    self->set_gps_test(false);

                                    _win_cit_test_item_list[self->_registry.win.cit.current_test_item_index].symbol = LV_SYMBOL_CLOSE;
                                    _win_cit_test_item_list[self->_registry.win.cit.current_test_item_index].color = 0xEE2C2C;

                                    self->init_win_cit();

                                    lv_screen_load_anim(self->_registry.win.cit.root, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 100, 0, true);
                                    break;
                                default:
                                    break;
                                } }, LV_EVENT_ALL, this);

        lv_obj_add_event_cb(_registry.win.cit.gps_test.root, [](lv_event_t *e)
                            {
                                    System *self = static_cast<System *>(lv_event_get_user_data(e));
                                    lv_event_code_t code = lv_event_get_code(e);
    
                                    if (code == LV_EVENT_GESTURE)
                                    {
                                        lv_dir_t gesture_dir = lv_indev_get_gesture_dir(lv_indev_active());
    
                                        // 边缘检测以及左右滑动
                                        if ((gesture_dir == LV_DIR_LEFT || gesture_dir == LV_DIR_RIGHT)&&(self->_edge_touch_flag == true))
                                        {
                                            self->set_gps_test(false);
                                            
                                            self->set_vibration();
                                            self->init_win_cit();
                                            
                                            lv_screen_load_anim(self->_registry.win.cit.root, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 100, 0, true);
    
                                            self->_edge_touch_flag = false;
                                        }
                                    } }, LV_EVENT_ALL, this);

        init_status_bar(_registry.win.cit.gps_test.root);

        lv_obj_update_layout(_registry.win.cit.gps_test.root);

        set_gps_test(true);
    }

    void System::init_win_cit_ethernet_test(void)
    {
        // 主界面
        _registry.win.cit.ethernet_test.root = lv_obj_create(NULL);
        lv_obj_set_style_bg_color(_registry.win.cit.ethernet_test.root, lv_color_hex(0xFF7F58), (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_size(_registry.win.cit.ethernet_test.root, _width, _height);
        lv_obj_set_scrollbar_mode(_registry.win.cit.ethernet_test.root, LV_SCROLLBAR_MODE_OFF);

        // 创建标题
        lv_obj_t *title_label = lv_label_create(_registry.win.cit.ethernet_test.root);
        lv_label_set_text(title_label, "Ethernet");
        lv_obj_set_style_text_color(title_label, lv_color_white(), (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_text_align(title_label, LV_TEXT_ALIGN_LEFT, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(title_label, &lv_font_montserrat_48, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_size(title_label, _width - 100, 40);
        lv_obj_align(title_label, LV_ALIGN_TOP_LEFT, 20, 10 + 50);

        // 创建容器
        lv_obj_t *container = lv_obj_create(_registry.win.cit.ethernet_test.root);
        lv_obj_set_size(container, _width, _height - 50 - 80 - 140);
        lv_obj_align(container, LV_ALIGN_TOP_MID, 0, 50 + 80);
        lv_obj_set_style_bg_color(container, lv_color_white(), (lv_style_selector_t)LV_PART_MAIN); // 设置背景颜色为白色
        lv_obj_set_style_radius(container, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_border_width(container, 0, (lv_style_selector_t)LV_PART_MAIN); // 移除边框
        lv_obj_set_scrollbar_mode(container, LV_SCROLLBAR_MODE_ACTIVE);

        // 创建一个标签用于显示电池健康数据
        _registry.win.cit.ethernet_test.data_label = lv_label_create(container);
        lv_obj_set_style_text_color(_registry.win.cit.ethernet_test.data_label, lv_color_black(), (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(_registry.win.cit.ethernet_test.data_label, &lv_font_montserrat_24, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_label_set_text(_registry.win.cit.ethernet_test.data_label, "ethernet data:");
        lv_obj_align(_registry.win.cit.ethernet_test.data_label, LV_ALIGN_CENTER, 0, 0);

        // 创建一个容器来存放两个按键
        lv_obj_t *button_container = lv_obj_create(_registry.win.cit.ethernet_test.root);
        lv_obj_set_size(button_container, _width, 140);
        lv_obj_align(button_container, LV_ALIGN_BOTTOM_MID, 0, 0);
        lv_obj_set_style_bg_color(button_container, lv_color_white(), (lv_style_selector_t)LV_PART_MAIN); // 设置背景颜色为白色
        lv_obj_set_style_radius(button_container, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_border_width(button_container, 0, (lv_style_selector_t)LV_PART_MAIN); // 移除边框

        // 创建PASS按键
        lv_obj_t *pass_button = lv_button_create(button_container);
        lv_obj_set_size(pass_button, 200, 60);
        lv_obj_align(pass_button, LV_ALIGN_LEFT_MID, 10, 0);
        lv_obj_set_style_radius(pass_button, 10, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_width(pass_button, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT); // 移除阴影

        lv_obj_t *pass_label = lv_label_create(pass_button);
        lv_obj_set_style_text_font(pass_label, &lv_font_montserrat_24, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_label_set_text(pass_label, "PASS");
        lv_obj_center(pass_label);

        lv_obj_add_event_cb(pass_button, [](lv_event_t *e)
                            {
                                System *self = static_cast<System *>(lv_event_get_user_data(e));
                                lv_event_code_t code = lv_event_get_code(e);

                                switch (code)
                                {
                                case LV_EVENT_CLICKED:
                                    self->set_ethernet_test(false);

                                    _win_cit_test_item_list[self->_registry.win.cit.current_test_item_index].symbol = LV_SYMBOL_OK;
                                    _win_cit_test_item_list[self->_registry.win.cit.current_test_item_index].color = 0x008B45;

                                    self->init_win_cit();

                                    lv_screen_load_anim(self->_registry.win.cit.root, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 100, 0, true);
                                    break;
                                default:
                                    break;
                                } }, LV_EVENT_ALL, this);

        // 创建FAIL按键
        lv_obj_t *fail_button = lv_button_create(button_container);
        lv_obj_set_size(fail_button, 200, 60);
        lv_obj_align(fail_button, LV_ALIGN_RIGHT_MID, -10, 0);
        lv_obj_set_style_radius(fail_button, 10, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_width(fail_button, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT); // 移除阴影

        lv_obj_t *fail_label = lv_label_create(fail_button);
        lv_obj_set_style_text_font(fail_label, &lv_font_montserrat_24, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_label_set_text(fail_label, "FAIL");
        lv_obj_center(fail_label);

        lv_obj_add_event_cb(fail_button, [](lv_event_t *e)
                            {
                                System *self = static_cast<System *>(lv_event_get_user_data(e));
                                lv_event_code_t code = lv_event_get_code(e);

                                switch (code)
                                {
                                case LV_EVENT_CLICKED:
                                    self->set_ethernet_test(false);

                                    _win_cit_test_item_list[self->_registry.win.cit.current_test_item_index].symbol = LV_SYMBOL_CLOSE;
                                    _win_cit_test_item_list[self->_registry.win.cit.current_test_item_index].color = 0xEE2C2C;

                                    self->init_win_cit();

                                    lv_screen_load_anim(self->_registry.win.cit.root, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 100, 0, true);
                                    break;
                                default:
                                    break;
                                } }, LV_EVENT_ALL, this);

        lv_obj_add_event_cb(_registry.win.cit.ethernet_test.root, [](lv_event_t *e)
                            {
                                    System *self = static_cast<System *>(lv_event_get_user_data(e));
                                    lv_event_code_t code = lv_event_get_code(e);
    
                                    if (code == LV_EVENT_GESTURE)
                                    {
                                        lv_dir_t gesture_dir = lv_indev_get_gesture_dir(lv_indev_active());
    
                                        // 边缘检测以及左右滑动
                                        if ((gesture_dir == LV_DIR_LEFT || gesture_dir == LV_DIR_RIGHT)&&(self->_edge_touch_flag == true))
                                        {
                                            self->set_ethernet_test(false);
                                            
                                            self->set_vibration();
                                            self->init_win_cit();
                                            
                                            lv_screen_load_anim(self->_registry.win.cit.root, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 100, 0, true);
    
                                            self->_edge_touch_flag = false;
                                        }
                                    } }, LV_EVENT_ALL, this);

        init_status_bar(_registry.win.cit.ethernet_test.root);

        lv_obj_update_layout(_registry.win.cit.ethernet_test.root);

        set_ethernet_test(true);
    }

    void System::init_win_cit_rtc_test(void)
    {
        // 主界面
        _registry.win.cit.rtc_test.root = lv_obj_create(NULL);
        lv_obj_set_style_bg_color(_registry.win.cit.rtc_test.root, lv_color_hex(0xFF7F58), (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_size(_registry.win.cit.rtc_test.root, _width, _height);
        lv_obj_set_scrollbar_mode(_registry.win.cit.rtc_test.root, LV_SCROLLBAR_MODE_OFF);

        // 创建标题
        lv_obj_t *title_label = lv_label_create(_registry.win.cit.rtc_test.root);
        lv_label_set_text(title_label, "Rtc");
        lv_obj_set_style_text_color(title_label, lv_color_white(), (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_text_align(title_label, LV_TEXT_ALIGN_LEFT, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(title_label, &lv_font_montserrat_48, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_size(title_label, _width - 100, 40);
        lv_obj_align(title_label, LV_ALIGN_TOP_LEFT, 20, 10 + 50);

        // 创建容器
        lv_obj_t *container = lv_obj_create(_registry.win.cit.rtc_test.root);
        lv_obj_set_size(container, _width, _height - 50 - 80 - 140);
        lv_obj_align(container, LV_ALIGN_TOP_MID, 0, 50 + 80);
        lv_obj_set_style_bg_color(container, lv_color_white(), (lv_style_selector_t)LV_PART_MAIN); // 设置背景颜色为白色
        lv_obj_set_style_radius(container, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_border_width(container, 0, (lv_style_selector_t)LV_PART_MAIN); // 移除边框
        lv_obj_set_scrollbar_mode(container, LV_SCROLLBAR_MODE_ACTIVE);

        // 创建一个标签用于显示电池健康数据
        _registry.win.cit.rtc_test.data_label = lv_label_create(container);
        lv_obj_set_style_text_color(_registry.win.cit.rtc_test.data_label, lv_color_black(), (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(_registry.win.cit.rtc_test.data_label, &lv_font_montserrat_24, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_label_set_text(_registry.win.cit.rtc_test.data_label, "rtc data:");
        lv_obj_align(_registry.win.cit.rtc_test.data_label, LV_ALIGN_CENTER, 0, 0);

        lv_obj_add_event_cb(_registry.win.cit.rtc_test.root, [](lv_event_t *e)
                            {
                                    System *self = static_cast<System *>(lv_event_get_user_data(e));
                                    lv_event_code_t code = lv_event_get_code(e);
    
                                    if (code == LV_EVENT_GESTURE)
                                    {
                                        lv_dir_t gesture_dir = lv_indev_get_gesture_dir(lv_indev_active());
    
                                        // 边缘检测以及左右滑动
                                        if ((gesture_dir == LV_DIR_LEFT || gesture_dir == LV_DIR_RIGHT)&&(self->_edge_touch_flag == true))
                                        {
                                            
                                            self->set_vibration();
                                            self->init_win_cit();
                                            
                                            lv_screen_load_anim(self->_registry.win.cit.root, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 100, 0, true);
    
                                            self->_edge_touch_flag = false;
                                        }
                                    } }, LV_EVENT_ALL, this);

        add_win_cit_test_item_pass_fail_button(_registry.win.cit.rtc_test.root);

        init_status_bar(_registry.win.cit.rtc_test.root);

        lv_obj_update_layout(_registry.win.cit.rtc_test.root);

        _current_win = Current_Win::CIT_RTC_TEST;
    }

    void System::init_win_cit_esp32c6_at_test(void)
    {
        // 主界面
        _registry.win.cit.esp32c6_at_test.root = lv_obj_create(NULL);
        lv_obj_set_style_bg_color(_registry.win.cit.esp32c6_at_test.root, lv_color_hex(0xFF7F58), (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_size(_registry.win.cit.esp32c6_at_test.root, _width, _height);
        lv_obj_set_scrollbar_mode(_registry.win.cit.esp32c6_at_test.root, LV_SCROLLBAR_MODE_OFF);

        // 创建标题
        lv_obj_t *title_label = lv_label_create(_registry.win.cit.esp32c6_at_test.root);
        lv_label_set_text(title_label, "ADS-B");
        lv_obj_set_style_text_color(title_label, lv_color_white(), (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_text_align(title_label, LV_TEXT_ALIGN_LEFT, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(title_label, &lv_font_montserrat_48, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_size(title_label, _width - 100, 40);
        lv_obj_align(title_label, LV_ALIGN_TOP_LEFT, 20, 10 + 50);

        // 创建容器
        lv_obj_t *container = lv_obj_create(_registry.win.cit.esp32c6_at_test.root);
        lv_obj_set_size(container, _width, _height - 50 - 80 - 140);
        lv_obj_align(container, LV_ALIGN_TOP_MID, 0, 50 + 80);
        lv_obj_set_style_bg_color(container, lv_color_white(), (lv_style_selector_t)LV_PART_MAIN); // 设置背景颜色为白色
        lv_obj_set_style_radius(container, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_border_width(container, 0, (lv_style_selector_t)LV_PART_MAIN); // 移除边框
        lv_obj_set_scrollbar_mode(container, LV_SCROLLBAR_MODE_ACTIVE);

        // 创建一个标签用于显示电池健康数据
        _registry.win.cit.esp32c6_at_test.data_label = lv_label_create(container);
        lv_obj_set_style_text_color(_registry.win.cit.esp32c6_at_test.data_label, lv_color_black(), (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(_registry.win.cit.esp32c6_at_test.data_label, &lv_font_montserrat_24, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_label_set_text(_registry.win.cit.esp32c6_at_test.data_label, "ADS-B receiver status:");
        lv_obj_align(_registry.win.cit.esp32c6_at_test.data_label, LV_ALIGN_CENTER, 0, 0);

        // 创建一个容器来存放两个按键
        lv_obj_t *button_container = lv_obj_create(_registry.win.cit.esp32c6_at_test.root);
        lv_obj_set_size(button_container, _width, 140);
        lv_obj_align(button_container, LV_ALIGN_BOTTOM_MID, 0, 0);
        lv_obj_set_style_bg_color(button_container, lv_color_white(), (lv_style_selector_t)LV_PART_MAIN); // 设置背景颜色为白色
        lv_obj_set_style_radius(button_container, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_border_width(button_container, 0, (lv_style_selector_t)LV_PART_MAIN); // 移除边框

        // 创建PASS按键
        lv_obj_t *pass_button = lv_button_create(button_container);
        lv_obj_set_size(pass_button, 200, 60);
        lv_obj_align(pass_button, LV_ALIGN_LEFT_MID, 10, 0);
        lv_obj_set_style_radius(pass_button, 10, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_width(pass_button, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT); // 移除阴影

        lv_obj_t *pass_label = lv_label_create(pass_button);
        lv_obj_set_style_text_font(pass_label, &lv_font_montserrat_24, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_label_set_text(pass_label, "PASS");
        lv_obj_center(pass_label);

        lv_obj_add_event_cb(pass_button, [](lv_event_t *e)
                            {
                                System *self = static_cast<System *>(lv_event_get_user_data(e));
                                lv_event_code_t code = lv_event_get_code(e);

                                switch (code)
                                {
                                case LV_EVENT_CLICKED:
                                    self->set_esp32c6_at_test(false);

                                    _win_cit_test_item_list[self->_registry.win.cit.current_test_item_index].symbol = LV_SYMBOL_OK;
                                    _win_cit_test_item_list[self->_registry.win.cit.current_test_item_index].color = 0x008B45;

                                    self->init_win_cit();

                                    lv_screen_load_anim(self->_registry.win.cit.root, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 100, 0, true);
                                    break;
                                default:
                                    break;
                                } }, LV_EVENT_ALL, this);

        // 创建FAIL按键
        lv_obj_t *fail_button = lv_button_create(button_container);
        lv_obj_set_size(fail_button, 200, 60);
        lv_obj_align(fail_button, LV_ALIGN_RIGHT_MID, -10, 0);
        lv_obj_set_style_radius(fail_button, 10, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_width(fail_button, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT); // 移除阴影

        lv_obj_t *fail_label = lv_label_create(fail_button);
        lv_obj_set_style_text_font(fail_label, &lv_font_montserrat_24, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_label_set_text(fail_label, "FAIL");
        lv_obj_center(fail_label);

        lv_obj_add_event_cb(fail_button, [](lv_event_t *e)
                            {
                                System *self = static_cast<System *>(lv_event_get_user_data(e));
                                lv_event_code_t code = lv_event_get_code(e);

                                switch (code)
                                {
                                case LV_EVENT_CLICKED:
                                    self->set_esp32c6_at_test(false);

                                    _win_cit_test_item_list[self->_registry.win.cit.current_test_item_index].symbol = LV_SYMBOL_CLOSE;
                                    _win_cit_test_item_list[self->_registry.win.cit.current_test_item_index].color = 0xEE2C2C;

                                    self->init_win_cit();

                                    lv_screen_load_anim(self->_registry.win.cit.root, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 100, 0, true);
                                    break;
                                default:
                                    break;
                                } }, LV_EVENT_ALL, this);

        lv_obj_add_event_cb(_registry.win.cit.esp32c6_at_test.root, [](lv_event_t *e)
                            {
                                    System *self = static_cast<System *>(lv_event_get_user_data(e));
                                    lv_event_code_t code = lv_event_get_code(e);
    
                                    if (code == LV_EVENT_GESTURE)
                                    {
                                        lv_dir_t gesture_dir = lv_indev_get_gesture_dir(lv_indev_active());
    
                                        // 边缘检测以及左右滑动
                                        if ((gesture_dir == LV_DIR_LEFT || gesture_dir == LV_DIR_RIGHT)&&(self->_edge_touch_flag == true))
                                        {
                                            self->set_esp32c6_at_test(false);
                                            
                                            self->set_vibration();
                                            self->init_win_cit();
                                            
                                            lv_screen_load_anim(self->_registry.win.cit.root, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 100, 0, true);
    
                                            self->_edge_touch_flag = false;
                                        }
                                    } }, LV_EVENT_ALL, this);

        init_status_bar(_registry.win.cit.esp32c6_at_test.root);

        lv_obj_update_layout(_registry.win.cit.esp32c6_at_test.root);

        set_esp32c6_at_test(true);
    }

    // void System::init_win_cit_sleep_test(void)
    // {
    //     // 主界面
    //     _registry.win.cit.sleep_test = lv_obj_create(NULL);
    //     lv_obj_set_style_bg_color(_registry.win.cit.sleep_test, lv_color_hex(0xFF7F58), (lv_style_selector_t)LV_PART_MAIN);
    //     lv_obj_set_size(_registry.win.cit.sleep_test, _width, _height);
    //     lv_obj_set_scrollbar_mode(_registry.win.cit.sleep_test, LV_SCROLLBAR_MODE_OFF);

    //     // 创建标题
    //     lv_obj_t *title_label = lv_label_create(_registry.win.cit.sleep_test);
    //     lv_label_set_text(title_label, "Sleep");
    //     lv_obj_set_style_text_color(title_label, lv_color_white(), (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
    //     lv_obj_set_style_text_align(title_label, LV_TEXT_ALIGN_LEFT, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
    //     lv_obj_set_style_text_font(title_label, &lv_font_montserrat_48, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
    //     lv_obj_set_size(title_label, _width - 100, 40);
    //     lv_obj_align(title_label, LV_ALIGN_TOP_LEFT, 20, 10 + 50);

    //     // 创建容器
    //     lv_obj_t *container = lv_obj_create(_registry.win.cit.sleep_test);
    //     lv_obj_set_size(container, _width, _height - 50 - 80 - 140);
    //     lv_obj_align(container, LV_ALIGN_TOP_MID, 0, 50 + 80);
    //     lv_obj_set_style_bg_color(container, lv_color_white(), (lv_style_selector_t)LV_PART_MAIN); // 设置背景颜色为白色
    //     lv_obj_set_style_radius(container, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
    //     lv_obj_set_style_border_width(container, 0, (lv_style_selector_t)LV_PART_MAIN); // 移除边框

    //     // 创建NORMAL_SLEEP按键
    //     lv_obj_t *light_sleep_button = lv_button_create(container);
    //     lv_obj_set_size(light_sleep_button, 250, 80);
    //     lv_obj_align(light_sleep_button, LV_ALIGN_RIGHT_MID, -10, 0);
    //     lv_obj_set_style_radius(light_sleep_button, 10, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
    //     lv_obj_set_style_shadow_width(light_sleep_button, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT); // 移除阴影
    //     lv_obj_set_style_bg_color(light_sleep_button, lv_color_hex(0xFF6A6A), (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
    //     lv_obj_align(light_sleep_button, LV_ALIGN_CENTER, 0, -50);

    //     lv_obj_t *light_sleep_label = lv_label_create(light_sleep_button);
    //     lv_obj_set_style_text_font(light_sleep_label, &lv_font_montserrat_24, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
    //     lv_label_set_text(light_sleep_label, "NORMAL SLEEP");
    //     lv_obj_center(light_sleep_label);

    //     lv_obj_add_event_cb(light_sleep_button, [](lv_event_t *e)
    //                         {
    //                             System *self = static_cast<System *>(lv_event_get_user_data(e));
    //                             lv_event_code_t code = lv_event_get_code(e);

    //                             switch (code)
    //                             {
    //                             case LV_EVENT_CLICKED:

    //                                 break;
    //                             default:
    //                                 break;
    //                             } }, LV_EVENT_ALL, this);

    //     // 创建LIGHT_SLEEP按键
    //     lv_obj_t *deep_sleep_button = lv_button_create(container);
    //     lv_obj_set_size(deep_sleep_button, 250, 80);
    //     lv_obj_align(deep_sleep_button, LV_ALIGN_RIGHT_MID, -10, 0);
    //     lv_obj_set_style_radius(deep_sleep_button, 10, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
    //     lv_obj_set_style_shadow_width(deep_sleep_button, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT); // 移除阴影
    //     lv_obj_set_style_bg_color(deep_sleep_button, lv_color_hex(0xFF6A6A), (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
    //     lv_obj_align(deep_sleep_button, LV_ALIGN_CENTER, 0, 50);

    //     lv_obj_t *deep_sleep_label = lv_label_create(deep_sleep_button);
    //     lv_obj_set_style_text_font(deep_sleep_label, &lv_font_montserrat_24, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
    //     lv_label_set_text(deep_sleep_label, "LIGHT SLEEP");
    //     lv_obj_center(deep_sleep_label);

    //     lv_obj_add_event_cb(deep_sleep_button, [](lv_event_t *e)
    //                         {
    //                             System *self = static_cast<System *>(lv_event_get_user_data(e));
    //                             lv_event_code_t code = lv_event_get_code(e);

    //                             switch (code)
    //                             {
    //                             case LV_EVENT_CLICKED:
    //                             self->start_sleep_test(Sleep_Mode::LIGHT_SLEEP);

    //                                 break;
    //                             default:
    //                                 break;
    //                             } }, LV_EVENT_ALL, this);

    //     add_win_cit_test_item_pass_fail_button(_registry.win.cit.sleep_test);

    //     add_event_cb_win_return_to_cit(_registry.win.cit.sleep_test);

    //     init_status_bar(_registry.win.cit.sleep_test);

    //     lv_obj_update_layout(_registry.win.cit.sleep_test);
    // }

    void System::init_win_camera(void)
    {
        // 主界面
        _registry.win.camera.root = lv_obj_create(NULL);

        lv_obj_set_style_bg_color(_registry.win.camera.root, lv_color_black(), (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_size(_registry.win.camera.root, _width, _height);
        lv_obj_set_scrollbar_mode(_registry.win.camera.root, LV_SCROLLBAR_MODE_OFF);

        // // 创建画布来显示摄像头数据
        // _registry.win.camera.canvas = lv_canvas_create(_registry.win.camera.root);
        // lv_canvas_set_buffer(_registry.win.camera.canvas, _lv_color_win_draw_buf.get(), _width, _height, LVGL_COLOR_FORMAT);
        // lv_canvas_fill_bg(_registry.win.camera.canvas, lv_color_black(), LV_OPA_COVER);
        // lv_obj_center(_registry.win.camera.canvas);

        lv_obj_add_event_cb(_registry.win.camera.root, [](lv_event_t *e)
                            {
                                System *self = static_cast<System *>(lv_event_get_user_data(e));
                                lv_event_code_t code = lv_event_get_code(e);

                                if (code == LV_EVENT_GESTURE)
                                {
                                    lv_dir_t gesture_dir = lv_indev_get_gesture_dir(lv_indev_active());

                                    // 边缘检测以及左右滑动
                                    if ((gesture_dir == LV_DIR_LEFT || gesture_dir == LV_DIR_RIGHT)&&(self->_edge_touch_flag == true))
                                    {
                                        self->set_camera_status(false);
                                        
                                        self->set_vibration();
                                        self->init_win_home();
                                        
                                        lv_screen_load_anim(self->_registry.win.home.root, LV_SCR_LOAD_ANIM_FADE_OUT, 100, 0, true);

                                        self->_edge_touch_flag = false;
                                    }
                                } }, LV_EVENT_ALL, this);

        init_status_bar(_registry.win.camera.root);

        lv_obj_update_layout(_registry.win.camera.root);

        set_camera_status(true);

        _current_win = Current_Win::CAMERA;
    }

    void System::init_win_adsb(void)
    {
        lv_display_t *disp = lv_display_get_default();

        // Get display dimensions (current orientation)
        int32_t w = lv_display_get_horizontal_resolution(disp);
        int32_t h = lv_display_get_vertical_resolution(disp);
        bool is_landscape = (w > h);

        // Sync sort state with backend
        adsb_set_sort(_registry.win.adsb.sort_col,
                      _registry.win.adsb.sort_asc);

        // Root screen
        _registry.win.adsb.root = lv_obj_create(NULL);
        lv_obj_set_style_bg_color(_registry.win.adsb.root, lv_color_hex(0x0A1520), (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_size(_registry.win.adsb.root, w, h);
        lv_obj_set_scrollbar_mode(_registry.win.adsb.root, LV_SCROLLBAR_MODE_OFF);

        // Title bar
        int32_t status_h = is_landscape ? 0 : 50;  // hide status bar in landscape for max space
        int32_t title_h = is_landscape ? 54 : 80;
        lv_obj_t *title_bar = lv_obj_create(_registry.win.adsb.root);
        lv_obj_set_size(title_bar, w, title_h);
        lv_obj_align(title_bar, LV_ALIGN_TOP_MID, 0, status_h);
        lv_obj_set_style_bg_color(title_bar, lv_color_hex(0x1A3A5C), (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_bg_opa(title_bar, LV_OPA_COVER, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_border_width(title_bar, 0, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_radius(title_bar, 0, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_remove_flag(title_bar, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t *title_label = lv_label_create(title_bar);
        lv_label_set_text(title_label, is_landscape ? LV_SYMBOL_GPS " ADS-B" : LV_SYMBOL_GPS " ADS-B Receiver");
        lv_obj_set_style_text_color(title_label, lv_color_hex(0x00DD00), (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_text_font(title_label, is_landscape ? &lv_font_montserrat_22 : &lv_font_montserrat_28, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_align(title_label, LV_ALIGN_LEFT_MID, 10, 0);

        // Close / back button (rightmost)
        lv_obj_t *close_btn = lv_button_create(title_bar);
        lv_obj_set_size(close_btn, 50, 50);
        lv_obj_align(close_btn, LV_ALIGN_RIGHT_MID, -5, 0);
        lv_obj_set_style_bg_color(close_btn, lv_color_hex(0x404040), (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_shadow_width(close_btn, 0, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_border_width(close_btn, 0, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_radius(close_btn, 6, (lv_style_selector_t)LV_PART_MAIN);

        lv_obj_t *close_lbl = lv_label_create(close_btn);
        lv_label_set_text(close_lbl, LV_SYMBOL_CLOSE);
        lv_obj_set_style_text_color(close_lbl, lv_color_white(), (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_center(close_lbl);

        lv_obj_add_event_cb(close_btn, [](lv_event_t *e)
                            {
                                System *self = static_cast<System *>(lv_event_get_user_data(e));
                                if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;

                                if (self->_win_adsb_status_callback)
                                    self->_win_adsb_status_callback(false);

                                // Restore home rotation
                                lv_display_set_rotation(lv_display_get_default(),
                                    self->_home_rotation);
                                self->_registry.win.adsb.rotated = false;

                                self->set_vibration();
                                self->init_win_home();
                                lv_screen_load_anim(self->_registry.win.home.root, LV_SCR_LOAD_ANIM_FADE_OUT, 100, 0, true);
                            }, LV_EVENT_ALL, this);

        // Rotate toggle button (left of close button)
        lv_obj_t *toggle_btn = lv_button_create(title_bar);
        lv_obj_set_size(toggle_btn, is_landscape ? 100 : 110, 50);
        lv_obj_align_to(toggle_btn, close_btn, LV_ALIGN_OUT_LEFT_MID, -5, 0);
        lv_obj_set_style_bg_color(toggle_btn, lv_color_hex(0x2A5A8C), (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_shadow_width(toggle_btn, 0, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_border_width(toggle_btn, 0, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_radius(toggle_btn, 6, (lv_style_selector_t)LV_PART_MAIN);

        lv_obj_t *toggle_lbl = lv_label_create(toggle_btn);
        lv_label_set_text(toggle_lbl, LV_SYMBOL_LOOP " 90°");
        lv_obj_set_style_text_color(toggle_lbl, lv_color_white(), (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_center(toggle_lbl);

        lv_obj_add_event_cb(toggle_btn, [](lv_event_t *e)
                            {
                                System *self = static_cast<System *>(lv_event_get_user_data(e));
                                if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;

                                lv_display_t *d = lv_display_get_default();
                                // Mark as rotated (home rotation restored via _home_rotation on exit)
                                self->_registry.win.adsb.rotated = true;
                                // Cycle to next rotation (0 → 90 → 180 → 270 → 0 ...)
                                lv_display_rotation_t cur = lv_display_get_rotation(d);
                                lv_display_rotation_t next = (lv_display_rotation_t)((cur + 1) % 4);
                                lv_display_set_rotation(d, next);
                                // Remember for next visit
                                self->_registry.win.adsb.user_rotation = next;
                                self->_registry.win.adsb.has_user_rotation = true;
                                self->_registry.win.adsb.divider_y = 0;  // reset divider for new aspect
                                // Rebuild with new dimensions
                                self->init_win_adsb();
                                lv_screen_load(self->_registry.win.adsb.root);
                            }, LV_EVENT_ALL, this);

        int32_t content_top = status_h + title_h + 4;
        int32_t content_h = h - content_top - 4;

        // Default divider position — just below the 5 stats lines.
        // Only set on first entry; user's drag position persists across visits.
        if (_registry.win.adsb.divider_y == 0) {
            _registry.win.adsb.divider_y = content_top + (is_landscape ? 70 : 155);
        }

        // Clamp divider to sensible range
        int32_t div_min = content_top + 50;
        int32_t div_max = h - 120;
        if (_registry.win.adsb.divider_y < div_min) _registry.win.adsb.divider_y = div_min;
        if (_registry.win.adsb.divider_y > div_max) _registry.win.adsb.divider_y = div_max;

        int32_t div_y = _registry.win.adsb.divider_y;

        // Stats panel — above divider
        _registry.win.adsb.stats_panel = lv_obj_create(_registry.win.adsb.root);
        lv_obj_set_size(_registry.win.adsb.stats_panel, w - 20, div_y - content_top);
        lv_obj_align(_registry.win.adsb.stats_panel, LV_ALIGN_TOP_MID, 0, content_top);
        lv_obj_set_style_bg_color(_registry.win.adsb.stats_panel, lv_color_hex(0x0F1F2E), (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_bg_opa(_registry.win.adsb.stats_panel, LV_OPA_COVER, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_border_color(_registry.win.adsb.stats_panel, lv_color_hex(0x1A3A5C), (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_border_width(_registry.win.adsb.stats_panel, 1, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_radius(_registry.win.adsb.stats_panel, 8, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_pad_all(_registry.win.adsb.stats_panel, 8, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_remove_flag(_registry.win.adsb.stats_panel, LV_OBJ_FLAG_SCROLLABLE);

        _registry.win.adsb.stats_label = lv_label_create(_registry.win.adsb.stats_panel);
        lv_obj_set_style_text_color(_registry.win.adsb.stats_label, lv_color_hex(0xCCDDEE), (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_text_font(_registry.win.adsb.stats_label, &lv_font_montserrat_22, (lv_style_selector_t)LV_PART_MAIN);
        lv_label_set_text(_registry.win.adsb.stats_label, "Waiting for data...");
        lv_obj_set_width(_registry.win.adsb.stats_label, w - 40);

        // Draggable divider bar
        _registry.win.adsb.divider = lv_obj_create(_registry.win.adsb.root);
        lv_obj_set_size(_registry.win.adsb.divider, w - 40, 18);
        lv_obj_align(_registry.win.adsb.divider, LV_ALIGN_TOP_MID, 0, div_y);
        lv_obj_set_style_bg_color(_registry.win.adsb.divider, lv_color_hex(0x3A6A9C), (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_bg_opa(_registry.win.adsb.divider, LV_OPA_COVER, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_radius(_registry.win.adsb.divider, 6, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_border_width(_registry.win.adsb.divider, 0, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_add_flag(_registry.win.adsb.divider, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_remove_flag(_registry.win.adsb.divider, LV_OBJ_FLAG_SCROLLABLE);

        // Drag handle visual (three small dots)
        lv_obj_t *handle_lbl = lv_label_create(_registry.win.adsb.divider);
        lv_label_set_text(handle_lbl, "= = =");
        lv_obj_set_style_text_color(handle_lbl, lv_color_hex(0x88AACC), (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_center(handle_lbl);

        lv_obj_add_event_cb(_registry.win.adsb.divider, [](lv_event_t *e)
                            {
                                System *self = static_cast<System *>(lv_event_get_user_data(e));
                                lv_event_code_t code = lv_event_get_code(e);
                                if (code != LV_EVENT_PRESSING) return;

                                lv_indev_t *indev = lv_indev_active();
                                lv_point_t point;
                                lv_indev_get_point(indev, &point);

                                lv_display_t *d = lv_display_get_default();
                                int32_t h_now = lv_display_get_vertical_resolution(d);
                                int32_t w_now = lv_display_get_horizontal_resolution(d);
                                bool landscape = (w_now > h_now);
                                int32_t status_h_now = landscape ? 0 : 50;
                                int32_t title_h_now = landscape ? 40 : 80;
                                int32_t ct = status_h_now + title_h_now + 4;
                                int32_t d_min = ct + 50;
                                int32_t d_max = h_now - 120;

                                int32_t new_y = point.y;
                                if (new_y < d_min) new_y = d_min;
                                if (new_y > d_max) new_y = d_max;
                                self->_registry.win.adsb.divider_y = new_y;

                                // Move divider
                                lv_obj_align(self->_registry.win.adsb.divider, LV_ALIGN_TOP_MID, 0, new_y);

                                // Resize stats panel
                                lv_obj_set_height(self->_registry.win.adsb.stats_panel, new_y - ct);

                                // Move and resize list header + list panel
                                int32_t hdr_y = new_y + 14;
                                lv_obj_align(self->_registry.win.adsb.list_header, LV_ALIGN_TOP_MID, 0, hdr_y);
                                lv_obj_set_pos(self->_registry.win.adsb.list_panel, 10, hdr_y + 34);
                                lv_obj_set_height(self->_registry.win.adsb.list_panel, h_now - hdr_y - 38);
                            }, LV_EVENT_ALL, this);

        // Column header — below divider
        int32_t hdr_y = div_y + 20;
        _registry.win.adsb.list_header = lv_obj_create(_registry.win.adsb.root);
        lv_obj_set_size(_registry.win.adsb.list_header, w - 20, 32);
        lv_obj_align(_registry.win.adsb.list_header, LV_ALIGN_TOP_MID, 0, hdr_y);
        lv_obj_set_style_bg_color(_registry.win.adsb.list_header, lv_color_hex(0x1A3A5C), (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_bg_opa(_registry.win.adsb.list_header, LV_OPA_COVER, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_border_width(_registry.win.adsb.list_header, 0, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_radius(_registry.win.adsb.list_header, 6, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_pad_all(_registry.win.adsb.list_header, 4, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_remove_flag(_registry.win.adsb.list_header, LV_OBJ_FLAG_SCROLLABLE);

        // Sortable column headers — each is a clickable label
        // Sort indicator: ▲ ascending, ▼ descending appended to active column
        struct { const char *name; int x; int sort_col; } cols[] = {
            {"ICAO",  0,   1},  // ADSB_SORT_ICAO
            {"CALL",  78,  2},  // ADSB_SORT_CALL
            {"ALT",   185, 3},  // ADSB_SORT_ALT
            {"SPD",   260, 4},  // ADSB_SORT_SPD
            {"HDG",   320, 5},  // ADSB_SORT_HDG
            {"DIST",  385, 0},  // ADSB_SORT_DIST
        };
        for (int c = 0; c < 6; c++) {
            lv_obj_t *col_btn = lv_label_create(_registry.win.adsb.list_header);
            char hdr_txt[16];
            if (_registry.win.adsb.sort_col == cols[c].sort_col) {
                snprintf(hdr_txt, sizeof(hdr_txt), "%s%s", cols[c].name,
                         _registry.win.adsb.sort_asc ? LV_SYMBOL_UP : LV_SYMBOL_DOWN);
            } else {
                snprintf(hdr_txt, sizeof(hdr_txt), "%s", cols[c].name);
            }
            lv_label_set_text(col_btn, hdr_txt);
            lv_obj_set_style_text_font(col_btn, &lv_font_montserrat_22, (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_set_style_text_color(col_btn,
                (_registry.win.adsb.sort_col == cols[c].sort_col)
                    ? lv_color_hex(0xFFFFFF)
                    : lv_color_hex(0x88AACC),
                (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_align(col_btn, LV_ALIGN_LEFT_MID, cols[c].x, 0);
            lv_obj_add_flag(col_btn, LV_OBJ_FLAG_CLICKABLE);

            // Store sort_col in user_data for the click handler
            lv_obj_set_user_data(col_btn, (void *)(intptr_t)cols[c].sort_col);
            lv_obj_add_event_cb(col_btn, [](lv_event_t *e) {
                if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
                System *self = static_cast<System *>(lv_event_get_user_data(e));
                lv_obj_t *target = lv_event_get_target(e);
                int col_id = (int)(intptr_t)lv_obj_get_user_data(target);

                if (self->_registry.win.adsb.sort_col == col_id) {
                    // Same column — toggle direction
                    self->_registry.win.adsb.sort_asc = !self->_registry.win.adsb.sort_asc;
                } else {
                    // New column — default ascending (except DIST defaults descending... no, ascending = nearest first)
                    self->_registry.win.adsb.sort_col = col_id;
                    self->_registry.win.adsb.sort_asc = true;
                }
                adsb_set_sort(self->_registry.win.adsb.sort_col,
                              self->_registry.win.adsb.sort_asc);
                // Rebuild to update header visuals
                self->init_win_adsb();
                lv_screen_load(self->_registry.win.adsb.root);
            }, LV_EVENT_ALL, this);
        }

        // Aircraft list — below header, fills remaining space
        int32_t list_top = hdr_y + 34;
        _registry.win.adsb.list_panel = lv_obj_create(_registry.win.adsb.root);
        lv_obj_set_size(_registry.win.adsb.list_panel, w - 20, h - list_top - 4);
        lv_obj_set_pos(_registry.win.adsb.list_panel, 10, list_top);
        lv_obj_set_style_bg_color(_registry.win.adsb.list_panel, lv_color_hex(0x0A1520), (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_bg_opa(_registry.win.adsb.list_panel, LV_OPA_COVER, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_border_color(_registry.win.adsb.list_panel, lv_color_hex(0x1A3A5C), (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_border_width(_registry.win.adsb.list_panel, 1, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_radius(_registry.win.adsb.list_panel, 8, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_pad_all(_registry.win.adsb.list_panel, 6, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_scrollbar_mode(_registry.win.adsb.list_panel, LV_SCROLLBAR_MODE_ACTIVE);

        _registry.win.adsb.list_label = lv_label_create(_registry.win.adsb.list_panel);
        lv_obj_set_style_text_color(_registry.win.adsb.list_label, lv_color_hex(0x00DD00), (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_text_font(_registry.win.adsb.list_label, &lv_font_montserrat_22, (lv_style_selector_t)LV_PART_MAIN);
        lv_label_set_text(_registry.win.adsb.list_label, "No aircraft");
        lv_obj_set_width(_registry.win.adsb.list_label, w - 36);

        // Swipe to return home (restore rotation)
        lv_obj_add_event_cb(_registry.win.adsb.root, [](lv_event_t *e)
                            {
                                System *self = static_cast<System *>(lv_event_get_user_data(e));
                                lv_event_code_t code = lv_event_get_code(e);

                                if (code == LV_EVENT_GESTURE)
                                {
                                    lv_dir_t gesture_dir = lv_indev_get_gesture_dir(lv_indev_active());

                                    if ((gesture_dir == LV_DIR_LEFT || gesture_dir == LV_DIR_RIGHT) && (self->_edge_touch_flag == true))
                                    {
                                        if (self->_win_adsb_status_callback)
                                            self->_win_adsb_status_callback(false);

                                        // Always restore home rotation on exit.
                                        lv_display_set_rotation(lv_display_get_default(),
                                            self->_home_rotation);
                                        self->_registry.win.adsb.rotated = false;

                                        self->set_vibration();
                                        self->init_win_home();

                                        lv_screen_load_anim(self->_registry.win.home.root, LV_SCR_LOAD_ANIM_FADE_OUT, 100, 0, true);

                                        self->_edge_touch_flag = false;
                                    }
                                } }, LV_EVENT_ALL, this);

        if (!is_landscape)
            init_status_bar(_registry.win.adsb.root);

        lv_obj_update_layout(_registry.win.adsb.root);

        if (_win_adsb_status_callback)
            _win_adsb_status_callback(true);

        _current_win = Current_Win::ADSB;
    }

    void System::win_adsb_update(const char *stats_text, const char *list_text)
    {
        if (_registry.win.adsb.stats_label)
            lv_label_set_text(_registry.win.adsb.stats_label, stats_text);
        if (_registry.win.adsb.list_label)
            lv_label_set_text(_registry.win.adsb.list_label, list_text);
    }

    // ================================================================
    // Meshy — Meshtastic mesh messaging app
    // ================================================================

    void System::init_win_meshy(void)
    {
        if (_win_meshy_status_callback)
            _win_meshy_status_callback(true);

        lv_display_t *disp = lv_display_get_default();
        int32_t w = lv_display_get_horizontal_resolution(disp);
        int32_t h = lv_display_get_vertical_resolution(disp);

        // Root screen — dark green tint
        _registry.win.meshy.root = lv_obj_create(NULL);
        lv_obj_set_style_bg_color(_registry.win.meshy.root, lv_color_hex(0x0A1A10), (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_size(_registry.win.meshy.root, w, h);
        lv_obj_set_scrollbar_mode(_registry.win.meshy.root, LV_SCROLLBAR_MODE_OFF);

        // Status bar
        int32_t status_h = 50;
        init_status_bar(_registry.win.meshy.root);

        // Title bar
        int32_t title_h = 80;
        lv_obj_t *title_bar = lv_obj_create(_registry.win.meshy.root);
        lv_obj_set_size(title_bar, w, title_h);
        lv_obj_set_pos(title_bar, 0, status_h);
        lv_obj_set_style_bg_color(title_bar, lv_color_hex(0x2D8C3A), (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_bg_opa(title_bar, LV_OPA_COVER, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_border_width(title_bar, 0, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_radius(title_bar, 0, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_pad_all(title_bar, 8, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_remove_flag(title_bar, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t *title_icon = lv_label_create(title_bar);
        lv_label_set_text(title_icon, LV_SYMBOL_WIFI);
        lv_obj_set_style_text_font(title_icon, &lv_font_montserrat_28, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_text_color(title_icon, lv_color_white(), (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_align(title_icon, LV_ALIGN_LEFT_MID, 0, 0);

        lv_obj_t *title_label = lv_label_create(title_bar);
        lv_label_set_text(title_label, " Meshy");
        lv_obj_set_style_text_font(title_label, &lv_font_montserrat_28, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_text_color(title_label, lv_color_white(), (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_align_to(title_label, title_icon, LV_ALIGN_OUT_RIGHT_MID, 4, 0);

        // Close button
        lv_obj_t *close_btn = lv_button_create(title_bar);
        lv_obj_set_size(close_btn, 50, 50);
        lv_obj_align(close_btn, LV_ALIGN_RIGHT_MID, -5, 0);
        lv_obj_set_style_bg_color(close_btn, lv_color_hex(0x1A6628), (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_shadow_width(close_btn, 0, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_border_width(close_btn, 0, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_radius(close_btn, 6, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_t *close_lbl = lv_label_create(close_btn);
        lv_label_set_text(close_lbl, LV_SYMBOL_CLOSE);
        lv_obj_set_style_text_color(close_lbl, lv_color_white(), (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_center(close_lbl);
        lv_obj_add_event_cb(close_btn, [](lv_event_t *e) {
            System *self = static_cast<System *>(lv_event_get_user_data(e));
            if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
            if (self->_win_meshy_status_callback)
                self->_win_meshy_status_callback(false);
            lv_display_set_rotation(lv_display_get_default(), self->_home_rotation);
            self->set_vibration();
            self->init_win_home();
            lv_screen_load_anim(self->_registry.win.home.root, LV_SCR_LOAD_ANIM_FADE_OUT, 100, 0, true);
        }, LV_EVENT_ALL, this);

        // Content area
        int32_t content_top = status_h + title_h + 4;

        // Stats panel — connection info, node count, freq
        lv_obj_t *stats_panel = lv_obj_create(_registry.win.meshy.root);
        lv_obj_set_size(stats_panel, w - 20, 110);
        lv_obj_set_pos(stats_panel, 10, content_top);
        lv_obj_set_style_bg_color(stats_panel, lv_color_hex(0x143020), (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_bg_opa(stats_panel, LV_OPA_COVER, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_border_color(stats_panel, lv_color_hex(0x2D8C3A), (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_border_width(stats_panel, 1, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_radius(stats_panel, 8, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_pad_all(stats_panel, 8, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_remove_flag(stats_panel, LV_OBJ_FLAG_SCROLLABLE);

        _registry.win.meshy.stats_label = lv_label_create(stats_panel);
        lv_obj_set_style_text_color(_registry.win.meshy.stats_label, lv_color_hex(0x00DD00), (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_text_font(_registry.win.meshy.stats_label, &lv_font_montserrat_22, (lv_style_selector_t)LV_PART_MAIN);
        lv_label_set_text(_registry.win.meshy.stats_label, "Initializing...");
        lv_obj_set_width(_registry.win.meshy.stats_label, w - 40);
        lv_obj_align(_registry.win.meshy.stats_label, LV_ALIGN_TOP_LEFT, 0, 0);

        // Message list — fills remaining space minus TX bar
        int32_t tx_bar_h = 56;
        int32_t msg_top = content_top + 116;
        lv_obj_t *msg_panel = lv_obj_create(_registry.win.meshy.root);
        lv_obj_set_size(msg_panel, w - 20, h - msg_top - tx_bar_h - 8);
        lv_obj_set_pos(msg_panel, 10, msg_top);
        lv_obj_set_style_bg_color(msg_panel, lv_color_hex(0x0A1A10), (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_bg_opa(msg_panel, LV_OPA_COVER, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_border_color(msg_panel, lv_color_hex(0x2D8C3A), (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_border_width(msg_panel, 1, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_radius(msg_panel, 8, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_pad_all(msg_panel, 8, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_scrollbar_mode(msg_panel, LV_SCROLLBAR_MODE_ACTIVE);
        lv_obj_add_flag(msg_panel, LV_OBJ_FLAG_SCROLL_MOMENTUM);
        lv_obj_add_flag(msg_panel, LV_OBJ_FLAG_SCROLL_ELASTIC);

        // TX input bar
        lv_obj_t *tx_bar = lv_obj_create(_registry.win.meshy.root);
        lv_obj_set_size(tx_bar, w - 20, tx_bar_h);
        lv_obj_set_pos(tx_bar, 10, h - tx_bar_h - 4);
        lv_obj_set_style_bg_color(tx_bar, lv_color_hex(0x0A1A10), (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_bg_opa(tx_bar, LV_OPA_COVER, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_border_color(tx_bar, lv_color_hex(0x2D8C3A), (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_border_width(tx_bar, 1, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_radius(tx_bar, 8, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_pad_all(tx_bar, 6, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_remove_flag(tx_bar, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_flex_flow(tx_bar, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(tx_bar, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        lv_obj_set_style_pad_column(tx_bar, 6, (lv_style_selector_t)LV_PART_MAIN);

        // Textarea
        _registry.win.meshy.tx_textarea = lv_textarea_create(tx_bar);
        lv_obj_set_flex_grow(_registry.win.meshy.tx_textarea, 1);
        lv_obj_set_height(_registry.win.meshy.tx_textarea, 40);
        lv_textarea_set_one_line(_registry.win.meshy.tx_textarea, true);
        lv_textarea_set_max_length(_registry.win.meshy.tx_textarea, 230);
        lv_textarea_set_placeholder_text(_registry.win.meshy.tx_textarea, "Send message...");
        lv_obj_set_style_bg_color(_registry.win.meshy.tx_textarea, lv_color_hex(0x143020), (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_border_color(_registry.win.meshy.tx_textarea, lv_color_hex(0x2D8C3A), (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_text_color(_registry.win.meshy.tx_textarea, lv_color_hex(0x00DD00), (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_text_font(_registry.win.meshy.tx_textarea, &lv_font_montserrat_16, (lv_style_selector_t)LV_PART_MAIN);

        // Send button
        lv_obj_t *tx_send = lv_button_create(tx_bar);
        lv_obj_set_size(tx_send, 70, 40);
        lv_obj_set_style_bg_color(tx_send, lv_color_hex(0x2D8C3A), (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_shadow_width(tx_send, 0, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_radius(tx_send, 6, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_t *tx_lbl = lv_label_create(tx_send);
        lv_label_set_text(tx_lbl, "Send");
        lv_obj_set_style_text_color(tx_lbl, lv_color_white(), (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_text_font(tx_lbl, &lv_font_montserrat_16, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_center(tx_lbl);

        // On-screen keyboard — overlays message area when textarea focused
        lv_obj_t *kb = lv_keyboard_create(_registry.win.meshy.root);
        lv_obj_set_size(kb, w, h * 2 / 5);
        lv_obj_align(kb, LV_ALIGN_BOTTOM_MID, 0, 0);
        lv_keyboard_set_textarea(kb, _registry.win.meshy.tx_textarea);
        lv_obj_add_flag(kb, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_style_bg_color(kb, lv_color_hex(0x0A1A10), (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_border_color(kb, lv_color_hex(0x2D8C3A), (lv_style_selector_t)LV_PART_MAIN);

        // Show keyboard on textarea focus, hide on defocus
        lv_obj_add_event_cb(_registry.win.meshy.tx_textarea, [](lv_event_t *e) {
            lv_obj_t *kb = (lv_obj_t *)lv_event_get_user_data(e);
            if (lv_event_get_code(e) == LV_EVENT_FOCUSED) {
                lv_keyboard_set_textarea(kb, lv_event_get_target_obj(e));
                lv_obj_remove_flag(kb, LV_OBJ_FLAG_HIDDEN);
            } else if (lv_event_get_code(e) == LV_EVENT_DEFOCUSED ||
                       lv_event_get_code(e) == LV_EVENT_READY) {
                lv_obj_add_flag(kb, LV_OBJ_FLAG_HIDDEN);
            }
        }, LV_EVENT_ALL, kb);

        // Send button callback
        lv_obj_add_event_cb(tx_send, [](lv_event_t *e) {
            System *self = static_cast<System *>(lv_event_get_user_data(e));
            if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
            lv_obj_t *ta = self->_registry.win.meshy.tx_textarea;
            const char *text = lv_textarea_get_text(ta);
            if (!text || text[0] == '\0') return;
            if (meshy_send_text(text)) {
                lv_textarea_set_text(ta, "");
            }
        }, LV_EVENT_ALL, this);

        _registry.win.meshy.msg_label = nullptr;
        _registry.win.meshy.msg_canvas = nullptr;

        // Message display — 1800px scrollable canvas for emoji-capable rendering.
        // Messages render top-down from line 0. Panel scrolls to show newest.
        int32_t msg_canvas_w = w - 40;   // panel width minus padding
        int32_t msg_canvas_h = 1800;      // ~64 lines at 28px
        _registry.win.meshy.msg_canvas_w = msg_canvas_w;
        _registry.win.meshy.msg_canvas_max_h = msg_canvas_h;
        _registry.win.meshy.msg_canvas_stride = 0;

        lv_obj_t *mc = lv_canvas_create(msg_panel);
        lv_draw_buf_t *db = lv_draw_buf_create(msg_canvas_w, msg_canvas_h,
                                                 LV_COLOR_FORMAT_RGB565, LV_STRIDE_AUTO);
        if (mc && db) {
            lv_canvas_set_draw_buf(mc, db);
            lv_canvas_fill_bg(mc, lv_color_hex(0x0A1A10), LV_OPA_COVER);
            _registry.win.meshy.msg_canvas = mc;
            _registry.win.meshy.msg_canvas_buf = db->data;
            _registry.win.meshy.msg_canvas_stride = db->header.stride;
            ESP_LOGI("MESHY", "Canvas %ldx%ld via lv_draw_buf_create, data=%p stride=%u",
                     (long)msg_canvas_w, (long)msg_canvas_h, db->data, (unsigned)db->header.stride);
        } else {
            ESP_LOGW("MESHY", "Canvas alloc failed — falling back to label");
            if (mc) lv_obj_delete(mc);
            if (db) lv_draw_buf_destroy(db);
            _registry.win.meshy.msg_canvas = nullptr;
            _registry.win.meshy.msg_canvas_buf = nullptr;
            _registry.win.meshy.msg_label = lv_label_create(msg_panel);
            lv_obj_set_style_text_color(_registry.win.meshy.msg_label, lv_color_hex(0x88DDAA), (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_set_style_text_font(_registry.win.meshy.msg_label, &lv_font_montserrat_22, (lv_style_selector_t)LV_PART_MAIN);
            lv_label_set_text(_registry.win.meshy.msg_label, "Listening...");
            lv_obj_set_width(_registry.win.meshy.msg_label, w - 40);
        }

        // Enable vertical scrolling on the message panel
        lv_obj_set_scrollbar_mode(msg_panel, LV_SCROLLBAR_MODE_ACTIVE);
        lv_obj_add_flag(msg_panel, LV_OBJ_FLAG_SCROLLABLE);

        // Swipe to return home
        lv_obj_add_event_cb(_registry.win.meshy.root, [](lv_event_t *e) {
            System *self = static_cast<System *>(lv_event_get_user_data(e));
            if (lv_event_get_code(e) != LV_EVENT_GESTURE) return;
            lv_dir_t dir = lv_indev_get_gesture_dir(lv_indev_active());
            if ((dir == LV_DIR_LEFT || dir == LV_DIR_RIGHT) && self->_edge_touch_flag) {
                if (self->_win_meshy_status_callback)
                    self->_win_meshy_status_callback(false);
                lv_display_set_rotation(lv_display_get_default(), self->_home_rotation);
                self->set_vibration();
                self->init_win_home();
                lv_screen_load_anim(self->_registry.win.home.root, LV_SCR_LOAD_ANIM_FADE_OUT, 100, 0, true);
                self->_edge_touch_flag = false;
            }
        }, LV_EVENT_ALL, this);

        lv_obj_update_layout(_registry.win.meshy.root);
        _current_win = Current_Win::MESHY;
    }

    void System::win_meshy_update(const char *stats_text, const char *msg_text)
    {
        if (_registry.win.meshy.stats_label)
            lv_label_set_text(_registry.win.meshy.stats_label, stats_text);

        // Canvas path — 1800px scrollable, newest messages, auto-scroll
        if (_registry.win.meshy.msg_canvas && _registry.win.meshy.msg_canvas_buf) {
            lv_obj_t *canvas = _registry.win.meshy.msg_canvas;
            lv_obj_t *panel = lv_obj_get_parent(canvas);
            int32_t cw = _registry.win.meshy.msg_canvas_w;
            int32_t buf_h = _registry.win.meshy.msg_canvas_max_h;  // 1800
            int32_t line_h = 28;
            lv_color_t text_color = lv_color_hex(0x88DDAA);
            int32_t max_lines = (buf_h - 10) / line_h;

            // Parse lines from message text
            static const char *line_starts[200];
            static int line_lens[200];
            int total_lines = 0;
            const char *p = msg_text;
            while (*p && total_lines < 200) {
                line_starts[total_lines] = p;
                const char *eol = p;
                while (*eol && *eol != '\n') eol++;
                line_lens[total_lines] = eol - p;
                total_lines++;
                p = *eol ? eol + 1 : eol;
            }

            // Show newest lines that fit in canvas buffer
            int render_start = 0;
            int render_count = total_lines;
            if (render_count > max_lines) {
                render_start = total_lines - max_lines;
                render_count = max_lines;
            }

            // Check if user is at bottom before redraw
            bool at_bottom = true;
            if (panel) {
                int32_t scroll_remaining = lv_obj_get_scroll_bottom(panel);
                at_bottom = (scroll_remaining < 60);
            }

            // Clear and redraw
            lv_canvas_fill_bg(canvas, lv_color_hex(0x0A1A10), LV_OPA_COVER);

            draw_text_emoji_reset_pool();
            lv_layer_t layer;
            lv_canvas_init_layer(canvas, &layer);

            int32_t y = 4;
            static char line_buf[512];
            for (int i = render_start; i < render_start + render_count && y < buf_h - line_h; i++) {
                int len = line_lens[i];
                if (len > (int)sizeof(line_buf) - 1) len = sizeof(line_buf) - 1;
                memcpy(line_buf, line_starts[i], len);
                line_buf[len] = '\0';

                draw_text_with_emoji((uint16_t *)_registry.win.meshy.msg_canvas_buf,
                                     cw, buf_h, &layer,
                                     line_buf, 4, y,
                                     &lv_font_montserrat_22, text_color);
                y += line_h;
            }

            lv_canvas_finish_layer(canvas, &layer);

            // Canvas stays at full buf_h — don't resize widget.
            // LVGL canvas fill_bg/layer ops need widget height == draw_buf height.
            lv_obj_invalidate(canvas);

            // Auto-scroll panel so the last drawn line is visible
            if (panel && at_bottom && y > 100) {
                // Scroll to put the last line at the bottom of the visible panel
                int32_t panel_h = lv_obj_get_content_height(panel);
                int32_t target_y = y - panel_h + 20;
                if (target_y < 0) target_y = 0;
                lv_obj_scroll_to_y(panel, target_y, LV_ANIM_OFF);
            }
            return;
        }

        // Fallback: plain label (no emoji)
        if (_registry.win.meshy.msg_label) {
            lv_obj_t *panel = lv_obj_get_parent(_registry.win.meshy.msg_label);
            bool at_bottom = true;
            if (panel) {
                int32_t scroll_remaining = lv_obj_get_scroll_bottom(panel);
                at_bottom = (scroll_remaining < 60);
            }

            lv_label_set_text(_registry.win.meshy.msg_label, msg_text);

            if (panel && at_bottom) {
                lv_obj_scroll_to_y(panel, LV_COORD_MAX, LV_ANIM_OFF);
            }
        }
    }

    // ================================================================
    // Scope — Radar-style aircraft display
    // ================================================================

    void System::init_win_scope(void)
    {
        if (_win_scope_status_callback)
            _win_scope_status_callback(true);

        lv_display_t *disp = lv_display_get_default();
        int32_t w = lv_display_get_horizontal_resolution(disp);
        int32_t h = lv_display_get_vertical_resolution(disp);

        // Root screen — dark radar background
        _registry.win.scope.root = lv_obj_create(NULL);
        lv_obj_set_style_bg_color(_registry.win.scope.root, lv_color_hex(0x0a0e14), (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_size(_registry.win.scope.root, w, h);
        lv_obj_set_scrollbar_mode(_registry.win.scope.root, LV_SCROLLBAR_MODE_OFF);

        // Status bar
        int32_t status_h = 50;
        init_status_bar(_registry.win.scope.root);

        // Canvas — rectangular, width fills screen, height is 1.5× width (or max available)
        int32_t canvas_margin = 4;   // minimal margin below status bar
        int32_t canvas_w = w;
        int32_t canvas_top = status_h + canvas_margin;
        int32_t canvas_avail_h = h - canvas_top - 80; // room for info + detail labels below
        int32_t canvas_h = (int32_t)(canvas_w * 1.5f);
        if (canvas_h > canvas_avail_h) canvas_h = canvas_avail_h;

        _registry.win.scope.canvas_w = canvas_w;
        _registry.win.scope.canvas_h = canvas_h;

        // Allocate canvas buffer in PSRAM — aligned per LVGL 9 PPA requirements
        size_t buf_size = canvas_w * canvas_h * 2; // RGB565
        if (!_registry.win.scope.canvas_buf) {
            size_t align = LV_DRAW_BUF_ALIGN < 64 ? 64 : LV_DRAW_BUF_ALIGN;
            _registry.win.scope.canvas_buf = heap_caps_aligned_alloc(align, buf_size, MALLOC_CAP_SPIRAM);
            ESP_LOGI("SCOPE", "Canvas %dx%d buf=%zuB at %p (align=%zu, mod=%zu)",
                     (int)canvas_w, (int)canvas_h, buf_size,
                     _registry.win.scope.canvas_buf, align,
                     _registry.win.scope.canvas_buf ? ((uintptr_t)_registry.win.scope.canvas_buf % align) : 0);
        }
        if (!_registry.win.scope.canvas_buf) {
            ESP_LOGE("SCOPE", "Failed to allocate canvas buffer (%zu bytes)", buf_size);
            return;
        }

        _registry.win.scope.canvas = lv_canvas_create(_registry.win.scope.root);
        lv_canvas_set_buffer(_registry.win.scope.canvas,
                             _registry.win.scope.canvas_buf,
                             canvas_w, canvas_h, LV_COLOR_FORMAT_RGB565);
        lv_obj_set_pos(_registry.win.scope.canvas, (w - canvas_w) / 2, canvas_top);

        // Info label below canvas
        _registry.win.scope.info_label = lv_label_create(_registry.win.scope.root);
        lv_obj_set_style_text_color(_registry.win.scope.info_label, lv_color_hex(0x5c7080), (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_text_font(_registry.win.scope.info_label, &lv_font_montserrat_16, (lv_style_selector_t)LV_PART_MAIN);
        lv_label_set_text(_registry.win.scope.info_label, "Scope initializing...");
        lv_obj_set_width(_registry.win.scope.info_label, w - 20);
        lv_obj_set_style_text_align(_registry.win.scope.info_label, LV_TEXT_ALIGN_CENTER, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_pos(_registry.win.scope.info_label, 10, canvas_top + canvas_h + 4);

        // Detail label for selected aircraft (below info)
        _registry.win.scope.detail_label = lv_label_create(_registry.win.scope.root);
        lv_obj_set_style_text_color(_registry.win.scope.detail_label, lv_color_hex(0x4dabf7), (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_text_font(_registry.win.scope.detail_label, &lv_font_montserrat_16, (lv_style_selector_t)LV_PART_MAIN);
        lv_label_set_text(_registry.win.scope.detail_label, "");
        lv_obj_set_width(_registry.win.scope.detail_label, w - 20);
        lv_obj_set_style_text_align(_registry.win.scope.detail_label, LV_TEXT_ALIGN_CENTER, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_pos(_registry.win.scope.detail_label, 10, canvas_top + canvas_h + 26);

        // Zoom +/- buttons (bottom-right of canvas, overlaid)
        int32_t zoom_btn_size = 60;
        int32_t btn_inset = 8;
        int32_t canvas_left = (w - canvas_w) / 2;
        // Zoom +/- at bottom corners of canvas, 5px inset
        auto make_zoom_btn = [&](const char *symbol, int32_t x_pos, int32_t y_pos) -> lv_obj_t * {
            lv_obj_t *btn = lv_button_create(_registry.win.scope.root);
            lv_obj_set_size(btn, zoom_btn_size, zoom_btn_size);
            lv_obj_set_pos(btn, x_pos, y_pos);
            lv_obj_set_style_bg_color(btn, lv_color_hex(0x1a3a2a), (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_set_style_bg_opa(btn, 200, (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_set_style_shadow_width(btn, 0, (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_set_style_border_color(btn, lv_color_hex(0x1a5c3a), (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_set_style_border_width(btn, 1, (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_set_style_radius(btn, 6, (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_t *lbl = lv_label_create(btn);
            lv_label_set_text(lbl, symbol);
            lv_obj_set_style_text_color(lbl, lv_color_hex(0x00e5a0), (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_set_style_text_font(lbl, &lv_font_montserrat_22, (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_center(lbl);
            return btn;
        };

        lv_obj_t *zoom_out_btn = make_zoom_btn(LV_SYMBOL_MINUS, canvas_left + btn_inset, canvas_top + canvas_h - zoom_btn_size - btn_inset);
        lv_obj_t *zoom_in_btn = make_zoom_btn(LV_SYMBOL_PLUS, canvas_left + canvas_w - zoom_btn_size - btn_inset, canvas_top + canvas_h - zoom_btn_size - btn_inset);

        lv_obj_add_event_cb(zoom_in_btn, [](lv_event_t *e) {
            System *self = static_cast<System *>(lv_event_get_user_data(e));
            if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
            auto &s = self->_registry.win.scope;
            s.zoom *= 1.5f;
            if (s.zoom > 8.0f) s.zoom = 8.0f;
        }, LV_EVENT_ALL, this);

        lv_obj_add_event_cb(zoom_out_btn, [](lv_event_t *e) {
            System *self = static_cast<System *>(lv_event_get_user_data(e));
            if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
            auto &s = self->_registry.win.scope;
            s.zoom /= 1.5f;
            if (s.zoom < 0.25f) { s.zoom = 0.25f; s.pan_x = 0; s.pan_y = 0; }
        }, LV_EVENT_ALL, this);

        // Close button (top-right, created after canvas so it's on top)
        lv_obj_t *close_btn = lv_button_create(_registry.win.scope.root);
        lv_obj_set_size(close_btn, 60, 60);
        lv_obj_set_pos(close_btn, canvas_left + canvas_w - 60 - btn_inset, canvas_top + btn_inset);
        lv_obj_set_style_bg_color(close_btn, lv_color_hex(0x1a3a2a), (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_bg_opa(close_btn, 200, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_shadow_width(close_btn, 0, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_border_color(close_btn, lv_color_hex(0x1a5c3a), (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_border_width(close_btn, 1, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_radius(close_btn, 6, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_t *close_lbl = lv_label_create(close_btn);
        lv_label_set_text(close_lbl, LV_SYMBOL_CLOSE);
        lv_obj_set_style_text_color(close_lbl, lv_color_hex(0x00e5a0), (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_text_font(close_lbl, &lv_font_montserrat_22, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_center(close_lbl);
        lv_obj_add_event_cb(close_btn, [](lv_event_t *e) {
            System *self = static_cast<System *>(lv_event_get_user_data(e));
            if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
            // Stop redraw timer
            if (self->_registry.win.scope.redraw_timer) {
                lv_timer_delete(self->_registry.win.scope.redraw_timer);
                self->_registry.win.scope.redraw_timer = nullptr;
            }
            if (self->_win_scope_status_callback)
                self->_win_scope_status_callback(false);
            lv_display_set_rotation(lv_display_get_default(), self->_home_rotation);
            self->set_vibration();
            self->init_win_home();
            lv_screen_load_anim(self->_registry.win.home.root, LV_SCR_LOAD_ANIM_FADE_OUT, 100, 0, true);
        }, LV_EVENT_ALL, this);

        // Color mode button (top-left of canvas)
        static const char *color_mode_labels[] = {"MONO", "RNBO", "ALT", "SPD"};
        lv_obj_t *color_btn = lv_button_create(_registry.win.scope.root);
        lv_obj_set_size(color_btn, 80, 60);
        lv_obj_set_pos(color_btn, canvas_left + btn_inset, canvas_top + btn_inset);
        lv_obj_set_style_bg_color(color_btn, lv_color_hex(0x1a3a2a), (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_bg_opa(color_btn, 200, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_shadow_width(color_btn, 0, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_border_color(color_btn, lv_color_hex(0x1a5c3a), (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_border_width(color_btn, 1, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_radius(color_btn, 6, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_t *color_lbl = lv_label_create(color_btn);
        lv_label_set_text(color_lbl, color_mode_labels[_registry.win.scope.color_mode]);
        lv_obj_set_style_text_color(color_lbl, lv_color_hex(0x00e5a0), (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_text_font(color_lbl, &lv_font_montserrat_14, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_center(color_lbl);
        lv_obj_add_event_cb(color_btn, [](lv_event_t *e) {
            System *self = static_cast<System *>(lv_event_get_user_data(e));
            if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
            auto &s = self->_registry.win.scope;
            s.color_mode = (s.color_mode + 1) % 4;
            lv_obj_t *lbl = lv_obj_get_child(lv_event_get_target_obj(e), 0);
            if (lbl) lv_label_set_text(lbl, color_mode_labels[s.color_mode]);
        }, LV_EVENT_ALL, this);

        // Reset pan/zoom on entry (keep selected_icao and color_mode across visits)
        _registry.win.scope.pan_x = 0;
        _registry.win.scope.pan_y = 0;
        _registry.win.scope.zoom = 1.0f;

        // Touch events on canvas — drag to pan, tap to select aircraft
        lv_obj_add_flag(_registry.win.scope.canvas, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_remove_flag(_registry.win.scope.canvas, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_flag(_registry.win.scope.canvas, LV_OBJ_FLAG_GESTURE_BUBBLE);
        lv_obj_add_event_cb(_registry.win.scope.canvas, [](lv_event_t *e) {
            System *self = static_cast<System *>(lv_event_get_user_data(e));
            lv_event_code_t code = lv_event_get_code(e);
            if (code != LV_EVENT_PRESSED && code != LV_EVENT_PRESSING &&
                code != LV_EVENT_RELEASED && code != LV_EVENT_PRESS_LOST) return;
            lv_indev_t *indev = lv_indev_active();
            if (!indev) return;
            lv_point_t p;
            lv_indev_get_point(indev, &p);

            auto &s = self->_registry.win.scope;
            // Track last known good touch position
            static int32_t last_x = 0, last_y = 0;

            if (code == LV_EVENT_PRESSED) {
                s.touch_active = true;
                s.touch_start_x = p.x;
                s.touch_start_y = p.y;
                s.pan_start_x = s.pan_x;
                s.pan_start_y = s.pan_y;
                last_x = p.x; last_y = p.y;
            }
            else if (code == LV_EVENT_PRESSING) {
                if (s.touch_active) {
                    s.pan_x = s.pan_start_x + (p.x - s.touch_start_x);
                    s.pan_y = s.pan_start_y + (p.y - s.touch_start_y);
                    last_x = p.x; last_y = p.y;
                }
            }
            else if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) {
                if (s.touch_active) {
                    // Use last known position — RELEASED may not have valid coords
                    int32_t dx = last_x - s.touch_start_x;
                    int32_t dy = last_y - s.touch_start_y;
                    // If barely moved, treat as tap → select aircraft
                    if (dx * dx + dy * dy < 400) {  // ~20px movement threshold
                        int32_t canvas_x_off = lv_obj_get_x(s.canvas);
                        int32_t canvas_y_off = lv_obj_get_y(s.canvas);
                        int32_t tap_cx = last_x - canvas_x_off;
                        int32_t tap_cy = last_y - canvas_y_off;

                        // Check if this tap is near the last tap (cycle mode)
                        int32_t ldx = tap_cx - s.last_tap_x;
                        int32_t ldy = tap_cy - s.last_tap_y;
                        bool same_area = (s.last_tap_x >= 0 && ldx * ldx + ldy * ldy < 2500);

                        if (same_area && s.tap_candidate_count > 1) {
                            // Cycle to next candidate
                            s.tap_cycle_idx = (s.tap_cycle_idx + 1) % s.tap_candidate_count;
                            s.selected_icao = s.tap_candidates[s.tap_cycle_idx];
                        } else {
                            // Build new candidate list — static to avoid stack overflow in LVGL task
                            static scope_aircraft_t ac_buf[64];
                            int ac_cnt = adsb_get_aircraft_for_scope(ac_buf, 64);
                            receiver_pos_t rx = adsb_get_receiver_pos();
                            bool has_fix = rx.fix_valid;
                            double center_lat = rx.lat, center_lon = rx.lon;

                            if (!has_fix) {
                                double sl = 0, sn = 0; int np = 0;
                                for (int i = 0; i < ac_cnt; i++) {
                                    if (ac_buf[i].has_position) { sl += ac_buf[i].lat; sn += ac_buf[i].lon; np++; }
                                }
                                if (np > 0) { center_lat = sl / np; center_lon = sn / np; }
                            }

                            int32_t cx = s.canvas_w / 2;
                            int32_t cy = s.canvas_h / 2;
                            double cos_lat = cos(center_lat * M_PI / 180.0);
                            double scale = (double)(cx - 20) / (double)s.range_nm;

                            // Collect candidates within 60px, sorted by distance
                            struct { uint32_t icao; int32_t dsq; } cands[8];
                            int ncands = 0;
                            for (int i = 0; i < ac_cnt && ncands < 8; i++) {
                                if (!ac_buf[i].has_position) continue;
                                int32_t ax, ay;
                                if (has_fix) {
                                    double brd = ac_buf[i].bearing_deg * M_PI / 180.0;
                                    ax = cx + (int32_t)(ac_buf[i].dist_nm * sin(brd) * scale + s.pan_x);
                                    ay = cy + (int32_t)(-ac_buf[i].dist_nm * cos(brd) * scale + s.pan_y);
                                } else {
                                    double ddx = (ac_buf[i].lon - center_lon) * 60.0 * cos_lat;
                                    double ddy = -(ac_buf[i].lat - center_lat) * 60.0;
                                    ax = cx + (int32_t)(ddx * scale + s.pan_x);
                                    ay = cy + (int32_t)(ddy * scale + s.pan_y);
                                }
                                int32_t tdx = tap_cx - ax, tdy = tap_cy - ay;
                                int32_t dsq = tdx * tdx + tdy * tdy;
                                if (dsq < 3600) { // 60px radius
                                    cands[ncands++] = { ac_buf[i].icao, dsq };
                                }
                            }
                            // Sort by distance
                            for (int i = 0; i < ncands - 1; i++)
                                for (int j = i + 1; j < ncands; j++)
                                    if (cands[j].dsq < cands[i].dsq) {
                                        auto tmp = cands[i]; cands[i] = cands[j]; cands[j] = tmp;
                                    }

                            s.tap_candidate_count = ncands;
                            s.tap_cycle_idx = 0;
                            for (int i = 0; i < ncands; i++) s.tap_candidates[i] = cands[i].icao;
                            s.last_tap_x = tap_cx;
                            s.last_tap_y = tap_cy;

                            if (ncands > 0) {
                                s.selected_icao = (s.tap_candidates[0] == s.selected_icao && ncands == 1) ? 0 : s.tap_candidates[0];
                            } else {
                                s.selected_icao = 0;
                            }
                        }
                    } else {
                        // Was a drag, clear tap cycle state
                        s.last_tap_x = -1;
                        s.tap_candidate_count = 0;
                    }
                    s.touch_active = false;
                }
            }
        }, LV_EVENT_ALL, this);

        // Swipe to return home
        lv_obj_add_event_cb(_registry.win.scope.root, [](lv_event_t *e) {
            System *self = static_cast<System *>(lv_event_get_user_data(e));
            if (lv_event_get_code(e) != LV_EVENT_GESTURE) return;
            lv_dir_t dir = lv_indev_get_gesture_dir(lv_indev_active());
            if ((dir == LV_DIR_LEFT || dir == LV_DIR_RIGHT) && self->_edge_touch_flag) {
                // Stop redraw timer
                if (self->_registry.win.scope.redraw_timer) {
                    lv_timer_delete(self->_registry.win.scope.redraw_timer);
                    self->_registry.win.scope.redraw_timer = nullptr;
                }
                if (self->_win_scope_status_callback)
                    self->_win_scope_status_callback(false);
                lv_display_set_rotation(lv_display_get_default(), self->_home_rotation);
                self->set_vibration();
                self->init_win_home();
                lv_screen_load_anim(self->_registry.win.home.root, LV_SCR_LOAD_ANIM_FADE_OUT, 100, 0, true);
                self->_edge_touch_flag = false;
            }
        }, LV_EVENT_ALL, this);

        // Create LVGL timer for periodic redraw (runs inside lv_timer_handler context)
        // This is critical — canvas pixel buffer changes are only visible to LVGL
        // when invalidated from within the LVGL timer/render cycle.
        if (_registry.win.scope.redraw_timer) {
            lv_timer_delete(_registry.win.scope.redraw_timer);
        }
        _registry.win.scope.redraw_timer = lv_timer_create([](lv_timer_t *t) {
            System *self = static_cast<System *>(lv_timer_get_user_data(t));
            if (self->get_current_win() == Current_Win::SCOPE) {
                self->win_scope_redraw();
            }
        }, 100, this);  // initial 100ms, adaptive code adjusts per frame

        // Initial draw
        win_scope_redraw();

        lv_obj_update_layout(_registry.win.scope.root);
        _current_win = Current_Win::SCOPE;
    }

    // --- Scope color helpers (matching ADS-B Scope web app) ---
    static lv_color_t hsl_to_lv_color(float h, float s, float l) {
        // h: 0-360, s: 0-1, l: 0-1
        float c = (1.0f - fabsf(2.0f * l - 1.0f)) * s;
        float x = c * (1.0f - fabsf(fmodf(h / 60.0f, 2.0f) - 1.0f));
        float m = l - c / 2.0f;
        float r = 0, g = 0, b = 0;
        if (h < 60)       { r = c; g = x; }
        else if (h < 120) { r = x; g = c; }
        else if (h < 180) { g = c; b = x; }
        else if (h < 240) { g = x; b = c; }
        else if (h < 300) { r = x; b = c; }
        else              { r = c; b = x; }
        return lv_color_make((uint8_t)((r + m) * 255), (uint8_t)((g + m) * 255), (uint8_t)((b + m) * 255));
    }

    static lv_color_t scope_aircraft_color(int color_mode, uint32_t icao, int altitude, int speed) {
        switch (color_mode) {
            case 0: // MONO — green
                return lv_color_hex(0x00e5a0);
            case 1: { // RAINBOW — hash of ICAO
                uint32_t h = 0;
                // Same DJB2-like hash as ADS-B Scope: h = (h*31 + byte) & 0xFFFF
                h = ((icao >> 20) & 0xF); // hash the hex digits
                for (int i = 0; i < 6; i++) {
                    uint8_t nibble = (icao >> (20 - i * 4)) & 0xF;
                    uint8_t ch = nibble < 10 ? '0' + nibble : 'A' + nibble - 10;
                    h = (h * 31 + ch) & 0xFFFF;
                }
                return hsl_to_lv_color((float)(h % 360), 0.85f, 0.60f);
            }
            case 2: { // ALT — blue(low) → green → yellow → red(high)
                float a = (float)(altitude < 0 ? 0 : (altitude > 45000 ? 45000 : altitude));
                float hue = 240.0f - (a / 45000.0f) * 240.0f;
                return hsl_to_lv_color(hue, 0.85f, 0.55f);
            }
            case 3: { // SPD — green(slow) → yellow → red(fast)
                float s = (float)(speed < 0 ? 0 : (speed > 500 ? 500 : speed));
                float hue = 120.0f - (s / 500.0f) * 120.0f;
                return hsl_to_lv_color(hue, 0.85f, 0.55f);
            }
            default:
                return lv_color_hex(0x00e5a0);
        }
    }

    void System::win_scope_redraw(void)
    {
        lv_obj_t *canvas = _registry.win.scope.canvas;
        if (!canvas || !_registry.win.scope.canvas_buf) return;

        int64_t render_start = esp_timer_get_time();

        int32_t cw = _registry.win.scope.canvas_w;
        int32_t ch = _registry.win.scope.canvas_h;
        int32_t cx = cw / 2;
        int32_t cy = ch / 2;

        // Static text buffers — LVGL 9 canvas draw is deferred; dsc.text must
        // persist until lv_canvas_finish_layer() renders everything.
        static char ring_labels[8][16];
        static char ac_labels[64][28];

        // Colors
        lv_color_t col_bg    = lv_color_hex(0x0a0e14);
        lv_color_t col_ring  = lv_color_hex(0x1a5c3a);  // green rings (outer)
        lv_color_t col_ring2 = lv_color_hex(0x0e3320);  // green rings (inner, dimmer)
        lv_color_t col_rx    = lv_color_hex(0x00e5a0);
        lv_color_t col_label = lv_color_hex(0x1a5c3a);  // green label text

        // Clear canvas
        lv_canvas_fill_bg(canvas, col_bg, LV_OPA_COVER);

        // Get aircraft data — apply max aircraft limit from settings
        static scope_aircraft_t ac[64];
        int max_ac = 64;
        if (g_settings.scope_max_aircraft > 0 && g_settings.scope_max_aircraft < 64)
            max_ac = g_settings.scope_max_aircraft;
        int ac_count = adsb_get_aircraft_for_scope(ac, max_ac);
        receiver_pos_t rx = adsb_get_receiver_pos();

        // Count positioned aircraft
        int positioned = 0;
        for (int i = 0; i < ac_count; i++) {
            if (ac[i].has_position) positioned++;
        }

        // Determine center and scale
        bool has_fix = rx.fix_valid;
        double center_lat = rx.lat, center_lon = rx.lon;

        // If no GPS fix, center on average of aircraft positions
        if (!has_fix && positioned > 0) {
            double sum_lat = 0, sum_lon = 0;
            for (int i = 0; i < ac_count; i++) {
                if (ac[i].has_position) {
                    sum_lat += ac[i].lat;
                    sum_lon += ac[i].lon;
                }
            }
            center_lat = sum_lat / positioned;
            center_lon = sum_lon / positioned;
        }

        // Compute pixel positions for each aircraft
        // We need this before range calc. Use haversine-like approximation.
        // For display purposes, simple equirectangular projection is fine.
        double cos_lat = cos(center_lat * M_PI / 180.0);
        double nm_per_deg_lat = 60.0;       // 1 deg lat ≈ 60 nm
        double nm_per_deg_lon = 60.0 * cos_lat;

        // Calculate distance from center for all positioned aircraft
        static double ac_dx[64], ac_dy[64], ac_dist[64];
        memset(ac_dx, 0, sizeof(ac_dx));
        memset(ac_dy, 0, sizeof(ac_dy));
        memset(ac_dist, 0, sizeof(ac_dist));
        double max_range_nm = 0;
        for (int i = 0; i < ac_count; i++) {
            if (!ac[i].has_position) continue;
            if (has_fix) {
                // Use precomputed haversine distance/bearing
                ac_dx[i] = ac[i].dist_nm * sin(ac[i].bearing_deg * M_PI / 180.0);
                ac_dy[i] = -ac[i].dist_nm * cos(ac[i].bearing_deg * M_PI / 180.0);  // -Y = north
                ac_dist[i] = ac[i].dist_nm;
            } else {
                // Equirectangular from centroid
                ac_dx[i] = (ac[i].lon - center_lon) * nm_per_deg_lon;
                ac_dy[i] = -(ac[i].lat - center_lat) * nm_per_deg_lat;  // -Y = north
                ac_dist[i] = sqrt(ac_dx[i] * ac_dx[i] + ac_dy[i] * ac_dy[i]);
            }
            if (ac_dist[i] > max_range_nm) max_range_nm = ac_dist[i];
        }

        // Snap to nice range values (default 50nm when no positioned aircraft)
        double range_nm;
        if (positioned == 0) range_nm = 50.0;  // no positioned aircraft — nice default
        else if (max_range_nm <= 10.0) range_nm = 10.0;
        else if (max_range_nm <= 25.0) range_nm = 25.0;
        else if (max_range_nm <= 50.0) range_nm = 50.0;
        else if (max_range_nm <= 100.0) range_nm = 100.0;
        else range_nm = 200.0;

        // Apply zoom
        range_nm /= _registry.win.scope.zoom;
        _registry.win.scope.range_nm = (float)range_nm;
        double scale = (double)(cx - 20) / range_nm; // pixels per nm

        // Pan offset
        float pan_x = _registry.win.scope.pan_x;
        float pan_y = _registry.win.scope.pan_y;

        lv_layer_t layer;
        lv_canvas_init_layer(canvas, &layer);

        if (has_fix) {
            // Dynamic range rings — pick from standard distances, draw all that
            // are visible on screen with enough spacing to be readable
            static const double ring_nms[] = {1, 2, 5, 10, 25, 50, 100, 150, 200};
            static const int ring_nm_count = sizeof(ring_nms) / sizeof(ring_nms[0]);
            int ring_drawn = 0;

            for (int r = 0; r < ring_nm_count && ring_drawn < 8; r++) {
                int32_t radius = (int32_t)(ring_nms[r] * scale);

                // Skip if too small to be useful
                if (radius < 35) continue;
                // Cap radius to prevent LVGL renderer overflow
                if (radius > cw) break;

                // Skip if arc doesn't intersect canvas at all
                int32_t arc_cx = cx + (int32_t)pan_x;
                int32_t arc_cy = cy + (int32_t)pan_y;
                if (arc_cx + radius < 0 || arc_cx - radius > cw ||
                    arc_cy + radius < 0 || arc_cy - radius > ch) continue;

                // Outer rings brighter, inner rings dimmer
                bool is_outermost = (r == ring_nm_count - 1) ||
                    ((int32_t)(ring_nms[r + 1] * scale) > cw);
                lv_color_t ring_col = is_outermost ? col_ring : col_ring2;

                lv_draw_arc_dsc_t arc_dsc;
                lv_draw_arc_dsc_init(&arc_dsc);
                arc_dsc.color = ring_col;
                arc_dsc.width = 2;
                arc_dsc.center.x = cx + (int32_t)pan_x;
                arc_dsc.center.y = cy + (int32_t)pan_y;
                arc_dsc.radius = radius;
                arc_dsc.start_angle = 0;
                arc_dsc.end_angle = 360;
                arc_dsc.opa = LV_OPA_COVER;
                lv_draw_arc(&layer, &arc_dsc);

                // Label just inside the ring, north side (skip if off-canvas)
                int32_t lbl_y = cy + (int32_t)pan_y - radius + 4;
                if (lbl_y > -16 && lbl_y < ch) {
                    int nm_int = (int)ring_nms[r];
                    snprintf(ring_labels[ring_drawn], sizeof(ring_labels[ring_drawn]),
                             "%d nm", nm_int);
                    lv_draw_label_dsc_t lbl_dsc;
                    lv_draw_label_dsc_init(&lbl_dsc);
                    lbl_dsc.color = col_label;
                    lbl_dsc.font = &lv_font_montserrat_14;
                    lbl_dsc.opa = LV_OPA_COVER;
                    lbl_dsc.text = ring_labels[ring_drawn];
                    lv_area_t lbl_area;
                    lbl_area.x1 = cx + (int32_t)pan_x + 4;
                    lbl_area.y1 = lbl_y;
                    lbl_area.x2 = lbl_area.x1 + 60;
                    lbl_area.y2 = lbl_area.y1 + 16;
                    lv_draw_label(&layer, &lbl_dsc, &lbl_area);
                }

                ring_drawn++;
            }

            // Receiver dot (skip if center is way off canvas)
            int32_t rx_cx = cx + (int32_t)pan_x;
            int32_t rx_cy = cy + (int32_t)pan_y;
            if (rx_cx > -50 && rx_cx < cw + 50 && rx_cy > -50 && rx_cy < ch + 50) {
                lv_draw_arc_dsc_t rx_dsc;
                lv_draw_arc_dsc_init(&rx_dsc);
                rx_dsc.color = col_rx;
                rx_dsc.width = 4;
                rx_dsc.center.x = rx_cx;
                rx_dsc.center.y = rx_cy;
                rx_dsc.radius = 4;
                rx_dsc.start_angle = 0;
                rx_dsc.end_angle = 360;
                rx_dsc.opa = LV_OPA_COVER;
                lv_draw_arc(&layer, &rx_dsc);
            }
        } else {
            // No GPS fix — draw a scale bar in bottom-left corner
            // Pick a nice scale: find largest round distance that fits in ~1/4 canvas width
            double bar_target_px = (double)cw * 0.25;
            double bar_nm_options[] = {1, 2, 5, 10, 25, 50, 100};
            double bar_nm = 10;
            for (int i = 6; i >= 0; i--) {
                if (bar_nm_options[i] * scale <= bar_target_px) { bar_nm = bar_nm_options[i]; break; }
            }
            int32_t bar_px = (int32_t)(bar_nm * scale);
            if (bar_px < 20) bar_px = 20;
            int32_t bar_x = 16;
            int32_t bar_y = ch - 84;  // above zoom-out button (60px + 8px inset + margin)

            // Scale bar line
            lv_draw_line_dsc_t bar_dsc;
            lv_draw_line_dsc_init(&bar_dsc);
            bar_dsc.color = col_ring;
            bar_dsc.width = 2;
            bar_dsc.opa = LV_OPA_COVER;
            bar_dsc.p1.x = bar_x; bar_dsc.p1.y = bar_y;
            bar_dsc.p2.x = bar_x + bar_px; bar_dsc.p2.y = bar_y;
            lv_draw_line(&layer, &bar_dsc);
            // End caps
            bar_dsc.p1.y = bar_y - 4; bar_dsc.p2.x = bar_x; bar_dsc.p2.y = bar_y + 4;
            lv_draw_line(&layer, &bar_dsc);
            bar_dsc.p1.x = bar_x + bar_px; bar_dsc.p1.y = bar_y - 4;
            bar_dsc.p2.x = bar_x + bar_px; bar_dsc.p2.y = bar_y + 4;
            lv_draw_line(&layer, &bar_dsc);

            // Scale label
            static char scale_label[16];
            snprintf(scale_label, sizeof(scale_label), "%dnm", (int)bar_nm);
            lv_draw_label_dsc_t slbl;
            lv_draw_label_dsc_init(&slbl);
            slbl.color = col_label;
            slbl.font = &lv_font_montserrat_14;
            slbl.opa = LV_OPA_COVER;
            slbl.text = scale_label;
            lv_area_t sa;
            sa.x1 = bar_x + bar_px + 6;
            sa.y1 = bar_y - 8;
            sa.x2 = sa.x1 + 60;
            sa.y2 = sa.y1 + 16;
            lv_draw_label(&layer, &slbl, &sa);
        }

        // Draw crosshair lines (always)
        lv_draw_line_dsc_t line_dsc;
        lv_draw_line_dsc_init(&line_dsc);
        line_dsc.color = col_ring2;
        line_dsc.width = 1;
        line_dsc.opa = 60;
        line_dsc.p1.x = cx; line_dsc.p1.y = 0;
        line_dsc.p2.x = cx; line_dsc.p2.y = ch;
        lv_draw_line(&layer, &line_dsc);
        line_dsc.p1.x = 0; line_dsc.p1.y = cy;
        line_dsc.p2.x = cw; line_dsc.p2.y = cy;
        lv_draw_line(&layer, &line_dsc);

        // Cardinal labels (string literals — safe for deferred draw)
        static const char *cardinals[] = {"N", "E", "S", "W"};
        int32_t card_x[] = {cx - 4, cw - 18, cx - 4, 4};
        int32_t card_y[] = {4, cy - 8, ch - 20, cy - 8};
        for (int i = 0; i < 4; i++) {
            lv_draw_label_dsc_t cdsc;
            lv_draw_label_dsc_init(&cdsc);
            cdsc.color = col_label;
            cdsc.font = &lv_font_montserrat_14;
            cdsc.opa = LV_OPA_COVER;
            cdsc.text = cardinals[i];
            lv_area_t ca;
            ca.x1 = card_x[i]; ca.y1 = card_y[i];
            ca.x2 = ca.x1 + 20; ca.y2 = ca.y1 + 16;
            lv_draw_label(&layer, &cdsc, &ca);
        }

        // --- Trail data (recorded by scope_trail_timer_cb at 1Hz from boot) ---
        // Just reference the file-scope g_trails / g_trail_count for drawing.

        // Draw trails — budget-based to prevent watchdog on busy airspace
        // Total trail line draws capped at ~800 to stay under render budget
        #define SCOPE_TRAIL_BUDGET 800
        #define SCOPE_TRAIL_MAX_DRAW 60
        if (g_trails) {
          // Count visible trails to compute per-trail segment budget
          int visible_trails = 0;
          for (int t = 0; t < g_trail_count; t++) {
              if (g_trails[t].count < 2) continue;
              // Match the age/selection filter used in the draw loop
              int32_t t_age = 999000;
              for (int a = 0; a < ac_count; a++) {
                  if (ac[a].icao == g_trails[t].icao) { t_age = ac[a].age_ms; break; }
              }
              bool t_sel = (g_trails[t].icao == _registry.win.scope.selected_icao);
              if (t_age > 180000 && !t_sel) continue;
              visible_trails++;
          }
          int segs_per_trail = (visible_trails > 0) ?
              (SCOPE_TRAIL_BUDGET / visible_trails) : SCOPE_TRAIL_MAX_DRAW;
          if (segs_per_trail > SCOPE_TRAIL_MAX_DRAW) segs_per_trail = SCOPE_TRAIL_MAX_DRAW;
          if (segs_per_trail < 4) segs_per_trail = 4;  // minimum for recognizable trail

          for (int t = 0; t < g_trail_count; t++) {
            if (g_trails[t].count < 2) continue;
            int trail_alt = 0, trail_spd = 0;
            int32_t trail_age_ms = 999000;  // assume stale if not found
            for (int a = 0; a < ac_count; a++) {
                if (ac[a].icao == g_trails[t].icao) {
                    trail_alt = ac[a].altitude; trail_spd = ac[a].speed;
                    trail_age_ms = ac[a].age_ms; break;
                }
            }
            bool trail_sel = (g_trails[t].icao == _registry.win.scope.selected_icao);
            // Hide trails for aircraft aged out >180s (unless selected)
            if (trail_age_ms > 180000 && !trail_sel) continue;
            bool trail_gray = (trail_age_ms > 60000);
            lv_color_t trail_ac_color = trail_gray ? lv_color_hex(0x556677) :
                scope_aircraft_color(_registry.win.scope.color_mode,
                                     g_trails[t].icao, trail_alt, trail_spd);
            if (trail_sel && _registry.win.scope.color_mode == 0 && !trail_gray)
                trail_ac_color = lv_color_hex(0x4dabf7);
            int trail_width = (visible_trails > 20) ? 1 : 2;
            if (trail_sel) trail_width = 3;  // selected trail always prominent
            int total = g_trails[t].count;
            int step = (total > segs_per_trail) ? total / segs_per_trail : 1;
            int start = (g_trails[t].head - total + SCOPE_TRAIL_DEPTH) % SCOPE_TRAIL_DEPTH;
            int32_t prev_x = -1, prev_y = -1;
            int draw_idx = 0;
            int actual_segs = total / step;
            for (int j = 0; j < total; j += step) {
                int idx = (start + j) % SCOPE_TRAIL_DEPTH;
                double pt_dx = (g_trails[t].pts[idx].lon - center_lon) * nm_per_deg_lon;
                double pt_dy = -(g_trails[t].pts[idx].lat - center_lat) * nm_per_deg_lat;
                int32_t tx = cx + (int32_t)(pt_dx * scale + pan_x);
                int32_t ty = cy + (int32_t)(pt_dy * scale + pan_y);
                if (tx < 0 || tx > cw || ty < 0 || ty > ch) { prev_x = -1; draw_idx++; continue; }
                if (prev_x >= 0) {
                    lv_draw_line_dsc_t tdsc;
                    lv_draw_line_dsc_init(&tdsc);
                    tdsc.color = trail_ac_color;
                    tdsc.width = trail_width;
                    tdsc.opa = trail_sel ? 220 : (uint8_t)(80 + (draw_idx * 140) / (actual_segs > 0 ? actual_segs : 1));
                    tdsc.p1.x = prev_x; tdsc.p1.y = prev_y;
                    tdsc.p2.x = tx; tdsc.p2.y = ty;
                    lv_draw_line(&layer, &tdsc);
                }
                prev_x = tx; prev_y = ty;
                draw_idx++;
            }
            // Connect to most recent point
            if (total > 1 && step > 1) {
                int last_idx = (g_trails[t].head - 1 + SCOPE_TRAIL_DEPTH) % SCOPE_TRAIL_DEPTH;
                double pt_dx = (g_trails[t].pts[last_idx].lon - center_lon) * nm_per_deg_lon;
                double pt_dy = -(g_trails[t].pts[last_idx].lat - center_lat) * nm_per_deg_lat;
                int32_t tx = cx + (int32_t)(pt_dx * scale + pan_x);
                int32_t ty = cy + (int32_t)(pt_dy * scale + pan_y);
                if (prev_x >= 0 && tx >= 0 && tx <= cw && ty >= 0 && ty <= ch) {
                    lv_draw_line_dsc_t tdsc;
                    lv_draw_line_dsc_init(&tdsc);
                    tdsc.color = trail_ac_color;
                    tdsc.width = trail_width;
                    tdsc.opa = 220;
                    tdsc.p1.x = prev_x; tdsc.p1.y = prev_y;
                    tdsc.p2.x = tx; tdsc.p2.y = ty;
                    lv_draw_line(&layer, &tdsc);
                }
            }
          }
        } // end if(g_trails)

        // Draw aircraft
        int drawn = 0;
        int visible = 0;
        lv_color_t col_gray = lv_color_hex(0x667788);
        for (int i = 0; i < ac_count && drawn < 64; i++) {
            if (!ac[i].has_position) continue;

            bool selected = (ac[i].icao == _registry.win.scope.selected_icao);

            // Age-based visibility (matching ADS-B Scope webapp rules):
            // <30s  → full opacity, normal color
            // 30-60s → 50% opacity, normal color
            // >60s  → 35% opacity, gray color
            // >180s → skip unless selected (accessor already filters at 180s)
            int32_t age_s = ac[i].age_ms / 1000;
            uint8_t age_opa;
            bool age_gray;
            if (age_s < 30)       { age_opa = 255; age_gray = false; }
            else if (age_s < 60)  { age_opa = 128; age_gray = false; }
            else                  { age_opa = 90;  age_gray = true;  }
            // Selected aircraft stay visible
            if (selected && age_opa < 200) age_opa = 200;

            // Convert to pixel coordinates
            int32_t ax = cx + (int32_t)(ac_dx[i] * scale + pan_x);
            int32_t ay = cy + (int32_t)(ac_dy[i] * scale + pan_y);

            // Clamp to canvas (with margin for labels)
            if (ax < 2 || ax > cw - 2 || ay < 2 || ay > ch - 2) { continue; }

            lv_color_t ac_color = age_gray ? col_gray :
                scope_aircraft_color(_registry.win.scope.color_mode,
                                     ac[i].icao, ac[i].altitude, ac[i].speed);
            lv_color_t dot_color = ac_color;
            // In MONO mode, selected aircraft uses blue
            if (selected && _registry.win.scope.color_mode == 0 && !age_gray) {
                dot_color = lv_color_hex(0x4dabf7);
            }

            // Aircraft dot — selected gets bigger + outer ring for emphasis
            lv_draw_arc_dsc_t ac_dsc;
            lv_draw_arc_dsc_init(&ac_dsc);
            ac_dsc.color = dot_color;
            ac_dsc.width = selected ? 5 : 3;
            ac_dsc.center.x = ax;
            ac_dsc.center.y = ay;
            ac_dsc.radius = selected ? 5 : 3;
            ac_dsc.start_angle = 0;
            ac_dsc.end_angle = 360;
            ac_dsc.opa = age_opa;
            lv_draw_arc(&layer, &ac_dsc);

            // Selection ring — white outline to make selected aircraft pop
            if (selected) {
                lv_draw_arc_dsc_t sel_ring;
                lv_draw_arc_dsc_init(&sel_ring);
                sel_ring.color = lv_color_white();
                sel_ring.width = 2;
                sel_ring.center.x = ax;
                sel_ring.center.y = ay;
                sel_ring.radius = 14;
                sel_ring.start_angle = 0;
                sel_ring.end_angle = 360;
                sel_ring.opa = 220;
                lv_draw_arc(&layer, &sel_ring);
            }

            // Heading line
            if (ac[i].heading > 0) {
                double hdg_rad = ac[i].heading * M_PI / 180.0;
                int32_t hlen = selected ? 22 : 12;
                int32_t hx = ax + (int32_t)(hlen * sin(hdg_rad));
                int32_t hy = ay - (int32_t)(hlen * cos(hdg_rad));

                lv_draw_line_dsc_t hdg_dsc;
                lv_draw_line_dsc_init(&hdg_dsc);
                hdg_dsc.color = dot_color;
                hdg_dsc.width = selected ? 3 : 2;
                hdg_dsc.opa = age_opa;
                hdg_dsc.p1.x = ax; hdg_dsc.p1.y = ay;
                hdg_dsc.p2.x = hx; hdg_dsc.p2.y = hy;
                lv_draw_line(&layer, &hdg_dsc);
            }

            // Callsign + FL label (static buffer!)
            if (ac[i].callsign[0]) {
                snprintf(ac_labels[drawn], sizeof(ac_labels[drawn]), "%s FL%d",
                         ac[i].callsign, ac[i].altitude / 100);
            } else {
                snprintf(ac_labels[drawn], sizeof(ac_labels[drawn]), "%06lX FL%d",
                         (unsigned long)ac[i].icao, ac[i].altitude / 100);
            }

            lv_draw_label_dsc_t acdsc;
            lv_draw_label_dsc_init(&acdsc);
            acdsc.color = age_gray ? col_gray : ((selected && _registry.win.scope.color_mode == 0) ? lv_color_hex(0x4dabf7) : ac_color);
            acdsc.font = &lv_font_montserrat_14;
            acdsc.opa = (age_opa < 200) ? age_opa : 200;
            acdsc.text = ac_labels[drawn];
            lv_area_t ac_area;
            ac_area.x1 = ax + 18;
            ac_area.y1 = ay - 8;
            ac_area.x2 = ac_area.x1 + 160;
            ac_area.y2 = ac_area.y1 + 16;
            lv_draw_label(&layer, &acdsc, &ac_area);

            drawn++;
            visible++;
        }

        lv_canvas_finish_layer(canvas, &layer);
        // Force LVGL to see the new pixel data:
        // 1. Re-set buffer flushes the image cache (LVGL 9 canvas = image widget)
        // 2. Invalidate marks the area dirty for the current render cycle
        // Both are needed. This runs inside lv_timer_handler() context via LVGL timer.
        lv_canvas_set_buffer(canvas, _registry.win.scope.canvas_buf,
                             _registry.win.scope.canvas_w, _registry.win.scope.canvas_h,
                             LV_COLOR_FORMAT_RGB565);
        lv_obj_invalidate(canvas);

        // Store visible count
        _registry.win.scope.visible_count = visible;

        // --- Adaptive FPS based on actual render time ---
        int64_t render_end = esp_timer_get_time();
        uint32_t render_us = (uint32_t)(render_end - render_start);
        _registry.win.scope.last_render_us = render_us;

        // Determine target period
        uint32_t target_period_ms;
        if (g_settings.scope_fps_cap > 0) {
            // User-set FPS cap
            target_period_ms = 1000 / g_settings.scope_fps_cap;
        } else {
            // Auto: aim for render time < 60% of frame period (leave headroom for LVGL + touch)
            // Minimum: render_ms * 1.7 (so render is ~60% of frame time)
            uint32_t render_ms = render_us / 1000;
            uint32_t min_period = (render_ms < 10) ? 33 :     // <10ms render → 30fps
                                  (render_ms < 30) ? 50 :     // <30ms → 20fps
                                  (render_ms < 60) ? 100 :    // <60ms → 10fps
                                  (render_ms < 100) ? 200 :   // <100ms → 5fps
                                  (render_ms < 200) ? 500 :   // <200ms → 2fps
                                  1000;                        // >200ms → 1fps
            target_period_ms = min_period;
        }

        // Clamp to 1-30 FPS range
        if (target_period_ms < 33) target_period_ms = 33;    // max 30fps
        if (target_period_ms > 1000) target_period_ms = 1000; // min 1fps

        if (_registry.win.scope.redraw_timer) {
            lv_timer_set_period(_registry.win.scope.redraw_timer, target_period_ms);
        }

        // Update info label with visible count
        char info[128];
        if (has_fix) {
            snprintf(info, sizeof(info), "%.0fnm  %d ac  %d pos  %d vis  ~%lu FPS",
                     range_nm, ac_count, positioned, visible, (unsigned long)(1000 / target_period_ms));
        } else {
            snprintf(info, sizeof(info), "%d ac  %d pos  %d vis  ~%lu FPS  no GPS",
                     ac_count, positioned, visible, (unsigned long)(1000 / target_period_ms));
        }
        if (_registry.win.scope.info_label)
            lv_label_set_text(_registry.win.scope.info_label, info);

        // Update detail label for selected aircraft
        if (_registry.win.scope.detail_label) {
            if (_registry.win.scope.selected_icao != 0) {
                for (int i = 0; i < ac_count; i++) {
                    if (ac[i].icao == _registry.win.scope.selected_icao) {
                        bool det_gray = (ac[i].age_ms > 60000);
                        lv_color_t det_color = det_gray ? lv_color_hex(0x667788) :
                            ((_registry.win.scope.color_mode == 0)
                            ? lv_color_hex(0x4dabf7)
                            : scope_aircraft_color(_registry.win.scope.color_mode,
                                                   ac[i].icao, ac[i].altitude, ac[i].speed));
                        lv_obj_set_style_text_color(_registry.win.scope.detail_label, det_color, (lv_style_selector_t)LV_PART_MAIN);
                        char det[300];
                        int p = 0;
                        // Line 1: callsign + ICAO + msgs
                        p += snprintf(det + p, sizeof(det) - p, "%s  %06lX  %lu msgs",
                                     ac[i].callsign[0] ? ac[i].callsign : "----",
                                     (unsigned long)ac[i].icao,
                                     (unsigned long)ac[i].msg_count);
                        // Line 2: ALT SPD HDG V/S
                        p += snprintf(det + p, sizeof(det) - p, "\nALT %d  SPD %d  HDG %d\xC2\xB0  V/S %+d",
                                     ac[i].altitude, ac[i].speed, ac[i].heading, ac[i].vert_rate);
                        // Line 3: distance + bearing + age
                        if (has_fix && ac[i].has_position) {
                            p += snprintf(det + p, sizeof(det) - p, "\n%.1fnm  %d\xC2\xB0  %ds ago",
                                         ac[i].dist_nm, (int)ac[i].bearing_deg, (int)(ac[i].age_ms / 1000));
                        } else {
                            p += snprintf(det + p, sizeof(det) - p, "\n%ds ago%s",
                                         (int)(ac[i].age_ms / 1000), has_fix ? "" : "  (no GPS)");
                        }
                        lv_label_set_text(_registry.win.scope.detail_label, det);
                        break;
                    }
                }
            } else {
                lv_label_set_text(_registry.win.scope.detail_label, "");
            }
        }
    }

    // ================================================================
    // Settings — Device configuration
    // ================================================================
    void System::init_win_settings(void)
    {
        lv_display_t *disp = lv_display_get_default();
        int32_t w = lv_display_get_horizontal_resolution(disp);
        int32_t h = lv_display_get_vertical_resolution(disp);

        // Root screen
        _registry.win.settings.root = lv_obj_create(NULL);
        lv_obj_set_style_bg_color(_registry.win.settings.root, lv_color_hex(0x0A0E14), (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_size(_registry.win.settings.root, w, h);
        lv_obj_set_scrollbar_mode(_registry.win.settings.root, LV_SCROLLBAR_MODE_OFF);

        // Status bar
        int32_t status_h = 50;
        init_status_bar(_registry.win.settings.root);

        // Title bar
        int32_t title_h = 60;
        lv_obj_t *title_bar = lv_obj_create(_registry.win.settings.root);
        lv_obj_set_size(title_bar, w, title_h);
        lv_obj_set_pos(title_bar, 0, status_h);
        lv_obj_set_style_bg_color(title_bar, lv_color_hex(0x1a2a36), (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_bg_opa(title_bar, LV_OPA_COVER, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_border_width(title_bar, 0, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_radius(title_bar, 0, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_pad_all(title_bar, 8, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_remove_flag(title_bar, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t *title_icon = lv_label_create(title_bar);
        lv_label_set_text(title_icon, LV_SYMBOL_SETTINGS);
        lv_obj_set_style_text_font(title_icon, &lv_font_montserrat_24, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_text_color(title_icon, lv_color_white(), (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_align(title_icon, LV_ALIGN_LEFT_MID, 0, 0);

        lv_obj_t *title_label = lv_label_create(title_bar);
        lv_label_set_text(title_label, " Settings");
        lv_obj_set_style_text_font(title_label, &lv_font_montserrat_24, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_text_color(title_label, lv_color_white(), (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_align_to(title_label, title_icon, LV_ALIGN_OUT_RIGHT_MID, 4, 0);

        // Close button
        lv_obj_t *close_btn = lv_button_create(title_bar);
        lv_obj_set_size(close_btn, 50, 42);
        lv_obj_align(close_btn, LV_ALIGN_RIGHT_MID, -5, 0);
        lv_obj_set_style_bg_color(close_btn, lv_color_hex(0x2a3a46), (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_shadow_width(close_btn, 0, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_border_width(close_btn, 0, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_radius(close_btn, 6, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_t *close_lbl = lv_label_create(close_btn);
        lv_label_set_text(close_lbl, LV_SYMBOL_CLOSE);
        lv_obj_set_style_text_color(close_lbl, lv_color_white(), (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_center(close_lbl);
        lv_obj_add_event_cb(close_btn, [](lv_event_t *e) {
            System *self = static_cast<System *>(lv_event_get_user_data(e));
            if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
            settings_save();
            lv_display_set_rotation(lv_display_get_default(), self->_home_rotation);
            self->set_vibration();
            self->init_win_home();
            lv_screen_load_anim(self->_registry.win.home.root, LV_SCR_LOAD_ANIM_FADE_OUT, 100, 0, true);
        }, LV_EVENT_ALL, this);

        // Scrollable content area
        int32_t content_top = status_h + title_h;
        lv_obj_t *scroll = lv_obj_create(_registry.win.settings.root);
        lv_obj_set_size(scroll, w, h - content_top);
        lv_obj_set_pos(scroll, 0, content_top);
        lv_obj_set_style_bg_color(scroll, lv_color_hex(0x0A0E14), (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_bg_opa(scroll, LV_OPA_COVER, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_border_width(scroll, 0, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_radius(scroll, 0, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_style_pad_all(scroll, 12, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_flex_flow(scroll, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_flex_align(scroll, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
        lv_obj_set_style_pad_row(scroll, 4, (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_scrollbar_mode(scroll, LV_SCROLLBAR_MODE_ACTIVE);
        _registry.win.settings.scroll_container = scroll;

        // Style constants
        lv_color_t col_section = lv_color_hex(0x4dabf7);
        lv_color_t col_label = lv_color_hex(0xc8d6e5);
        lv_color_t col_value = lv_color_hex(0x00e5a0);
        lv_color_t col_row_bg = lv_color_hex(0x141e28);
        int32_t row_w = w - 28;

        // --- Helper lambdas ---

        // Section header
        auto add_section = [&](const char *text) {
            lv_obj_t *lbl = lv_label_create(scroll);
            lv_label_set_text(lbl, text);
            lv_obj_set_style_text_color(lbl, col_section, (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_set_style_text_font(lbl, &lv_font_montserrat_22, (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_set_width(lbl, row_w);
            lv_obj_set_style_pad_top(lbl, 12, (lv_style_selector_t)LV_PART_MAIN);
        };

        // Setting row container
        auto add_row = [&]() -> lv_obj_t * {
            lv_obj_t *row = lv_obj_create(scroll);
            lv_obj_set_size(row, row_w, 52);
            lv_obj_set_style_bg_color(row, col_row_bg, (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_set_style_bg_opa(row, LV_OPA_COVER, (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_set_style_border_width(row, 0, (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_set_style_radius(row, 8, (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_set_style_pad_all(row, 8, (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_remove_flag(row, LV_OBJ_FLAG_SCROLLABLE);
            return row;
        };

        // Label on left side of a row
        auto add_row_label = [&](lv_obj_t *row, const char *text) {
            lv_obj_t *lbl = lv_label_create(row);
            lv_label_set_text(lbl, text);
            lv_obj_set_style_text_color(lbl, col_label, (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_set_style_text_font(lbl, &lv_font_montserrat_16, (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_align(lbl, LV_ALIGN_LEFT_MID, 0, 0);
        };

        // Toggle switch on right side of a row
        auto add_toggle = [&](lv_obj_t *row, bool *setting) -> lv_obj_t * {
            lv_obj_t *sw = lv_switch_create(row);
            lv_obj_set_size(sw, 56, 30);
            lv_obj_align(sw, LV_ALIGN_RIGHT_MID, 0, 0);
            lv_obj_set_style_bg_color(sw, lv_color_hex(0x2a3a46), (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_set_style_bg_color(sw, lv_color_hex(0x00e5a0), (lv_style_selector_t)(LV_PART_INDICATOR | LV_STATE_CHECKED));
            if (*setting) lv_obj_add_state(sw, LV_STATE_CHECKED);
            // Store pointer to setting in user_data for the callback
            lv_obj_set_user_data(sw, (void *)setting);
            lv_obj_add_event_cb(sw, [](lv_event_t *e) {
                if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) return;
                lv_obj_t *sw = lv_event_get_target_obj(e);
                bool *s = (bool *)lv_obj_get_user_data(sw);
                if (s) *s = lv_obj_has_state(sw, LV_STATE_CHECKED);
                settings_save();
            }, LV_EVENT_ALL, nullptr);
            return sw;
        };

        // Value display button on right side (tap to cycle)
        auto add_value_btn = [&](lv_obj_t *row, const char *text) -> lv_obj_t * {
            lv_obj_t *btn = lv_button_create(row);
            lv_obj_set_size(btn, LV_SIZE_CONTENT, 36);
            lv_obj_set_style_min_width(btn, 80, (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_align(btn, LV_ALIGN_RIGHT_MID, 0, 0);
            lv_obj_set_style_bg_color(btn, lv_color_hex(0x1a2a36), (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_set_style_shadow_width(btn, 0, (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_set_style_border_color(btn, lv_color_hex(0x2a4a56), (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_set_style_border_width(btn, 1, (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_set_style_radius(btn, 6, (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_set_style_pad_hor(btn, 12, (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_t *lbl = lv_label_create(btn);
            lv_label_set_text(lbl, text);
            lv_obj_set_style_text_color(lbl, col_value, (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_set_style_text_font(lbl, &lv_font_montserrat_16, (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_center(lbl);
            return btn;
        };

        // Slider on right side of a row
        auto add_slider = [&](lv_obj_t *row, int32_t min, int32_t max, int32_t value) -> lv_obj_t * {
            lv_obj_t *slider = lv_slider_create(row);
            lv_obj_set_size(slider, 180, 20);
            lv_obj_align(slider, LV_ALIGN_RIGHT_MID, 0, 0);
            lv_slider_set_range(slider, min, max);
            lv_slider_set_value(slider, value, LV_ANIM_OFF);
            lv_obj_set_style_bg_color(slider, lv_color_hex(0x2a3a46), (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_set_style_bg_color(slider, lv_color_hex(0x00e5a0), (lv_style_selector_t)LV_PART_INDICATOR);
            lv_obj_set_style_bg_color(slider, lv_color_hex(0xc8d6e5), (lv_style_selector_t)LV_PART_KNOB);
            return slider;
        };

        // Info row (read-only value on right)
        auto add_info_row = [&](const char *label, const char *value) {
            lv_obj_t *row = add_row();
            add_row_label(row, label);
            lv_obj_t *val = lv_label_create(row);
            lv_label_set_text(val, value);
            lv_obj_set_style_text_color(val, col_value, (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_set_style_text_font(val, &lv_font_montserrat_16, (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_align(val, LV_ALIGN_RIGHT_MID, 0, 0);
        };

        // ═══════════════════════════════════════════════════════════════
        // DISPLAY
        // ═══════════════════════════════════════════════════════════════
        add_section(LV_SYMBOL_IMAGE "  Display");

        // Display type (read-only)
        add_info_row("Display", screen_type_name());

        // Brightness slider
        {
            lv_obj_t *row = add_row();
            add_row_label(row, "Brightness");
            lv_obj_t *slider = add_slider(row, 5, 100, (int32_t)g_settings.brightness);
            lv_obj_add_event_cb(slider, [](lv_event_t *e) {
                if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) return;
                System *self = static_cast<System *>(lv_event_get_user_data(e));
                g_settings.brightness = (uint8_t)lv_slider_get_value(lv_event_get_target_obj(e));
                if (self->_device_brightness_callback)
                    self->_device_brightness_callback(g_settings.brightness);
            }, LV_EVENT_ALL, this);
            // Save on release
            lv_obj_add_event_cb(slider, [](lv_event_t *e) {
                if (lv_event_get_code(e) != LV_EVENT_RELEASED) return;
                settings_save();
            }, LV_EVENT_ALL, nullptr);
        }

        // Screen timeout
        {
            lv_obj_t *row = add_row();
            add_row_label(row, "Screen Timeout");
            static const uint16_t timeout_vals[] = {0, 30, 60, 120, 300, 600};
            static const char *timeout_labels[] = {"Never", "30s", "1m", "2m", "5m", "10m"};
            int cur = 0;
            for (int i = 0; i < 6; i++) {
                if (g_settings.screen_timeout_s == timeout_vals[i]) { cur = i; break; }
            }
            lv_obj_t *btn = add_value_btn(row, timeout_labels[cur]);
            lv_obj_add_event_cb(btn, [](lv_event_t *e) {
                if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
                static const uint16_t vals[] = {0, 30, 60, 120, 300, 600};
                static const char *labels[] = {"Never", "30s", "1m", "2m", "5m", "10m"};
                int cur = 0;
                for (int i = 0; i < 6; i++) if (g_settings.screen_timeout_s == vals[i]) { cur = i; break; }
                cur = (cur + 1) % 6;
                g_settings.screen_timeout_s = vals[cur];
                // Reset touch timer so the new timeout starts from now
                g_last_touch_ms = esp_log_timestamp();
                lv_obj_t *lbl = lv_obj_get_child(lv_event_get_target_obj(e), 0);
                if (lbl) lv_label_set_text(lbl, labels[cur]);
                settings_save();
            }, LV_EVENT_ALL, nullptr);
        }

        // Double-tap wake/sleep
        {
            lv_obj_t *row = add_row();
            add_row_label(row, "Double-Tap Wake");
            add_toggle(row, &g_settings.double_tap_wake);
        }

        // Auto-rotation
        {
            lv_obj_t *row = add_row();
            add_row_label(row, "Auto Rotation");
            add_toggle(row, &g_settings.auto_rotation);
        }

        // ═══════════════════════════════════════════════════════════════
        // TIME & DATE
        // ═══════════════════════════════════════════════════════════════
        add_section(LV_SYMBOL_BELL "  Time & Date");

        // Timezone controls — shared state for refresh
        static lv_obj_t *tz_val_lbl = nullptr;   // timezone value label
        static lv_obj_t *tz_minus_btn = nullptr;  // − button
        static lv_obj_t *tz_plus_btn = nullptr;   // + button
        static lv_obj_t *dst_sw = nullptr;        // DST toggle
        static lv_obj_t *dst_info_lbl = nullptr;  // DST auto info label

        // Helper: get auto-detected timezone string for display
        auto get_auto_tz = [&]() -> const char * {
            static char abuf[24];
            receiver_pos_t rx = adsb_get_receiver_pos();
            if (rx.fix_valid) {
                struct timeval tv; gettimeofday(&tv, NULL);
                struct tm tm_utc; gmtime_r(&tv.tv_sec, &tm_utc);
                tz_result_t tz = tz_lookup(rx.lat, rx.lon,
                    tm_utc.tm_year + 1900, tm_utc.tm_mon + 1, tm_utc.tm_mday);
                int h = tz.total_offset_min / 60;
                int m = abs(tz.total_offset_min) % 60;
                snprintf(abuf, sizeof(abuf), "UTC%+d:%02d%s", h, m,
                         tz.dst_active ? " DST" : "");
                return abuf;
            }
            return "No GPS fix";
        };

        // Helper: get manual timezone string (without DST — shown separately)
        auto get_manual_tz = [&]() -> const char * {
            static char mbuf[16];
            snprintf(mbuf, sizeof(mbuf), "UTC%+d:%02d",
                     g_settings.tz_offset_h, abs(g_settings.tz_offset_m));
            return mbuf;
        };

        // Helper: refresh timezone UI state based on tz_auto
        auto tz_ui_refresh = [&]() {
            bool manual = !g_settings.tz_auto;
            lv_color_t dim = lv_color_hex(0x445566);
            lv_color_t active = lv_color_hex(0xccddee);

            // Timezone value label
            if (tz_val_lbl) {
                lv_label_set_text(tz_val_lbl, g_settings.tz_auto ? get_auto_tz() : get_manual_tz());
                lv_obj_set_style_text_color(tz_val_lbl, manual ? active : dim, (lv_style_selector_t)LV_PART_MAIN);
            }
            // ± buttons visibility
            if (tz_minus_btn) { if (manual) lv_obj_remove_flag(tz_minus_btn, LV_OBJ_FLAG_HIDDEN); else lv_obj_add_flag(tz_minus_btn, LV_OBJ_FLAG_HIDDEN); }
            if (tz_plus_btn)  { if (manual) lv_obj_remove_flag(tz_plus_btn, LV_OBJ_FLAG_HIDDEN);  else lv_obj_add_flag(tz_plus_btn, LV_OBJ_FLAG_HIDDEN); }
            // DST toggle vs info label
            if (dst_sw) { if (manual) lv_obj_remove_flag(dst_sw, LV_OBJ_FLAG_HIDDEN); else lv_obj_add_flag(dst_sw, LV_OBJ_FLAG_HIDDEN); }
            if (dst_info_lbl) {
                if (manual) {
                    lv_obj_add_flag(dst_info_lbl, LV_OBJ_FLAG_HIDDEN);
                } else {
                    lv_obj_remove_flag(dst_info_lbl, LV_OBJ_FLAG_HIDDEN);
                    receiver_pos_t rx = adsb_get_receiver_pos();
                    if (rx.fix_valid) {
                        struct timeval tv; gettimeofday(&tv, NULL);
                        struct tm tm_utc; gmtime_r(&tv.tv_sec, &tm_utc);
                        tz_result_t tz = tz_lookup(rx.lat, rx.lon,
                            tm_utc.tm_year + 1900, tm_utc.tm_mon + 1, tm_utc.tm_mday);
                        lv_label_set_text(dst_info_lbl, tz.has_dst ? (tz.dst_active ? "Active" : "Inactive") : "N/A");
                    } else {
                        lv_label_set_text(dst_info_lbl, "---");
                    }
                    lv_obj_set_style_text_color(dst_info_lbl, dim, (lv_style_selector_t)LV_PART_MAIN);
                }
            }
        };

        // Auto Timezone toggle
        {
            lv_obj_t *row = add_row();
            add_row_label(row, "Auto Timezone");
            lv_obj_t *sw = lv_switch_create(row);
            lv_obj_set_size(sw, 56, 30);
            lv_obj_align(sw, LV_ALIGN_RIGHT_MID, 0, 0);
            lv_obj_set_style_bg_color(sw, lv_color_hex(0x2a3a46), (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_set_style_bg_color(sw, lv_color_hex(0x00e5a0), (lv_style_selector_t)(LV_PART_INDICATOR | LV_STATE_CHECKED));
            if (g_settings.tz_auto) lv_obj_add_state(sw, LV_STATE_CHECKED);
            lv_obj_add_event_cb(sw, [](lv_event_t *e) {
                if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) return;
                g_settings.tz_auto = lv_obj_has_state(lv_event_get_target_obj(e), LV_STATE_CHECKED);
                settings_apply_timezone();
                settings_save();
                // Refresh timezone UI widgets
                bool manual = !g_settings.tz_auto;
                lv_color_t dim = lv_color_hex(0x445566);
                lv_color_t active = lv_color_hex(0xccddee);
                if (tz_val_lbl) {
                    if (g_settings.tz_auto) {
                        // Show auto-detected offset
                        receiver_pos_t rx = adsb_get_receiver_pos();
                        if (rx.fix_valid) {
                            struct timeval tv; gettimeofday(&tv, NULL);
                            struct tm tm_utc; gmtime_r(&tv.tv_sec, &tm_utc);
                            tz_result_t tz = tz_lookup(rx.lat, rx.lon,
                                tm_utc.tm_year + 1900, tm_utc.tm_mon + 1, tm_utc.tm_mday);
                            lv_label_set_text(tz_val_lbl, settings_format_offset(tz.total_offset_min, tz.dst_active));
                        } else {
                            lv_label_set_text(tz_val_lbl, "No GPS fix");
                        }
                    } else {
                        lv_label_set_text(tz_val_lbl, settings_format_offset(
                            g_settings.tz_offset_h * 60 + g_settings.tz_offset_m, false));
                    }
                    lv_obj_set_style_text_color(tz_val_lbl, manual ? active : dim, (lv_style_selector_t)LV_PART_MAIN);
                }
                if (tz_minus_btn) { if (manual) lv_obj_remove_flag(tz_minus_btn, LV_OBJ_FLAG_HIDDEN); else lv_obj_add_flag(tz_minus_btn, LV_OBJ_FLAG_HIDDEN); }
                if (tz_plus_btn)  { if (manual) lv_obj_remove_flag(tz_plus_btn, LV_OBJ_FLAG_HIDDEN);  else lv_obj_add_flag(tz_plus_btn, LV_OBJ_FLAG_HIDDEN); }
                if (dst_sw)       { if (manual) lv_obj_remove_flag(dst_sw, LV_OBJ_FLAG_HIDDEN);       else lv_obj_add_flag(dst_sw, LV_OBJ_FLAG_HIDDEN); }
                if (dst_info_lbl) {
                    if (manual) {
                        lv_obj_add_flag(dst_info_lbl, LV_OBJ_FLAG_HIDDEN);
                    } else {
                        lv_obj_remove_flag(dst_info_lbl, LV_OBJ_FLAG_HIDDEN);
                        receiver_pos_t rx = adsb_get_receiver_pos();
                        if (rx.fix_valid) {
                            struct timeval tv; gettimeofday(&tv, NULL);
                            struct tm tm_utc; gmtime_r(&tv.tv_sec, &tm_utc);
                            tz_result_t tz = tz_lookup(rx.lat, rx.lon,
                                tm_utc.tm_year + 1900, tm_utc.tm_mon + 1, tm_utc.tm_mday);
                            lv_label_set_text(dst_info_lbl, tz.has_dst ? (tz.dst_active ? "Active" : "Inactive") : "N/A");
                        } else {
                            lv_label_set_text(dst_info_lbl, "---");
                        }
                        lv_obj_set_style_text_color(dst_info_lbl, dim, (lv_style_selector_t)LV_PART_MAIN);
                    }
                }
            }, LV_EVENT_ALL, nullptr);
        }

        // Timezone row — with ± buttons for manual mode
        {
            lv_obj_t *row = add_row();
            add_row_label(row, "Timezone");

            // Container for [−] [value] [+] on the right
            lv_obj_t *ctl = lv_obj_create(row);
            lv_obj_set_size(ctl, LV_SIZE_CONTENT, 36);
            lv_obj_align(ctl, LV_ALIGN_RIGHT_MID, 0, 0);
            lv_obj_set_style_bg_opa(ctl, 0, (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_set_style_border_width(ctl, 0, (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_set_style_pad_all(ctl, 0, (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_remove_flag(ctl, LV_OBJ_FLAG_SCROLLABLE);
            lv_obj_set_flex_flow(ctl, LV_FLEX_FLOW_ROW);
            lv_obj_set_flex_align(ctl, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
            lv_obj_set_style_pad_column(ctl, 4, (lv_style_selector_t)LV_PART_MAIN);

            // − button
            tz_minus_btn = lv_button_create(ctl);
            lv_obj_set_size(tz_minus_btn, 36, 32);
            lv_obj_set_style_bg_color(tz_minus_btn, lv_color_hex(0x1a2a36), (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_set_style_shadow_width(tz_minus_btn, 0, (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_set_style_radius(tz_minus_btn, 4, (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_t *ml = lv_label_create(tz_minus_btn);
            lv_label_set_text(ml, LV_SYMBOL_MINUS);
            lv_obj_set_style_text_color(ml, lv_color_hex(0xccddee), (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_center(ml);
            lv_obj_add_event_cb(tz_minus_btn, [](lv_event_t *e) {
                if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
                g_settings.tz_offset_h--;
                if (g_settings.tz_offset_h < -12) g_settings.tz_offset_h = 14;
                settings_apply_timezone();
                settings_save();
                if (tz_val_lbl) lv_label_set_text(tz_val_lbl,
                    settings_format_offset(g_settings.tz_offset_h * 60 + g_settings.tz_offset_m, false));
            }, LV_EVENT_ALL, nullptr);

            // Value label
            tz_val_lbl = lv_label_create(ctl);
            lv_obj_set_style_text_font(tz_val_lbl, &lv_font_montserrat_16, (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_set_style_min_width(tz_val_lbl, 90, (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_set_style_text_align(tz_val_lbl, LV_TEXT_ALIGN_CENTER, (lv_style_selector_t)LV_PART_MAIN);

            // + button
            tz_plus_btn = lv_button_create(ctl);
            lv_obj_set_size(tz_plus_btn, 36, 32);
            lv_obj_set_style_bg_color(tz_plus_btn, lv_color_hex(0x1a2a36), (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_set_style_shadow_width(tz_plus_btn, 0, (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_set_style_radius(tz_plus_btn, 4, (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_t *pl = lv_label_create(tz_plus_btn);
            lv_label_set_text(pl, LV_SYMBOL_PLUS);
            lv_obj_set_style_text_color(pl, lv_color_hex(0xccddee), (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_center(pl);
            lv_obj_add_event_cb(tz_plus_btn, [](lv_event_t *e) {
                if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
                g_settings.tz_offset_h++;
                if (g_settings.tz_offset_h > 14) g_settings.tz_offset_h = -12;
                settings_apply_timezone();
                settings_save();
                if (tz_val_lbl) lv_label_set_text(tz_val_lbl,
                    settings_format_offset(g_settings.tz_offset_h * 60 + g_settings.tz_offset_m, false));
            }, LV_EVENT_ALL, nullptr);
        }

        // DST row — toggle (manual) or info label (auto)
        {
            lv_obj_t *row = add_row();
            add_row_label(row, "Daylight Saving");

            // Manual mode: toggle switch
            dst_sw = lv_switch_create(row);
            lv_obj_set_size(dst_sw, 56, 30);
            lv_obj_align(dst_sw, LV_ALIGN_RIGHT_MID, 0, 0);
            lv_obj_set_style_bg_color(dst_sw, lv_color_hex(0x2a3a46), (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_set_style_bg_color(dst_sw, lv_color_hex(0x00e5a0), (lv_style_selector_t)(LV_PART_INDICATOR | LV_STATE_CHECKED));
            if (g_settings.dst_enabled) lv_obj_add_state(dst_sw, LV_STATE_CHECKED);
            lv_obj_add_event_cb(dst_sw, [](lv_event_t *e) {
                if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) return;
                g_settings.dst_enabled = lv_obj_has_state(lv_event_get_target_obj(e), LV_STATE_CHECKED);
                settings_apply_timezone();
                settings_save();
            }, LV_EVENT_ALL, nullptr);

            // Auto mode: info label (read-only)
            dst_info_lbl = lv_label_create(row);
            lv_obj_align(dst_info_lbl, LV_ALIGN_RIGHT_MID, 0, 0);
            lv_obj_set_style_text_font(dst_info_lbl, &lv_font_montserrat_16, (lv_style_selector_t)LV_PART_MAIN);
        }

        // Apply initial state
        tz_ui_refresh();

        // 24h format
        {
            lv_obj_t *row = add_row();
            add_row_label(row, "24-Hour Format");
            add_toggle(row, &g_settings.time_24h);
        }

        // Show seconds
        {
            lv_obj_t *row = add_row();
            add_row_label(row, "Show Seconds");
            add_toggle(row, &g_settings.show_seconds);
        }

        // ═══════════════════════════════════════════════════════════════
        // ADS-B
        // ═══════════════════════════════════════════════════════════════
        add_section(LV_SYMBOL_GPS "  ADS-B");

        // ADS-B enable
        {
            lv_obj_t *row = add_row();
            add_row_label(row, "RTL-SDR Receiver");
            add_toggle(row, &g_settings.adsb_enabled);
        }

        // SD logging
        {
            lv_obj_t *row = add_row();
            add_row_label(row, "SD Card Logging");
            add_toggle(row, &g_settings.adsb_sd_logging);
        }

        // Bias-T power (live toggle — no radio reset needed)
        {
            lv_obj_t *row = add_row();
            add_row_label(row, "Bias-T Power");
            lv_obj_t *sw = lv_switch_create(row);
            lv_obj_set_size(sw, 56, 30);
            lv_obj_align(sw, LV_ALIGN_RIGHT_MID, 0, 0);
            lv_obj_set_style_bg_color(sw, lv_color_hex(0x2a3a46), (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_set_style_bg_color(sw, lv_color_hex(0x00e5a0), (lv_style_selector_t)(LV_PART_INDICATOR | LV_STATE_CHECKED));
            if (g_settings.adsb_bias_tee) lv_obj_add_state(sw, LV_STATE_CHECKED);
            lv_obj_add_event_cb(sw, [](lv_event_t *e) {
                if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) return;
                lv_obj_t *sw = lv_event_get_target_obj(e);
                bool on = lv_obj_has_state(sw, LV_STATE_CHECKED);
                g_settings.adsb_bias_tee = on;
                adsb_set_bias_tee(on);
                settings_save();
            }, LV_EVENT_ALL, nullptr);
        }

        // ═══════════════════════════════════════════════════════════════
        // MESHTASTIC
        // ═══════════════════════════════════════════════════════════════
        add_section(LV_SYMBOL_WIFI "  Meshy");

        // Enable — actually start/stop radio
        {
            lv_obj_t *row = add_row();
            add_row_label(row, "LoRa Radio");
            lv_obj_t *sw = lv_switch_create(row);
            lv_obj_set_size(sw, 56, 30);
            lv_obj_align(sw, LV_ALIGN_RIGHT_MID, 0, 0);
            lv_obj_set_style_bg_color(sw, lv_color_hex(0x2a3a46), (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_set_style_bg_color(sw, lv_color_hex(0x00e5a0), (lv_style_selector_t)(LV_PART_INDICATOR | LV_STATE_CHECKED));
            if (g_settings.meshy_enabled) lv_obj_add_state(sw, LV_STATE_CHECKED);
            lv_obj_add_event_cb(sw, [](lv_event_t *e) {
                if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) return;
                lv_obj_t *sw = lv_event_get_target_obj(e);
                bool on = lv_obj_has_state(sw, LV_STATE_CHECKED);
                g_settings.meshy_enabled = on;
                settings_save();
                if (on) meshy_start();
                else    meshy_stop();
            }, LV_EVENT_ALL, nullptr);
        }

        // SD logging
        {
            lv_obj_t *row = add_row();
            add_row_label(row, "SD Card Logging");
            add_toggle(row, &g_settings.meshy_sd_logging);
        }

        // Region
        {
            lv_obj_t *row = add_row();
            add_row_label(row, "Region");
            lv_obj_t *btn = add_value_btn(row, settings_region_name(g_settings.meshy_region));
            lv_obj_add_event_cb(btn, [](lv_event_t *e) {
                if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
                g_settings.meshy_region = (g_settings.meshy_region + 1) % 15;
                lv_obj_t *lbl = lv_obj_get_child(lv_event_get_target_obj(e), 0);
                if (lbl) lv_label_set_text(lbl, settings_region_name(g_settings.meshy_region));
                settings_save();
                if (g_settings.meshy_enabled) meshy_restart();
            }, LV_EVENT_ALL, nullptr);
        }

        // Channel preset
        {
            lv_obj_t *row = add_row();
            add_row_label(row, "Preset");
            lv_obj_t *btn = add_value_btn(row, settings_preset_name(g_settings.meshy_preset));
            lv_obj_add_event_cb(btn, [](lv_event_t *e) {
                if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
                g_settings.meshy_preset = (g_settings.meshy_preset + 1) % 6;
                lv_obj_t *lbl = lv_obj_get_child(lv_event_get_target_obj(e), 0);
                if (lbl) lv_label_set_text(lbl, settings_preset_name(g_settings.meshy_preset));
                settings_save();
                if (g_settings.meshy_enabled) meshy_restart();
            }, LV_EVENT_ALL, nullptr);
        }

        // Frequency slot override
        {
            lv_obj_t *row = add_row();
            add_row_label(row, "Freq Slot");
            static char slot_buf[8];
            if (g_settings.meshy_freq_slot == 0)
                snprintf(slot_buf, sizeof(slot_buf), "Auto");
            else
                snprintf(slot_buf, sizeof(slot_buf), "%d", g_settings.meshy_freq_slot);
            lv_obj_t *btn = add_value_btn(row, slot_buf);
            lv_obj_add_event_cb(btn, [](lv_event_t *e) {
                if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
                // Cycle: 0 (Auto) → 1 → 2 → ... → 50 → 0
                g_settings.meshy_freq_slot = (g_settings.meshy_freq_slot + 1) % 51;
                static char buf[8];
                if (g_settings.meshy_freq_slot == 0)
                    snprintf(buf, sizeof(buf), "Auto");
                else
                    snprintf(buf, sizeof(buf), "%d", g_settings.meshy_freq_slot);
                lv_obj_t *lbl = lv_obj_get_child(lv_event_get_target_obj(e), 0);
                if (lbl) lv_label_set_text(lbl, buf);
                settings_save();
                if (g_settings.meshy_enabled) meshy_restart();
            }, LV_EVENT_ALL, nullptr);
        }

        // OK to MQTT
        {
            lv_obj_t *row = add_row();
            add_row_label(row, "OK to MQTT");
            add_toggle(row, &g_settings.meshy_ok_to_mqtt);
        }

        // Hop Limit
        {
            lv_obj_t *row = add_row();
            add_row_label(row, "Hop Limit");
            static char hop_buf[4];
            snprintf(hop_buf, sizeof(hop_buf), "%d", g_settings.meshy_hop_limit);
            lv_obj_t *btn = add_value_btn(row, hop_buf);
            lv_obj_add_event_cb(btn, [](lv_event_t *e) {
                if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
                uint8_t h = g_settings.meshy_hop_limit;
                h = (h >= 7) ? 1 : h + 1;
                g_settings.meshy_hop_limit = h;
                static char buf[4];
                snprintf(buf, sizeof(buf), "%d", h);
                lv_obj_t *lbl = lv_obj_get_child(lv_event_get_target_obj(e), 0);
                if (lbl) lv_label_set_text(lbl, buf);
                settings_save();
            }, LV_EVENT_ALL, nullptr);
        }

        // TX power
        {
            lv_obj_t *row = add_row();
            add_row_label(row, "TX Power");
            static char pwr_buf[8];
            snprintf(pwr_buf, sizeof(pwr_buf), "%ddBm", g_settings.meshy_tx_power);
            lv_obj_t *btn = add_value_btn(row, pwr_buf);
            lv_obj_add_event_cb(btn, [](lv_event_t *e) {
                if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
                static const uint8_t powers[] = {10, 17, 20, 22, 27, 30};
                int cur = 0;
                for (int i = 0; i < 6; i++) if (g_settings.meshy_tx_power == powers[i]) { cur = i; break; }
                cur = (cur + 1) % 6;
                g_settings.meshy_tx_power = powers[cur];
                static char buf[8];
                snprintf(buf, sizeof(buf), "%ddBm", g_settings.meshy_tx_power);
                lv_obj_t *lbl = lv_obj_get_child(lv_event_get_target_obj(e), 0);
                if (lbl) lv_label_set_text(lbl, buf);
                settings_save();
                if (g_settings.meshy_enabled) meshy_restart();
            }, LV_EVENT_ALL, nullptr);
        }

        // Role
        {
            lv_obj_t *row = add_row();
            add_row_label(row, "Role");
            lv_obj_t *btn = add_value_btn(row, settings_role_name(g_settings.meshy_role));
            lv_obj_add_event_cb(btn, [](lv_event_t *e) {
                if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
                g_settings.meshy_role = (g_settings.meshy_role + 1) % 3;
                lv_obj_t *lbl = lv_obj_get_child(lv_event_get_target_obj(e), 0);
                if (lbl) lv_label_set_text(lbl, settings_role_name(g_settings.meshy_role));
                settings_save();
                if (g_settings.meshy_enabled) meshy_restart();
            }, LV_EVENT_ALL, nullptr);
        }

        // NODEINFO Period
        {
            lv_obj_t *row = add_row();
            add_row_label(row, "NODEINFO");
            static const uint16_t ni_vals[] = {15, 30, 60, 120, 240, 480, 1440};
            static const char *ni_labels[] = {"15m", "30m", "1h", "2h", "4h", "8h", "24h"};
            int cur = 1; // default 30m
            for (int i = 0; i < 7; i++) if (g_settings.meshy_nodeinfo_period_m == ni_vals[i]) { cur = i; break; }
            lv_obj_t *btn = add_value_btn(row, ni_labels[cur]);
            lv_obj_add_event_cb(btn, [](lv_event_t *e) {
                if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
                static const uint16_t vals[] = {15, 30, 60, 120, 240, 480, 1440};
                static const char *labels[] = {"15m", "30m", "1h", "2h", "4h", "8h", "24h"};
                int cur = 1;
                for (int i = 0; i < 7; i++) if (g_settings.meshy_nodeinfo_period_m == vals[i]) { cur = i; break; }
                cur = (cur + 1) % 7;
                g_settings.meshy_nodeinfo_period_m = vals[cur];
                lv_obj_t *lbl = lv_obj_get_child(lv_event_get_target_obj(e), 0);
                if (lbl) lv_label_set_text(lbl, labels[cur]);
                settings_save();
            }, LV_EVENT_ALL, nullptr);
        }

        // ═══════════════════════════════════════════════════════════════
        // GPS
        // ═══════════════════════════════════════════════════════════════
        add_section(LV_SYMBOL_GPS "  GPS");

        {
            lv_obj_t *row = add_row();
            add_row_label(row, "GPS Module");
            add_toggle(row, &g_settings.gps_enabled);
        }

        // ═══════════════════════════════════════════════════════════════
        // SCOPE DISPLAY
        // ═══════════════════════════════════════════════════════════════
        add_section(LV_SYMBOL_EYE_OPEN "  Scope Display");

        // FPS cap
        {
            lv_obj_t *row = add_row();
            add_row_label(row, "FPS Cap");
            static const uint8_t fps_vals[] = {0, 5, 10, 15, 30};
            static const char *fps_labels[] = {"Auto", "5", "10", "15", "30"};
            int cur = 0;
            for (int i = 0; i < 5; i++) if (g_settings.scope_fps_cap == fps_vals[i]) { cur = i; break; }
            lv_obj_t *btn = add_value_btn(row, fps_labels[cur]);
            lv_obj_add_event_cb(btn, [](lv_event_t *e) {
                if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
                static const uint8_t vals[] = {0, 5, 10, 15, 30};
                static const char *labels[] = {"Auto", "5", "10", "15", "30"};
                int cur = 0;
                for (int i = 0; i < 5; i++) if (g_settings.scope_fps_cap == vals[i]) { cur = i; break; }
                cur = (cur + 1) % 5;
                g_settings.scope_fps_cap = vals[cur];
                lv_obj_t *lbl = lv_obj_get_child(lv_event_get_target_obj(e), 0);
                if (lbl) lv_label_set_text(lbl, labels[cur]);
                settings_save();
            }, LV_EVENT_ALL, nullptr);
        }

        // Max aircraft
        {
            lv_obj_t *row = add_row();
            add_row_label(row, "Max Aircraft");
            static const uint8_t ac_vals[] = {0, 16, 32, 48, 64};
            static const char *ac_labels[] = {"All", "16", "32", "48", "64"};
            int cur = 0;
            for (int i = 0; i < 5; i++) if (g_settings.scope_max_aircraft == ac_vals[i]) { cur = i; break; }
            lv_obj_t *btn = add_value_btn(row, ac_labels[cur]);
            lv_obj_add_event_cb(btn, [](lv_event_t *e) {
                if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
                static const uint8_t vals[] = {0, 16, 32, 48, 64};
                static const char *labels[] = {"All", "16", "32", "48", "64"};
                int cur = 0;
                for (int i = 0; i < 5; i++) if (g_settings.scope_max_aircraft == vals[i]) { cur = i; break; }
                cur = (cur + 1) % 5;
                g_settings.scope_max_aircraft = vals[cur];
                lv_obj_t *lbl = lv_obj_get_child(lv_event_get_target_obj(e), 0);
                if (lbl) lv_label_set_text(lbl, labels[cur]);
                settings_save();
            }, LV_EVENT_ALL, nullptr);
        }

        // ═══════════════════════════════════════════════════════════════
        // AUDIO & HAPTICS
        // ═══════════════════════════════════════════════════════════════
        add_section(LV_SYMBOL_VOLUME_MAX "  Audio & Haptics");

        // Volume slider
        {
            lv_obj_t *row = add_row();
            add_row_label(row, "Volume");
            lv_obj_t *slider = add_slider(row, 0, 100, (int32_t)g_settings.volume);
            lv_obj_add_event_cb(slider, [](lv_event_t *e) {
                if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) return;
                System *self = static_cast<System *>(lv_event_get_user_data(e));
                g_settings.volume = (uint8_t)lv_slider_get_value(lv_event_get_target_obj(e));
                if (self->_device_volume_callback)
                    self->_device_volume_callback(g_settings.volume);
            }, LV_EVENT_ALL, this);
            lv_obj_add_event_cb(slider, [](lv_event_t *e) {
                if (lv_event_get_code(e) != LV_EVENT_RELEASED) return;
                settings_save();
            }, LV_EVENT_ALL, nullptr);
        }

        // Haptic
        {
            lv_obj_t *row = add_row();
            add_row_label(row, "Haptic Feedback");
            add_toggle(row, &g_settings.haptic_enabled);
        }

        // ═══════════════════════════════════════════════════════════════
        // SYSTEM
        // ═══════════════════════════════════════════════════════════════
        add_section(LV_SYMBOL_CHARGE "  System");

        // Reboot device
        {
            lv_obj_t *row = add_row();
            add_row_label(row, "Reboot Device");
            lv_obj_t *btn = add_value_btn(row, "REBOOT");
            lv_obj_set_style_bg_color(btn, lv_color_hex(0x1a2a3a), (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_set_style_border_color(btn, lv_color_hex(0x2a4a6a), (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_t *rlbl = lv_obj_get_child(btn, 0);
            if (rlbl) lv_obj_set_style_text_color(rlbl, lv_color_hex(0x4dabf7), (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_add_event_cb(btn, [](lv_event_t *e) {
                if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
                esp_restart();
            }, LV_EVENT_ALL, nullptr);
        }

        // Memory info
        {
            static char mem_buf[64];
            snprintf(mem_buf, sizeof(mem_buf), "%luKB / %.1fMB",
                     (unsigned long)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024),
                     (double)heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / (1024.0 * 1024.0));
            add_info_row("Free RAM / PSRAM", mem_buf);
        }

        // Console heartbeat
        {
            lv_obj_t *row = add_row();
            add_row_label(row, "Console Heartbeat");
            add_toggle(row, &g_settings.heartbeat_enabled);
        }
        {
            static const uint8_t hb_vals[] = {5, 10, 15, 30, 60, 120};
            static const char *hb_labels[] = {"5s", "10s", "15s", "30s", "60s", "120s"};
            lv_obj_t *row = add_row();
            add_row_label(row, "Heartbeat Period");
            static char hb_buf[8];
            snprintf(hb_buf, sizeof(hb_buf), "%ds", g_settings.heartbeat_period_s);
            lv_obj_t *btn = add_value_btn(row, hb_buf);
            lv_obj_add_event_cb(btn, [](lv_event_t *e) {
                if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
                static const uint8_t vals[] = {5, 10, 15, 30, 60, 120};
                static const char *labels[] = {"5s", "10s", "15s", "30s", "60s", "120s"};
                int cur = 3; // default 30s
                for (int i = 0; i < 6; i++) {
                    if (g_settings.heartbeat_period_s == vals[i]) { cur = i; break; }
                }
                cur = (cur + 1) % 6;
                g_settings.heartbeat_period_s = vals[cur];
                lv_obj_t *lbl = lv_obj_get_child(lv_event_get_target_obj(e), 0);
                if (lbl) lv_label_set_text(lbl, labels[cur]);
                settings_save();
            }, LV_EVENT_ALL, nullptr);
        }

        // Firmware version
        {
            const esp_app_desc_t *app = esp_app_get_description();
            add_info_row("Firmware", app->version);
        }

        // Factory reset
        {
            lv_obj_t *row = add_row();
            add_row_label(row, "Factory Reset");
            lv_obj_t *btn = add_value_btn(row, "RESET...");
            lv_obj_set_style_bg_color(btn, lv_color_hex(0x3a1a1a), (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_set_style_border_color(btn, lv_color_hex(0x6a2a2a), (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_t *rlbl = lv_obj_get_child(btn, 0);
            if (rlbl) lv_obj_set_style_text_color(rlbl, lv_color_hex(0xff4444), (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_add_event_cb(btn, [](lv_event_t *e) {
                System *self = static_cast<System *>(lv_event_get_user_data(e));
                if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;

                // ─── Reset Options Dialog ──────────────────────────────
                lv_obj_t *overlay = lv_obj_create(self->_registry.win.settings.root);
                lv_obj_set_size(overlay, self->_width, self->_height);
                lv_obj_align(overlay, LV_ALIGN_CENTER, 0, 0);
                lv_obj_set_style_bg_color(overlay, lv_color_hex(0x000000), LV_PART_MAIN);
                lv_obj_set_style_bg_opa(overlay, 200, LV_PART_MAIN);
                lv_obj_set_style_border_width(overlay, 0, LV_PART_MAIN);

                lv_obj_t *dialog = lv_obj_create(overlay);
                lv_obj_set_size(dialog, 440, 500);
                lv_obj_center(dialog);
                lv_obj_set_style_bg_color(dialog, lv_color_hex(0x1A1A1A), LV_PART_MAIN);
                lv_obj_set_style_radius(dialog, 20, LV_PART_MAIN);
                lv_obj_set_style_border_width(dialog, 0, LV_PART_MAIN);
                lv_obj_set_style_pad_all(dialog, 20, LV_PART_MAIN);
                lv_obj_set_flex_flow(dialog, LV_FLEX_FLOW_COLUMN);
                lv_obj_set_style_pad_row(dialog, 12, LV_PART_MAIN);

                // Title
                lv_obj_t *title = lv_label_create(dialog);
                lv_label_set_text(title, "Factory Reset");
                lv_obj_set_style_text_color(title, lv_color_hex(0xff4444), LV_PART_MAIN);
                lv_obj_set_style_text_font(title, &lv_font_montserrat_26, LV_PART_MAIN);

                lv_obj_t *desc = lv_label_create(dialog);
                lv_label_set_text(desc, "Select what to reset:");
                lv_obj_set_style_text_color(desc, lv_color_hex(0xAAAAAA), LV_PART_MAIN);
                lv_obj_set_style_text_font(desc, &lv_font_montserrat_22, LV_PART_MAIN);

                // Checkbox: Device Settings
                lv_obj_t *cb_settings = lv_checkbox_create(dialog);
                lv_checkbox_set_text(cb_settings, "Device Settings");
                lv_obj_add_state(cb_settings, LV_STATE_CHECKED);  // default on
                lv_obj_set_style_text_color(cb_settings, lv_color_white(), LV_PART_MAIN);
                lv_obj_set_style_text_font(cb_settings, &lv_font_montserrat_22, LV_PART_MAIN);

                // Checkbox: Meshy Identity + Channels
                lv_obj_t *cb_meshy = lv_checkbox_create(dialog);
                lv_checkbox_set_text(cb_meshy, "Meshy Identity + Channels");
                lv_obj_set_style_text_color(cb_meshy, lv_color_white(), LV_PART_MAIN);
                lv_obj_set_style_text_font(cb_meshy, &lv_font_montserrat_22, LV_PART_MAIN);

                // Checkbox: PKI Keypair
                lv_obj_t *cb_pki = lv_checkbox_create(dialog);
                lv_checkbox_set_text(cb_pki, "PKI Keypair (X25519)");
                lv_obj_set_style_text_color(cb_pki, lv_color_white(), LV_PART_MAIN);
                lv_obj_set_style_text_font(cb_pki, &lv_font_montserrat_22, LV_PART_MAIN);

                // Warning label
                lv_obj_t *warn = lv_label_create(dialog);
                lv_label_set_text(warn, "PKI reset generates a new key.\nOther nodes won't recognize you.");
                lv_obj_set_style_text_color(warn, lv_color_hex(0x996633), LV_PART_MAIN);
                lv_obj_set_style_text_font(warn, &lv_font_montserrat_16, LV_PART_MAIN);
                lv_obj_set_width(warn, 390);

                // Button row
                lv_obj_t *btn_row = lv_obj_create(dialog);
                lv_obj_set_size(btn_row, 400, 55);
                lv_obj_set_style_bg_opa(btn_row, 0, LV_PART_MAIN);
                lv_obj_set_style_border_width(btn_row, 0, LV_PART_MAIN);
                lv_obj_set_style_pad_all(btn_row, 0, LV_PART_MAIN);
                lv_obj_set_flex_flow(btn_row, LV_FLEX_FLOW_ROW);
                lv_obj_set_style_pad_column(btn_row, 15, LV_PART_MAIN);
                lv_obj_set_flex_align(btn_row, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

                // Cancel button
                lv_obj_t *cancel_btn = lv_button_create(btn_row);
                lv_obj_set_size(cancel_btn, 180, 45);
                lv_obj_set_style_bg_color(cancel_btn, lv_color_hex(0x333333), LV_PART_MAIN);
                lv_obj_set_style_radius(cancel_btn, 10, LV_PART_MAIN);
                lv_obj_set_style_shadow_width(cancel_btn, 0, LV_PART_MAIN);
                lv_obj_t *cancel_lbl = lv_label_create(cancel_btn);
                lv_label_set_text(cancel_lbl, "Cancel");
                lv_obj_set_style_text_color(cancel_lbl, lv_color_white(), LV_PART_MAIN);
                lv_obj_set_style_text_font(cancel_lbl, &lv_font_montserrat_22, LV_PART_MAIN);
                lv_obj_center(cancel_lbl);
                lv_obj_set_user_data(cancel_btn, overlay);
                lv_obj_add_event_cb(cancel_btn, [](lv_event_t *e) {
                    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
                    lv_obj_t *ov = (lv_obj_t *)lv_obj_get_user_data(lv_event_get_target_obj(e));
                    if (ov) lv_obj_delete(ov);
                }, LV_EVENT_ALL, nullptr);

                // Confirm button — stores checkbox refs in user_data via static struct
                lv_obj_t *confirm_btn = lv_button_create(btn_row);
                lv_obj_set_size(confirm_btn, 180, 45);
                lv_obj_set_style_bg_color(confirm_btn, lv_color_hex(0x661111), LV_PART_MAIN);
                lv_obj_set_style_radius(confirm_btn, 10, LV_PART_MAIN);
                lv_obj_set_style_shadow_width(confirm_btn, 0, LV_PART_MAIN);
                lv_obj_t *confirm_lbl = lv_label_create(confirm_btn);
                lv_label_set_text(confirm_lbl, "RESET");
                lv_obj_set_style_text_color(confirm_lbl, lv_color_hex(0xff4444), LV_PART_MAIN);
                lv_obj_set_style_text_font(confirm_lbl, &lv_font_montserrat_22, LV_PART_MAIN);
                lv_obj_center(confirm_lbl);

                // Pack refs for the confirm callback
                struct ResetCtx {
                    lv_obj_t *overlay;
                    lv_obj_t *cb_settings;
                    lv_obj_t *cb_meshy;
                    lv_obj_t *cb_pki;
                    System   *self;
                };
                static ResetCtx s_reset_ctx;
                s_reset_ctx = { overlay, cb_settings, cb_meshy, cb_pki, self };
                lv_obj_set_user_data(confirm_btn, &s_reset_ctx);

                lv_obj_add_event_cb(confirm_btn, [](lv_event_t *e) {
                    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
                    ResetCtx *ctx = (ResetCtx *)lv_obj_get_user_data(lv_event_get_target_obj(e));
                    if (!ctx) return;

                    bool reset_settings = lv_obj_has_state(ctx->cb_settings, LV_STATE_CHECKED);
                    bool reset_meshy    = lv_obj_has_state(ctx->cb_meshy, LV_STATE_CHECKED);
                    bool reset_pki      = lv_obj_has_state(ctx->cb_pki, LV_STATE_CHECKED);

                    // 1. Optionally reset device settings
                    if (reset_settings) {
                        settings_reset();
                    }

                    // 2. Optionally reset Meshy identity + channels
                    //    Always erase the NVS blob first (clean slate, clears corruption).
                    //    Then rebuild with only the preserved fields.
                    if (reset_meshy || reset_pki) {
                        // Save PKI if keeping it
                        uint8_t saved_priv[32] = {0}, saved_pub[32] = {0};
                        uint8_t saved_valid = 0;
                        if (!reset_pki && g_meshy_channels.pki_valid) {
                            memcpy(saved_priv, g_meshy_channels.pki_private_key, 32);
                            memcpy(saved_pub, g_meshy_channels.pki_public_key, 32);
                            saved_valid = g_meshy_channels.pki_valid;
                        }

                        // Save identity if keeping it
                        char saved_long[MESHY_LONG_NAME_LEN] = {0};
                        char saved_short[MESHY_SHORT_NAME_LEN] = {0};
                        if (!reset_meshy) {
                            memcpy(saved_long, g_meshy_channels.node_long_name, sizeof(saved_long));
                            memcpy(saved_short, g_meshy_channels.node_short_name, sizeof(saved_short));
                        }

                        // Erase NVS — clean slate
                        nvs_handle_t nvs;
                        if (nvs_open(MESHY_CH_NVS_NAMESPACE, NVS_READWRITE, &nvs) == ESP_OK) {
                            nvs_erase_all(nvs);
                            nvs_commit(nvs);
                            nvs_close(nvs);
                        }

                        // Rebuild in-memory struct from zero
                        memset(&g_meshy_channels, 0, sizeof(g_meshy_channels));

                        // Restore preserved fields
                        if (saved_valid) {
                            memcpy(g_meshy_channels.pki_private_key, saved_priv, 32);
                            memcpy(g_meshy_channels.pki_public_key, saved_pub, 32);
                            g_meshy_channels.pki_valid = saved_valid;
                        }
                        if (!reset_meshy) {
                            memcpy(g_meshy_channels.node_long_name, saved_long, sizeof(saved_long));
                            memcpy(g_meshy_channels.node_short_name, saved_short, sizeof(saved_short));
                        }

                        // Wipe saved private key from stack
                        memset(saved_priv, 0, sizeof(saved_priv));

                        // Write the clean blob back to NVS
                        meshy_channels_save();

                        ESP_LOGI("SETTINGS", "Meshy reset: identity=%s pki=%s (NVS erased + rebuilt)",
                                 reset_meshy ? "cleared" : "kept",
                                 reset_pki ? "cleared" : "kept");
                    }

                    // Clean up and navigate home
                    g_last_touch_ms = esp_log_timestamp();
                    lv_obj_delete(ctx->overlay);
                    lv_display_set_rotation(lv_display_get_default(), ctx->self->_home_rotation);
                    ctx->self->set_vibration();
                    ctx->self->init_win_home();
                    lv_screen_load_anim(ctx->self->_registry.win.home.root, LV_SCR_LOAD_ANIM_FADE_OUT, 100, 0, true);
                }, LV_EVENT_ALL, nullptr);

            }, LV_EVENT_ALL, this);
        }

        // Bottom padding
        {
            lv_obj_t *pad = lv_obj_create(scroll);
            lv_obj_set_size(pad, row_w, 40);
            lv_obj_set_style_bg_opa(pad, 0, (lv_style_selector_t)LV_PART_MAIN);
            lv_obj_set_style_border_width(pad, 0, (lv_style_selector_t)LV_PART_MAIN);
        }

        // Swipe to return home
        lv_obj_add_event_cb(_registry.win.settings.root, [](lv_event_t *e) {
            System *self = static_cast<System *>(lv_event_get_user_data(e));
            if (lv_event_get_code(e) != LV_EVENT_GESTURE) return;
            lv_dir_t dir = lv_indev_get_gesture_dir(lv_indev_active());
            if ((dir == LV_DIR_LEFT || dir == LV_DIR_RIGHT) && self->_edge_touch_flag) {
                settings_save();
                lv_display_set_rotation(lv_display_get_default(), self->_home_rotation);
                self->set_vibration();
                self->init_win_home();
                lv_screen_load_anim(self->_registry.win.home.root, LV_SCR_LOAD_ANIM_FADE_OUT, 100, 0, true);
                self->_edge_touch_flag = false;
            }
        }, LV_EVENT_ALL, this);

        lv_obj_update_layout(_registry.win.settings.root);
        _current_win = Current_Win::SETTINGS;
    }

    void System::init_win_rf(void)
    {
        // 主界面
        _registry.win.rf.root = lv_obj_create(NULL);
        lv_obj_set_style_bg_color(_registry.win.rf.root, lv_color_hex(0xA69CDB), (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_size(_registry.win.rf.root, _width, _height);
        lv_obj_set_scrollbar_mode(_registry.win.rf.root, LV_SCROLLBAR_MODE_OFF);

        // 添加 symbol list 图标
        lv_obj_t *symbol_icon = lv_label_create(_registry.win.rf.root);
        lv_label_set_text(symbol_icon, LV_SYMBOL_LIST);
        lv_obj_set_style_text_color(symbol_icon, lv_color_white(), (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(symbol_icon, &lv_font_montserrat_48, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_align(symbol_icon, LV_ALIGN_TOP_RIGHT, -30, 15 + 50);

        // 设置标签为可点击
        lv_obj_add_flag(symbol_icon, LV_OBJ_FLAG_CLICKABLE);

        lv_obj_add_event_cb(symbol_icon, [](lv_event_t *e)
                            {
                                System *self = static_cast<System *>(lv_event_get_user_data(e));
                                lv_event_code_t code = lv_event_get_code(e);

                                switch (code)
                                {
                                case LV_EVENT_CLICKED:
                                    self->set_rf_status_callback(false);
                                    self->init_win_rf_setings();

                                    lv_screen_load_anim(self->_registry.win.rf.setings.root, LV_SCR_LOAD_ANIM_MOVE_LEFT, 100, 0, true);
                                break;
                                default:
                                break;
                                } }, LV_EVENT_ALL, this);

        // 创建标题
        lv_obj_t *title_label = lv_label_create(_registry.win.rf.root);
        lv_label_set_text(title_label, "Rf");
        lv_obj_set_style_text_color(title_label, lv_color_white(), (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_text_align(title_label, LV_TEXT_ALIGN_LEFT, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(title_label, &lv_font_montserrat_48, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_size(title_label, _width - 300, 40);
        lv_obj_align(title_label, LV_ALIGN_TOP_LEFT, 30, 10 + 50);

        // 创建发送框容器
        _registry.win.rf.send_box_container = lv_obj_create(_registry.win.rf.root);
        lv_obj_set_size(_registry.win.rf.send_box_container, _width, 100);
        lv_obj_align(_registry.win.rf.send_box_container, LV_ALIGN_BOTTOM_MID, 0, 0);
        lv_obj_set_style_bg_color(_registry.win.rf.send_box_container, lv_color_hex(0xEEE9E9), (lv_style_selector_t)LV_PART_MAIN); // 设置背景颜色为灰色
        lv_obj_set_style_radius(_registry.win.rf.send_box_container, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_border_width(_registry.win.rf.send_box_container, 0, (lv_style_selector_t)LV_PART_MAIN); // 移除边框
        lv_obj_remove_flag(_registry.win.rf.send_box_container, LV_OBJ_FLAG_SCROLLABLE);                          // 禁止滚动
        lv_obj_remove_flag(_registry.win.rf.send_box_container, LV_OBJ_FLAG_CLICKABLE);                           // 禁止触摸

        _registry.win.rf.chat_textarea = lv_textarea_create(_registry.win.rf.send_box_container);
        lv_textarea_set_one_line(_registry.win.rf.chat_textarea, true);
        lv_textarea_set_password_mode(_registry.win.rf.chat_textarea, false);
        lv_obj_set_style_pad_top(_registry.win.rf.chat_textarea, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_bottom(_registry.win.rf.chat_textarea, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_left(_registry.win.rf.chat_textarea, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_right(_registry.win.rf.chat_textarea, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        // 设置初始内容为_registry.win.rf.chat_textarea_data
        lv_textarea_set_text(_registry.win.rf.chat_textarea, _registry.win.rf.chat_textarea_data.c_str());
        lv_obj_set_style_text_font(_registry.win.rf.chat_textarea, &lv_font_montserrat_24, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_width(_registry.win.rf.chat_textarea, _width - 20 - 150);
        lv_obj_align(_registry.win.rf.chat_textarea, LV_ALIGN_BOTTOM_LEFT, -20, 20); // 调整位置到底部往上一点点

        lv_obj_t *send_button = lv_button_create(_registry.win.rf.send_box_container);
        lv_obj_set_size(send_button, 120, 55);
        lv_obj_align_to(send_button, _registry.win.rf.chat_textarea, LV_ALIGN_OUT_RIGHT_MID, 15, 0);
        lv_obj_set_style_radius(send_button, 10, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_width(send_button, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_t *label = lv_label_create(send_button);
        lv_label_set_text(label, "send");
        lv_obj_set_style_text_font(label, &lv_font_montserrat_26, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_center(label);

        lv_obj_add_event_cb(send_button, [](lv_event_t *e)
                            {
                                System *self = static_cast<System *>(lv_event_get_user_data(e));
                                lv_event_code_t code = lv_event_get_code(e);

                                if (code == LV_EVENT_CLICKED)
                                {
                                // 获取当前输入框内容
                                std::string text = lv_textarea_get_text(self->_registry.win.rf.chat_textarea);

                                    //如果消息不为空
                                    if (!text.empty())
                                    {
                                        char buffer_time[15];
                                        snprintf(buffer_time, sizeof(buffer_time), "%02d:%02d:%02d", self->_time.hour , self-> _time.minute , self->_time.second);
        
                                        Win_Rf_Chat_Message wlcm =
                                        {
                                            .direction = Chat_Message_Direction::SEND,
                                            .time = buffer_time,
                                            .data = text,
                                        };
                                        self->_registry.win.rf.chat_message_data.push_back(wlcm);

                                        // 清空输入框
                                        lv_textarea_set_text(self->_registry.win.rf.chat_textarea, "");

                                        // 更新聊天容器
                                        self->win_rf_chat_message_data_update(self->_registry.win.rf.chat_message_data);

                                        //发送数据
                                        self->set_rf_send_data_callback(text);
                                    }
                                } }, LV_EVENT_ALL, this);

#if defined CONFIG_BOARD_TYPE_T_DISPLAY_P4
        create_keyboard(_registry.win.rf.root);

        lv_keyboard_set_textarea(_registry.keyboard, _registry.win.rf.chat_textarea);
        lv_textarea_set_accepted_chars(_registry.win.rf.chat_textarea, DEFAULT_KEYBOARD_ALLOWED_CHARS); // 只允许默认字符输入
#endif

        lv_obj_add_event_cb(_registry.win.rf.chat_textarea, [](lv_event_t *e)
                            {
                                System *self = static_cast<System *>(lv_event_get_user_data(e));
                                lv_event_code_t code = lv_event_get_code(e);
                                lv_obj_t *textarea = lv_event_get_target_obj(e);

                                switch (code)
                                {
#if defined CONFIG_BOARD_TYPE_T_DISPLAY_P4
                                case LV_EVENT_FOCUSED:
                                        lv_keyboard_set_textarea(self->_registry.keyboard, textarea);
                                        lv_textarea_set_accepted_chars(textarea, DEFAULT_KEYBOARD_ALLOWED_CHARS); // 只允许默认字符输入
                                        lv_obj_remove_flag(self->_registry.keyboard, LV_OBJ_FLAG_HIDDEN); // 显示键盘
                                        lv_obj_align(self->_registry.keyboard, LV_ALIGN_BOTTOM_MID, 0, 0); // 对齐到屏幕底部

                                        // 调整聊天框的大小
                                        lv_obj_set_size(self->_registry.win.rf.chat_message_container, self->_width, self->_height - 100 - 130 - lv_obj_get_height(self->_registry.keyboard));

                                        // 调整容器位置
                                        lv_obj_align_to(self->_registry.win.rf.send_box_container, self->_registry.keyboard, LV_ALIGN_OUT_TOP_MID, 0, 0);
                                        lv_obj_align_to(self->_registry.win.rf.chat_message_container, self->_registry.win.rf.send_box_container, LV_ALIGN_OUT_TOP_MID, 0, 0);
                                    break;
#elif defined CONFIG_BOARD_TYPE_T_DISPLAY_P4_KEYBOARD
                                case LV_EVENT_PRESSED:
                                    lv_group_remove_all_objs(self->_registry.keyboard_group);
                                    lv_group_add_obj(self->_registry.keyboard_group, textarea);
                                    break;
#else
#error "unknown macro definition, please select the correct macro definition."
#endif
                                // case LV_EVENT_DEFOCUSED:
                                    //     lv_obj_add_flag(self->_registry.keyboard, LV_OBJ_FLAG_HIDDEN); // 隐藏键盘

                                    //     // 调整聊天框的大小
                                    //     lv_obj_set_size(self->_registry.win.rf.chat_message_container, self->_width, self->_height - 100 - 130);
                                    //     // 恢复容器位置
                                    //     lv_obj_align(self->_registry.win.rf.send_box_container, LV_ALIGN_BOTTOM_MID, 0, 0);
                                    //     lv_obj_align_to(self->_registry.win.rf.chat_message_container, self->_registry.win.rf.send_box_container, LV_ALIGN_OUT_TOP_MID, 0, 0);
                                    // break;
                                case LV_EVENT_READY:
                                    {
                                        // printf("Ready, current text: %s", lv_textarea_get_text(textarea));

                                        // lv_obj_add_flag(self->_registry.keyboard, LV_OBJ_FLAG_HIDDEN); // 隐藏键盘

                                        // // 调整聊天框的大小
                                        // lv_obj_set_size(self->_registry.win.rf.chat_message_container, self->_width, self->_height - 100 - 130);
                                        // // 恢复容器位置
                                        // lv_obj_align(self->_registry.win.rf.send_box_container, LV_ALIGN_BOTTOM_MID, 0, 0);
                                        // lv_obj_align_to(self->_registry.win.rf.chat_message_container, self->_registry.win.rf.send_box_container, LV_ALIGN_OUT_TOP_MID, 0, 0);

                                        // 获取当前输入框内容
                                        std::string text = lv_textarea_get_text(textarea);

                                        //如果消息不为空
                                        if (!text.empty())
                                        {
                                            char buffer_time[15];
                                            snprintf(buffer_time, sizeof(buffer_time), "%02d:%02d:%02d", self->_time.hour , self-> _time.minute , self->_time.second);

                                            Win_Rf_Chat_Message wlcm =
                                            {
                                                .direction = Chat_Message_Direction::SEND,
                                                .time = buffer_time,
                                                .data = text,
                                            };
                                            self->_registry.win.rf.chat_message_data.push_back(wlcm);

                                            // 清空输入框
                                            lv_textarea_set_text(textarea, "");

                                            // 更新聊天容器
                                            self->win_rf_chat_message_data_update(self->_registry.win.rf.chat_message_data);

                                            //发送数据
                                            self->set_rf_send_data_callback(text);
                                        }
                                    }
                                    break;

                                default:
                                    break;
                                } }, LV_EVENT_ALL, this);

        // 创建聊天容器
        _registry.win.rf.chat_message_container = lv_obj_create(_registry.win.rf.root);
        lv_obj_set_size(_registry.win.rf.chat_message_container, _width, _height - 100 - 130);
        lv_obj_set_style_bg_color(_registry.win.rf.chat_message_container, lv_color_hex(0xEEE9E9), (lv_style_selector_t)LV_PART_MAIN); // 设置背景颜色为灰
        lv_obj_set_style_radius(_registry.win.rf.chat_message_container, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_border_width(_registry.win.rf.chat_message_container, 0, (lv_style_selector_t)LV_PART_MAIN); // 移除边框
        lv_obj_align_to(_registry.win.rf.chat_message_container, _registry.win.rf.send_box_container, LV_ALIGN_OUT_TOP_MID, 0, 0);
        lv_obj_set_scrollbar_mode(_registry.win.rf.chat_message_container, LV_SCROLLBAR_MODE_ACTIVE);

#if defined CONFIG_BOARD_TYPE_T_DISPLAY_P4
        // 触摸聊天区域时隐藏键盘
        lv_obj_add_event_cb(_registry.win.rf.chat_message_container, [](lv_event_t *e)
                            {
                                System *self = static_cast<System *>(lv_event_get_user_data(e));
                                lv_event_code_t code = lv_event_get_code(e);

                                if (code == LV_EVENT_CLICKED)
                                {
                                lv_obj_add_flag(self->_registry.keyboard, LV_OBJ_FLAG_HIDDEN); // 隐藏键盘

                                // 调整聊天框的大小
                                lv_obj_set_size(self->_registry.win.rf.chat_message_container, self->_width, self->_height - 100 - 130);
                                // 恢复容器位置
                                lv_obj_align(self->_registry.win.rf.send_box_container, LV_ALIGN_BOTTOM_MID, 0, 0);
                                lv_obj_align_to(self->_registry.win.rf.chat_message_container, self->_registry.win.rf.send_box_container, LV_ALIGN_OUT_TOP_MID, 0, 0);
                                } }, LV_EVENT_ALL, this);
#elif defined CONFIG_BOARD_TYPE_T_DISPLAY_P4_KEYBOARD
        lv_obj_add_event_cb(_registry.win.rf.chat_message_container, [](lv_event_t *e)
                            {
                                System *self = static_cast<System *>(lv_event_get_user_data(e));
                                lv_event_code_t code = lv_event_get_code(e);

                                if (code == LV_EVENT_CLICKED)
                                {
                                    lv_group_remove_obj(self->_registry.win.rf.chat_textarea);
                                } }, LV_EVENT_ALL, this);
#endif

        win_rf_chat_message_data_update(_registry.win.rf.chat_message_data);

        lv_obj_add_event_cb(_registry.win.rf.root, [](lv_event_t *e)
                            {
                                System *self = static_cast<System *>(lv_event_get_user_data(e));
                                lv_event_code_t code = lv_event_get_code(e);

                                if (code == LV_EVENT_GESTURE)
                                {
                                    lv_dir_t gesture_dir = lv_indev_get_gesture_dir(lv_indev_active());

                                    // 边缘检测以及左右滑动
                                    if ((gesture_dir == LV_DIR_LEFT || gesture_dir == LV_DIR_RIGHT)&&(self->_edge_touch_flag == true))
                                    {   
                                        self->set_rf_status_callback(false);
                                        self->_registry.win.rf.chat_textarea_data = lv_textarea_get_text(self->_registry.win.rf.chat_textarea);

                                        self->set_vibration();
                                        self->init_win_home();
                                        
                                        lv_screen_load_anim(self->_registry.win.home.root, LV_SCR_LOAD_ANIM_FADE_OUT, 100, 0, true);

                                        self->_edge_touch_flag = false;
                                    }
                                } }, LV_EVENT_ALL, this);

        init_status_bar(_registry.win.rf.root);

        lv_obj_update_layout(_registry.win.rf.root);

        set_rf_status_callback(true);

        _current_win = Current_Win::RF;
    }

    void System::win_rf_chat_message_data_update(std::vector<Win_Rf_Chat_Message> wlcm)
    {
        // 清空 _registry.win.rf.chat_message_container 的所有子对象
        lv_obj_clean(_registry.win.rf.chat_message_container);

        if (wlcm.size() > 100)
        {
            // 删除最早的数据直到只剩下100条
            wlcm.erase(wlcm.begin(), wlcm.begin() + (wlcm.size() - 100));
        }

        lv_obj_t *previous_message_btn = nullptr;
        for (uint8_t i = 0; i < wlcm.size(); i++)
        {
            // 聊天按钮
            lv_obj_t *message_btn = lv_button_create(_registry.win.rf.chat_message_container);
            lv_obj_set_width(message_btn, LV_SIZE_CONTENT);
            lv_obj_set_height(message_btn, LV_SIZE_CONTENT);
            lv_obj_set_style_pad_left(message_btn, 15, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_pad_right(message_btn, 15, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_pad_top(message_btn, 15, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_pad_bottom(message_btn, 15, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_shadow_width(message_btn, 0, LV_PART_MAIN | LV_STATE_DEFAULT); // 去除按钮阴影

            // 时间标签
            lv_obj_t *time_label = lv_label_create(_registry.win.rf.chat_message_container);
            lv_label_set_text(time_label, wlcm[i].time.c_str());
            lv_obj_set_style_text_font(time_label, &lv_font_montserrat_22, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_text_color(time_label, lv_color_hex(0x888888), LV_PART_MAIN | LV_STATE_DEFAULT);

            // 设置按钮颜色和对齐
            if (wlcm[i].direction == Chat_Message_Direction::SEND)
            {
                lv_obj_set_style_bg_color(message_btn, lv_color_hex(0x4A90E2), LV_PART_MAIN | LV_STATE_DEFAULT); // 蓝色
                if (previous_message_btn != nullptr)
                {
                    lv_obj_update_layout(previous_message_btn); // 确保布局更新
                    lv_obj_align(message_btn, LV_ALIGN_TOP_RIGHT, 0, lv_obj_get_y(previous_message_btn) + lv_obj_get_height(previous_message_btn) + 40);
                }
                else
                {
                    lv_obj_align(message_btn, LV_ALIGN_TOP_RIGHT, 0, 20);
                }

                lv_obj_align_to(time_label, message_btn, LV_ALIGN_OUT_TOP_RIGHT, 0, -5);
            }
            else
            {
                // 数据信息标签
                lv_obj_t *data_info_label = lv_label_create(_registry.win.rf.chat_message_container);
                lv_label_set_text(data_info_label, wlcm[i].data_info.c_str());
                lv_obj_set_style_text_font(data_info_label, &lv_font_montserrat_22, LV_PART_MAIN | LV_STATE_DEFAULT);
                lv_obj_set_style_text_color(data_info_label, lv_color_hex(0x888888), LV_PART_MAIN | LV_STATE_DEFAULT);

                lv_obj_set_style_bg_color(message_btn, lv_color_white(), LV_PART_MAIN | LV_STATE_DEFAULT); // 白色

                if (previous_message_btn != nullptr)
                {
                    lv_obj_update_layout(previous_message_btn); // 确保布局更新
                    lv_obj_align(message_btn, LV_ALIGN_TOP_LEFT, 0, lv_obj_get_y(previous_message_btn) + lv_obj_get_height(previous_message_btn) + 70);
                }
                else
                {
                    lv_obj_align(message_btn, LV_ALIGN_TOP_LEFT, 0, 50);
                }

                lv_obj_align_to(data_info_label, message_btn, LV_ALIGN_OUT_TOP_LEFT, 0, -5);
                lv_obj_align_to(time_label, data_info_label, LV_ALIGN_OUT_TOP_LEFT, 0, 0);
            }

            // 判断 wlcm[i].data 的长度并插入换行符
            std::string message_text = wlcm[i].data;
#if defined SCREEN_ROTATION_DIRECTION_0
            if (wlcm[i].data.size() > 15)
            {
                for (size_t pos = 15; pos < message_text.length(); pos += 16)
                {
                    message_text.insert(pos, "\n");
                }
            }
#elif defined SCREEN_ROTATION_DIRECTION_90
            if (wlcm[i].data.size() > 30)
            {
                for (size_t pos = 30; pos < message_text.length(); pos += 31)
                {
                    message_text.insert(pos, "\n");
                }
            }
#else
#error "unknown macro definition, please select the correct macro definition."
#endif

            // 消息内容
            lv_obj_t *message_label = lv_label_create(message_btn);
            lv_label_set_text(message_label, message_text.c_str());
            lv_obj_set_style_text_font(message_label, &lv_font_montserrat_26, LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_text_color(message_label, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT);
            lv_obj_set_style_text_align(message_label, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN | LV_STATE_DEFAULT);

            previous_message_btn = message_btn;
        }

        lv_obj_update_layout(_registry.win.rf.chat_message_container); // 确保布局更新
        // 滚动到聊天容器的最底部
        lv_obj_scroll_to_y(_registry.win.rf.chat_message_container,
                           lv_obj_get_scroll_bottom(_registry.win.rf.chat_message_container), LV_ANIM_OFF);
    }

    void System::init_win_rf_setings(void)
    {
        // 主界面
        _registry.win.rf.setings.root = lv_obj_create(NULL);
        lv_obj_set_style_bg_color(_registry.win.rf.setings.root, lv_color_hex(0xA69CDB), (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_size(_registry.win.rf.setings.root, _width, _height);
        lv_obj_set_scrollbar_mode(_registry.win.rf.setings.root, LV_SCROLLBAR_MODE_OFF);

        // 添加 symbol 退出图标
        lv_obj_t *symbol_icon = lv_label_create(_registry.win.rf.setings.root);
        lv_label_set_text(symbol_icon, LV_SYMBOL_LEFT);
        lv_obj_set_style_text_color(symbol_icon, lv_color_white(), (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(symbol_icon, &lv_font_montserrat_48, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_align(symbol_icon, LV_ALIGN_TOP_LEFT, 30, 15 + 50);

        // 设置标签为可点击
        lv_obj_add_flag(symbol_icon, LV_OBJ_FLAG_CLICKABLE);

        lv_obj_add_event_cb(symbol_icon, [](lv_event_t *e)
                            {
                                System *self = static_cast<System *>(lv_event_get_user_data(e));
                                lv_event_code_t code = lv_event_get_code(e);

                                switch (code)
                                {
                                case LV_EVENT_CLICKED:
                                    self->init_win_rf();

                                    lv_screen_load_anim(self->_registry.win.rf.root, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 100, 0, true);
                                break;
                                default:
                                break;
                            } }, LV_EVENT_ALL, this);

        // 创建标题
        lv_obj_t *title_label = lv_label_create(_registry.win.rf.setings.root);
        lv_label_set_text(title_label, "Rf Setings");
        lv_obj_set_style_text_color(title_label, lv_color_white(), (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_text_align(title_label, LV_TEXT_ALIGN_LEFT, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(title_label, &lv_font_montserrat_48, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_size(title_label, _width - 200, 40);
        lv_obj_align(title_label, LV_ALIGN_TOP_LEFT, 20 + 90, 10 + 50);

        // 创建列表
        lv_obj_t *list = lv_list_create(_registry.win.rf.setings.root);
        lv_obj_set_size(list, _width, _height - 130);
        lv_obj_align(list, LV_ALIGN_BOTTOM_MID, 0, 0);
        lv_obj_set_style_pad_left(list, 20, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_right(list, 20, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(list, &lv_font_montserrat_24, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_border_width(list, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_radius(list, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_scrollbar_mode(list, LV_SCROLLBAR_MODE_ACTIVE);

        lv_obj_t *list_button_rf_chip_type = lv_list_add_button(list, NULL, "rf chip type");
        lv_obj_set_style_text_font(list_button_rf_chip_type, &lv_font_montserrat_30, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);

        lv_obj_add_event_cb(list_button_rf_chip_type, [](lv_event_t *e)
                            {
                                System *self = static_cast<System *>(lv_event_get_user_data(e));
                                lv_event_code_t code = lv_event_get_code(e);

                                switch (code)
                                {
                                case LV_EVENT_CLICKED:
                                {
                                    self->init_win_rf_setings_rf_chip_type_message_box();
                                }
                                    break;
                                default:
                                    break;
                            } }, LV_EVENT_ALL, this);

        lv_obj_t *list_button_config_rf_params = lv_list_add_button(list, NULL, "config rf params");
        lv_obj_set_style_text_font(list_button_config_rf_params, &lv_font_montserrat_30, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);

        lv_obj_add_event_cb(list_button_config_rf_params, [](lv_event_t *e)
                            {
                                System *self = static_cast<System *>(lv_event_get_user_data(e));
                                lv_event_code_t code = lv_event_get_code(e);

                                switch (code)
                                {
                                case LV_EVENT_CLICKED:
                                {
                                    switch (self->_rf_chip_type)
                                    {
                                    case Rf_Chip_Type::SX1262:
                                        self->init_win_rf_setings_config_sx1262_params_message_box();
                                        break;
#if defined CONFIG_BOARD_TYPE_T_DISPLAY_P4_KEYBOARD
                                    case Rf_Chip_Type::CC1101:
                                        self->init_win_rf_setings_config_cc1101_params_message_box();
                                        break;
                                    case Rf_Chip_Type::NRF24L01:
                                        self->init_win_rf_setings_config_nrf24l01_params_message_box();
                                        break;
#endif
                                    
                                    default:
                                        break;
                                    }

                                }
                                    break;
                                default:
                                    break;
                            } }, LV_EVENT_ALL, this);

        lv_obj_t *list_button_auto_send = lv_list_add_button(list, NULL, "auto send");
        lv_obj_set_style_text_font(list_button_auto_send, &lv_font_montserrat_30, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);

        lv_obj_add_event_cb(list_button_auto_send, [](lv_event_t *e)
                            {
                                System *self = static_cast<System *>(lv_event_get_user_data(e));
                                lv_event_code_t code = lv_event_get_code(e);

                                switch (code)
                                {
                                case LV_EVENT_CLICKED:
                                {
                                    self->init_win_rf_setings_auto_send_message_box();
                                }
                                    break;
                                default:
                                    break;
                            } }, LV_EVENT_ALL, this);

        // // 创建开关并添加到列表项中
        // lv_obj_t *sw = lv_switch_create(list_button);
        // lv_obj_set_size(sw, 80, 50);

        lv_obj_add_event_cb(_registry.win.rf.setings.root, [](lv_event_t *e)
                            {
                                System *self = static_cast<System *>(lv_event_get_user_data(e));
                                lv_event_code_t code = lv_event_get_code(e);

                                if (code == LV_EVENT_GESTURE)
                                {
                                    lv_dir_t gesture_dir = lv_indev_get_gesture_dir(lv_indev_active());

                                    // 边缘检测以及左右滑动
                                    if ((gesture_dir == LV_DIR_LEFT || gesture_dir == LV_DIR_RIGHT)&&(self->_edge_touch_flag == true))
                                    {   
                                        self->set_vibration();
                                        self->init_win_rf();
                                        
                                        lv_screen_load_anim(self->_registry.win.rf.root, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 100, 0, true);

                                        self->_edge_touch_flag = false;
                                    }
                                } }, LV_EVENT_ALL, this);

        init_status_bar(_registry.win.rf.setings.root);

        lv_obj_update_layout(_registry.win.rf.setings.root);

        _current_win = Current_Win::RF_SETINGS;
    }

    void System::init_win_rf_setings_keyboard_position_event_cb(lv_obj_t *parent)
    {
        lv_obj_add_event_cb(parent, [](lv_event_t *e)
                            {
                                System *self = static_cast<System *>(lv_event_get_user_data(e));
                                lv_event_code_t code = lv_event_get_code(e);
                                lv_obj_t *textarea = lv_event_get_target_obj(e); // 直接获取目标文本区域

#if defined CONFIG_BOARD_TYPE_T_DISPLAY_P4
                                switch (code)
                                {
                                case LV_EVENT_FOCUSED:
                                    lv_keyboard_set_textarea(self->_registry.keyboard, textarea);
                                    lv_textarea_set_accepted_chars(textarea, DEFAULT_KEYBOARD_ALLOWED_CHARS); // 只允许默认字符输入
                                    lv_obj_remove_flag(self->_registry.keyboard, LV_OBJ_FLAG_HIDDEN);         // 显示键盘
                                    lv_obj_align(self->_registry.keyboard, LV_ALIGN_BOTTOM_MID, 0, 0);        // 对齐到屏幕底部

                                    // 调整消息框的大小
                                    lv_obj_set_size(self->_registry.win.rf.setings.message_box.root_container, 450, 800);
                                    lv_obj_align_to(self->_registry.win.rf.setings.message_box.root_container, self->_registry.keyboard, LV_ALIGN_OUT_TOP_MID, 0, 0);
                                    lv_obj_align(self->_registry.win.rf.setings.message_box.btn_container, LV_ALIGN_BOTTOM_MID, 0, 0);
                                    lv_obj_set_size(self->_registry.win.rf.setings.message_box.parameter_container, 450, 680);
                                    lv_obj_align_to(self->_registry.win.rf.setings.message_box.parameter_container, self->_registry.win.rf.setings.message_box.btn_container, LV_ALIGN_OUT_TOP_MID, 0, 0);

                                    break;

                                // case LV_EVENT_DEFOCUSED:
                                //         lv_obj_add_flag(self->_registry.keyboard, LV_OBJ_FLAG_HIDDEN); // 隐藏键盘

                                //     break;
                                case LV_EVENT_READY:
                                    // printf("Ready, current text: %s", lv_textarea_get_text(textarea));

                                    lv_obj_add_flag(self->_registry.keyboard, LV_OBJ_FLAG_HIDDEN); // 隐藏键盘

                                    // 调整聊天框的大小
                                    lv_obj_set_size(self->_registry.win.rf.setings.message_box.root_container, 450, self->_height * 0.7);
                                    lv_obj_center(self->_registry.win.rf.setings.message_box.root_container);

                                    break;

                                default:
                                    break;
                                }
#elif defined CONFIG_BOARD_TYPE_T_DISPLAY_P4_KEYBOARD
                                switch (code)
                                {
                                case LV_EVENT_PRESSED:
                                    lv_group_remove_all_objs(self->_registry.keyboard_group);
                                    lv_group_add_obj(self->_registry.keyboard_group, textarea);
                                    break;
                                case LV_EVENT_READY:
                                    // printf("Ready, current text: %s", lv_textarea_get_text(textarea));

                                    lv_group_remove_obj(textarea);
                                    break;

                                default:
                                    break;
                                }
#else
#error "unknown macro definition, please select the correct macro definition."
#endif
                            },
                            LV_EVENT_ALL, this);
    }

    void System::init_win_rf_setings_rf_chip_type_message_box(void)
    {
        // 创建全屏灰色透明遮罩
        _registry.win.rf.setings.message_box.root = lv_obj_create(_registry.win.rf.setings.root);
        lv_obj_set_style_pad_top(_registry.win.rf.setings.message_box.root, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_bottom(_registry.win.rf.setings.message_box.root, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_left(_registry.win.rf.setings.message_box.root, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_right(_registry.win.rf.setings.message_box.root, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_size(_registry.win.rf.setings.message_box.root, _width, _height);
        lv_obj_set_style_bg_color(_registry.win.rf.setings.message_box.root, lv_color_black(), LV_PART_MAIN);
        lv_obj_set_style_bg_opa(_registry.win.rf.setings.message_box.root, LV_OPA_30, LV_PART_MAIN);
        lv_obj_set_style_border_width(_registry.win.rf.setings.message_box.root, 0, LV_PART_MAIN);
        lv_obj_set_style_radius(_registry.win.rf.setings.message_box.root, 0, LV_PART_MAIN);
        lv_obj_align(_registry.win.rf.setings.message_box.root, LV_ALIGN_CENTER, 0, 0);
        lv_obj_add_flag(_registry.win.rf.setings.message_box.root, LV_OBJ_FLAG_CLICKABLE); // 添加触摸标志来禁止其他界面触摸

        _registry.win.rf.setings.message_box.root_container = lv_obj_create(_registry.win.rf.setings.message_box.root);
        lv_obj_set_style_pad_top(_registry.win.rf.setings.message_box.root_container, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_bottom(_registry.win.rf.setings.message_box.root_container, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_left(_registry.win.rf.setings.message_box.root_container, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_right(_registry.win.rf.setings.message_box.root_container, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_size(_registry.win.rf.setings.message_box.root_container, 450, _height * 0.7);
        lv_obj_set_style_radius(_registry.win.rf.setings.message_box.root_container, 15, LV_PART_MAIN);
        lv_obj_set_style_bg_color(_registry.win.rf.setings.message_box.root_container, lv_color_white(), LV_PART_MAIN);
        lv_obj_set_style_border_width(_registry.win.rf.setings.message_box.root_container, 0, LV_PART_MAIN);
        lv_obj_set_style_shadow_width(_registry.win.rf.setings.message_box.root_container, 16, LV_PART_MAIN);
        lv_obj_center(_registry.win.rf.setings.message_box.root_container);

        _registry.win.rf.setings.message_box.btn_container = lv_obj_create(_registry.win.rf.setings.message_box.root_container);
        lv_obj_set_style_pad_top(_registry.win.rf.setings.message_box.btn_container, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_bottom(_registry.win.rf.setings.message_box.btn_container, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_left(_registry.win.rf.setings.message_box.btn_container, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_right(_registry.win.rf.setings.message_box.btn_container, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_size(_registry.win.rf.setings.message_box.btn_container, 450, 110);
        lv_obj_set_style_border_width(_registry.win.rf.setings.message_box.btn_container, 0, (lv_style_selector_t)LV_PART_MAIN); // 移除边框
        lv_obj_align(_registry.win.rf.setings.message_box.btn_container, LV_ALIGN_BOTTOM_MID, 0, 0);

        lv_obj_t *btn_cancel = lv_button_create(_registry.win.rf.setings.message_box.btn_container);
        lv_obj_set_size(btn_cancel, 150, 60);
        lv_obj_align(btn_cancel, LV_ALIGN_BOTTOM_LEFT, 40, -30);
        lv_obj_set_style_radius(btn_cancel, 10, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_width(btn_cancel, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT); // 去除按钮阴影
        lv_obj_t *btn_cancel_label = lv_label_create(btn_cancel);
        lv_label_set_text(btn_cancel_label, "cancel");
        lv_obj_set_style_text_font(btn_cancel_label, &lv_font_montserrat_26, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_center(btn_cancel_label);

        // cancel 按钮回调
        lv_obj_add_event_cb(btn_cancel, [](lv_event_t *e)
                            {
                                System *self = static_cast<System *>(lv_event_get_user_data(e));
                                lv_obj_delete(self->_registry.win.rf.setings.message_box.root); }, LV_EVENT_CLICKED, this);

        lv_obj_t *btn_apply = lv_button_create(_registry.win.rf.setings.message_box.btn_container);
        lv_obj_set_size(btn_apply, 150, 60);
        lv_obj_align(btn_apply, LV_ALIGN_BOTTOM_RIGHT, -40, -30);
        lv_obj_set_style_radius(btn_apply, 10, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_width(btn_apply, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT); // 去除按钮阴影
        lv_obj_t *btn_apply_label = lv_label_create(btn_apply);
        lv_label_set_text(btn_apply_label, "apply");
        lv_obj_set_style_text_font(btn_apply_label, &lv_font_montserrat_26, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_center(btn_apply_label);

        // apply 按钮回调
        lv_obj_add_event_cb(btn_apply, [](lv_event_t *e)
                            {
                                System *self = static_cast<System *>(lv_event_get_user_data(e));

                                self->_rf_chip_type = static_cast<Rf_Chip_Type>(
                                                            lv_dropdown_get_selected(self->_registry.win.rf.setings.rf_chip_type.dropdown.rf_chip));

                                lv_obj_delete(self->_registry.win.rf.setings.message_box.root); }, LV_EVENT_CLICKED, this);

#if defined CONFIG_BOARD_TYPE_T_DISPLAY_P4
        create_keyboard(_registry.win.rf.setings.message_box.root);
#endif

        _registry.win.rf.setings.message_box.parameter_container = lv_obj_create(_registry.win.rf.setings.message_box.root_container);
        lv_obj_set_style_pad_top(_registry.win.rf.setings.message_box.parameter_container, 30, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_bottom(_registry.win.rf.setings.message_box.parameter_container, 30, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_left(_registry.win.rf.setings.message_box.parameter_container, 30, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_right(_registry.win.rf.setings.message_box.parameter_container, 30, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
#if defined SCREEN_ROTATION_DIRECTION_0
        lv_obj_set_size(_registry.win.rf.setings.message_box.parameter_container, 450, _height * 0.6);
#elif defined SCREEN_ROTATION_DIRECTION_90
        lv_obj_set_size(_registry.win.rf.setings.message_box.parameter_container, 450, _height * 0.5);
#else
#error "unknown macro definition, please select the correct macro definition."
#endif
        lv_obj_set_style_border_width(_registry.win.rf.setings.message_box.parameter_container, 0, (lv_style_selector_t)LV_PART_MAIN); // 移除边框
        lv_obj_set_scrollbar_mode(_registry.win.rf.setings.message_box.parameter_container, LV_SCROLLBAR_MODE_ACTIVE);
        lv_obj_align_to(_registry.win.rf.setings.message_box.parameter_container, _registry.win.rf.setings.message_box.btn_container, LV_ALIGN_OUT_TOP_MID, 0, 0);

        // 触摸设置消息框区域时隐藏键盘
        lv_obj_add_event_cb(_registry.win.rf.setings.message_box.parameter_container, [](lv_event_t *e)
                            {
                                System *self = static_cast<System *>(lv_event_get_user_data(e));
                                lv_event_code_t code = lv_event_get_code(e);

                                if (code == LV_EVENT_CLICKED)
                                {
#if defined CONFIG_BOARD_TYPE_T_DISPLAY_P4
                                    lv_obj_add_flag(self->_registry.keyboard, LV_OBJ_FLAG_HIDDEN); // 隐藏键盘

                                    // 调整聊天框的大小
                                    lv_obj_set_size(self->_registry.win.rf.setings.message_box.root_container, 450, self->_height * 0.7);
                                    lv_obj_center(self->_registry.win.rf.setings.message_box.root_container);
                                    lv_obj_align(self->_registry.win.rf.setings.message_box.btn_container, LV_ALIGN_BOTTOM_MID, 0, 0);
                                    lv_obj_set_size(self->_registry.win.rf.setings.message_box.parameter_container, 450, self->_height * 0.6);
                                    lv_obj_align_to(self->_registry.win.rf.setings.message_box.parameter_container, self->_registry.win.rf.setings.message_box.btn_container, LV_ALIGN_OUT_TOP_MID, 0, 0);
#elif defined CONFIG_BOARD_TYPE_T_DISPLAY_P4_KEYBOARD
                                    lv_group_remove_all_objs(self->_registry.keyboard_group);
#else
#error "unknown macro definition, please select the correct macro definition."
#endif
                                } }, LV_EVENT_ALL, this);

        lv_obj_t *msgbox_rf_chip_text = lv_label_create(_registry.win.rf.setings.message_box.parameter_container);
        lv_label_set_text(msgbox_rf_chip_text, "rf chip");
        lv_obj_set_size(msgbox_rf_chip_text, 300, 40);
        lv_obj_set_style_text_font(msgbox_rf_chip_text, &lv_font_montserrat_26, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_align(msgbox_rf_chip_text, LV_ALIGN_TOP_LEFT, 0, 0);

        _registry.win.rf.setings.rf_chip_type.dropdown.rf_chip = lv_dropdown_create(_registry.win.rf.setings.message_box.parameter_container);
        lv_dropdown_set_dir(_registry.win.rf.setings.rf_chip_type.dropdown.rf_chip, LV_DIR_BOTTOM);
#if defined CONFIG_BOARD_TYPE_T_DISPLAY_P4
        lv_dropdown_set_options(_registry.win.rf.setings.rf_chip_type.dropdown.rf_chip, "sx1262");
#elif defined CONFIG_BOARD_TYPE_T_DISPLAY_P4_KEYBOARD
        lv_dropdown_set_options(_registry.win.rf.setings.rf_chip_type.dropdown.rf_chip, "sx1262\n"
                                                                                        "cc1101\n"
                                                                                        "nrf24l01");
#else
#error "unknown macro definition, please select the correct macro definition."
#endif
        lv_dropdown_set_selected(_registry.win.rf.setings.rf_chip_type.dropdown.rf_chip, static_cast<uint32_t>(_rf_chip_type));
        lv_obj_set_style_pad_top(_registry.win.rf.setings.rf_chip_type.dropdown.rf_chip, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_bottom(_registry.win.rf.setings.rf_chip_type.dropdown.rf_chip, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_left(_registry.win.rf.setings.rf_chip_type.dropdown.rf_chip, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_right(_registry.win.rf.setings.rf_chip_type.dropdown.rf_chip, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_min_width(_registry.win.rf.setings.rf_chip_type.dropdown.rf_chip, 200, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_min_height(_registry.win.rf.setings.rf_chip_type.dropdown.rf_chip, 30, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(_registry.win.rf.setings.rf_chip_type.dropdown.rf_chip, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);  // 输入框字体
        lv_obj_set_style_text_font(_registry.win.rf.setings.rf_chip_type.dropdown.rf_chip, &lv_font_montserrat_24, LV_PART_ITEMS | LV_STATE_DEFAULT); // 下拉列表字体
        lv_obj_align_to(_registry.win.rf.setings.rf_chip_type.dropdown.rf_chip, msgbox_rf_chip_text, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);

        lv_obj_add_event_cb(_registry.win.rf.setings.rf_chip_type.dropdown.rf_chip, [](lv_event_t *e)
                            {
                                lv_obj_t *dropdown = lv_event_get_target_obj(e);
                                lv_event_code_t code = lv_event_get_code(e);
                                
                                if (code == LV_EVENT_CLICKED) 
                                {
                                    // // 强制下拉列表向下打开
                                    // lv_dropdown_set_dir(dropdown, LV_DIR_BOTTOM);
                                    // 获取弹出的下拉列表对象
                                    lv_obj_t *list = lv_dropdown_get_list(dropdown);
                                    // // 设置下拉列表最多显示100高度
                                    // lv_obj_set_height(list, 300);

                                    lv_obj_set_style_bg_color(list, lv_color_hex(0xEEE9E9), LV_PART_MAIN | LV_STATE_DEFAULT);
                                    lv_obj_set_scrollbar_mode(list, LV_SCROLLBAR_MODE_ACTIVE);
                                    lv_obj_set_style_border_width(list, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
                                    
                                    lv_obj_set_style_text_font(list, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
                                    lv_obj_set_style_text_font(list, &lv_font_montserrat_24, LV_PART_ITEMS | LV_STATE_DEFAULT);
                                } }, LV_EVENT_ALL, NULL);
    }

    void System::init_win_rf_setings_config_sx1262_params_message_box(void)
    {
        // 创建全屏灰色透明遮罩
        _registry.win.rf.setings.message_box.root = lv_obj_create(_registry.win.rf.setings.root);
        lv_obj_set_style_pad_top(_registry.win.rf.setings.message_box.root, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_bottom(_registry.win.rf.setings.message_box.root, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_left(_registry.win.rf.setings.message_box.root, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_right(_registry.win.rf.setings.message_box.root, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_size(_registry.win.rf.setings.message_box.root, _width, _height);
        lv_obj_set_style_bg_color(_registry.win.rf.setings.message_box.root, lv_color_black(), LV_PART_MAIN);
        lv_obj_set_style_bg_opa(_registry.win.rf.setings.message_box.root, LV_OPA_30, LV_PART_MAIN);
        lv_obj_set_style_border_width(_registry.win.rf.setings.message_box.root, 0, LV_PART_MAIN);
        lv_obj_set_style_radius(_registry.win.rf.setings.message_box.root, 0, LV_PART_MAIN);
        lv_obj_align(_registry.win.rf.setings.message_box.root, LV_ALIGN_CENTER, 0, 0);
        lv_obj_add_flag(_registry.win.rf.setings.message_box.root, LV_OBJ_FLAG_CLICKABLE); // 添加触摸标志来禁止其他界面触摸

        _registry.win.rf.setings.message_box.root_container = lv_obj_create(_registry.win.rf.setings.message_box.root);
        lv_obj_set_style_pad_top(_registry.win.rf.setings.message_box.root_container, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_bottom(_registry.win.rf.setings.message_box.root_container, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_left(_registry.win.rf.setings.message_box.root_container, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_right(_registry.win.rf.setings.message_box.root_container, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_size(_registry.win.rf.setings.message_box.root_container, 450, _height * 0.7);
        lv_obj_set_style_radius(_registry.win.rf.setings.message_box.root_container, 15, LV_PART_MAIN);
        lv_obj_set_style_bg_color(_registry.win.rf.setings.message_box.root_container, lv_color_white(), LV_PART_MAIN);
        lv_obj_set_style_border_width(_registry.win.rf.setings.message_box.root_container, 0, LV_PART_MAIN);
        lv_obj_set_style_shadow_width(_registry.win.rf.setings.message_box.root_container, 16, LV_PART_MAIN);
        lv_obj_center(_registry.win.rf.setings.message_box.root_container);

        _registry.win.rf.setings.message_box.btn_container = lv_obj_create(_registry.win.rf.setings.message_box.root_container);
        lv_obj_set_style_pad_top(_registry.win.rf.setings.message_box.btn_container, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_bottom(_registry.win.rf.setings.message_box.btn_container, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_left(_registry.win.rf.setings.message_box.btn_container, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_right(_registry.win.rf.setings.message_box.btn_container, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_size(_registry.win.rf.setings.message_box.btn_container, 450, 110);
        lv_obj_set_style_border_width(_registry.win.rf.setings.message_box.btn_container, 0, (lv_style_selector_t)LV_PART_MAIN); // 移除边框
        lv_obj_align(_registry.win.rf.setings.message_box.btn_container, LV_ALIGN_BOTTOM_MID, 0, 0);

        lv_obj_t *btn_cancel = lv_button_create(_registry.win.rf.setings.message_box.btn_container);
        lv_obj_set_size(btn_cancel, 150, 60);
        lv_obj_align(btn_cancel, LV_ALIGN_BOTTOM_LEFT, 40, -30);
        lv_obj_set_style_radius(btn_cancel, 10, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_width(btn_cancel, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT); // 去除按钮阴影
        lv_obj_t *btn_cancel_label = lv_label_create(btn_cancel);
        lv_label_set_text(btn_cancel_label, "cancel");
        lv_obj_set_style_text_font(btn_cancel_label, &lv_font_montserrat_26, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_center(btn_cancel_label);

        // cancel 按钮回调
        lv_obj_add_event_cb(btn_cancel, [](lv_event_t *e)
                            {
                                System *self = static_cast<System *>(lv_event_get_user_data(e));
                                lv_obj_delete(self->_registry.win.rf.setings.message_box.root); }, LV_EVENT_CLICKED, this);

        lv_obj_t *btn_apply = lv_button_create(_registry.win.rf.setings.message_box.btn_container);
        lv_obj_set_size(btn_apply, 150, 60);
        lv_obj_align(btn_apply, LV_ALIGN_BOTTOM_RIGHT, -40, -30);
        lv_obj_set_style_radius(btn_apply, 10, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_width(btn_apply, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT); // 去除按钮阴影
        lv_obj_t *btn_apply_label = lv_label_create(btn_apply);
        lv_label_set_text(btn_apply_label, "apply");
        lv_obj_set_style_text_font(btn_apply_label, &lv_font_montserrat_26, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_center(btn_apply_label);

        // apply 按钮回调
        lv_obj_add_event_cb(btn_apply, [](lv_event_t *e)
                            {
                                System *self = static_cast<System *>(lv_event_get_user_data(e));

                                Device_Sx1262 ds;

                                ds.params.rf_switch = static_cast<bool>(lv_dropdown_get_selected(self->_registry.win.rf.setings.config_rf_params.sx1262.dropdown.rf_switch));

                                const char* freq_text = lv_textarea_get_text(self->_registry.win.rf.setings.config_rf_params.sx1262.textarea.freq);
                                if (freq_text != nullptr && freq_text[0] != '\0') // 同时检查NULL和空字符串
                                {  
                                    double buffer = std::stod(freq_text, nullptr);  

                                    // 限制范围
                                    if(buffer <= 150.0)
                                    {
                                        ds.params.freq = 150.0;
                                    }
                                    else if(buffer <= 960.0)
                                    {
                                        ds.params.freq = buffer;
                                    }
                                    else
                                    {
                                        ds.params.freq = 960.0;
                                    }
                                }

                                uint32_t bandwidth_buffer_index = lv_dropdown_get_selected(self->_registry.win.rf.setings.config_rf_params.sx1262.dropdown.bandwidth);
                                if(bandwidth_buffer_index > 6)
                                {
                                    bandwidth_buffer_index++;
                                }
                                ds.params.bandwidth = static_cast<Sx126x::Lora_Bw>(bandwidth_buffer_index);

                                const char* current_limit_text = lv_textarea_get_text(self->_registry.win.rf.setings.config_rf_params.sx1262.textarea.current_limit);
                                if (current_limit_text != nullptr && current_limit_text[0] != '\0') // 同时检查NULL和空字符串
                                {  
                                    float buffer = std::stof(current_limit_text, nullptr);  

                                    // 限制范围
                                    if(buffer <= 0)
                                    {
                                        ds.params.current_limit = 0;
                                    }
                                    else if(buffer <= 140.0)
                                    {
                                        ds.params.current_limit = buffer;
                                    }
                                    else
                                    {
                                        ds.params.current_limit = 140.0;
                                    }
                                }

                                const char* power_text = lv_textarea_get_text(self->_registry.win.rf.setings.config_rf_params.sx1262.textarea.power);
                                if (power_text != nullptr && power_text[0] != '\0') // 同时检查NULL和空字符串
                                {  
                                    int8_t buffer = std::stoi(power_text);  

                                    // 限制范围
                                    if(buffer <= -9)
                                    {
                                        ds.params.power = -9;
                                    }
                                    else if(buffer <= 22)
                                    {
                                        ds.params.power = buffer;
                                    }
                                    else
                                    {
                                        ds.params.power = 22;
                                    }
                                }

                                ds.params.sf = static_cast<Sx126x::Sf>(
                                                            lv_dropdown_get_selected(self->_registry.win.rf.setings.config_rf_params.sx1262.dropdown.spreading_factor) + 5);

                                ds.params.cr = static_cast<Sx126x::Cr>(
                                                            lv_dropdown_get_selected(self->_registry.win.rf.setings.config_rf_params.sx1262.dropdown.coding_rate) + 1);

                                ds.params.crc_type = static_cast<Sx126x::Lora_Crc_Type>(
                                                            lv_dropdown_get_selected(self->_registry.win.rf.setings.config_rf_params.sx1262.dropdown.crc_type));

                                const char* preamble_length_text = lv_textarea_get_text(self->_registry.win.rf.setings.config_rf_params.sx1262.textarea.preamble_length);
                                if (preamble_length_text != nullptr && preamble_length_text[0] != '\0') // 同时检查NULL和空字符串
                                {  
                                    ds.params.preamble_length = std::stoi(preamble_length_text);  
                                }

                                const char* sync_word_text = lv_textarea_get_text(self->_registry.win.rf.setings.config_rf_params.sx1262.textarea.sync_word);
                                if (sync_word_text != nullptr && sync_word_text[0] != '\0') // 同时检查NULL和空字符串
                                {  
                                    ds.params.sync_word = std::stoi(sync_word_text);  
                                }

                                if(self->set_config_rf_params(ds) == true)
                                {
                                    self->_device_sx1262.params = ds.params;
                                }

                                lv_obj_delete(self->_registry.win.rf.setings.message_box.root); }, LV_EVENT_CLICKED, this);

#if defined CONFIG_BOARD_TYPE_T_DISPLAY_P4
        create_keyboard(_registry.win.rf.setings.message_box.root);
#endif

        _registry.win.rf.setings.message_box.parameter_container = lv_obj_create(_registry.win.rf.setings.message_box.root_container);
        lv_obj_set_style_pad_top(_registry.win.rf.setings.message_box.parameter_container, 30, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_bottom(_registry.win.rf.setings.message_box.parameter_container, 30, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_left(_registry.win.rf.setings.message_box.parameter_container, 30, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_right(_registry.win.rf.setings.message_box.parameter_container, 30, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
#if defined SCREEN_ROTATION_DIRECTION_0
        lv_obj_set_size(_registry.win.rf.setings.message_box.parameter_container, 450, _height * 0.6);
#elif defined SCREEN_ROTATION_DIRECTION_90
        lv_obj_set_size(_registry.win.rf.setings.message_box.parameter_container, 450, _height * 0.5);
#else
#error "unknown macro definition, please select the correct macro definition."
#endif
        lv_obj_set_style_border_width(_registry.win.rf.setings.message_box.parameter_container, 0, (lv_style_selector_t)LV_PART_MAIN); // 移除边框
        lv_obj_set_scrollbar_mode(_registry.win.rf.setings.message_box.parameter_container, LV_SCROLLBAR_MODE_ACTIVE);
        lv_obj_align_to(_registry.win.rf.setings.message_box.parameter_container, _registry.win.rf.setings.message_box.btn_container, LV_ALIGN_OUT_TOP_MID, 0, 0);

        // 触摸设置消息框区域时隐藏键盘
        lv_obj_add_event_cb(_registry.win.rf.setings.message_box.parameter_container, [](lv_event_t *e)
                            {
                                System *self = static_cast<System *>(lv_event_get_user_data(e));
                                lv_event_code_t code = lv_event_get_code(e);

                                if (code == LV_EVENT_CLICKED)
                                {
#if defined CONFIG_BOARD_TYPE_T_DISPLAY_P4
                                    lv_obj_add_flag(self->_registry.keyboard, LV_OBJ_FLAG_HIDDEN); // 隐藏键盘

                                    // 调整聊天框的大小
                                    lv_obj_set_size(self->_registry.win.rf.setings.message_box.root_container, 450, self->_height * 0.7);
                                    lv_obj_center(self->_registry.win.rf.setings.message_box.root_container);
                                    lv_obj_align(self->_registry.win.rf.setings.message_box.btn_container, LV_ALIGN_BOTTOM_MID, 0, 0);
                                    lv_obj_set_size(self->_registry.win.rf.setings.message_box.parameter_container, 450, self->_height * 0.6);
                                    lv_obj_align_to(self->_registry.win.rf.setings.message_box.parameter_container, self->_registry.win.rf.setings.message_box.btn_container, LV_ALIGN_OUT_TOP_MID, 0, 0);
#elif defined CONFIG_BOARD_TYPE_T_DISPLAY_P4_KEYBOARD
                                    lv_group_remove_all_objs(self->_registry.keyboard_group);
#else
#error "unknown macro definition, please select the correct macro definition."
#endif
                                } }, LV_EVENT_ALL, this);

        lv_obj_t *msgbox_rf_switch_text = lv_label_create(_registry.win.rf.setings.message_box.parameter_container);
        lv_label_set_text(msgbox_rf_switch_text, "rf switch");
        lv_obj_set_size(msgbox_rf_switch_text, 300, 40);
        lv_obj_set_style_text_font(msgbox_rf_switch_text, &lv_font_montserrat_26, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_align(msgbox_rf_switch_text, LV_ALIGN_TOP_LEFT, 0, 0);

        _registry.win.rf.setings.config_rf_params.sx1262.dropdown.rf_switch = lv_dropdown_create(_registry.win.rf.setings.message_box.parameter_container);
        lv_dropdown_set_dir(_registry.win.rf.setings.config_rf_params.sx1262.dropdown.rf_switch, LV_DIR_BOTTOM);
        lv_dropdown_set_options(_registry.win.rf.setings.config_rf_params.sx1262.dropdown.rf_switch, "RF1\n"
                                                                                                     "RF2");
        lv_dropdown_set_selected(_registry.win.rf.setings.config_rf_params.sx1262.dropdown.rf_switch, static_cast<uint32_t>(_device_sx1262.params.rf_switch));
        lv_obj_set_style_pad_top(_registry.win.rf.setings.config_rf_params.sx1262.dropdown.rf_switch, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_bottom(_registry.win.rf.setings.config_rf_params.sx1262.dropdown.rf_switch, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_left(_registry.win.rf.setings.config_rf_params.sx1262.dropdown.rf_switch, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_right(_registry.win.rf.setings.config_rf_params.sx1262.dropdown.rf_switch, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_min_width(_registry.win.rf.setings.config_rf_params.sx1262.dropdown.rf_switch, 200, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_min_height(_registry.win.rf.setings.config_rf_params.sx1262.dropdown.rf_switch, 30, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(_registry.win.rf.setings.config_rf_params.sx1262.dropdown.rf_switch, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);  // 输入框字体
        lv_obj_set_style_text_font(_registry.win.rf.setings.config_rf_params.sx1262.dropdown.rf_switch, &lv_font_montserrat_24, LV_PART_ITEMS | LV_STATE_DEFAULT); // 下拉列表字体
        lv_obj_align_to(_registry.win.rf.setings.config_rf_params.sx1262.dropdown.rf_switch, msgbox_rf_switch_text, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);

        lv_obj_add_event_cb(_registry.win.rf.setings.config_rf_params.sx1262.dropdown.rf_switch, [](lv_event_t *e)
                            {
                                lv_obj_t *dropdown = lv_event_get_target_obj(e);
                                lv_event_code_t code = lv_event_get_code(e);
                                
                                if (code == LV_EVENT_CLICKED) 
                                {
                                    // // 强制下拉列表向下打开
                                    // lv_dropdown_set_dir(dropdown, LV_DIR_BOTTOM);
                                    // 获取弹出的下拉列表对象
                                    lv_obj_t *list = lv_dropdown_get_list(dropdown);
                                    // // 设置下拉列表最多显示100高度
                                    // lv_obj_set_height(list, 300);

                                    lv_obj_set_style_bg_color(list, lv_color_hex(0xEEE9E9), LV_PART_MAIN | LV_STATE_DEFAULT);
                                    lv_obj_set_scrollbar_mode(list, LV_SCROLLBAR_MODE_ACTIVE);
                                    lv_obj_set_style_border_width(list, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
                                    
                                    lv_obj_set_style_text_font(list, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
                                    lv_obj_set_style_text_font(list, &lv_font_montserrat_24, LV_PART_ITEMS | LV_STATE_DEFAULT);
                                } }, LV_EVENT_ALL, NULL);

        lv_obj_t *msgbox_freq_text = lv_label_create(_registry.win.rf.setings.message_box.parameter_container);
        lv_label_set_text(msgbox_freq_text, "freq");
        lv_obj_set_size(msgbox_freq_text, 100, 40);
        lv_obj_set_style_text_font(msgbox_freq_text, &lv_font_montserrat_26, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_align_to(msgbox_freq_text, _registry.win.rf.setings.config_rf_params.sx1262.dropdown.rf_switch, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 10);

        _registry.win.rf.setings.config_rf_params.sx1262.textarea.freq = lv_textarea_create(_registry.win.rf.setings.message_box.parameter_container);
        lv_textarea_set_accepted_chars(_registry.win.rf.setings.config_rf_params.sx1262.textarea.freq, "0123456789."); // 只允许输入数字和小数点
        lv_obj_set_style_pad_top(_registry.win.rf.setings.config_rf_params.sx1262.textarea.freq, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_bottom(_registry.win.rf.setings.config_rf_params.sx1262.textarea.freq, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_left(_registry.win.rf.setings.config_rf_params.sx1262.textarea.freq, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_right(_registry.win.rf.setings.config_rf_params.sx1262.textarea.freq, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_width(_registry.win.rf.setings.config_rf_params.sx1262.textarea.freq, 300);
        lv_textarea_set_one_line(_registry.win.rf.setings.config_rf_params.sx1262.textarea.freq, true);
        char freq_str[15];
        snprintf(freq_str, sizeof(freq_str), "%.6f", _device_sx1262.params.freq);
        lv_textarea_set_text(_registry.win.rf.setings.config_rf_params.sx1262.textarea.freq, freq_str);
        lv_obj_set_style_text_font(_registry.win.rf.setings.config_rf_params.sx1262.textarea.freq, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_align_to(_registry.win.rf.setings.config_rf_params.sx1262.textarea.freq, msgbox_freq_text, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);

        init_win_rf_setings_keyboard_position_event_cb(_registry.win.rf.setings.config_rf_params.sx1262.textarea.freq);

        lv_obj_t *freq_unit_text = lv_label_create(_registry.win.rf.setings.message_box.parameter_container);
        lv_label_set_text(freq_unit_text, "mhz");
        lv_obj_set_size(freq_unit_text, 70, 40);
        lv_obj_set_style_text_font(freq_unit_text, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_align_to(freq_unit_text, _registry.win.rf.setings.config_rf_params.sx1262.textarea.freq, LV_ALIGN_OUT_RIGHT_BOTTOM, 10, 0);

        lv_obj_t *msgbox_bandwidth_text = lv_label_create(_registry.win.rf.setings.message_box.parameter_container);
        lv_label_set_text(msgbox_bandwidth_text, "bandwidth");
        lv_obj_set_size(msgbox_bandwidth_text, 200, 40);
        lv_obj_set_style_text_font(msgbox_bandwidth_text, &lv_font_montserrat_26, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_align_to(msgbox_bandwidth_text, _registry.win.rf.setings.config_rf_params.sx1262.textarea.freq, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 10);

        _registry.win.rf.setings.config_rf_params.sx1262.dropdown.bandwidth = lv_dropdown_create(_registry.win.rf.setings.message_box.parameter_container);
        lv_dropdown_set_dir(_registry.win.rf.setings.config_rf_params.sx1262.dropdown.bandwidth, LV_DIR_BOTTOM);
        lv_dropdown_set_options(_registry.win.rf.setings.config_rf_params.sx1262.dropdown.bandwidth, "BW_7810\n"
                                                                                                     "BW_15630\n"
                                                                                                     "BW_31250\n"
                                                                                                     "BW_62500\n"
                                                                                                     "BW_125000\n"
                                                                                                     "BW_250000\n"
                                                                                                     "BW_500000\n"
                                                                                                     "BW_10420\n"
                                                                                                     "BW_20830\n"
                                                                                                     "BW_41670");
        uint32_t buffer_index = static_cast<uint32_t>(_device_sx1262.params.bandwidth);
        if (buffer_index > 6)
        {
            buffer_index--;
        }
        lv_dropdown_set_selected(_registry.win.rf.setings.config_rf_params.sx1262.dropdown.bandwidth, buffer_index);
        lv_obj_set_style_pad_top(_registry.win.rf.setings.config_rf_params.sx1262.dropdown.bandwidth, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_bottom(_registry.win.rf.setings.config_rf_params.sx1262.dropdown.bandwidth, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_left(_registry.win.rf.setings.config_rf_params.sx1262.dropdown.bandwidth, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_right(_registry.win.rf.setings.config_rf_params.sx1262.dropdown.bandwidth, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_min_width(_registry.win.rf.setings.config_rf_params.sx1262.dropdown.bandwidth, 200, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_min_height(_registry.win.rf.setings.config_rf_params.sx1262.dropdown.bandwidth, 30, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(_registry.win.rf.setings.config_rf_params.sx1262.dropdown.bandwidth, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);  // 输入框字体
        lv_obj_set_style_text_font(_registry.win.rf.setings.config_rf_params.sx1262.dropdown.bandwidth, &lv_font_montserrat_24, LV_PART_ITEMS | LV_STATE_DEFAULT); // 下拉列表字体
        lv_obj_align_to(_registry.win.rf.setings.config_rf_params.sx1262.dropdown.bandwidth, msgbox_bandwidth_text, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);

        lv_obj_add_event_cb(_registry.win.rf.setings.config_rf_params.sx1262.dropdown.bandwidth, [](lv_event_t *e)
                            {
                                lv_obj_t *dropdown = lv_event_get_target_obj(e);
                                lv_event_code_t code = lv_event_get_code(e);
                                
                                if (code == LV_EVENT_CLICKED) 
                                {
                                    // // 强制下拉列表向下打开
                                    // lv_dropdown_set_dir(dropdown, LV_DIR_BOTTOM);
                                    // 获取弹出的下拉列表对象
                                    lv_obj_t *list = lv_dropdown_get_list(dropdown);
                                    // // 设置下拉列表最多显示100高度
                                    // lv_obj_set_height(list, 300);
                                    lv_obj_set_style_bg_color(list, lv_color_hex(0xEEE9E9), LV_PART_MAIN | LV_STATE_DEFAULT);
                                    lv_obj_set_scrollbar_mode(list, LV_SCROLLBAR_MODE_ACTIVE);
                                    lv_obj_set_style_border_width(list, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
                                    
                                    lv_obj_set_style_text_font(list, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
                                    lv_obj_set_style_text_font(list, &lv_font_montserrat_24, LV_PART_ITEMS | LV_STATE_DEFAULT);
                                } }, LV_EVENT_ALL, NULL);

        lv_obj_t *bandwidth_unit_text = lv_label_create(_registry.win.rf.setings.message_box.parameter_container);
        lv_label_set_text(bandwidth_unit_text, "hz");
        lv_obj_set_size(bandwidth_unit_text, 70, 40);
        lv_obj_set_style_text_font(bandwidth_unit_text, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_align_to(bandwidth_unit_text, _registry.win.rf.setings.config_rf_params.sx1262.dropdown.bandwidth, LV_ALIGN_OUT_RIGHT_BOTTOM, 10, 0);

        lv_obj_t *msgbox_current_limit_text = lv_label_create(_registry.win.rf.setings.message_box.parameter_container);
        lv_label_set_text(msgbox_current_limit_text, "current limit");
        lv_obj_set_size(msgbox_current_limit_text, 300, 40);
        lv_obj_set_style_text_font(msgbox_current_limit_text, &lv_font_montserrat_26, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_align_to(msgbox_current_limit_text, _registry.win.rf.setings.config_rf_params.sx1262.dropdown.bandwidth, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 10);

        _registry.win.rf.setings.config_rf_params.sx1262.textarea.current_limit = lv_textarea_create(_registry.win.rf.setings.message_box.parameter_container);
        lv_textarea_set_accepted_chars(_registry.win.rf.setings.config_rf_params.sx1262.textarea.current_limit, "0123456789."); // 只允许输入数字和小数点
        lv_obj_set_style_pad_top(_registry.win.rf.setings.config_rf_params.sx1262.textarea.current_limit, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_bottom(_registry.win.rf.setings.config_rf_params.sx1262.textarea.current_limit, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_left(_registry.win.rf.setings.config_rf_params.sx1262.textarea.current_limit, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_right(_registry.win.rf.setings.config_rf_params.sx1262.textarea.current_limit, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_width(_registry.win.rf.setings.config_rf_params.sx1262.textarea.current_limit, 300);
        lv_textarea_set_one_line(_registry.win.rf.setings.config_rf_params.sx1262.textarea.current_limit, true);
        char current_limit_str[10];
        snprintf(current_limit_str, sizeof(current_limit_str), "%.1f", _device_sx1262.params.current_limit);
        lv_textarea_set_text(_registry.win.rf.setings.config_rf_params.sx1262.textarea.current_limit, current_limit_str); // 设置初始内容
        lv_obj_set_style_text_font(_registry.win.rf.setings.config_rf_params.sx1262.textarea.current_limit, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_align_to(_registry.win.rf.setings.config_rf_params.sx1262.textarea.current_limit, msgbox_current_limit_text, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);

        init_win_rf_setings_keyboard_position_event_cb(_registry.win.rf.setings.config_rf_params.sx1262.textarea.current_limit);

        lv_obj_t *current_limit_unit_text = lv_label_create(_registry.win.rf.setings.message_box.parameter_container);
        lv_label_set_text(current_limit_unit_text, "ma");
        lv_obj_set_size(current_limit_unit_text, 70, 40);
        lv_obj_set_style_text_font(current_limit_unit_text, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_align_to(current_limit_unit_text, _registry.win.rf.setings.config_rf_params.sx1262.textarea.current_limit, LV_ALIGN_OUT_RIGHT_BOTTOM, 10, 0);

        lv_obj_t *msgbox_power_text = lv_label_create(_registry.win.rf.setings.message_box.parameter_container);
        lv_label_set_text(msgbox_power_text, "power");
        lv_obj_set_size(msgbox_power_text, 100, 40);
        lv_obj_set_style_text_font(msgbox_power_text, &lv_font_montserrat_26, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_align_to(msgbox_power_text, _registry.win.rf.setings.config_rf_params.sx1262.textarea.current_limit, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 10);

        _registry.win.rf.setings.config_rf_params.sx1262.textarea.power = lv_textarea_create(_registry.win.rf.setings.message_box.parameter_container);
        lv_textarea_set_accepted_chars(_registry.win.rf.setings.config_rf_params.sx1262.textarea.power, "0123456789-"); // 只允许输入数字和负号
        lv_obj_set_style_pad_top(_registry.win.rf.setings.config_rf_params.sx1262.textarea.power, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_bottom(_registry.win.rf.setings.config_rf_params.sx1262.textarea.power, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_left(_registry.win.rf.setings.config_rf_params.sx1262.textarea.power, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_right(_registry.win.rf.setings.config_rf_params.sx1262.textarea.power, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_width(_registry.win.rf.setings.config_rf_params.sx1262.textarea.power, 300);
        lv_textarea_set_one_line(_registry.win.rf.setings.config_rf_params.sx1262.textarea.power, true);
        char power_str[10];
        snprintf(power_str, sizeof(power_str), "%d", _device_sx1262.params.power);
        lv_textarea_set_text(_registry.win.rf.setings.config_rf_params.sx1262.textarea.power, power_str); // 设置初始内容
        lv_obj_set_style_text_font(_registry.win.rf.setings.config_rf_params.sx1262.textarea.power, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_align_to(_registry.win.rf.setings.config_rf_params.sx1262.textarea.power, msgbox_power_text, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);

        init_win_rf_setings_keyboard_position_event_cb(_registry.win.rf.setings.config_rf_params.sx1262.textarea.power);

        lv_obj_t *power_unit_text = lv_label_create(_registry.win.rf.setings.message_box.parameter_container);
        lv_label_set_text(power_unit_text, "dbm");
        lv_obj_set_size(power_unit_text, 70, 40);
        lv_obj_set_style_text_font(power_unit_text, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_align_to(power_unit_text, _registry.win.rf.setings.config_rf_params.sx1262.textarea.power, LV_ALIGN_OUT_RIGHT_BOTTOM, 10, 0);

        lv_obj_t *msgbox_spreading_factor_text = lv_label_create(_registry.win.rf.setings.message_box.parameter_container);
        lv_label_set_text(msgbox_spreading_factor_text, "spreading factor");
        lv_obj_set_size(msgbox_spreading_factor_text, 300, 40);
        lv_obj_set_style_text_font(msgbox_spreading_factor_text, &lv_font_montserrat_26, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_align_to(msgbox_spreading_factor_text, _registry.win.rf.setings.config_rf_params.sx1262.textarea.power, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 10);

        _registry.win.rf.setings.config_rf_params.sx1262.dropdown.spreading_factor = lv_dropdown_create(_registry.win.rf.setings.message_box.parameter_container);
        lv_dropdown_set_dir(_registry.win.rf.setings.config_rf_params.sx1262.dropdown.spreading_factor, LV_DIR_BOTTOM);
        lv_dropdown_set_options(_registry.win.rf.setings.config_rf_params.sx1262.dropdown.spreading_factor, "SF5\n"
                                                                                                            "SF6\n"
                                                                                                            "SF7\n"
                                                                                                            "SF8\n"
                                                                                                            "SF9\n"
                                                                                                            "SF10\n"
                                                                                                            "SF11\n"
                                                                                                            "SF12");
        lv_dropdown_set_selected(_registry.win.rf.setings.config_rf_params.sx1262.dropdown.spreading_factor, static_cast<uint32_t>(_device_sx1262.params.sf) - 5);
        lv_obj_set_style_pad_top(_registry.win.rf.setings.config_rf_params.sx1262.dropdown.spreading_factor, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_bottom(_registry.win.rf.setings.config_rf_params.sx1262.dropdown.spreading_factor, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_left(_registry.win.rf.setings.config_rf_params.sx1262.dropdown.spreading_factor, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_right(_registry.win.rf.setings.config_rf_params.sx1262.dropdown.spreading_factor, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_min_width(_registry.win.rf.setings.config_rf_params.sx1262.dropdown.spreading_factor, 200, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_min_height(_registry.win.rf.setings.config_rf_params.sx1262.dropdown.spreading_factor, 30, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(_registry.win.rf.setings.config_rf_params.sx1262.dropdown.spreading_factor, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);  // 输入框字体
        lv_obj_set_style_text_font(_registry.win.rf.setings.config_rf_params.sx1262.dropdown.spreading_factor, &lv_font_montserrat_24, LV_PART_ITEMS | LV_STATE_DEFAULT); // 下拉列表字体
        lv_obj_align_to(_registry.win.rf.setings.config_rf_params.sx1262.dropdown.spreading_factor, msgbox_spreading_factor_text, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);

        lv_obj_add_event_cb(_registry.win.rf.setings.config_rf_params.sx1262.dropdown.spreading_factor, [](lv_event_t *e)
                            {
                                lv_obj_t *dropdown = lv_event_get_target_obj(e);
                                lv_event_code_t code = lv_event_get_code(e);
                                
                                if (code == LV_EVENT_CLICKED) 
                                {
                                    // // 强制下拉列表向下打开
                                    // lv_dropdown_set_dir(dropdown, LV_DIR_BOTTOM);
                                    // 获取弹出的下拉列表对象
                                    lv_obj_t *list = lv_dropdown_get_list(dropdown);
                                    // // 设置下拉列表最多显示100高度
                                    // lv_obj_set_height(list, 300);

                                    lv_obj_set_style_bg_color(list, lv_color_hex(0xEEE9E9), LV_PART_MAIN | LV_STATE_DEFAULT);
                                    lv_obj_set_scrollbar_mode(list, LV_SCROLLBAR_MODE_ACTIVE);
                                    lv_obj_set_style_border_width(list, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
                                    
                                    lv_obj_set_style_text_font(list, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
                                    lv_obj_set_style_text_font(list, &lv_font_montserrat_24, LV_PART_ITEMS | LV_STATE_DEFAULT);
                                } }, LV_EVENT_ALL, NULL);

        lv_obj_t *msgbox_coding_rate_text = lv_label_create(_registry.win.rf.setings.message_box.parameter_container);
        lv_label_set_text(msgbox_coding_rate_text, "coding rate");
        lv_obj_set_size(msgbox_coding_rate_text, 300, 40);
        lv_obj_set_style_text_font(msgbox_coding_rate_text, &lv_font_montserrat_26, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_align_to(msgbox_coding_rate_text, _registry.win.rf.setings.config_rf_params.sx1262.dropdown.spreading_factor, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 10);

        _registry.win.rf.setings.config_rf_params.sx1262.dropdown.coding_rate = lv_dropdown_create(_registry.win.rf.setings.message_box.parameter_container);
        lv_dropdown_set_dir(_registry.win.rf.setings.config_rf_params.sx1262.dropdown.coding_rate, LV_DIR_BOTTOM);
        lv_dropdown_set_options(_registry.win.rf.setings.config_rf_params.sx1262.dropdown.coding_rate, "CR_4_5\n"
                                                                                                       "CR_4_6\n"
                                                                                                       "CR_4_7\n"
                                                                                                       "CR_4_8");
        lv_dropdown_set_selected(_registry.win.rf.setings.config_rf_params.sx1262.dropdown.coding_rate, static_cast<uint32_t>(_device_sx1262.params.cr) - 1);
        lv_obj_set_style_pad_top(_registry.win.rf.setings.config_rf_params.sx1262.dropdown.coding_rate, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_bottom(_registry.win.rf.setings.config_rf_params.sx1262.dropdown.coding_rate, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_left(_registry.win.rf.setings.config_rf_params.sx1262.dropdown.coding_rate, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_right(_registry.win.rf.setings.config_rf_params.sx1262.dropdown.coding_rate, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_min_width(_registry.win.rf.setings.config_rf_params.sx1262.dropdown.coding_rate, 200, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_min_height(_registry.win.rf.setings.config_rf_params.sx1262.dropdown.coding_rate, 30, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(_registry.win.rf.setings.config_rf_params.sx1262.dropdown.coding_rate, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);  // 输入框字体
        lv_obj_set_style_text_font(_registry.win.rf.setings.config_rf_params.sx1262.dropdown.coding_rate, &lv_font_montserrat_24, LV_PART_ITEMS | LV_STATE_DEFAULT); // 下拉列表字体
        lv_obj_align_to(_registry.win.rf.setings.config_rf_params.sx1262.dropdown.coding_rate, msgbox_coding_rate_text, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);

        lv_obj_add_event_cb(_registry.win.rf.setings.config_rf_params.sx1262.dropdown.coding_rate, [](lv_event_t *e)
                            {
                                lv_obj_t *dropdown = lv_event_get_target_obj(e);
                                lv_event_code_t code = lv_event_get_code(e);
                                
                                if (code == LV_EVENT_CLICKED) 
                                {
                                    // // 强制下拉列表向下打开
                                    // lv_dropdown_set_dir(dropdown, LV_DIR_BOTTOM);
                                    // 获取弹出的下拉列表对象
                                    lv_obj_t *list = lv_dropdown_get_list(dropdown);
                                    // // 设置下拉列表最多显示100高度
                                    // lv_obj_set_height(list, 300);

                                    lv_obj_set_style_bg_color(list, lv_color_hex(0xEEE9E9), LV_PART_MAIN | LV_STATE_DEFAULT);
                                    lv_obj_set_scrollbar_mode(list, LV_SCROLLBAR_MODE_ACTIVE);
                                    lv_obj_set_style_border_width(list, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
                                    
                                    lv_obj_set_style_text_font(list, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
                                    lv_obj_set_style_text_font(list, &lv_font_montserrat_24, LV_PART_ITEMS | LV_STATE_DEFAULT);
                                } }, LV_EVENT_ALL, NULL);

        lv_obj_t *msgbox_crc_type_text = lv_label_create(_registry.win.rf.setings.message_box.parameter_container);
        lv_label_set_text(msgbox_crc_type_text, "crc type");
        lv_obj_set_size(msgbox_crc_type_text, 300, 40);
        lv_obj_set_style_text_font(msgbox_crc_type_text, &lv_font_montserrat_26, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_align_to(msgbox_crc_type_text, _registry.win.rf.setings.config_rf_params.sx1262.dropdown.coding_rate, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 10);

        _registry.win.rf.setings.config_rf_params.sx1262.dropdown.crc_type = lv_dropdown_create(_registry.win.rf.setings.message_box.parameter_container);
        lv_dropdown_set_dir(_registry.win.rf.setings.config_rf_params.sx1262.dropdown.crc_type, LV_DIR_BOTTOM);
        lv_dropdown_set_options(_registry.win.rf.setings.config_rf_params.sx1262.dropdown.crc_type, "OFF\n"
                                                                                                    "ON");
        lv_dropdown_set_selected(_registry.win.rf.setings.config_rf_params.sx1262.dropdown.crc_type, static_cast<uint32_t>(_device_sx1262.params.crc_type));
        lv_obj_set_style_pad_top(_registry.win.rf.setings.config_rf_params.sx1262.dropdown.crc_type, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_bottom(_registry.win.rf.setings.config_rf_params.sx1262.dropdown.crc_type, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_left(_registry.win.rf.setings.config_rf_params.sx1262.dropdown.crc_type, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_right(_registry.win.rf.setings.config_rf_params.sx1262.dropdown.crc_type, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_min_width(_registry.win.rf.setings.config_rf_params.sx1262.dropdown.crc_type, 200, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_min_height(_registry.win.rf.setings.config_rf_params.sx1262.dropdown.crc_type, 30, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(_registry.win.rf.setings.config_rf_params.sx1262.dropdown.crc_type, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);  // 输入框字体
        lv_obj_set_style_text_font(_registry.win.rf.setings.config_rf_params.sx1262.dropdown.crc_type, &lv_font_montserrat_24, LV_PART_ITEMS | LV_STATE_DEFAULT); // 下拉列表字体
        lv_obj_align_to(_registry.win.rf.setings.config_rf_params.sx1262.dropdown.crc_type, msgbox_crc_type_text, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);

        lv_obj_add_event_cb(_registry.win.rf.setings.config_rf_params.sx1262.dropdown.crc_type, [](lv_event_t *e)
                            {
                                lv_obj_t *dropdown = lv_event_get_target_obj(e);
                                lv_event_code_t code = lv_event_get_code(e);
                                
                                if (code == LV_EVENT_CLICKED) 
                                {
                                    // // 强制下拉列表向下打开
                                    // lv_dropdown_set_dir(dropdown, LV_DIR_BOTTOM);
                                    // 获取弹出的下拉列表对象
                                    lv_obj_t *list = lv_dropdown_get_list(dropdown);
                                    // // 设置下拉列表最多显示100高度
                                    // lv_obj_set_height(list, 300);

                                    lv_obj_set_style_bg_color(list, lv_color_hex(0xEEE9E9), LV_PART_MAIN | LV_STATE_DEFAULT);
                                    lv_obj_set_scrollbar_mode(list, LV_SCROLLBAR_MODE_ACTIVE);
                                    lv_obj_set_style_border_width(list, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
                                    
                                    lv_obj_set_style_text_font(list, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
                                    lv_obj_set_style_text_font(list, &lv_font_montserrat_24, LV_PART_ITEMS | LV_STATE_DEFAULT);
                                } }, LV_EVENT_ALL, NULL);

        lv_obj_t *msgbox_preamble_length = lv_label_create(_registry.win.rf.setings.message_box.parameter_container);
        lv_label_set_text(msgbox_preamble_length, "preamble length");
        lv_obj_set_size(msgbox_preamble_length, 300, 40);
        lv_obj_set_style_text_font(msgbox_preamble_length, &lv_font_montserrat_26, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_align_to(msgbox_preamble_length, _registry.win.rf.setings.config_rf_params.sx1262.dropdown.crc_type, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 10);

        _registry.win.rf.setings.config_rf_params.sx1262.textarea.preamble_length = lv_textarea_create(_registry.win.rf.setings.message_box.parameter_container);
        lv_textarea_set_accepted_chars(_registry.win.rf.setings.config_rf_params.sx1262.textarea.preamble_length, "0123456789"); // 只允许输入数字
        lv_obj_set_style_pad_top(_registry.win.rf.setings.config_rf_params.sx1262.textarea.preamble_length, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_bottom(_registry.win.rf.setings.config_rf_params.sx1262.textarea.preamble_length, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_left(_registry.win.rf.setings.config_rf_params.sx1262.textarea.preamble_length, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_right(_registry.win.rf.setings.config_rf_params.sx1262.textarea.preamble_length, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_width(_registry.win.rf.setings.config_rf_params.sx1262.textarea.preamble_length, 300);
        lv_textarea_set_one_line(_registry.win.rf.setings.config_rf_params.sx1262.textarea.preamble_length, true);
        char preamble_length_str[10];
        snprintf(preamble_length_str, sizeof(preamble_length_str), "%d", _device_sx1262.params.preamble_length);
        lv_textarea_set_text(_registry.win.rf.setings.config_rf_params.sx1262.textarea.preamble_length, preamble_length_str); // 设置初始内容
        lv_obj_set_style_text_font(_registry.win.rf.setings.config_rf_params.sx1262.textarea.preamble_length, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_align_to(_registry.win.rf.setings.config_rf_params.sx1262.textarea.preamble_length, msgbox_preamble_length, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);

        init_win_rf_setings_keyboard_position_event_cb(_registry.win.rf.setings.config_rf_params.sx1262.textarea.preamble_length);

        lv_obj_t *msgbox_sync_word = lv_label_create(_registry.win.rf.setings.message_box.parameter_container);
        lv_label_set_text(msgbox_sync_word, "sync word");
        lv_obj_set_size(msgbox_sync_word, 300, 40);
        lv_obj_set_style_text_font(msgbox_sync_word, &lv_font_montserrat_26, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_align_to(msgbox_sync_word, _registry.win.rf.setings.config_rf_params.sx1262.textarea.preamble_length, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 10);

        _registry.win.rf.setings.config_rf_params.sx1262.textarea.sync_word = lv_textarea_create(_registry.win.rf.setings.message_box.parameter_container);
        lv_textarea_set_accepted_chars(_registry.win.rf.setings.config_rf_params.sx1262.textarea.sync_word, "0123456789"); // 只允许输入数字
        lv_obj_set_style_pad_top(_registry.win.rf.setings.config_rf_params.sx1262.textarea.sync_word, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_bottom(_registry.win.rf.setings.config_rf_params.sx1262.textarea.sync_word, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_left(_registry.win.rf.setings.config_rf_params.sx1262.textarea.sync_word, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_right(_registry.win.rf.setings.config_rf_params.sx1262.textarea.sync_word, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_width(_registry.win.rf.setings.config_rf_params.sx1262.textarea.sync_word, 300);
        lv_textarea_set_one_line(_registry.win.rf.setings.config_rf_params.sx1262.textarea.sync_word, true);
        char sync_word_str[10];
        snprintf(sync_word_str, sizeof(sync_word_str), "%d", _device_sx1262.params.sync_word);
        lv_textarea_set_text(_registry.win.rf.setings.config_rf_params.sx1262.textarea.sync_word, sync_word_str); // 设置初始内容
        lv_obj_set_style_text_font(_registry.win.rf.setings.config_rf_params.sx1262.textarea.sync_word, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_align_to(_registry.win.rf.setings.config_rf_params.sx1262.textarea.sync_word, msgbox_sync_word, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);

        init_win_rf_setings_keyboard_position_event_cb(_registry.win.rf.setings.config_rf_params.sx1262.textarea.sync_word);
    }

    void System::init_win_rf_setings_auto_send_message_box(void)
    {
        // 创建全屏灰色透明遮罩
        _registry.win.rf.setings.message_box.root = lv_obj_create(_registry.win.rf.setings.root);
        lv_obj_set_style_pad_top(_registry.win.rf.setings.message_box.root, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_bottom(_registry.win.rf.setings.message_box.root, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_left(_registry.win.rf.setings.message_box.root, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_right(_registry.win.rf.setings.message_box.root, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_size(_registry.win.rf.setings.message_box.root, _width, _height);
        lv_obj_set_style_bg_color(_registry.win.rf.setings.message_box.root, lv_color_black(), LV_PART_MAIN);
        lv_obj_set_style_bg_opa(_registry.win.rf.setings.message_box.root, LV_OPA_30, LV_PART_MAIN);
        lv_obj_set_style_border_width(_registry.win.rf.setings.message_box.root, 0, LV_PART_MAIN);
        lv_obj_set_style_radius(_registry.win.rf.setings.message_box.root, 0, LV_PART_MAIN);
        lv_obj_align(_registry.win.rf.setings.message_box.root, LV_ALIGN_CENTER, 0, 0);
        lv_obj_add_flag(_registry.win.rf.setings.message_box.root, LV_OBJ_FLAG_CLICKABLE); // 添加触摸标志来禁止其他界面触摸

        _registry.win.rf.setings.message_box.root_container = lv_obj_create(_registry.win.rf.setings.message_box.root);
        lv_obj_set_style_pad_top(_registry.win.rf.setings.message_box.root_container, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_bottom(_registry.win.rf.setings.message_box.root_container, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_left(_registry.win.rf.setings.message_box.root_container, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_right(_registry.win.rf.setings.message_box.root_container, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_size(_registry.win.rf.setings.message_box.root_container, 450, _height * 0.7);
        lv_obj_set_style_radius(_registry.win.rf.setings.message_box.root_container, 15, LV_PART_MAIN);
        lv_obj_set_style_bg_color(_registry.win.rf.setings.message_box.root_container, lv_color_white(), LV_PART_MAIN);
        lv_obj_set_style_border_width(_registry.win.rf.setings.message_box.root_container, 0, LV_PART_MAIN);
        lv_obj_set_style_shadow_width(_registry.win.rf.setings.message_box.root_container, 16, LV_PART_MAIN);
        lv_obj_center(_registry.win.rf.setings.message_box.root_container);

        _registry.win.rf.setings.message_box.btn_container = lv_obj_create(_registry.win.rf.setings.message_box.root_container);
        lv_obj_set_style_pad_top(_registry.win.rf.setings.message_box.btn_container, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_bottom(_registry.win.rf.setings.message_box.btn_container, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_left(_registry.win.rf.setings.message_box.btn_container, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_right(_registry.win.rf.setings.message_box.btn_container, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_size(_registry.win.rf.setings.message_box.btn_container, 450, 110);
        lv_obj_set_style_border_width(_registry.win.rf.setings.message_box.btn_container, 0, (lv_style_selector_t)LV_PART_MAIN); // 移除边框
        lv_obj_align(_registry.win.rf.setings.message_box.btn_container, LV_ALIGN_BOTTOM_MID, 0, 0);

        lv_obj_t *btn_cancel = lv_button_create(_registry.win.rf.setings.message_box.btn_container);
        lv_obj_set_size(btn_cancel, 150, 60);
        lv_obj_align(btn_cancel, LV_ALIGN_BOTTOM_LEFT, 40, -30);
        lv_obj_set_style_radius(btn_cancel, 10, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_width(btn_cancel, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT); // 去除按钮阴影
        lv_obj_t *btn_cancel_label = lv_label_create(btn_cancel);
        lv_label_set_text(btn_cancel_label, "cancel");
        lv_obj_set_style_text_font(btn_cancel_label, &lv_font_montserrat_26, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_center(btn_cancel_label);

        // cancel 按钮回调
        lv_obj_add_event_cb(btn_cancel, [](lv_event_t *e)
                            {
                                System *self = static_cast<System *>(lv_event_get_user_data(e));
                                lv_obj_delete(self->_registry.win.rf.setings.message_box.root); }, LV_EVENT_CLICKED, this);

        lv_obj_t *btn_apply = lv_button_create(_registry.win.rf.setings.message_box.btn_container);
        lv_obj_set_size(btn_apply, 150, 60);
        lv_obj_align(btn_apply, LV_ALIGN_BOTTOM_RIGHT, -40, -30);
        lv_obj_set_style_radius(btn_apply, 10, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_width(btn_apply, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT); // 去除按钮阴影
        lv_obj_t *btn_apply_label = lv_label_create(btn_apply);
        lv_label_set_text(btn_apply_label, "apply");
        lv_obj_set_style_text_font(btn_apply_label, &lv_font_montserrat_26, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_center(btn_apply_label);

        // apply 按钮回调
        lv_obj_add_event_cb(btn_apply, [](lv_event_t *e)
                            {
                                System *self = static_cast<System *>(lv_event_get_user_data(e));

                                switch (self->_rf_chip_type)
                                {
                                case Rf_Chip_Type::SX1262:
                                    {
                                        self->_device_sx1262.auto_send.flag = lv_obj_has_state(self->_registry.win.rf.setings.auto_send.control_switch, LV_STATE_CHECKED);
                                    
                                        const char* auto_send_text = lv_textarea_get_text(self->_registry.win.rf.setings.auto_send.textarea.auto_send_text);
                                        if((auto_send_text != nullptr) && (auto_send_text[0] != '\0'))  // 同时检查NULL和空字符串
                                        {
                                            self->_device_sx1262.auto_send.text = auto_send_text;
                                        }

                                        const char* auto_send_interval_text = lv_textarea_get_text(self->_registry.win.rf.setings.auto_send.textarea.auto_send_interval);
                                        if (auto_send_interval_text != nullptr && auto_send_interval_text[0] != '\0') // 同时检查NULL和空字符串
                                        {  
                                            uint32_t buffer = std::stoi(auto_send_interval_text);  

                                            // 限制范围
                                            if((buffer >= 1) && (buffer <= 1000000))
                                            {
                                                self->_device_sx1262.auto_send.interval = buffer;
                                            }
                                        }
                                    }
                                break;
#if defined CONFIG_BOARD_TYPE_T_DISPLAY_P4_KEYBOARD
                                case Rf_Chip_Type::CC1101:
                                    {
                                        self->_device_cc1101.auto_send.flag = lv_obj_has_state(self->_registry.win.rf.setings.auto_send.control_switch, LV_STATE_CHECKED);
                                        
                                        const char* auto_send_text = lv_textarea_get_text(self->_registry.win.rf.setings.auto_send.textarea.auto_send_text);
                                        if((auto_send_text != nullptr) && (auto_send_text[0] != '\0'))  // 同时检查NULL和空字符串
                                        {
                                            self->_device_cc1101.auto_send.text = auto_send_text;
                                        }

                                        const char* auto_send_interval_text = lv_textarea_get_text(self->_registry.win.rf.setings.auto_send.textarea.auto_send_interval);
                                        if (auto_send_interval_text != nullptr && auto_send_interval_text[0] != '\0') // 同时检查NULL和空字符串
                                        {  
                                            uint32_t buffer = std::stoi(auto_send_interval_text);  

                                            // 限制范围
                                            if((buffer >= 1) && (buffer <= 1000000))
                                            {
                                                self->_device_cc1101.auto_send.interval = buffer;
                                            }
                                        }
                                    }
                                    break;
                                case Rf_Chip_Type::NRF24L01:
                                    {
                                        self->_device_nrf24l01.auto_send.flag = lv_obj_has_state(self->_registry.win.rf.setings.auto_send.control_switch, LV_STATE_CHECKED);
                                        
                                        const char* auto_send_text = lv_textarea_get_text(self->_registry.win.rf.setings.auto_send.textarea.auto_send_text);
                                        if((auto_send_text != nullptr) && (auto_send_text[0] != '\0'))  // 同时检查NULL和空字符串
                                        {
                                            self->_device_nrf24l01.auto_send.text = auto_send_text;
                                        }

                                        const char* auto_send_interval_text = lv_textarea_get_text(self->_registry.win.rf.setings.auto_send.textarea.auto_send_interval);
                                        if (auto_send_interval_text != nullptr && auto_send_interval_text[0] != '\0') // 同时检查NULL和空字符串
                                        {  
                                            uint32_t buffer = std::stoi(auto_send_interval_text);  

                                            // 限制范围
                                            if((buffer >= 1) && (buffer <= 1000000))
                                            {
                                                self->_device_nrf24l01.auto_send.interval = buffer;
                                            }
                                        }
                                    }
                                    break;
#endif

                            default:
                                break;
                            }

                                lv_obj_delete(self->_registry.win.rf.setings.message_box.root); }, LV_EVENT_CLICKED, this);

#if defined CONFIG_BOARD_TYPE_T_DISPLAY_P4
        create_keyboard(_registry.win.rf.setings.message_box.root);
#endif

        _registry.win.rf.setings.message_box.parameter_container = lv_obj_create(_registry.win.rf.setings.message_box.root_container);
        lv_obj_set_style_pad_top(_registry.win.rf.setings.message_box.parameter_container, 30, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_bottom(_registry.win.rf.setings.message_box.parameter_container, 30, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_left(_registry.win.rf.setings.message_box.parameter_container, 30, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_right(_registry.win.rf.setings.message_box.parameter_container, 30, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
#if defined SCREEN_ROTATION_DIRECTION_0
        lv_obj_set_size(_registry.win.rf.setings.message_box.parameter_container, 450, _height * 0.6);
#elif defined SCREEN_ROTATION_DIRECTION_90
        lv_obj_set_size(_registry.win.rf.setings.message_box.parameter_container, 450, _height * 0.5);
#else
#error "unknown macro definition, please select the correct macro definition."
#endif
        lv_obj_set_style_border_width(_registry.win.rf.setings.message_box.parameter_container, 0, (lv_style_selector_t)LV_PART_MAIN); // 移除边框
        lv_obj_set_scrollbar_mode(_registry.win.rf.setings.message_box.parameter_container, LV_SCROLLBAR_MODE_ACTIVE);
        lv_obj_align_to(_registry.win.rf.setings.message_box.parameter_container, _registry.win.rf.setings.message_box.btn_container, LV_ALIGN_OUT_TOP_MID, 0, 0);

        // 触摸设置消息框区域时隐藏键盘
        lv_obj_add_event_cb(_registry.win.rf.setings.message_box.parameter_container, [](lv_event_t *e)
                            {
                                System *self = static_cast<System *>(lv_event_get_user_data(e));
                                lv_event_code_t code = lv_event_get_code(e);

                                if (code == LV_EVENT_CLICKED)
                                {
#if defined CONFIG_BOARD_TYPE_T_DISPLAY_P4
                                    lv_obj_add_flag(self->_registry.keyboard, LV_OBJ_FLAG_HIDDEN); // 隐藏键盘

                                    // 调整聊天框的大小
                                    lv_obj_set_size(self->_registry.win.rf.setings.message_box.root_container, 450, self->_height * 0.7);
                                    lv_obj_center(self->_registry.win.rf.setings.message_box.root_container);
                                    lv_obj_align(self->_registry.win.rf.setings.message_box.btn_container, LV_ALIGN_BOTTOM_MID, 0, 0);
                                    lv_obj_set_size(self->_registry.win.rf.setings.message_box.parameter_container, 450, self->_height * 0.6);
                                    lv_obj_align_to(self->_registry.win.rf.setings.message_box.parameter_container, self->_registry.win.rf.setings.message_box.btn_container, LV_ALIGN_OUT_TOP_MID, 0, 0);
#elif defined CONFIG_BOARD_TYPE_T_DISPLAY_P4_KEYBOARD
                                    lv_group_remove_all_objs(self->_registry.keyboard_group);
#else
#error "unknown macro definition, please select the correct macro definition."
#endif
                                } }, LV_EVENT_ALL, this);

        lv_obj_t *msgbox_auto_send_control_text = lv_label_create(_registry.win.rf.setings.message_box.parameter_container);
        lv_label_set_text(msgbox_auto_send_control_text, "auto send control");
        lv_obj_set_size(msgbox_auto_send_control_text, 280, 30);
        lv_obj_set_style_text_font(msgbox_auto_send_control_text, &lv_font_montserrat_26, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_align(msgbox_auto_send_control_text, LV_ALIGN_TOP_LEFT, 0, 0);

        _registry.win.rf.setings.auto_send.control_switch = lv_switch_create(_registry.win.rf.setings.message_box.parameter_container);
        lv_obj_set_size(_registry.win.rf.setings.auto_send.control_switch, 90, 50);
        lv_obj_align_to(_registry.win.rf.setings.auto_send.control_switch, msgbox_auto_send_control_text, LV_ALIGN_OUT_RIGHT_MID, 10, 0);

        switch (_rf_chip_type)
        {
        case Rf_Chip_Type::SX1262:
            if (_device_sx1262.auto_send.flag == true)
            {
                lv_obj_add_state(_registry.win.rf.setings.auto_send.control_switch, LV_STATE_CHECKED);
            }
            else
            {
                lv_obj_remove_state(_registry.win.rf.setings.auto_send.control_switch, LV_STATE_CHECKED);
            }
            break;
#if defined CONFIG_BOARD_TYPE_T_DISPLAY_P4_KEYBOARD
        case Rf_Chip_Type::CC1101:
            if (_device_cc1101.auto_send.flag == true)
            {
                lv_obj_add_state(_registry.win.rf.setings.auto_send.control_switch, LV_STATE_CHECKED);
            }
            else
            {
                lv_obj_remove_state(_registry.win.rf.setings.auto_send.control_switch, LV_STATE_CHECKED);
            }
            break;
        case Rf_Chip_Type::NRF24L01:
            if (_device_nrf24l01.auto_send.flag == true)
            {
                lv_obj_add_state(_registry.win.rf.setings.auto_send.control_switch, LV_STATE_CHECKED);
            }
            else
            {
                lv_obj_remove_state(_registry.win.rf.setings.auto_send.control_switch, LV_STATE_CHECKED);
            }
            break;
#endif

        default:
            break;
        }

        lv_obj_t *msgbox_auto_send_text = lv_label_create(_registry.win.rf.setings.message_box.parameter_container);
        lv_label_set_text(msgbox_auto_send_text, "auto send text");
        lv_obj_set_size(msgbox_auto_send_text, 300, 40);
        lv_obj_set_style_text_font(msgbox_auto_send_text, &lv_font_montserrat_26, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_align_to(msgbox_auto_send_text, msgbox_auto_send_control_text, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 30);

        _registry.win.rf.setings.auto_send.textarea.auto_send_text = lv_textarea_create(_registry.win.rf.setings.message_box.parameter_container);
        lv_obj_set_style_pad_top(_registry.win.rf.setings.auto_send.textarea.auto_send_text, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_bottom(_registry.win.rf.setings.auto_send.textarea.auto_send_text, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_left(_registry.win.rf.setings.auto_send.textarea.auto_send_text, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_right(_registry.win.rf.setings.auto_send.textarea.auto_send_text, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_width(_registry.win.rf.setings.auto_send.textarea.auto_send_text, 300);
        lv_textarea_set_one_line(_registry.win.rf.setings.auto_send.textarea.auto_send_text, true);

        switch (_rf_chip_type)
        {
        case Rf_Chip_Type::SX1262:
            lv_textarea_set_text(_registry.win.rf.setings.auto_send.textarea.auto_send_text, _device_sx1262.auto_send.text.c_str()); // 设置初始内容
            break;
#if defined CONFIG_BOARD_TYPE_T_DISPLAY_P4_KEYBOARD
        case Rf_Chip_Type::CC1101:
            lv_textarea_set_text(_registry.win.rf.setings.auto_send.textarea.auto_send_text, _device_cc1101.auto_send.text.c_str()); // 设置初始内容
            break;
        case Rf_Chip_Type::NRF24L01:
            lv_textarea_set_text(_registry.win.rf.setings.auto_send.textarea.auto_send_text, _device_nrf24l01.auto_send.text.c_str()); // 设置初始内容
            break;
#endif

        default:
            break;
        }

        lv_obj_set_style_text_font(_registry.win.rf.setings.auto_send.textarea.auto_send_text, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_align_to(_registry.win.rf.setings.auto_send.textarea.auto_send_text, msgbox_auto_send_text, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);

        init_win_rf_setings_keyboard_position_event_cb(_registry.win.rf.setings.auto_send.textarea.auto_send_text);

        lv_obj_t *msgbox_auto_send_interval = lv_label_create(_registry.win.rf.setings.message_box.parameter_container);
        lv_label_set_text(msgbox_auto_send_interval, "auto send interval");
        lv_obj_set_size(msgbox_auto_send_interval, 300, 40);
        lv_obj_set_style_text_font(msgbox_auto_send_interval, &lv_font_montserrat_26, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_align_to(msgbox_auto_send_interval, _registry.win.rf.setings.auto_send.textarea.auto_send_text, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 10);

        _registry.win.rf.setings.auto_send.textarea.auto_send_interval = lv_textarea_create(_registry.win.rf.setings.message_box.parameter_container);
        lv_textarea_set_accepted_chars(_registry.win.rf.setings.auto_send.textarea.auto_send_interval, "0123456789"); // 只允许输入数字
        lv_obj_set_style_pad_top(_registry.win.rf.setings.auto_send.textarea.auto_send_interval, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_bottom(_registry.win.rf.setings.auto_send.textarea.auto_send_interval, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_left(_registry.win.rf.setings.auto_send.textarea.auto_send_interval, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_right(_registry.win.rf.setings.auto_send.textarea.auto_send_interval, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_width(_registry.win.rf.setings.auto_send.textarea.auto_send_interval, 300);
        lv_textarea_set_one_line(_registry.win.rf.setings.auto_send.textarea.auto_send_interval, true);
        char auto_send_interval_str[10];

        switch (_rf_chip_type)
        {
        case Rf_Chip_Type::SX1262:
            snprintf(auto_send_interval_str, sizeof(auto_send_interval_str), "%ld", _device_sx1262.auto_send.interval);
            break;
#if defined CONFIG_BOARD_TYPE_T_DISPLAY_P4_KEYBOARD
        case Rf_Chip_Type::CC1101:
            snprintf(auto_send_interval_str, sizeof(auto_send_interval_str), "%ld", _device_cc1101.auto_send.interval);
            break;
        case Rf_Chip_Type::NRF24L01:
            snprintf(auto_send_interval_str, sizeof(auto_send_interval_str), "%ld", _device_nrf24l01.auto_send.interval);
            break;
#endif

        default:
            break;
        }

        lv_textarea_set_text(_registry.win.rf.setings.auto_send.textarea.auto_send_interval, auto_send_interval_str); // 设置初始内容
        lv_obj_set_style_text_font(_registry.win.rf.setings.auto_send.textarea.auto_send_interval, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_align_to(_registry.win.rf.setings.auto_send.textarea.auto_send_interval, msgbox_auto_send_interval, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);

        init_win_rf_setings_keyboard_position_event_cb(_registry.win.rf.setings.auto_send.textarea.auto_send_interval);

        lv_obj_t *auto_send_interval_unit_text = lv_label_create(_registry.win.rf.setings.message_box.parameter_container);
        lv_label_set_text(auto_send_interval_unit_text, "ms");
        lv_obj_set_size(auto_send_interval_unit_text, 70, 40);
        lv_obj_set_style_text_font(auto_send_interval_unit_text, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_align_to(auto_send_interval_unit_text, _registry.win.rf.setings.auto_send.textarea.auto_send_interval, LV_ALIGN_OUT_RIGHT_BOTTOM, 10, 0);
    }

    bool System::set_config_rf_params(Device_Sx1262 device_sx1262)
    {
        if (_win_rf_config_sx1262_params_callback != nullptr)
        {
            if (_win_rf_config_sx1262_params_callback(device_sx1262) == true)
            {
                return true;
            }
        }

        return false;
    }

    void System::set_rf_send_data_callback(std::string data)
    {
        if (_win_rf_send_data_callback != nullptr)
        {
            _win_rf_send_data_callback(data);
        }
    }

    void System::set_rf_status_callback(bool status)
    {
        if (_win_rf_status_callback != nullptr)
        {
            _win_rf_status_callback(status);
        }
    }

    // ─── Album Art: HW JPEG decode + fallback ──────────────────────────────

    // Persistent decoded pixel buffer and LVGL image descriptor (allocated once, never freed)
    static uint8_t  *s_art_pixels = nullptr;     // PSRAM: decoded RGB565 pixels
    static size_t    s_art_pixels_alloc = 0;     // allocated size
    static lv_image_dsc_t s_art_dsc = {};        // LVGL image descriptor
    static int       s_art_track_idx = -1;       // track index of current art (-1 = none)

    // Max art pixel buffer: 1024x1024 (padded to 16) RGB565 = 2MB PSRAM.
    // Covers virtually all embedded album art. After decode, we downscale
    // in-place to display size (544x544) to minimize LVGL render cost.
    #define ART_PIXEL_MAX_W  1024   // max decode width
    #define ART_PIXEL_MAX_H  1024   // max decode height
    #define ART_PIXEL_BUF_SIZE (ART_PIXEL_MAX_W * ART_PIXEL_MAX_H * 2)
    #define ART_DISPLAY_SIZE 544    // target display size (padded to 16 for HW compat)

    // Persistent HW JPEG decoder — acquired once, never released
    static jpeg_decoder_handle_t s_jpeg_decoder = NULL;

    // Lazy-init: allocate pixel buffer and JPEG engine once on first use
    static bool art_ensure_resources(void) {
        if (!s_art_pixels) {
            // Must use jpeg_alloc_decoder_mem for DMA-aligned output buffer
            jpeg_decode_memory_alloc_cfg_t mem_cfg = {
                .buffer_direction = JPEG_DEC_ALLOC_OUTPUT_BUFFER,
            };
            size_t actual_size = 0;
            s_art_pixels = (uint8_t *)jpeg_alloc_decoder_mem(ART_PIXEL_BUF_SIZE, &mem_cfg, &actual_size);
            s_art_pixels_alloc = s_art_pixels ? actual_size : 0;
            if (s_art_pixels)
                ESP_LOGI("ART", "Allocated %u bytes for art pixels (DMA-aligned, PSRAM)", (unsigned)actual_size);
            else
                ESP_LOGE("ART", "Failed to allocate art pixel buffer (%u bytes)", (unsigned)ART_PIXEL_BUF_SIZE);
        }
        if (!s_jpeg_decoder) {
            jpeg_decode_engine_cfg_t eng_cfg = {
                .intr_priority = 0,
                .timeout_ms = 1000,
            };
            esp_err_t err = jpeg_new_decoder_engine(&eng_cfg, &s_jpeg_decoder);
            if (err != ESP_OK || !s_jpeg_decoder) {
                ESP_LOGE("ART", "Failed to acquire JPEG decoder: 0x%x", err);
                s_jpeg_decoder = NULL;
            }
        }
        return s_art_pixels && s_jpeg_decoder;
    }

    // In-place box downscale RGB565 pixels in a buffer.
    // Safe because output is always smaller — we write rows we've already read.
    static void downscale_rgb565_inplace(uint8_t *buf, int src_w, int src_h, int dst_w, int dst_h) {
        uint16_t *src = (uint16_t *)buf;
        uint16_t *dst = (uint16_t *)buf;

        for (int dy = 0; dy < dst_h; dy++) {
            int sy0 = (dy * src_h) / dst_h;
            int sy1 = ((dy + 1) * src_h) / dst_h;
            if (sy1 <= sy0) sy1 = sy0 + 1;

            for (int dx = 0; dx < dst_w; dx++) {
                int sx0 = (dx * src_w) / dst_w;
                int sx1 = ((dx + 1) * src_w) / dst_w;
                if (sx1 <= sx0) sx1 = sx0 + 1;

                // Box filter: average all source pixels in this cell
                uint32_t r_acc = 0, g_acc = 0, b_acc = 0;
                int count = 0;
                for (int sy = sy0; sy < sy1 && sy < src_h; sy++) {
                    for (int sx = sx0; sx < sx1 && sx < src_w; sx++) {
                        uint16_t px = src[sy * src_w + sx];
                        r_acc += (px >> 11) & 0x1F;
                        g_acc += (px >> 5) & 0x3F;
                        b_acc += px & 0x1F;
                        count++;
                    }
                }
                if (count > 0) {
                    dst[dy * dst_w + dx] = ((r_acc / count) << 11) |
                                           ((g_acc / count) << 5) |
                                           (b_acc / count);
                }
            }
        }
    }

    // Fallback: generate a gradient image for tracks without embedded art.
    // Uses a simple color derived from the track title hash.
    // Writes directly into persistent s_art_pixels (no allocation).
    static void generate_fallback_art(int width, int height) {
        if (!s_art_pixels) return;
        size_t needed = width * height * 2;  // RGB565
        if (needed > s_art_pixels_alloc) return;  // shouldn't happen with 270x270

        // Hash track title for color seed
        music_player_info_t info = music_player_get_info();
        uint32_t hash = 0x811c9dc5;
        if (info.title) {
            for (const char *p = info.title; *p; p++)
                hash = (hash ^ (uint8_t)*p) * 0x01000193;
        }

        // Two complementary colors from hash
        uint8_t r1 = (hash >> 0) & 0x7F;   // darker tones
        uint8_t g1 = (hash >> 8) & 0x7F;
        uint8_t b1 = (hash >> 16) & 0x7F;
        uint8_t r2 = 0x40 + ((hash >> 4) & 0x3F);
        uint8_t g2 = 0x40 + ((hash >> 12) & 0x3F);
        uint8_t b2 = 0x40 + ((hash >> 20) & 0x3F);

        uint16_t *px = (uint16_t *)s_art_pixels;
        for (int y = 0; y < height; y++) {
            int t = (y * 255) / (height - 1);  // 0..255 vertical gradient
            uint8_t r = r1 + ((r2 - r1) * t) / 255;
            uint8_t g = g1 + ((g2 - g1) * t) / 255;
            uint8_t b = b1 + ((b2 - b1) * t) / 255;
            uint16_t rgb565 = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3);
            for (int x = 0; x < width; x++) {
                px[y * width + x] = rgb565;
            }
        }

        s_art_dsc.header.cf = LV_COLOR_FORMAT_RGB565;
        s_art_dsc.header.w = width;
        s_art_dsc.header.h = height;
        s_art_dsc.data_size = needed;
        s_art_dsc.data = s_art_pixels;
    }

    // Decode JPEG album art using ESP32-P4 hardware JPEG decoder.
    // Uses persistent s_art_pixels buffer and s_jpeg_decoder engine.
    // If the decoded image is larger than ART_DISPLAY_SIZE, downscales in-place.
    // Returns true if decode succeeded and s_art_dsc is ready for LVGL.
    static bool decode_album_art_jpeg(void) {
        size_t jpeg_size = 0;
        const uint8_t *jpeg_data = music_player_get_album_art(&jpeg_size);
        if (!jpeg_data || jpeg_size < 100) return false;
        if (!s_art_pixels || !s_jpeg_decoder) return false;

        // Get image dimensions
        jpeg_decode_picture_info_t pic_info = {};
        esp_err_t err = jpeg_decoder_get_info(jpeg_data, jpeg_size, &pic_info);
        if (err != ESP_OK) {
            ESP_LOGW("ART", "JPEG get_info failed: 0x%x", err);
            return false;
        }

        ESP_LOGI("ART", "JPEG: %lux%lu", (unsigned long)pic_info.width, (unsigned long)pic_info.height);

        // HW decoder pads output to multiples of 16
        uint32_t out_w = (pic_info.width + 15) & ~15;
        uint32_t out_h = (pic_info.height + 15) & ~15;
        size_t needed = out_w * out_h * 2;  // RGB565

        // Check if decoded image fits in our pre-allocated buffer
        if (needed > s_art_pixels_alloc) {
            ESP_LOGW("ART", "JPEG %lux%lu too large for buffer (%u > %u)",
                     (unsigned long)out_w, (unsigned long)out_h,
                     (unsigned)needed, (unsigned)s_art_pixels_alloc);
            return false;
        }

        // Decode to RGB565 using persistent HW decoder
        jpeg_decode_cfg_t dec_cfg = {
            .output_format = JPEG_DECODE_OUT_FORMAT_RGB565,
            .rgb_order = JPEG_DEC_RGB_ELEMENT_ORDER_BGR,  // LVGL expects BGR565
            .conv_std = JPEG_YUV_RGB_CONV_STD_BT601,
        };

        uint32_t out_size = 0;
        err = jpeg_decoder_process(s_jpeg_decoder, &dec_cfg, jpeg_data, jpeg_size,
                                   s_art_pixels, s_art_pixels_alloc, &out_size);

        if (err != ESP_OK) {
            ESP_LOGW("ART", "JPEG decode failed: 0x%x", err);
            return false;
        }

        // Downscale in-place if larger than display size.
        // Box filter averages source pixels — no extra allocation.
        uint32_t final_w = out_w;
        uint32_t final_h = out_h;
        if (out_w > ART_DISPLAY_SIZE || out_h > ART_DISPLAY_SIZE) {
            // Scale proportionally to fit ART_DISPLAY_SIZE, pad to 16
            float scale = (float)ART_DISPLAY_SIZE / (out_w > out_h ? out_w : out_h);
            final_w = ((uint32_t)(out_w * scale) + 15) & ~15;
            final_h = ((uint32_t)(out_h * scale) + 15) & ~15;
            if (final_w < 16) final_w = 16;
            if (final_h < 16) final_h = 16;

            ESP_LOGI("ART", "Downscaling %lux%lu → %lux%lu",
                     (unsigned long)out_w, (unsigned long)out_h,
                     (unsigned long)final_w, (unsigned long)final_h);
            downscale_rgb565_inplace(s_art_pixels, out_w, out_h, final_w, final_h);
        }

        ESP_LOGI("ART", "Art ready: %lux%lu RGB565", (unsigned long)final_w, (unsigned long)final_h);

        s_art_dsc.header.cf = LV_COLOR_FORMAT_RGB565;
        s_art_dsc.header.w = final_w;
        s_art_dsc.header.h = final_h;
        s_art_dsc.data_size = final_w * final_h * 2;
        s_art_dsc.data = s_art_pixels;

        return true;
    }

    // Update album art widget for the given track. Called from init_win_music
    // and from the 500ms timer on track change.
    static void refresh_album_art(lv_obj_t *art_widget, int track_idx) {
        if (!art_widget) return;
        if (track_idx == s_art_track_idx) return;  // already showing this track's art

        // Lazy-init persistent pixel buffer + JPEG engine on first call
        if (!art_ensure_resources()) {
            ESP_LOGW("ART", "Art resources unavailable");
            return;
        }

        bool has_art = false;
        if (track_idx >= 0) {
            has_art = music_player_load_album_art(track_idx);
            if (has_art) {
                has_art = decode_album_art_jpeg();
            }
        }

        if (!has_art) {
            // Generate fallback gradient (writes into persistent s_art_pixels)
            ESP_LOGI("ART", "No embedded art for track %d, using gradient", track_idx);
            generate_fallback_art(270, 270);  // half-size, LVGL will scale up to 540
        }

        s_art_track_idx = track_idx;

        if (s_art_dsc.data) {
            lv_image_set_src(art_widget, &s_art_dsc);
            lv_obj_set_size(art_widget, 540, 540);
            lv_image_set_inner_align(art_widget, LV_IMAGE_ALIGN_STRETCH);
            // Force redraw — same &s_art_dsc pointer but different pixel data
            lv_obj_invalidate(art_widget);
        }
    }

    void System::init_win_music(void)
    {
        // Re-scan SD card for tracks every time window opens.
        // Handles: SD card inserted/removed, files added/deleted since boot.
        // Only scan if not currently playing — avoids overwriting track data mid-playback.
        music_player_info_t pre_info = music_player_get_info();
        if (pre_info.state == MUSIC_STATE_STOPPED) {
            music_player_scan();
        }

        // 主界面
        _registry.win.music.root = lv_obj_create(NULL);
        lv_obj_set_style_bg_color(_registry.win.music.root, lv_color_hex(0xE0DFDE), (lv_style_selector_t)LV_PART_MAIN);

        lv_obj_t *album_cover_img = lv_image_create(_registry.win.music.root);
        _registry.win.music.album_art = album_cover_img;
        lv_obj_set_size(album_cover_img, 540, 540);
#if defined SCREEN_ROTATION_DIRECTION_0
        lv_obj_align(album_cover_img, LV_ALIGN_TOP_MID, 0, 50);
#elif defined SCREEN_ROTATION_DIRECTION_90
        lv_obj_align(album_cover_img, LV_ALIGN_TOP_MID, -330, 20);
#else
#error "unknown macro definition, please select the correct macro definition."
#endif
        // Load album art for current track (HW JPEG decode or fallback gradient)
        {
            music_player_info_t minfo_art = music_player_get_info();
            s_art_track_idx = -1;  // force refresh
            refresh_album_art(album_cover_img, minfo_art.track_index);
        }

        lv_obj_set_size(_registry.win.music.root, _width, _height);
        lv_obj_set_scrollbar_mode(_registry.win.music.root, LV_SCROLLBAR_MODE_OFF);

        // ─── Close button (upper right) ────────────────────────────────
        {
            lv_obj_t *close_btn = lv_button_create(_registry.win.music.root);
            lv_obj_set_size(close_btn, 50, 50);
            lv_obj_set_style_bg_color(close_btn, lv_color_black(), LV_PART_MAIN);
            lv_obj_set_style_bg_opa(close_btn, 120, LV_PART_MAIN);
            lv_obj_set_style_shadow_width(close_btn, 0, LV_PART_MAIN);
            lv_obj_set_style_border_width(close_btn, 0, LV_PART_MAIN);
            lv_obj_set_style_radius(close_btn, 25, LV_PART_MAIN);  // circle
#if defined SCREEN_ROTATION_DIRECTION_0
            lv_obj_align(close_btn, LV_ALIGN_TOP_RIGHT, -15, 55);
#elif defined SCREEN_ROTATION_DIRECTION_90
            lv_obj_align(close_btn, LV_ALIGN_TOP_RIGHT, -15, 25);
#endif
            lv_obj_t *close_lbl = lv_label_create(close_btn);
            lv_label_set_text(close_lbl, LV_SYMBOL_CLOSE);
            lv_obj_set_style_text_color(close_lbl, lv_color_white(), LV_PART_MAIN);
            lv_obj_center(close_lbl);

            lv_obj_add_event_cb(close_btn, [](lv_event_t *e) {
                if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
                System *self = static_cast<System *>(lv_event_get_user_data(e));
                self->set_vibration();
                self->init_win_home();
                lv_screen_load_anim(self->_registry.win.home.root, LV_SCR_LOAD_ANIM_FADE_OUT, 100, 0, true);
                self->_edge_touch_flag = false;
            }, LV_EVENT_ALL, this);
        }

        lv_obj_t *song_name_btn = lv_button_create(_registry.win.music.root);
        lv_obj_set_size(song_name_btn, 460, 60);
        lv_obj_set_style_radius(song_name_btn, LV_RADIUS_CIRCLE, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_color(song_name_btn, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_opa(song_name_btn, 60, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_width(song_name_btn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
#if defined SCREEN_ROTATION_DIRECTION_0
        lv_obj_align(song_name_btn, LV_ALIGN_TOP_MID, 0, 600);
#elif defined SCREEN_ROTATION_DIRECTION_90
        lv_obj_align(song_name_btn, LV_ALIGN_TOP_LEFT, 560, 70);
#else
#error "unknown macro definition, please select the correct macro definition."
#endif

        lv_obj_t *song_name_label = lv_label_create(song_name_btn);
        music_player_info_t minfo = music_player_get_info();
        lv_label_set_text(song_name_label, minfo.title);
        lv_label_set_long_mode(song_name_label, LV_LABEL_LONG_SCROLL_CIRCULAR);
        lv_obj_set_width(song_name_label, 420);
        _registry.win.music.label.song_name = song_name_label;
        lv_obj_set_style_text_color(song_name_label, lv_color_white(), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(song_name_label, &lvgl_font_misans_bold_27, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_text_align(song_name_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_center(song_name_label);

        lv_obj_t *artist_btn = lv_button_create(_registry.win.music.root);
        lv_obj_set_size(artist_btn, 300, 50);
        lv_obj_set_style_radius(artist_btn, LV_RADIUS_CIRCLE, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_color(artist_btn, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_opa(artist_btn, 60, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_width(artist_btn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_align_to(artist_btn, song_name_btn, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);

        lv_obj_t *artist_label = lv_label_create(artist_btn);
        lv_label_set_text(artist_label, minfo.artist);
        _registry.win.music.label.artist = artist_label;
        lv_obj_set_style_text_color(artist_label, lv_color_white(), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(artist_label, &lv_font_montserrat_26, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_text_align(artist_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_center(artist_label);

        // 创建播放按键图片按钮
        _registry.win.music.imagebutton.play = lv_imagebutton_create(_registry.win.music.root);
        lv_imagebutton_set_src(_registry.win.music.imagebutton.play, LV_IMAGEBUTTON_STATE_RELEASED, NULL, &win_music_play_start_1_140x140px_rgb565a8, NULL);
        lv_imagebutton_set_src(_registry.win.music.imagebutton.play, LV_IMAGEBUTTON_STATE_PRESSED, NULL, &win_music_play_start_2_140x140px_rgb565a8, NULL);
        lv_imagebutton_set_src(_registry.win.music.imagebutton.play, LV_IMAGEBUTTON_STATE_DISABLED, NULL, &win_music_play_start_1_140x140px_rgb565a8, NULL);
        lv_imagebutton_set_src(_registry.win.music.imagebutton.play, LV_IMAGEBUTTON_STATE_CHECKED_RELEASED, NULL, &win_music_play_pause_1_117x117px_rgb565a8, NULL);
        lv_imagebutton_set_src(_registry.win.music.imagebutton.play, LV_IMAGEBUTTON_STATE_CHECKED_PRESSED, NULL, &win_music_play_pause_2_117x117px_rgb565a8, NULL);
        lv_imagebutton_set_src(_registry.win.music.imagebutton.play, LV_IMAGEBUTTON_STATE_CHECKED_DISABLED, NULL, &win_music_play_pause_1_117x117px_rgb565a8, NULL);
        // 设置按钮大小和位置
        // lv_obj_set_size(_registry.win.music.imagebutton.play, 140, 140);
        // lv_obj_align(_registry.win.music.imagebutton.play, LV_ALIGN_BOTTOM_MID, 0, -120);

        // 添加点击事件切换播放/暂停状态
        lv_obj_add_event_cb(_registry.win.music.imagebutton.play, [](lv_event_t *e)
                            {
                                System *self = static_cast<System *>(lv_event_get_user_data(e));
                                lv_obj_t *btn = lv_event_get_target_obj(e);
                                lv_event_code_t code = lv_event_get_code(e);

                                if (code == LV_EVENT_CLICKED) 
                                {
                                    if (lv_obj_has_state(btn, LV_STATE_CHECKED)) 
                                    {
                                        self->_registry.win.music.play_flag = false;

                                        self->set_win_music_play_imagebutton_status(self->_registry.win.music.play_flag);

                                        // Actually pause the music player
                                        music_player_pause();
                                    }
                                    else // 按键点击 切换为播放模式
                                    {
                                        self->_registry.win.music.play_flag = true;

                                        self->set_win_music_play_imagebutton_status(self->_registry.win.music.play_flag);

                                        // Resume if paused, or start if stopped
                                        music_player_info_t pinfo = music_player_get_info();
                                        if (pinfo.state == MUSIC_STATE_PAUSED) {
                                            music_player_resume();
                                        } else {
                                            self->set_music_start_end(true);
                                        }
                                    }
                                } }, LV_EVENT_ALL, this);

        // 创建左切换按键图片按钮
        _registry.win.music.imagebutton.switch_left = lv_imagebutton_create(_registry.win.music.root);
        lv_imagebutton_set_src(_registry.win.music.imagebutton.switch_left, LV_IMAGEBUTTON_STATE_RELEASED, NULL, &win_music_play_switch_left_1_95x95px_rgb565a8, NULL);
        lv_imagebutton_set_src(_registry.win.music.imagebutton.switch_left, LV_IMAGEBUTTON_STATE_PRESSED, NULL, &win_music_play_switch_left_2_95x95px_rgb565a8, NULL);
        lv_imagebutton_set_src(_registry.win.music.imagebutton.switch_left, LV_IMAGEBUTTON_STATE_DISABLED, NULL, &win_music_play_switch_left_1_95x95px_rgb565a8, NULL);
        // 设置按钮大小和位置
        lv_obj_set_size(_registry.win.music.imagebutton.switch_left, 95, 95);
        // lv_obj_align_to(_registry.win.music.imagebutton.switch_left, _registry.win.music.imagebutton.play, LV_ALIGN_OUT_LEFT_MID, -10, 0);

        // Previous track callback
        lv_obj_add_event_cb(_registry.win.music.imagebutton.switch_left, [](lv_event_t *e) {
            if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
            System *self = static_cast<System *>(lv_event_get_user_data(e));
            music_player_prev();
            // Update labels
            music_player_info_t info = music_player_get_info();
            if (self->_registry.win.music.label.song_name)
                lv_label_set_text(self->_registry.win.music.label.song_name, info.title);
            if (self->_registry.win.music.label.artist)
                lv_label_set_text(self->_registry.win.music.label.artist, info.artist);
        }, LV_EVENT_ALL, this);

        // 创建右切换按键图片按钮
        _registry.win.music.imagebutton.switch_right = lv_imagebutton_create(_registry.win.music.root);
        lv_imagebutton_set_src(_registry.win.music.imagebutton.switch_right, LV_IMAGEBUTTON_STATE_RELEASED, NULL, &win_music_play_switch_right_1_95x95px_rgb565a8, NULL);
        lv_imagebutton_set_src(_registry.win.music.imagebutton.switch_right, LV_IMAGEBUTTON_STATE_PRESSED, NULL, &win_music_play_switch_right_2_95x95px_rgb565a8, NULL);
        lv_imagebutton_set_src(_registry.win.music.imagebutton.switch_right, LV_IMAGEBUTTON_STATE_DISABLED, NULL, &win_music_play_switch_right_1_95x95px_rgb565a8, NULL);
        // 设置按钮大小和位置
        lv_obj_set_size(_registry.win.music.imagebutton.switch_right, 95, 95);
        // lv_obj_align_to(_registry.win.music.imagebutton.switch_right, _registry.win.music.imagebutton.play, LV_ALIGN_OUT_RIGHT_MID, 10, 0);

        // Next track callback
        lv_obj_add_event_cb(_registry.win.music.imagebutton.switch_right, [](lv_event_t *e) {
            if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
            System *self = static_cast<System *>(lv_event_get_user_data(e));
            music_player_next();
            // Update labels
            music_player_info_t info = music_player_get_info();
            if (self->_registry.win.music.label.song_name)
                lv_label_set_text(self->_registry.win.music.label.song_name, info.title);
            if (self->_registry.win.music.label.artist)
                lv_label_set_text(self->_registry.win.music.label.artist, info.artist);
        }, LV_EVENT_ALL, this);

        // 创建左侧当前播放时间按钮
        lv_obj_t *current_time_btn = lv_button_create(_registry.win.music.root);
        lv_obj_set_size(current_time_btn, 90, 40);
        lv_obj_set_style_bg_color(current_time_btn, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_radius(current_time_btn, LV_RADIUS_CIRCLE, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_width(current_time_btn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
#if defined SCREEN_ROTATION_DIRECTION_0
        lv_obj_align(current_time_btn, LV_ALIGN_TOP_LEFT, 30, 795);
#elif defined SCREEN_ROTATION_DIRECTION_90
        lv_obj_align(current_time_btn, LV_ALIGN_TOP_LEFT, 560, 210);
#else
#error "unknown macro definition, please select the correct macro definition."
#endif

        _registry.win.music.label.current_time = lv_label_create(current_time_btn);
        // lv_label_set_text(_registry.win.music.label.current_time, "00:00");
        lv_obj_set_style_text_color(_registry.win.music.label.current_time, lv_color_white(), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(_registry.win.music.label.current_time, &lv_font_montserrat_22, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_center(_registry.win.music.label.current_time);

        // 创建右侧总时长按钮
        lv_obj_t *total_time_btn = lv_button_create(_registry.win.music.root);
        lv_obj_set_size(total_time_btn, 90, 40);
        lv_obj_set_style_bg_color(total_time_btn, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_radius(total_time_btn, LV_RADIUS_CIRCLE, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_width(total_time_btn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
#if defined SCREEN_ROTATION_DIRECTION_0
        lv_obj_align(total_time_btn, LV_ALIGN_TOP_RIGHT, -30, 795);
#elif defined SCREEN_ROTATION_DIRECTION_90
        lv_obj_align(total_time_btn, LV_ALIGN_TOP_RIGHT, -30, 210);
#else
#error "unknown macro definition, please select the correct macro definition."
#endif

        _registry.win.music.label.total_time = lv_label_create(total_time_btn);
        // lv_label_set_text(_registry.win.music.label.total_time, "04:20");
        lv_obj_set_style_text_color(_registry.win.music.label.total_time, lv_color_white(), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(_registry.win.music.label.total_time, &lv_font_montserrat_22, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_center(_registry.win.music.label.total_time);

        // 创建音乐滚动条
        static lv_style_t style_indic;
        lv_style_init(&style_indic);
        lv_style_set_bg_color(&style_indic, lv_color_white());
        lv_style_set_bg_grad_color(&style_indic, lv_color_white());
        lv_style_set_bg_grad_dir(&style_indic, LV_GRAD_DIR_HOR);

        static lv_style_t style_indic_pr;
        lv_style_init(&style_indic_pr);
        lv_style_set_shadow_color(&style_indic_pr, lv_color_white());
        lv_style_set_shadow_width(&style_indic_pr, 4);
        lv_style_set_shadow_spread(&style_indic_pr, 3);

        _registry.win.music.slider = lv_slider_create(_registry.win.music.root);
        lv_obj_set_style_bg_color(_registry.win.music.slider, lv_color_black(), LV_PART_MAIN); // 设置进度条背景（未填充部分）为灰色
        // 去除蓝色圆形按钮
        lv_obj_set_style_pad_left(_registry.win.music.slider, 0, LV_PART_KNOB);
        lv_obj_set_style_pad_right(_registry.win.music.slider, 0, LV_PART_KNOB);
        lv_obj_set_style_bg_opa(_registry.win.music.slider, LV_OPA_COVER, LV_PART_KNOB);
        lv_obj_set_style_bg_color(_registry.win.music.slider, lv_color_white(), LV_PART_KNOB);
        lv_obj_set_style_radius(_registry.win.music.slider, 50, LV_PART_KNOB);
        lv_obj_set_style_width(_registry.win.music.slider, 8, LV_PART_KNOB);
        lv_obj_set_style_height(_registry.win.music.slider, 8, LV_PART_KNOB);
#if defined SCREEN_ROTATION_DIRECTION_0
        lv_obj_set_size(_registry.win.music.slider, 460, 8);
#elif defined SCREEN_ROTATION_DIRECTION_90
        lv_obj_set_size(_registry.win.music.slider, 640, 8);
#else
#error "unknown macro definition, please select the correct macro definition."
#endif
        lv_obj_add_style(_registry.win.music.slider, &style_indic, LV_PART_INDICATOR);
        lv_obj_add_style(_registry.win.music.slider, &style_indic_pr, LV_PART_INDICATOR | LV_STATE_PRESSED);
// lv_slider_set_value(_registry.win.music.slider, 0, LV_ANIM_OFF);
#if defined SCREEN_ROTATION_DIRECTION_0
        lv_obj_align(_registry.win.music.slider, LV_ALIGN_TOP_LEFT, 40, 860);
#elif defined SCREEN_ROTATION_DIRECTION_90
        lv_obj_align(_registry.win.music.slider, LV_ALIGN_TOP_LEFT, 560, 270);
#else
#error "unknown macro definition, please select the correct macro definition."
#endif

        lv_obj_add_event_cb(_registry.win.music.slider, [](lv_event_t *e)
                            {
                                System *self = static_cast<System *>(lv_event_get_user_data(e));
                                lv_event_code_t code = lv_event_get_code(e);

                                if (code == LV_EVENT_RELEASED)
                                {
                                    int32_t  percent = lv_slider_get_value(self->_registry.win.music.slider);
                                    // printf("slider percent: %d\n", percent);

                                    double set_current_time_s = self->_registry.win.music.total_time_s * (static_cast<double>(percent) / 100.0);

                                    self->set_music_current_time_s(set_current_time_s);

                                    self->set_win_music_current_total_time(set_current_time_s, self->_registry.win.music.total_time_s);
                                } }, LV_EVENT_ALL, this);

        // ─── Volume slider ──────────────────────────────────────────────────
        {
            // Volume icon (speaker symbol)
            lv_obj_t *vol_icon = lv_label_create(_registry.win.music.root);
            lv_label_set_text(vol_icon, LV_SYMBOL_VOLUME_MID);
            lv_obj_set_style_text_color(vol_icon, lv_color_hex(0x555555), LV_PART_MAIN);
            lv_obj_set_style_text_font(vol_icon, &lv_font_montserrat_22, LV_PART_MAIN);
#if defined SCREEN_ROTATION_DIRECTION_0
            lv_obj_align(vol_icon, LV_ALIGN_BOTTOM_LEFT, 30, -128);
#elif defined SCREEN_ROTATION_DIRECTION_90
            lv_obj_align(vol_icon, LV_ALIGN_BOTTOM_LEFT, 560, -128);
#endif

            lv_obj_t *vol_slider = lv_slider_create(_registry.win.music.root);
            lv_slider_set_range(vol_slider, 0, 100);
            lv_slider_set_value(vol_slider, (int32_t)g_settings.volume, LV_ANIM_OFF);
            lv_obj_set_style_bg_color(vol_slider, lv_color_hex(0x999999), LV_PART_MAIN);
            lv_obj_set_style_bg_color(vol_slider, lv_color_white(), LV_PART_INDICATOR);
            lv_obj_set_style_bg_color(vol_slider, lv_color_white(), LV_PART_KNOB);
            lv_obj_set_style_radius(vol_slider, 4, LV_PART_MAIN);
            lv_obj_set_style_radius(vol_slider, 4, LV_PART_INDICATOR);
            lv_obj_set_style_radius(vol_slider, 50, LV_PART_KNOB);
            lv_obj_set_style_pad_left(vol_slider, 0, LV_PART_KNOB);
            lv_obj_set_style_pad_right(vol_slider, 0, LV_PART_KNOB);
            lv_obj_set_style_width(vol_slider, 10, LV_PART_KNOB);
            lv_obj_set_style_height(vol_slider, 10, LV_PART_KNOB);
#if defined SCREEN_ROTATION_DIRECTION_0
            lv_obj_set_size(vol_slider, 380, 6);
            lv_obj_align(vol_slider, LV_ALIGN_BOTTOM_MID, 20, -133);
#elif defined SCREEN_ROTATION_DIRECTION_90
            lv_obj_set_size(vol_slider, 540, 6);
            lv_obj_align(vol_slider, LV_ALIGN_BOTTOM_MID, 280, -133);
#endif

            lv_obj_add_event_cb(vol_slider, [](lv_event_t *e) {
                if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) return;
                System *self = static_cast<System *>(lv_event_get_user_data(e));
                g_settings.volume = (uint8_t)lv_slider_get_value(lv_event_get_target_obj(e));
                if (self->_device_volume_callback)
                    self->_device_volume_callback(g_settings.volume);
            }, LV_EVENT_ALL, this);
            lv_obj_add_event_cb(vol_slider, [](lv_event_t *e) {
                if (lv_event_get_code(e) != LV_EVENT_RELEASED) return;
                settings_save();
            }, LV_EVENT_ALL, nullptr);
        }

        // 创建底部黑色长条按钮
        lv_obj_t *bottom_bar_btn = lv_button_create(_registry.win.music.root);
        lv_obj_set_style_bg_color(bottom_bar_btn, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_radius(bottom_bar_btn, 20, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_width(bottom_bar_btn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
#if defined SCREEN_ROTATION_DIRECTION_0
        lv_obj_set_size(bottom_bar_btn, _width - 60, 70);
        lv_obj_align(bottom_bar_btn, LV_ALIGN_BOTTOM_MID, 0, -30);
#elif defined SCREEN_ROTATION_DIRECTION_90
        lv_obj_set_size(bottom_bar_btn, _width / 2 + 20, 70);
        lv_obj_align(bottom_bar_btn, LV_ALIGN_BOTTOM_MID, 260, -30);
#else
#error "unknown macro definition, please select the correct macro definition."
#endif

        // 长条按钮添加一个标签
        lv_obj_t *bottom_bar_label = lv_label_create(bottom_bar_btn);
        lv_label_set_text(bottom_bar_label, "Music");
        lv_obj_set_style_text_color(bottom_bar_label, lv_color_white(), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(bottom_bar_label, &lv_font_montserrat_26, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_center(bottom_bar_label);

        // ─── Playlist panel (hidden, shown on bottom bar tap) ──────────
        // Full-screen overlay with scrollable track list
        lv_obj_t *playlist_panel = lv_obj_create(_registry.win.music.root);
        lv_obj_set_size(playlist_panel, _width, _height - 50);
        lv_obj_align(playlist_panel, LV_ALIGN_BOTTOM_MID, 0, 0);
        lv_obj_set_style_bg_color(playlist_panel, lv_color_hex(0x1A1A1A), LV_PART_MAIN);
        lv_obj_set_style_bg_opa(playlist_panel, 240, LV_PART_MAIN);
        lv_obj_set_style_radius(playlist_panel, 20, LV_PART_MAIN);
        lv_obj_set_style_border_width(playlist_panel, 0, LV_PART_MAIN);
        lv_obj_set_style_pad_all(playlist_panel, 15, LV_PART_MAIN);
        lv_obj_set_style_pad_row(playlist_panel, 8, LV_PART_MAIN);
        lv_obj_set_scrollbar_mode(playlist_panel, LV_SCROLLBAR_MODE_AUTO);
        lv_obj_set_flex_flow(playlist_panel, LV_FLEX_FLOW_COLUMN);
        lv_obj_add_flag(playlist_panel, LV_OBJ_FLAG_HIDDEN);  // start hidden

        // Static refs for rebuild from callbacks
        static lv_obj_t *s_playlist_panel = nullptr;
        static System *s_playlist_self = nullptr;
        static int32_t s_playlist_width = 0;
        static void (*s_rebuild_fn)(void) = nullptr;
        s_playlist_panel = playlist_panel;
        s_playlist_self = this;
        s_playlist_width = _width;

        // Rebuild playlist contents — called on open and after track selection
        s_rebuild_fn = []() {
            if (!s_playlist_panel || !s_playlist_self) return;

            // Delete all children and rebuild
            lv_obj_clean(s_playlist_panel);

            // Header
            lv_obj_t *pl_header = lv_label_create(s_playlist_panel);
            lv_label_set_text(pl_header, "Up Next");
            lv_obj_set_style_text_color(pl_header, lv_color_hex(0xAAAAAA), LV_PART_MAIN);
            lv_obj_set_style_text_font(pl_header, &lv_font_montserrat_22, LV_PART_MAIN);
            lv_obj_set_style_pad_bottom(pl_header, 8, LV_PART_MAIN);

            music_player_info_t pl_info = music_player_get_info();
            int count = music_player_track_count();

            for (int i = 0; i < count; i++) {
                const music_track_t *trk = music_player_get_track(i);
                if (!trk) continue;

                lv_obj_t *row_btn = lv_button_create(s_playlist_panel);
                lv_obj_set_size(row_btn, s_playlist_width - 60, 65);
                lv_obj_set_style_radius(row_btn, 12, LV_PART_MAIN);
                lv_obj_set_style_shadow_width(row_btn, 0, LV_PART_MAIN);
                lv_obj_set_style_border_width(row_btn, 0, LV_PART_MAIN);
                lv_obj_set_style_pad_left(row_btn, 15, LV_PART_MAIN);
                lv_obj_set_style_pad_ver(row_btn, 8, LV_PART_MAIN);

                bool is_current = (i == pl_info.track_index);
                lv_obj_set_style_bg_color(row_btn, is_current ? lv_color_hex(0x333333) : lv_color_hex(0x222222), LV_PART_MAIN);

                // Track number
                lv_obj_t *num_lbl = lv_label_create(row_btn);
                char num_str[12];
                snprintf(num_str, sizeof(num_str), "%d.", i + 1);
                lv_label_set_text(num_lbl, num_str);
                lv_obj_set_style_text_color(num_lbl, is_current ? lv_color_hex(0x4dabf7) : lv_color_hex(0x888888), LV_PART_MAIN);
                lv_obj_set_style_text_font(num_lbl, &lv_font_montserrat_22, LV_PART_MAIN);
                lv_obj_align(num_lbl, LV_ALIGN_LEFT_MID, 0, 0);

                // Title
                lv_obj_t *title_lbl = lv_label_create(row_btn);
                lv_label_set_text(title_lbl, trk->title);
                lv_label_set_long_mode(title_lbl, LV_LABEL_LONG_DOT);
                lv_obj_set_width(title_lbl, s_playlist_width - 160);
                lv_obj_set_style_text_color(title_lbl, is_current ? lv_color_white() : lv_color_hex(0xCCCCCC), LV_PART_MAIN);
                lv_obj_set_style_text_font(title_lbl, &lv_font_montserrat_22, LV_PART_MAIN);
                lv_obj_align(title_lbl, LV_ALIGN_LEFT_MID, 35, 0);

                // Now-playing indicator
                if (is_current) {
                    lv_obj_t *now_lbl = lv_label_create(row_btn);
                    lv_label_set_text(now_lbl, LV_SYMBOL_PLAY);
                    lv_obj_set_style_text_color(now_lbl, lv_color_hex(0x4dabf7), LV_PART_MAIN);
                    lv_obj_align(now_lbl, LV_ALIGN_RIGHT_MID, -5, 0);
                }

                // Tap to jump to track — stays open, rebuilds to show new highlight
                lv_obj_add_event_cb(row_btn, [](lv_event_t *e) {
                    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
                    int idx = (int)(intptr_t)lv_obj_get_user_data(lv_event_get_target_obj(e));

                    int cnt = music_player_track_count();
                    if (cnt > 0) {
                        music_player_set_track((idx - 1 + cnt) % cnt);
                        music_player_next();
                    }

                    // Update labels
                    music_player_info_t info = music_player_get_info();
                    if (s_playlist_self->_registry.win.music.label.song_name)
                        lv_label_set_text(s_playlist_self->_registry.win.music.label.song_name, info.title);
                    if (s_playlist_self->_registry.win.music.label.artist)
                        lv_label_set_text(s_playlist_self->_registry.win.music.label.artist, info.artist);

                    // Rebuild playlist to refresh highlights (panel stays open)
                    // Use lv_async to avoid modifying tree during event processing
                    lv_async_call([](void *) {
                        if (s_rebuild_fn) s_rebuild_fn();
                    }, nullptr);
                }, LV_EVENT_ALL, s_playlist_self);
                lv_obj_set_user_data(row_btn, (void *)(intptr_t)i);
            }

            // Close button at bottom
            lv_obj_t *close_pl_btn = lv_button_create(s_playlist_panel);
            lv_obj_set_size(close_pl_btn, s_playlist_width - 60, 55);
            lv_obj_set_style_bg_color(close_pl_btn, lv_color_hex(0x333333), LV_PART_MAIN);
            lv_obj_set_style_radius(close_pl_btn, 12, LV_PART_MAIN);
            lv_obj_set_style_shadow_width(close_pl_btn, 0, LV_PART_MAIN);
            lv_obj_set_style_border_width(close_pl_btn, 0, LV_PART_MAIN);
            lv_obj_t *close_pl_lbl = lv_label_create(close_pl_btn);
            lv_label_set_text(close_pl_lbl, LV_SYMBOL_DOWN "  Close");
            lv_obj_set_style_text_color(close_pl_lbl, lv_color_white(), LV_PART_MAIN);
            lv_obj_set_style_text_font(close_pl_lbl, &lv_font_montserrat_22, LV_PART_MAIN);
            lv_obj_center(close_pl_lbl);
            lv_obj_add_event_cb(close_pl_btn, [](lv_event_t *e) {
                if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
                if (s_playlist_panel)
                    lv_obj_add_flag(s_playlist_panel, LV_OBJ_FLAG_HIDDEN);
            }, LV_EVENT_ALL, nullptr);
        };

        // Build initial playlist content
        s_rebuild_fn();

        // Bottom bar: toggle playlist on tap, rebuild on show
        lv_obj_add_event_cb(bottom_bar_btn, [](lv_event_t *e) {
            if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
            if (!s_playlist_panel) return;
            if (lv_obj_has_flag(s_playlist_panel, LV_OBJ_FLAG_HIDDEN)) {
                s_rebuild_fn();  // refresh highlights before showing
                lv_obj_remove_flag(s_playlist_panel, LV_OBJ_FLAG_HIDDEN);
            } else {
                lv_obj_add_flag(s_playlist_panel, LV_OBJ_FLAG_HIDDEN);
            }
        }, LV_EVENT_ALL, nullptr);

        set_win_music_play_imagebutton_status(_registry.win.music.play_flag);

        set_win_music_current_total_time(_registry.win.music.current_time_s, _registry.win.music.total_time_s);

        // Periodic timer to update UI from music_player state (position, track info)
        static lv_timer_t *s_music_timer = nullptr;
        if (s_music_timer) { lv_timer_delete(s_music_timer); s_music_timer = nullptr; }
        s_music_timer = lv_timer_create([](lv_timer_t *t) {
            System *self = static_cast<System *>(lv_timer_get_user_data(t));
            if (self->_current_win != Current_Win::MUSIC) return;
            music_player_info_t info = music_player_get_info();
            if (info.state == MUSIC_STATE_PLAYING || info.state == MUSIC_STATE_PAUSED) {
                self->set_win_music_current_total_time(info.position_s, info.duration_s);
            }
            // Update song/artist if track changed
            if (self->_registry.win.music.label.song_name)
                lv_label_set_text(self->_registry.win.music.label.song_name, info.title);
            if (self->_registry.win.music.label.artist)
                lv_label_set_text(self->_registry.win.music.label.artist, info.artist);
            // Refresh album art if track changed
            refresh_album_art(self->_registry.win.music.album_art, info.track_index);
        }, 500, this);

        lv_obj_add_event_cb(_registry.win.music.root, [](lv_event_t *e)
                            {
                                System *self = static_cast<System *>(lv_event_get_user_data(e));
                                lv_event_code_t code = lv_event_get_code(e);

                                if (code == LV_EVENT_GESTURE)
                                {
                                    lv_dir_t gesture_dir = lv_indev_get_gesture_dir(lv_indev_active());

                                    // 边缘检测以及左右滑动
                                    if ((gesture_dir == LV_DIR_LEFT || gesture_dir == LV_DIR_RIGHT)&&(self->_edge_touch_flag == true))
                                    {   
                                        // self->set_music_start_end(false);
                                        self->set_vibration();
                                        self->init_win_home();
                                        
                                        lv_screen_load_anim(self->_registry.win.home.root, LV_SCR_LOAD_ANIM_FADE_OUT, 100, 0, true);

                                        self->_edge_touch_flag = false;
                                    }
                                } }, LV_EVENT_ALL, this);

        // ─── Cleanup on screen destroy ─────────────────────────────────
        // When navigating away, lv_screen_load_anim(..., true) destroys
        // this screen. Null all statics to prevent use-after-free from
        // queued lv_async_call or lingering timer callbacks.
        lv_obj_add_event_cb(_registry.win.music.root, [](lv_event_t *e) {
            if (lv_event_get_code(e) != LV_EVENT_DELETE) return;
            // Kill the timer before its next tick can touch deleted widgets
            if (s_music_timer) {
                lv_timer_delete(s_music_timer);
                s_music_timer = nullptr;
            }
            // Null all statics that point into the destroyed screen tree
            s_playlist_panel = nullptr;
            s_playlist_self = nullptr;
            s_rebuild_fn = nullptr;
            // Reset art track index so next open forces a refresh
            s_art_track_idx = -1;
        }, LV_EVENT_DELETE, nullptr);

        init_status_bar(_registry.win.music.root);

        lv_obj_update_layout(_registry.win.music.root);

        // Auto-play if tracks available
        if (music_player_track_count() > 0) {
            _registry.win.music.play_flag = true;
            set_win_music_play_imagebutton_status(true);
            set_music_start_end(true);
        } else {
            // No tracks — show instructions
            if (_registry.win.music.label.song_name)
                lv_label_set_text(_registry.win.music.label.song_name, "No mp3s found in");
            if (_registry.win.music.label.artist)
                lv_label_set_text(_registry.win.music.label.artist, "/sdcard/music/");
        }

        _current_win = Current_Win::MUSIC;
    }

    void System::set_win_music_play_imagebutton_status(bool status)
    {
        if (status == false)
        {
            lv_obj_remove_state(_registry.win.music.imagebutton.play, LV_STATE_CHECKED);

            lv_obj_set_size(_registry.win.music.imagebutton.play, 140, 140);
#if defined SCREEN_ROTATION_DIRECTION_0
            lv_obj_align(_registry.win.music.imagebutton.play, LV_ALIGN_BOTTOM_MID, 0, -165);
#elif defined SCREEN_ROTATION_DIRECTION_90
            lv_obj_align(_registry.win.music.imagebutton.play, LV_ALIGN_BOTTOM_MID, 260, -165);
#else
#error "unknown macro definition, please select the correct macro definition."
#endif
            lv_obj_align_to(_registry.win.music.imagebutton.switch_left, _registry.win.music.imagebutton.play, LV_ALIGN_OUT_LEFT_MID, -10, 0);
            lv_obj_align_to(_registry.win.music.imagebutton.switch_right, _registry.win.music.imagebutton.play, LV_ALIGN_OUT_RIGHT_MID, 10, 0);
        }
        else
        {
            lv_obj_add_state(_registry.win.music.imagebutton.play, LV_STATE_CHECKED);

            lv_obj_set_size(_registry.win.music.imagebutton.play, 117, 117);
#if defined SCREEN_ROTATION_DIRECTION_0
            lv_obj_align(_registry.win.music.imagebutton.play, LV_ALIGN_BOTTOM_MID, 0, -177);
#elif defined SCREEN_ROTATION_DIRECTION_90
            lv_obj_align(_registry.win.music.imagebutton.play, LV_ALIGN_BOTTOM_MID, 260, -177);
#else
#error "unknown macro definition, please select the correct macro definition."
#endif
            lv_obj_align_to(_registry.win.music.imagebutton.switch_left, _registry.win.music.imagebutton.play, LV_ALIGN_OUT_LEFT_MID, -10, 0);
            lv_obj_align_to(_registry.win.music.imagebutton.switch_right, _registry.win.music.imagebutton.play, LV_ALIGN_OUT_RIGHT_MID, 10, 0);
        }
    }

    void System::set_win_music_current_total_time(double current_time_s, double total_time_s)
    {
        uint8_t sliding_percentage = 0;

        if (current_time_s == 0)
        {
            sliding_percentage = 0;
            _registry.win.music.current_time_s = 0;
            _registry.win.music.total_time_s = total_time_s;
        }
        else if (current_time_s > total_time_s)
        {
            sliding_percentage = 100;
            _registry.win.music.current_time_s = total_time_s;
            _registry.win.music.total_time_s = total_time_s;
        }
        else
        {
            sliding_percentage = (current_time_s / total_time_s) * 100;
            _registry.win.music.current_time_s = current_time_s;
            _registry.win.music.total_time_s = total_time_s;
        }

        // 格式化 current_time_s 和 total_time_s 格式
        char current_time_s_str[20];
        char total_time_s_str[20];
        snprintf(current_time_s_str, sizeof(current_time_s_str), "%ld:%02ld", static_cast<uint32_t>(_registry.win.music.current_time_s) / 60,
                 static_cast<uint32_t>(_registry.win.music.current_time_s) % 60);
        snprintf(total_time_s_str, sizeof(total_time_s_str), "%ld:%02ld", static_cast<uint32_t>(_registry.win.music.total_time_s) / 60,
                 static_cast<uint32_t>(_registry.win.music.total_time_s) % 60);

        lv_label_set_text(_registry.win.music.label.current_time, current_time_s_str);
        lv_label_set_text(_registry.win.music.label.total_time, total_time_s_str);
        lv_slider_set_value(_registry.win.music.slider, sliding_percentage, LV_ANIM_OFF);
    }

    void System::set_music_current_time_s(double current_time_s)
    {
        if (_set_music_current_time_s_callback != nullptr)
        {
            _set_music_current_time_s_callback(current_time_s);
        }
    }

    void System::set_music_start_end(bool status)
    {
        if (_win_music_start_end_callback != nullptr)
        {
            _win_music_start_end_callback(status);
        }
    }

    void System::create_keyboard(lv_obj_t *parent)
    {
        _registry.keyboard = lv_keyboard_create(parent);
        lv_obj_set_size(_registry.keyboard, _width, _height / 3.5);
        lv_obj_set_style_radius(_registry.keyboard, 8, (lv_style_selector_t)LV_PART_ITEMS | (lv_style_selector_t)LV_STATE_DEFAULT);
        // 设置键盘按钮间距更密集
        lv_obj_set_style_pad_row(_registry.keyboard, 8, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_column(_registry.keyboard, 4, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(_registry.keyboard, &lv_font_montserrat_26, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_align(_registry.keyboard, LV_ALIGN_BOTTOM_MID, 0, 0); // 对齐到屏幕底部
        lv_obj_add_flag(_registry.keyboard, LV_OBJ_FLAG_HIDDEN);     // 初始隐藏键盘

        static const char *kb_map_lower[] = {
            "q", "w", "e", "r", "t", "y", "u", "i", "o", "p", "\n",
            " ", "a", "s", "d", "f", "g", "h", "j", "k", "l", " ", "\n",
            LV_SYMBOL_EJECT, "z", "x", "c", "v", "b", "n", "m", LV_SYMBOL_BACKSPACE, "\n",
            LV_SYMBOL_KEYBOARD, LV_SYMBOL_LEFT, " ", LV_SYMBOL_RIGHT, LV_SYMBOL_NEW_LINE, NULL};

        static const char *kb_map_upper[] = {
            "Q", "W", "E", "R", "T", "Y", "U", "I", "O", "P", "\n",
            " ", "A", "S", "D", "F", "G", "H", "J", "K", "L", " ", "\n",
            LV_SYMBOL_EJECT, "Z", "X", "C", "V", "B", "N", "M", LV_SYMBOL_BACKSPACE, "\n",
            LV_SYMBOL_KEYBOARD, LV_SYMBOL_LEFT, " ", LV_SYMBOL_RIGHT, LV_SYMBOL_NEW_LINE, NULL};

        static const char *kb_map_num[] = {
            "1", "2", "3", LV_SYMBOL_BACKSPACE, "\n",
            "4", "5", "6", LV_SYMBOL_LEFT, LV_SYMBOL_RIGHT, "\n",
            "7", "8", "9", LV_SYMBOL_NEW_LINE, "\n",
            "+/-", "0", ".", LV_SYMBOL_KEYBOARD, NULL};

        static const char *kb_map_sym[] = {
            "!", "@", "#", "$", "%", "^", "&", "*", "(", ")", "\n",
            "_", "-", "+", "=", "[", "]", "{", "}", "|", ";", "\n",
            ":", "'", "\"", ",", "<", ">", ".", "?", "/", "\\", "`", "~", "\n",
            LV_SYMBOL_KEYBOARD, LV_SYMBOL_LEFT, LV_SYMBOL_RIGHT, LV_SYMBOL_BACKSPACE, NULL};

        static const lv_buttonmatrix_ctrl_t kb_ctrl[] =
            {
                // 字母键盘第一行
                LV_BUTTONMATRIX_CTRL_WIDTH_1,
                LV_BUTTONMATRIX_CTRL_WIDTH_1,
                LV_BUTTONMATRIX_CTRL_WIDTH_1,
                LV_BUTTONMATRIX_CTRL_WIDTH_1,
                LV_BUTTONMATRIX_CTRL_WIDTH_1,
                LV_BUTTONMATRIX_CTRL_WIDTH_1,
                LV_BUTTONMATRIX_CTRL_WIDTH_1,
                LV_BUTTONMATRIX_CTRL_WIDTH_1,
                LV_BUTTONMATRIX_CTRL_WIDTH_1,
                LV_BUTTONMATRIX_CTRL_WIDTH_1,

                // 字母键盘第二行
                LV_BUTTONMATRIX_CTRL_HIDDEN | LV_BUTTONMATRIX_CTRL_WIDTH_1,
                LV_BUTTONMATRIX_CTRL_WIDTH_2,
                LV_BUTTONMATRIX_CTRL_WIDTH_2,
                LV_BUTTONMATRIX_CTRL_WIDTH_2,
                LV_BUTTONMATRIX_CTRL_WIDTH_2,
                LV_BUTTONMATRIX_CTRL_WIDTH_2,
                LV_BUTTONMATRIX_CTRL_WIDTH_2,
                LV_BUTTONMATRIX_CTRL_WIDTH_2,
                LV_BUTTONMATRIX_CTRL_WIDTH_2,
                LV_BUTTONMATRIX_CTRL_WIDTH_2,
                LV_BUTTONMATRIX_CTRL_HIDDEN | LV_BUTTONMATRIX_CTRL_WIDTH_1,

                // 字母键盘第三行
                LV_BUTTONMATRIX_CTRL_CHECKED | LV_BUTTONMATRIX_CTRL_WIDTH_2,
                LV_BUTTONMATRIX_CTRL_WIDTH_1,
                LV_BUTTONMATRIX_CTRL_WIDTH_1,
                LV_BUTTONMATRIX_CTRL_WIDTH_1,
                LV_BUTTONMATRIX_CTRL_WIDTH_1,
                LV_BUTTONMATRIX_CTRL_WIDTH_1,
                LV_BUTTONMATRIX_CTRL_WIDTH_1,
                LV_BUTTONMATRIX_CTRL_WIDTH_1,
                LV_BUTTONMATRIX_CTRL_CHECKED | LV_BUTTONMATRIX_CTRL_WIDTH_2,

                // 字母键盘第四行
                LV_BUTTONMATRIX_CTRL_CHECKED | LV_BUTTONMATRIX_CTRL_WIDTH_2, // 键盘切换按钮
                LV_BUTTONMATRIX_CTRL_CHECKED | LV_BUTTONMATRIX_CTRL_WIDTH_1, // 左箭头
                LV_BUTTONMATRIX_CTRL_WIDTH_3,                                // Space
                LV_BUTTONMATRIX_CTRL_CHECKED | LV_BUTTONMATRIX_CTRL_WIDTH_1, // 右箭头
                LV_BUTTONMATRIX_CTRL_CHECKED | LV_BUTTONMATRIX_CTRL_WIDTH_2, // New line
            };

        // 为数字键盘创建单独的控制数组
        static const lv_buttonmatrix_ctrl_t kb_ctrl_num[] =
            {
                LV_BUTTONMATRIX_CTRL_WIDTH_1, LV_BUTTONMATRIX_CTRL_WIDTH_1, LV_BUTTONMATRIX_CTRL_WIDTH_1,
                LV_BUTTONMATRIX_CTRL_CHECKED | LV_BUTTONMATRIX_CTRL_WIDTH_2,

                LV_BUTTONMATRIX_CTRL_WIDTH_1, LV_BUTTONMATRIX_CTRL_WIDTH_1, LV_BUTTONMATRIX_CTRL_WIDTH_1,
                LV_BUTTONMATRIX_CTRL_CHECKED | LV_BUTTONMATRIX_CTRL_WIDTH_1, LV_BUTTONMATRIX_CTRL_CHECKED | LV_BUTTONMATRIX_CTRL_WIDTH_1,

                LV_BUTTONMATRIX_CTRL_WIDTH_1, LV_BUTTONMATRIX_CTRL_WIDTH_1, LV_BUTTONMATRIX_CTRL_WIDTH_1,
                LV_BUTTONMATRIX_CTRL_CHECKED | LV_BUTTONMATRIX_CTRL_WIDTH_2,

                LV_BUTTONMATRIX_CTRL_WIDTH_1, LV_BUTTONMATRIX_CTRL_WIDTH_1, LV_BUTTONMATRIX_CTRL_WIDTH_1,
                LV_BUTTONMATRIX_CTRL_CHECKED | LV_BUTTONMATRIX_CTRL_WIDTH_2};

        // 为符号键盘创建单独的控制数组
        static const lv_buttonmatrix_ctrl_t kb_ctrl_sym[] =
            {
                LV_BUTTONMATRIX_CTRL_WIDTH_1, LV_BUTTONMATRIX_CTRL_WIDTH_1, LV_BUTTONMATRIX_CTRL_WIDTH_1,
                LV_BUTTONMATRIX_CTRL_WIDTH_1, LV_BUTTONMATRIX_CTRL_WIDTH_1, LV_BUTTONMATRIX_CTRL_WIDTH_1,
                LV_BUTTONMATRIX_CTRL_WIDTH_1, LV_BUTTONMATRIX_CTRL_WIDTH_1, LV_BUTTONMATRIX_CTRL_WIDTH_1,
                LV_BUTTONMATRIX_CTRL_WIDTH_1,

                LV_BUTTONMATRIX_CTRL_WIDTH_1, LV_BUTTONMATRIX_CTRL_WIDTH_1, LV_BUTTONMATRIX_CTRL_WIDTH_1,
                LV_BUTTONMATRIX_CTRL_WIDTH_1, LV_BUTTONMATRIX_CTRL_WIDTH_1, LV_BUTTONMATRIX_CTRL_WIDTH_1,
                LV_BUTTONMATRIX_CTRL_WIDTH_1, LV_BUTTONMATRIX_CTRL_WIDTH_1, LV_BUTTONMATRIX_CTRL_WIDTH_1,
                LV_BUTTONMATRIX_CTRL_WIDTH_1,

                LV_BUTTONMATRIX_CTRL_WIDTH_1, LV_BUTTONMATRIX_CTRL_WIDTH_1, LV_BUTTONMATRIX_CTRL_WIDTH_1,
                LV_BUTTONMATRIX_CTRL_WIDTH_1, LV_BUTTONMATRIX_CTRL_WIDTH_1, LV_BUTTONMATRIX_CTRL_WIDTH_1,
                LV_BUTTONMATRIX_CTRL_WIDTH_1, LV_BUTTONMATRIX_CTRL_WIDTH_1, LV_BUTTONMATRIX_CTRL_WIDTH_1,
                LV_BUTTONMATRIX_CTRL_WIDTH_1, LV_BUTTONMATRIX_CTRL_WIDTH_1, LV_BUTTONMATRIX_CTRL_WIDTH_1,

                LV_BUTTONMATRIX_CTRL_CHECKED | LV_BUTTONMATRIX_CTRL_WIDTH_2,
                LV_BUTTONMATRIX_CTRL_CHECKED | LV_BUTTONMATRIX_CTRL_WIDTH_1,
                LV_BUTTONMATRIX_CTRL_CHECKED | LV_BUTTONMATRIX_CTRL_WIDTH_1,
                LV_BUTTONMATRIX_CTRL_CHECKED | LV_BUTTONMATRIX_CTRL_WIDTH_2};

        lv_keyboard_set_map(_registry.keyboard, LV_KEYBOARD_MODE_USER_1, kb_map_lower, kb_ctrl);
        lv_keyboard_set_map(_registry.keyboard, LV_KEYBOARD_MODE_USER_2, kb_map_upper, kb_ctrl);
        lv_keyboard_set_map(_registry.keyboard, LV_KEYBOARD_MODE_USER_3, kb_map_num, kb_ctrl_num);
        lv_keyboard_set_map(_registry.keyboard, LV_KEYBOARD_MODE_USER_4, kb_map_sym, kb_ctrl_sym);

        lv_keyboard_set_mode(_registry.keyboard, LV_KEYBOARD_MODE_USER_1);

        // 使用lambda表达式添加事件处理回调
        lv_obj_add_event_cb(_registry.keyboard, [](lv_event_t *e)
                            {
                                lv_obj_t * kb = lv_event_get_target_obj(e);
                                uint16_t btn_id = lv_keyboard_get_selected_button(kb);
                                
                                // 安全地获取按钮文本
                                const char * txt = lv_keyboard_get_button_text(kb, btn_id);
                                if(txt == nullptr) 
                                {
                                    return;  // 如果文本为空，直接返回
                                }
                                
                                // 处理键盘切换按钮
                                if(strcmp(txt, LV_SYMBOL_KEYBOARD) == 0) 
                                {
                                    // 获取当前模式并循环切换
                                    lv_keyboard_mode_t current_mode = lv_keyboard_get_mode(kb);
                                    lv_keyboard_mode_t next_mode;
                                    
                                    switch(current_mode) {
                                        case LV_KEYBOARD_MODE_USER_1: // 小写英文 -> 数字
                                            next_mode = LV_KEYBOARD_MODE_USER_3;
                                            break;
                                        case LV_KEYBOARD_MODE_USER_2: // 大写英文 -> 数字
                                            next_mode = LV_KEYBOARD_MODE_USER_3;
                                            break;
                                        case LV_KEYBOARD_MODE_USER_3: // 数字 -> 符号
                                            next_mode = LV_KEYBOARD_MODE_USER_4;
                                            break;
                                        case LV_KEYBOARD_MODE_USER_4: // 符号 -> 小写英文
                                            next_mode = LV_KEYBOARD_MODE_USER_1;
                                            break;
                                        default: // 默认回到小写英文
                                            next_mode = LV_KEYBOARD_MODE_USER_1;
                                            break;
                                    }
                                    
                                    lv_keyboard_set_mode(kb, next_mode);
                                }
                                else if(strcmp(txt, LV_SYMBOL_EJECT) == 0) 
                                {
                                    // 大小写切换功能保持不变
                                    lv_keyboard_mode_t mode = lv_keyboard_get_mode(kb);
                                    if(mode == LV_KEYBOARD_MODE_USER_1) 
                                    {
                                        lv_keyboard_set_mode(kb, LV_KEYBOARD_MODE_USER_2);
                                    } 
                                    else if(mode == LV_KEYBOARD_MODE_USER_2) 
                                    {
                                        lv_keyboard_set_mode(kb, LV_KEYBOARD_MODE_USER_1);
                                    }
                                } }, LV_EVENT_VALUE_CHANGED, nullptr);
    }

    void System::create_system_message_box(lv_obj_t *parent, std::string message_title, std::string message_data)
    {
        // 创建全屏灰色透明遮罩
        lv_obj_t *message_box_mask = lv_obj_create(parent);
        lv_obj_set_style_pad_top(message_box_mask, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_bottom(message_box_mask, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_left(message_box_mask, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_right(message_box_mask, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_size(message_box_mask, _width, _height);
        lv_obj_set_style_bg_color(message_box_mask, lv_color_black(), LV_PART_MAIN);
        lv_obj_set_style_bg_opa(message_box_mask, LV_OPA_30, LV_PART_MAIN);
        lv_obj_set_style_border_width(message_box_mask, 0, LV_PART_MAIN);
        lv_obj_set_style_radius(message_box_mask, 0, LV_PART_MAIN);
        lv_obj_align(message_box_mask, LV_ALIGN_CENTER, 0, 0);
        lv_obj_remove_flag(message_box_mask, LV_OBJ_FLAG_SCROLLABLE); // 禁止滚动

        // 灰色透明遮罩回调函数，删除整个消息框
        lv_obj_add_event_cb(message_box_mask, [](lv_event_t *e)
                            {
                                lv_obj_t *message_box_mask = lv_event_get_target_obj(e);
                                System *self = static_cast<System *>(lv_event_get_user_data(e));

                                lv_obj_remove_flag(message_box_mask, LV_OBJ_FLAG_CLICKABLE); // 禁止触摸

                                // 创建向下消失的动画
                                lv_anim_t anim;
                                lv_anim_init(&anim);
                                lv_anim_set_var(&anim, self->_registry.system_message_box.message_box);
                                lv_anim_set_exec_cb(&anim, (lv_anim_exec_xcb_t)lv_obj_set_y);
                                // 从当前位置移动到屏幕底部
                                lv_anim_set_values(&anim, -((self->_width / 7) / 4), self->_registry.system_message_box.system_message_height); // 屏幕底部
                                lv_anim_set_duration(&anim, 300);
                                lv_anim_set_delay(&anim, 0);
                                lv_anim_set_path_cb(&anim, lv_anim_path_ease_in);

                                lv_anim_set_completed_cb(&anim, [](lv_anim_t *anim)
                                                         {
                                    lv_obj_t *message_box = (lv_obj_t *)anim->var;
                                    lv_obj_t *message_box_mask = lv_obj_get_parent(message_box);
                                    lv_obj_delete(message_box_mask); });

                                lv_anim_start(&anim);

                                self->_registry.system_message_box.occupancy_flag = false; }, LV_EVENT_CLICKED, this);

        // 创建消息框容器
        _registry.system_message_box.message_box = lv_obj_create(message_box_mask);
        lv_obj_set_style_pad_all(_registry.system_message_box.message_box, 0, LV_PART_MAIN);
        lv_obj_set_size(_registry.system_message_box.message_box, _width - (_width / 7), _registry.system_message_box.system_message_height);
        lv_obj_set_style_bg_color(_registry.system_message_box.message_box, lv_color_white(), LV_PART_MAIN);
        lv_obj_set_style_radius(_registry.system_message_box.message_box, 45, LV_PART_MAIN);
        lv_obj_set_style_border_width(_registry.system_message_box.message_box, 0, LV_PART_MAIN);
        lv_obj_set_style_shadow_width(_registry.system_message_box.message_box, 16, LV_PART_MAIN);

        // 创建消息标题标签，顶部居中
        lv_obj_t *message_title_label = lv_label_create(_registry.system_message_box.message_box);
        lv_label_set_text(message_title_label, message_title.c_str());
        lv_obj_set_style_text_align(message_title_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
        lv_obj_set_style_text_font(message_title_label, &lv_font_montserrat_36, LV_PART_MAIN);
        lv_obj_set_style_text_color(message_title_label, lv_color_black(), LV_PART_MAIN);
        lv_obj_align(message_title_label, LV_ALIGN_TOP_MID, 0, 35);

        // 创建消息内容标签，居中显示
        lv_obj_t *message_label = lv_label_create(_registry.system_message_box.message_box);
        lv_label_set_text(message_label, message_data.c_str());
        lv_obj_set_style_text_align(message_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
        lv_obj_set_width(message_label, _width - (_width / 7) - 80);
        lv_label_set_long_mode(message_label, LV_LABEL_LONG_WRAP); // 自动换行
        lv_obj_set_style_text_font(message_label, &lv_font_montserrat_26, LV_PART_MAIN);
        lv_obj_set_style_text_color(message_label, lv_color_black(), LV_PART_MAIN);
        lv_obj_align_to(message_label, message_title_label, LV_ALIGN_OUT_BOTTOM_MID, 0, 30);

        // 创建OK按钮
        lv_obj_t *ok_btn = lv_button_create(_registry.system_message_box.message_box);
        lv_obj_set_size(ok_btn, (_width - _width / 8) * (2.0 / 3.0), 70);
        lv_obj_align(ok_btn, LV_ALIGN_CENTER, 0, 0);
        lv_obj_set_style_radius(ok_btn, 20, LV_PART_MAIN);
        lv_obj_set_style_shadow_width(ok_btn, 0, LV_PART_MAIN);
        lv_obj_align(ok_btn, LV_ALIGN_BOTTOM_MID, 0, -30);

        lv_obj_t *ok_label = lv_label_create(ok_btn);
        lv_label_set_text(ok_label, "OK");
        lv_obj_set_style_text_font(ok_label, &lv_font_montserrat_26, LV_PART_MAIN);
        lv_obj_center(ok_label);

        // 重新设置消息框正确高度
        _registry.system_message_box.system_message_height = 35 + 30 + lv_obj_get_height(message_title_label) + 40 + lv_obj_get_height(message_label) + 30 + 30 + 70;
        lv_obj_set_height(_registry.system_message_box.message_box, _registry.system_message_box.system_message_height);
        // 初始位置：隐藏在屏幕下方
        lv_obj_align(_registry.system_message_box.message_box, LV_ALIGN_BOTTOM_MID, 0, _registry.system_message_box.system_message_height);

        // OK按钮回调函数，删除整个消息框
        lv_obj_add_event_cb(ok_btn, [](lv_event_t *e)
                            {
                                lv_obj_t *ok_btn = lv_event_get_target_obj(e);
                                System *self = static_cast<System *>(lv_event_get_user_data(e));

                                lv_obj_remove_flag(ok_btn, LV_OBJ_FLAG_CLICKABLE); // 禁止触摸

                                // 创建向下消失的动画
                                lv_anim_t anim;
                                lv_anim_init(&anim);
                                lv_anim_set_var(&anim, self->_registry.system_message_box.message_box);
                                lv_anim_set_exec_cb(&anim, (lv_anim_exec_xcb_t)lv_obj_set_y);
                                // 从当前位置移动到屏幕底部
                                lv_anim_set_values(&anim, -((self->_width / 7) / 4), self->_registry.system_message_box.system_message_height);
                                lv_anim_set_duration(&anim, 300);
                                lv_anim_set_delay(&anim, 0);
                                lv_anim_set_path_cb(&anim, lv_anim_path_ease_in);

                                lv_anim_set_completed_cb(&anim, [](lv_anim_t *anim)
                                                         {
                                    lv_obj_t *message_box = (lv_obj_t *)anim->var;
                                    lv_obj_t *message_box_mask = lv_obj_get_parent(message_box);
                                    lv_obj_delete(message_box_mask); });

                                lv_anim_start(&anim);

                                self->_registry.system_message_box.occupancy_flag = false; }, LV_EVENT_CLICKED, this);

        lv_obj_add_event_cb(parent, [](lv_event_t *e)
                            {
                                System *self = static_cast<System *>(lv_event_get_user_data(e));
                                lv_event_code_t code = lv_event_get_code(e);

                                if (code == LV_EVENT_GESTURE)
                                {
                                    lv_dir_t gesture_dir = lv_indev_get_gesture_dir(lv_indev_active());

                                    // 边缘检测以及左右滑动
                                    if ((gesture_dir == LV_DIR_LEFT || gesture_dir == LV_DIR_RIGHT)&&(self->_edge_touch_flag == true))
                                    {   
                                        // 创建向下消失的动画
                                        lv_anim_t anim;
                                        lv_anim_init(&anim);
                                        lv_anim_set_var(&anim, self->_registry.system_message_box.message_box);
                                        lv_anim_set_exec_cb(&anim, (lv_anim_exec_xcb_t)lv_obj_set_y);
                                        // 从当前位置移动到屏幕底部
                                        lv_anim_set_values(&anim, -((self->_width / 7) / 4), self->_registry.system_message_box.system_message_height);             // 屏幕底部
                                        lv_anim_set_duration(&anim, 300);
                                        lv_anim_set_delay(&anim, 0);
                                        lv_anim_set_path_cb(&anim, lv_anim_path_ease_in);

                                        lv_anim_set_completed_cb(&anim, [](lv_anim_t *anim)
                                        {
                                            lv_obj_t *message_box = (lv_obj_t *)anim->var;
                                            lv_obj_t *message_box_mask = lv_obj_get_parent(message_box);
                                            lv_obj_delete(message_box_mask);
                                        });

                                        lv_anim_start(&anim); 

                                        self->_registry.system_message_box.occupancy_flag = false;

                                        self->set_vibration();

                                        self->_edge_touch_flag = false;
                                    }
                                } }, LV_EVENT_ALL, this);

        // 创建向上弹出的动画
        lv_anim_t anim;
        lv_anim_init(&anim);
        lv_anim_set_var(&anim, _registry.system_message_box.message_box);
        lv_anim_set_exec_cb(&anim, (lv_anim_exec_xcb_t)lv_obj_set_y);
        lv_anim_set_values(&anim, _registry.system_message_box.system_message_height, -((_width / 7) / 4)); // 从下方移动到正常位置
        lv_anim_set_duration(&anim, 300);
        lv_anim_set_delay(&anim, 0);
        lv_anim_set_path_cb(&anim, lv_anim_path_ease_out);

        lv_anim_start(&anim);

        _registry.system_message_box.occupancy_flag = true;
    }

#if defined CONFIG_BOARD_TYPE_T_DISPLAY_P4_KEYBOARD
    void System::init_win_cit_keyboard_test(void)
    {
        // 主界面
        _registry.win.cit.keyboard_test.root = lv_obj_create(NULL);
        lv_obj_set_style_bg_color(_registry.win.cit.keyboard_test.root, lv_color_hex(0xFF7F58), (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_size(_registry.win.cit.keyboard_test.root, _width, _height);
        lv_obj_set_scrollbar_mode(_registry.win.cit.keyboard_test.root, LV_SCROLLBAR_MODE_OFF);

        // 创建标题
        lv_obj_t *title_label = lv_label_create(_registry.win.cit.keyboard_test.root);
        lv_label_set_text(title_label, "Keyboard");
        lv_obj_set_style_text_color(title_label, lv_color_white(), (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_text_align(title_label, LV_TEXT_ALIGN_LEFT, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(title_label, &lv_font_montserrat_48, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_size(title_label, _width - 100, 40);
        lv_obj_align(title_label, LV_ALIGN_TOP_LEFT, 20, 10 + 50);

        // 创建容器
        lv_obj_t *container = lv_obj_create(_registry.win.cit.keyboard_test.root);
        lv_obj_set_size(container, _width, _height - 50 - 80 - 140);
        lv_obj_align(container, LV_ALIGN_TOP_MID, 0, 50 + 80);
        lv_obj_set_style_bg_color(container, lv_color_white(), (lv_style_selector_t)LV_PART_MAIN); // 设置背景颜色为白色
        lv_obj_set_style_radius(container, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_border_width(container, 0, (lv_style_selector_t)LV_PART_MAIN); // 移除边框

        // 创建一个标签用于显示键盘数据
        _registry.win.cit.keyboard_test.data_label = lv_label_create(container);
        lv_obj_set_style_text_color(_registry.win.cit.keyboard_test.data_label, lv_color_black(), (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(_registry.win.cit.keyboard_test.data_label, &lv_font_montserrat_48, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_label_set_text(_registry.win.cit.keyboard_test.data_label, "null");
        lv_obj_align(_registry.win.cit.keyboard_test.data_label, LV_ALIGN_TOP_LEFT, 0, 0);

        // 创建一个半透明文本框
        lv_obj_t *textarea = lv_textarea_create(container);
        lv_obj_set_width(textarea, 600);
        lv_obj_set_height(textarea, 200);
        lv_obj_set_style_text_font(textarea, &lv_font_montserrat_24, 0);
        lv_obj_center(textarea);

        lv_group_remove_all_objs(_registry.keyboard_group);
        lv_group_add_obj(_registry.keyboard_group, textarea);

        lv_obj_add_event_cb(_registry.win.cit.keyboard_test.root, [](lv_event_t *e)
                            {
                                    System *self = static_cast<System *>(lv_event_get_user_data(e));
                                    lv_event_code_t code = lv_event_get_code(e);
    
                                    if (code == LV_EVENT_GESTURE)
                                    {
                                        lv_dir_t gesture_dir = lv_indev_get_gesture_dir(lv_indev_active());
    
                                        // 边缘检测以及左右滑动
                                        if ((gesture_dir == LV_DIR_LEFT || gesture_dir == LV_DIR_RIGHT)&&(self->_edge_touch_flag == true))
                                        {
                                            lv_group_remove_all_objs(self->_registry.keyboard_group);
                                            
                                            self->set_vibration();
                                            self->init_win_cit();
                                            
                                            lv_screen_load_anim(self->_registry.win.cit.root, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 100, 0, true);
    
                                            self->_edge_touch_flag = false;
                                        }
                                    } }, LV_EVENT_ALL, this);

        add_win_cit_test_item_pass_fail_button(_registry.win.cit.keyboard_test.root);

        add_event_cb_win_return_to_cit(_registry.win.cit.keyboard_test.root);

        init_status_bar(_registry.win.cit.keyboard_test.root);

        lv_obj_update_layout(_registry.win.cit.keyboard_test.root);

        _current_win = Current_Win::CIT_KEYBOARD_TEST;
    }

    bool System::set_keyboard_group(lv_group_t *group)
    {
        // 查找类型为KEYPAD的输入设备
        lv_indev_t *kb_indev = nullptr;
        lv_indev_t *indev_iter = lv_indev_get_next(NULL);
        while (indev_iter)
        {
            if (lv_indev_get_type(indev_iter) == LV_INDEV_TYPE_KEYPAD)
            {
                kb_indev = indev_iter;
                break;
            }
            indev_iter = lv_indev_get_next(indev_iter);
        }
        if (kb_indev == nullptr)
        {
            return false;
        }

        lv_indev_set_group(kb_indev, group);
        return true;
    }

    void System::set_nfc_test(bool status)
    {
        if (_win_cit_nfc_test_callback != nullptr)
        {
            _win_cit_nfc_test_callback(status);
        }
    }

    void System::init_win_cit_nfc_test(void)
    {
        // 主界面
        _registry.win.cit.nfc_test.root = lv_obj_create(NULL);
        lv_obj_set_style_bg_color(_registry.win.cit.nfc_test.root, lv_color_hex(0xFF7F58), (lv_style_selector_t)LV_PART_MAIN);
        lv_obj_set_size(_registry.win.cit.nfc_test.root, _width, _height);
        lv_obj_set_scrollbar_mode(_registry.win.cit.nfc_test.root, LV_SCROLLBAR_MODE_OFF);

        // 创建标题
        lv_obj_t *title_label = lv_label_create(_registry.win.cit.nfc_test.root);
        lv_label_set_text(title_label, "Nfc");
        lv_obj_set_style_text_color(title_label, lv_color_white(), (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_text_align(title_label, LV_TEXT_ALIGN_LEFT, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(title_label, &lv_font_montserrat_48, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_size(title_label, _width - 100, 40);
        lv_obj_align(title_label, LV_ALIGN_TOP_LEFT, 20, 10 + 50);

        // 创建容器
        lv_obj_t *container = lv_obj_create(_registry.win.cit.nfc_test.root);
        lv_obj_set_size(container, _width, _height - 50 - 80 - 140);
        lv_obj_align(container, LV_ALIGN_TOP_MID, 0, 50 + 80);
        lv_obj_set_style_bg_color(container, lv_color_white(), (lv_style_selector_t)LV_PART_MAIN); // 设置背景颜色为白色
        lv_obj_set_style_radius(container, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_border_width(container, 0, (lv_style_selector_t)LV_PART_MAIN); // 移除边框

        // 创建一个标签用于显示nfc数据
        _registry.win.cit.nfc_test.data_label = lv_label_create(container);
        lv_obj_set_style_text_color(_registry.win.cit.nfc_test.data_label, lv_color_black(), (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(_registry.win.cit.nfc_test.data_label, &lv_font_montserrat_24, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_label_set_text(_registry.win.cit.nfc_test.data_label, "1. Tap a tag to read its content");
        lv_obj_align(_registry.win.cit.nfc_test.data_label, LV_ALIGN_CENTER, 0, 0);

        lv_obj_add_event_cb(_registry.win.cit.nfc_test.root, [](lv_event_t *e)
                            {
                                    System *self = static_cast<System *>(lv_event_get_user_data(e));
                                    lv_event_code_t code = lv_event_get_code(e);
    
                                    if (code == LV_EVENT_GESTURE)
                                    {
                                        lv_dir_t gesture_dir = lv_indev_get_gesture_dir(lv_indev_active());
    
                                        // 边缘检测以及左右滑动
                                        if ((gesture_dir == LV_DIR_LEFT || gesture_dir == LV_DIR_RIGHT)&&(self->_edge_touch_flag == true))
                                        {   
                                            self->set_nfc_test(false);

                                            self->set_vibration();
                                            self->init_win_cit();
                                            
                                            lv_screen_load_anim(self->_registry.win.cit.root, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 100, 0, true);
    
                                            self->_edge_touch_flag = false;
                                        }
                                    } }, LV_EVENT_ALL, this);

        add_win_cit_test_item_pass_fail_button(_registry.win.cit.nfc_test.root);

        add_event_cb_win_return_to_cit(_registry.win.cit.nfc_test.root);

        init_status_bar(_registry.win.cit.nfc_test.root);

        lv_obj_update_layout(_registry.win.cit.nfc_test.root);

        _current_win = Current_Win::CIT_NFC_TEST;

        set_nfc_test(true);
    }

    void System::init_win_rf_setings_config_cc1101_params_message_box(void)
    {
        // 创建全屏灰色透明遮罩
        _registry.win.rf.setings.message_box.root = lv_obj_create(_registry.win.rf.setings.root);
        lv_obj_set_style_pad_top(_registry.win.rf.setings.message_box.root, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_bottom(_registry.win.rf.setings.message_box.root, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_left(_registry.win.rf.setings.message_box.root, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_right(_registry.win.rf.setings.message_box.root, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_size(_registry.win.rf.setings.message_box.root, _width, _height);
        lv_obj_set_style_bg_color(_registry.win.rf.setings.message_box.root, lv_color_black(), LV_PART_MAIN);
        lv_obj_set_style_bg_opa(_registry.win.rf.setings.message_box.root, LV_OPA_30, LV_PART_MAIN);
        lv_obj_set_style_border_width(_registry.win.rf.setings.message_box.root, 0, LV_PART_MAIN);
        lv_obj_set_style_radius(_registry.win.rf.setings.message_box.root, 0, LV_PART_MAIN);
        lv_obj_align(_registry.win.rf.setings.message_box.root, LV_ALIGN_CENTER, 0, 0);
        lv_obj_add_flag(_registry.win.rf.setings.message_box.root, LV_OBJ_FLAG_CLICKABLE); // 添加触摸标志来禁止其他界面触摸

        _registry.win.rf.setings.message_box.root_container = lv_obj_create(_registry.win.rf.setings.message_box.root);
        lv_obj_set_style_pad_top(_registry.win.rf.setings.message_box.root_container, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_bottom(_registry.win.rf.setings.message_box.root_container, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_left(_registry.win.rf.setings.message_box.root_container, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_right(_registry.win.rf.setings.message_box.root_container, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_size(_registry.win.rf.setings.message_box.root_container, 450, _height * 0.7);
        lv_obj_set_style_radius(_registry.win.rf.setings.message_box.root_container, 15, LV_PART_MAIN);
        lv_obj_set_style_bg_color(_registry.win.rf.setings.message_box.root_container, lv_color_white(), LV_PART_MAIN);
        lv_obj_set_style_border_width(_registry.win.rf.setings.message_box.root_container, 0, LV_PART_MAIN);
        lv_obj_set_style_shadow_width(_registry.win.rf.setings.message_box.root_container, 16, LV_PART_MAIN);
        lv_obj_center(_registry.win.rf.setings.message_box.root_container);

        _registry.win.rf.setings.message_box.btn_container = lv_obj_create(_registry.win.rf.setings.message_box.root_container);
        lv_obj_set_style_pad_top(_registry.win.rf.setings.message_box.btn_container, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_bottom(_registry.win.rf.setings.message_box.btn_container, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_left(_registry.win.rf.setings.message_box.btn_container, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_right(_registry.win.rf.setings.message_box.btn_container, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_size(_registry.win.rf.setings.message_box.btn_container, 450, 110);
        lv_obj_set_style_border_width(_registry.win.rf.setings.message_box.btn_container, 0, (lv_style_selector_t)LV_PART_MAIN); // 移除边框
        lv_obj_align(_registry.win.rf.setings.message_box.btn_container, LV_ALIGN_BOTTOM_MID, 0, 0);

        lv_obj_t *btn_cancel = lv_button_create(_registry.win.rf.setings.message_box.btn_container);
        lv_obj_set_size(btn_cancel, 150, 60);
        lv_obj_align(btn_cancel, LV_ALIGN_BOTTOM_LEFT, 40, -30);
        lv_obj_set_style_radius(btn_cancel, 10, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_width(btn_cancel, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT); // 去除按钮阴影
        lv_obj_t *btn_cancel_label = lv_label_create(btn_cancel);
        lv_label_set_text(btn_cancel_label, "cancel");
        lv_obj_set_style_text_font(btn_cancel_label, &lv_font_montserrat_26, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_center(btn_cancel_label);

        // cancel 按钮回调
        lv_obj_add_event_cb(btn_cancel, [](lv_event_t *e)
                            {
                                System *self = static_cast<System *>(lv_event_get_user_data(e));
                                lv_obj_delete(self->_registry.win.rf.setings.message_box.root); }, LV_EVENT_CLICKED, this);

        lv_obj_t *btn_apply = lv_button_create(_registry.win.rf.setings.message_box.btn_container);
        lv_obj_set_size(btn_apply, 150, 60);
        lv_obj_align(btn_apply, LV_ALIGN_BOTTOM_RIGHT, -40, -30);
        lv_obj_set_style_radius(btn_apply, 10, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_width(btn_apply, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT); // 去除按钮阴影
        lv_obj_t *btn_apply_label = lv_label_create(btn_apply);
        lv_label_set_text(btn_apply_label, "apply");
        lv_obj_set_style_text_font(btn_apply_label, &lv_font_montserrat_26, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_center(btn_apply_label);

        // apply 按钮回调
        lv_obj_add_event_cb(btn_apply, [](lv_event_t *e)
                            {
                                System *self = static_cast<System *>(lv_event_get_user_data(e));

                                Device_Cc1101 dc;

                                dc.params.rf_switch = static_cast<uint8_t>(lv_dropdown_get_selected(self->_registry.win.rf.setings.config_rf_params.cc1101.dropdown.rf_switch));

                                const char* freq_text = lv_textarea_get_text(self->_registry.win.rf.setings.config_rf_params.cc1101.textarea.freq);
                                if (freq_text != nullptr && freq_text[0] != '\0') // 同时检查NULL和空字符串
                                {  
                                    double buffer = std::stod(freq_text, nullptr);  

                                    // 限制范围
                                    if (buffer <= 300.0) 
                                    {
                                        dc.params.freq = 300.0;
                                    } 
                                    else if (buffer <= 348.0) 
                                    {
                                        dc.params.freq = buffer;
                                    } 
                                    else if (buffer < 387.0) 
                                    {
                                        dc.params.freq = 348.0;
                                    } 
                                    else if (buffer <= 464.0) 
                                    {
                                        dc.params.freq = buffer;
                                    } 
                                    else if (buffer < 779.0)
                                    {
                                        dc.params.freq = 464.0;
                                    } 
                                    else if (buffer <= 928.0) 
                                    {
                                        dc.params.freq = buffer;
                                    } 
                                    else 
                                    {
                                        dc.params.freq = 928.0;
                                    }
                                }

                                uint32_t bandwidth_buffer_index = lv_dropdown_get_selected(self->_registry.win.rf.setings.config_rf_params.cc1101.dropdown.bandwidth);
                                dc.params.bandwidth = static_cast<Cc1101_Bw>(bandwidth_buffer_index);

                                const char* bit_rate_text = lv_textarea_get_text(self->_registry.win.rf.setings.config_rf_params.cc1101.textarea.bit_rate);
                                if (bit_rate_text != nullptr && bit_rate_text[0] != '\0') // 同时检查NULL和空字符串
                                {  
                                    float buffer = std::stof(bit_rate_text, nullptr);  

                                    // 限制范围
                                    if(buffer <= 0.025)
                                    {
                                        dc.params.bit_rate = 0.025;
                                    }
                                    else if(buffer <= 600.0)
                                    {
                                        dc.params.bit_rate = buffer;
                                    }
                                    else
                                    {
                                        dc.params.bit_rate = 600.0;
                                    }
                                }

                                const char* freq_deviation_khz_text = lv_textarea_get_text(self->_registry.win.rf.setings.config_rf_params.cc1101.textarea.freq_deviation_khz);
                                if (freq_deviation_khz_text != nullptr && freq_deviation_khz_text[0] != '\0') // 同时检查NULL和空字符串
                                {  
                                    float buffer = std::stof(freq_deviation_khz_text, nullptr);  

                                    // 限制范围
                                    if(buffer <= 1.587)
                                    {
                                        dc.params.freq_deviation_khz = 1.587;
                                    }
                                    else if(buffer <= 380.8)
                                    {
                                        dc.params.freq_deviation_khz = buffer;
                                    }
                                    else
                                    {
                                        dc.params.freq_deviation_khz = 380.8;
                                    }
                                }

                                const char* power_text = lv_textarea_get_text(self->_registry.win.rf.setings.config_rf_params.cc1101.textarea.power);
                                if (power_text != nullptr && power_text[0] != '\0') // 同时检查NULL和空字符串
                                {  
                                    int8_t buffer = std::stoi(power_text);  

                                    // 限制范围
                                    // 如果在范围内，选择最接近的较大允许值
                                    // 允许的值：-30, -20, -15, -10, 0, 5, 7, 10
                                    if(buffer <= -30) 
                                    {
                                        dc.params.power = -30;
                                    }
                                    else if(buffer <= -20) 
                                    {
                                        dc.params.power = -20;
                                    }
                                    else if(buffer <= -15) 
                                    {
                                        dc.params.power = -15;
                                    }
                                    else if(buffer <= -10) 
                                    {
                                        dc.params.power = -10;
                                    }
                                    else if(buffer <= 0) 
                                    {
                                        dc.params.power = 0;
                                    }
                                    else if(buffer <= 5) 
                                    {
                                        dc.params.power = 5;
                                    }
                                    else if(buffer <= 7) 
                                    {
                                        dc.params.power = 7;
                                    }
                                    else 
                                    {
                                        dc.params.power = 10;
                                    }
                                }

                                const char* preamble_length_text = lv_textarea_get_text(self->_registry.win.rf.setings.config_rf_params.cc1101.textarea.preamble_length);
                                if (preamble_length_text != nullptr && preamble_length_text[0] != '\0') // 同时检查NULL和空字符串
                                {  
                                    dc.params.preamble_length = std::stoi(preamble_length_text);  
                                }

                                const char* sync_word_text = lv_textarea_get_text(self->_registry.win.rf.setings.config_rf_params.cc1101.textarea.sync_word);
                                if (sync_word_text != nullptr && sync_word_text[0] != '\0') // 同时检查NULL和空字符串
                                {  
                                    dc.params.sync_word = std::stoi(sync_word_text);  
                                }

                                if(self->set_config_rf_params(dc) == true)
                                {
                                    self->_device_cc1101.params = dc.params;
                                }

                                lv_obj_delete(self->_registry.win.rf.setings.message_box.root); }, LV_EVENT_CLICKED, this);

        _registry.win.rf.setings.message_box.parameter_container = lv_obj_create(_registry.win.rf.setings.message_box.root_container);
        lv_obj_set_style_pad_top(_registry.win.rf.setings.message_box.parameter_container, 30, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_bottom(_registry.win.rf.setings.message_box.parameter_container, 30, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_left(_registry.win.rf.setings.message_box.parameter_container, 30, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_right(_registry.win.rf.setings.message_box.parameter_container, 30, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
#if defined SCREEN_ROTATION_DIRECTION_0
        lv_obj_set_size(_registry.win.rf.setings.message_box.parameter_container, 450, _height * 0.6);
#elif defined SCREEN_ROTATION_DIRECTION_90
        lv_obj_set_size(_registry.win.rf.setings.message_box.parameter_container, 450, _height * 0.5);
#else
#error "unknown macro definition, please select the correct macro definition."
#endif
        lv_obj_set_style_border_width(_registry.win.rf.setings.message_box.parameter_container, 0, (lv_style_selector_t)LV_PART_MAIN); // 移除边框
        lv_obj_set_scrollbar_mode(_registry.win.rf.setings.message_box.parameter_container, LV_SCROLLBAR_MODE_ACTIVE);
        lv_obj_align_to(_registry.win.rf.setings.message_box.parameter_container, _registry.win.rf.setings.message_box.btn_container, LV_ALIGN_OUT_TOP_MID, 0, 0);

        // 触摸设置消息框区域时隐藏键盘
        lv_obj_add_event_cb(_registry.win.rf.setings.message_box.parameter_container, [](lv_event_t *e)
                            {
                                System *self = static_cast<System *>(lv_event_get_user_data(e));
                                lv_event_code_t code = lv_event_get_code(e);

                                if (code == LV_EVENT_CLICKED)
                                {
                                    lv_group_remove_all_objs(self->_registry.keyboard_group);
                                } }, LV_EVENT_ALL, this);

        lv_obj_t *msgbox_rf_switch_text = lv_label_create(_registry.win.rf.setings.message_box.parameter_container);
        lv_label_set_text(msgbox_rf_switch_text, "rf switch");
        lv_obj_set_size(msgbox_rf_switch_text, 300, 40);
        lv_obj_set_style_text_font(msgbox_rf_switch_text, &lv_font_montserrat_26, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_align(msgbox_rf_switch_text, LV_ALIGN_TOP_LEFT, 0, 0);

        _registry.win.rf.setings.config_rf_params.cc1101.dropdown.rf_switch = lv_dropdown_create(_registry.win.rf.setings.message_box.parameter_container);
        lv_dropdown_set_dir(_registry.win.rf.setings.config_rf_params.cc1101.dropdown.rf_switch, LV_DIR_BOTTOM);
        lv_dropdown_set_options(_registry.win.rf.setings.config_rf_params.cc1101.dropdown.rf_switch, "RF_315MHZ\n"
                                                                                                     "RF_434MHZ\n"
                                                                                                     "RF_868_915MHZ");
        lv_dropdown_set_selected(_registry.win.rf.setings.config_rf_params.cc1101.dropdown.rf_switch, static_cast<uint32_t>(_device_cc1101.params.rf_switch));
        lv_obj_set_style_pad_top(_registry.win.rf.setings.config_rf_params.cc1101.dropdown.rf_switch, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_bottom(_registry.win.rf.setings.config_rf_params.cc1101.dropdown.rf_switch, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_left(_registry.win.rf.setings.config_rf_params.cc1101.dropdown.rf_switch, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_right(_registry.win.rf.setings.config_rf_params.cc1101.dropdown.rf_switch, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_min_width(_registry.win.rf.setings.config_rf_params.cc1101.dropdown.rf_switch, 200, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_min_height(_registry.win.rf.setings.config_rf_params.cc1101.dropdown.rf_switch, 30, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(_registry.win.rf.setings.config_rf_params.cc1101.dropdown.rf_switch, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);  // 输入框字体
        lv_obj_set_style_text_font(_registry.win.rf.setings.config_rf_params.cc1101.dropdown.rf_switch, &lv_font_montserrat_24, LV_PART_ITEMS | LV_STATE_DEFAULT); // 下拉列表字体
        lv_obj_align_to(_registry.win.rf.setings.config_rf_params.cc1101.dropdown.rf_switch, msgbox_rf_switch_text, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);

        lv_obj_add_event_cb(_registry.win.rf.setings.config_rf_params.cc1101.dropdown.rf_switch, [](lv_event_t *e)
                            {
                                lv_obj_t *dropdown = lv_event_get_target_obj(e);
                                lv_event_code_t code = lv_event_get_code(e);
                                
                                if (code == LV_EVENT_CLICKED) 
                                {
                                    // // 强制下拉列表向下打开
                                    // lv_dropdown_set_dir(dropdown, LV_DIR_BOTTOM);
                                    // 获取弹出的下拉列表对象
                                    lv_obj_t *list = lv_dropdown_get_list(dropdown);
                                    // // 设置下拉列表最多显示100高度
                                    // lv_obj_set_height(list, 300);

                                    lv_obj_set_style_bg_color(list, lv_color_hex(0xEEE9E9), LV_PART_MAIN | LV_STATE_DEFAULT);
                                    lv_obj_set_scrollbar_mode(list, LV_SCROLLBAR_MODE_ACTIVE);
                                    lv_obj_set_style_border_width(list, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
                                    
                                    lv_obj_set_style_text_font(list, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
                                    lv_obj_set_style_text_font(list, &lv_font_montserrat_24, LV_PART_ITEMS | LV_STATE_DEFAULT);
                                } }, LV_EVENT_ALL, NULL);

        lv_obj_t *msgbox_freq_text = lv_label_create(_registry.win.rf.setings.message_box.parameter_container);
        lv_label_set_text(msgbox_freq_text, "freq");
        lv_obj_set_size(msgbox_freq_text, 100, 40);
        lv_obj_set_style_text_font(msgbox_freq_text, &lv_font_montserrat_26, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_align_to(msgbox_freq_text, _registry.win.rf.setings.config_rf_params.cc1101.dropdown.rf_switch, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 10);

        _registry.win.rf.setings.config_rf_params.cc1101.textarea.freq = lv_textarea_create(_registry.win.rf.setings.message_box.parameter_container);
        lv_textarea_set_accepted_chars(_registry.win.rf.setings.config_rf_params.cc1101.textarea.freq, "0123456789."); // 只允许输入数字和小数点
        lv_obj_set_style_pad_top(_registry.win.rf.setings.config_rf_params.cc1101.textarea.freq, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_bottom(_registry.win.rf.setings.config_rf_params.cc1101.textarea.freq, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_left(_registry.win.rf.setings.config_rf_params.cc1101.textarea.freq, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_right(_registry.win.rf.setings.config_rf_params.cc1101.textarea.freq, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_width(_registry.win.rf.setings.config_rf_params.cc1101.textarea.freq, 300);
        lv_textarea_set_one_line(_registry.win.rf.setings.config_rf_params.cc1101.textarea.freq, true);
        char freq_str[15];
        snprintf(freq_str, sizeof(freq_str), "%.6f", _device_cc1101.params.freq);
        lv_textarea_set_text(_registry.win.rf.setings.config_rf_params.cc1101.textarea.freq, freq_str);
        lv_obj_set_style_text_font(_registry.win.rf.setings.config_rf_params.cc1101.textarea.freq, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_align_to(_registry.win.rf.setings.config_rf_params.cc1101.textarea.freq, msgbox_freq_text, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);

        init_win_rf_setings_keyboard_position_event_cb(_registry.win.rf.setings.config_rf_params.cc1101.textarea.freq);

        lv_obj_t *freq_unit_text = lv_label_create(_registry.win.rf.setings.message_box.parameter_container);
        lv_label_set_text(freq_unit_text, "mhz");
        lv_obj_set_size(freq_unit_text, 70, 40);
        lv_obj_set_style_text_font(freq_unit_text, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_align_to(freq_unit_text, _registry.win.rf.setings.config_rf_params.cc1101.textarea.freq, LV_ALIGN_OUT_RIGHT_BOTTOM, 10, 0);

        lv_obj_t *msgbox_bandwidth_text = lv_label_create(_registry.win.rf.setings.message_box.parameter_container);
        lv_label_set_text(msgbox_bandwidth_text, "bandwidth");
        lv_obj_set_size(msgbox_bandwidth_text, 200, 40);
        lv_obj_set_style_text_font(msgbox_bandwidth_text, &lv_font_montserrat_26, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_align_to(msgbox_bandwidth_text, _registry.win.rf.setings.config_rf_params.cc1101.textarea.freq, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 10);

        _registry.win.rf.setings.config_rf_params.cc1101.dropdown.bandwidth = lv_dropdown_create(_registry.win.rf.setings.message_box.parameter_container);
        lv_dropdown_set_dir(_registry.win.rf.setings.config_rf_params.cc1101.dropdown.bandwidth, LV_DIR_BOTTOM);
        lv_dropdown_set_options(_registry.win.rf.setings.config_rf_params.cc1101.dropdown.bandwidth, "BW_58\n"
                                                                                                     "BW_68\n"
                                                                                                     "BW_81\n"
                                                                                                     "BW_102\n"
                                                                                                     "BW_116\n"
                                                                                                     "BW_135\n"
                                                                                                     "BW_162\n"
                                                                                                     "BW_203\n"
                                                                                                     "BW_232\n"
                                                                                                     "BW_270\n"
                                                                                                     "BW_325\n"
                                                                                                     "BW_406\n"
                                                                                                     "BW_464\n"
                                                                                                     "BW_541\n"
                                                                                                     "BW_650\n"
                                                                                                     "BW_812");
        uint32_t buffer_index = static_cast<uint32_t>(_device_cc1101.params.bandwidth);
        lv_dropdown_set_selected(_registry.win.rf.setings.config_rf_params.cc1101.dropdown.bandwidth, buffer_index);
        lv_obj_set_style_pad_top(_registry.win.rf.setings.config_rf_params.cc1101.dropdown.bandwidth, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_bottom(_registry.win.rf.setings.config_rf_params.cc1101.dropdown.bandwidth, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_left(_registry.win.rf.setings.config_rf_params.cc1101.dropdown.bandwidth, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_right(_registry.win.rf.setings.config_rf_params.cc1101.dropdown.bandwidth, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_min_width(_registry.win.rf.setings.config_rf_params.cc1101.dropdown.bandwidth, 200, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_min_height(_registry.win.rf.setings.config_rf_params.cc1101.dropdown.bandwidth, 30, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(_registry.win.rf.setings.config_rf_params.cc1101.dropdown.bandwidth, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);  // 输入框字体
        lv_obj_set_style_text_font(_registry.win.rf.setings.config_rf_params.cc1101.dropdown.bandwidth, &lv_font_montserrat_24, LV_PART_ITEMS | LV_STATE_DEFAULT); // 下拉列表字体
        lv_obj_align_to(_registry.win.rf.setings.config_rf_params.cc1101.dropdown.bandwidth, msgbox_bandwidth_text, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);

        lv_obj_add_event_cb(_registry.win.rf.setings.config_rf_params.cc1101.dropdown.bandwidth, [](lv_event_t *e)
                            {
                                lv_obj_t *dropdown = lv_event_get_target_obj(e);
                                lv_event_code_t code = lv_event_get_code(e);
                                
                                if (code == LV_EVENT_CLICKED) 
                                {
                                    // // 强制下拉列表向下打开
                                    // lv_dropdown_set_dir(dropdown, LV_DIR_BOTTOM);
                                    // 获取弹出的下拉列表对象
                                    lv_obj_t *list = lv_dropdown_get_list(dropdown);
                                    // // 设置下拉列表最多显示100高度
                                    // lv_obj_set_height(list, 300);
                                    lv_obj_set_style_bg_color(list, lv_color_hex(0xEEE9E9), LV_PART_MAIN | LV_STATE_DEFAULT);
                                    lv_obj_set_scrollbar_mode(list, LV_SCROLLBAR_MODE_ACTIVE);
                                    lv_obj_set_style_border_width(list, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
                                    
                                    lv_obj_set_style_text_font(list, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
                                    lv_obj_set_style_text_font(list, &lv_font_montserrat_24, LV_PART_ITEMS | LV_STATE_DEFAULT);
                                } }, LV_EVENT_ALL, NULL);

        lv_obj_t *bandwidth_unit_text = lv_label_create(_registry.win.rf.setings.message_box.parameter_container);
        lv_label_set_text(bandwidth_unit_text, "khz");
        lv_obj_set_size(bandwidth_unit_text, 70, 40);
        lv_obj_set_style_text_font(bandwidth_unit_text, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_align_to(bandwidth_unit_text, _registry.win.rf.setings.config_rf_params.cc1101.dropdown.bandwidth, LV_ALIGN_OUT_RIGHT_BOTTOM, 10, 0);

        lv_obj_t *msgbox_current_limit_text = lv_label_create(_registry.win.rf.setings.message_box.parameter_container);
        lv_label_set_text(msgbox_current_limit_text, "bit rate");
        lv_obj_set_size(msgbox_current_limit_text, 300, 40);
        lv_obj_set_style_text_font(msgbox_current_limit_text, &lv_font_montserrat_26, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_align_to(msgbox_current_limit_text, _registry.win.rf.setings.config_rf_params.cc1101.dropdown.bandwidth, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 10);

        _registry.win.rf.setings.config_rf_params.cc1101.textarea.bit_rate = lv_textarea_create(_registry.win.rf.setings.message_box.parameter_container);
        lv_textarea_set_accepted_chars(_registry.win.rf.setings.config_rf_params.cc1101.textarea.bit_rate, "0123456789."); // 只允许输入数字和小数点
        lv_obj_set_style_pad_top(_registry.win.rf.setings.config_rf_params.cc1101.textarea.bit_rate, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_bottom(_registry.win.rf.setings.config_rf_params.cc1101.textarea.bit_rate, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_left(_registry.win.rf.setings.config_rf_params.cc1101.textarea.bit_rate, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_right(_registry.win.rf.setings.config_rf_params.cc1101.textarea.bit_rate, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_width(_registry.win.rf.setings.config_rf_params.cc1101.textarea.bit_rate, 300);
        lv_textarea_set_one_line(_registry.win.rf.setings.config_rf_params.cc1101.textarea.bit_rate, true);
        char bit_rate_str[10];
        snprintf(bit_rate_str, sizeof(bit_rate_str), "%.3f", _device_cc1101.params.bit_rate);
        lv_textarea_set_text(_registry.win.rf.setings.config_rf_params.cc1101.textarea.bit_rate, bit_rate_str); // 设置初始内容
        lv_obj_set_style_text_font(_registry.win.rf.setings.config_rf_params.cc1101.textarea.bit_rate, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_align_to(_registry.win.rf.setings.config_rf_params.cc1101.textarea.bit_rate, msgbox_current_limit_text, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);

        init_win_rf_setings_keyboard_position_event_cb(_registry.win.rf.setings.config_rf_params.cc1101.textarea.bit_rate);

        lv_obj_t *msgbox_freq_deviation_khz_text = lv_label_create(_registry.win.rf.setings.message_box.parameter_container);
        lv_label_set_text(msgbox_freq_deviation_khz_text, "freq deviation");
        lv_obj_set_size(msgbox_freq_deviation_khz_text, 300, 40);
        lv_obj_set_style_text_font(msgbox_freq_deviation_khz_text, &lv_font_montserrat_26, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_align_to(msgbox_freq_deviation_khz_text, _registry.win.rf.setings.config_rf_params.cc1101.textarea.bit_rate, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 10);

        _registry.win.rf.setings.config_rf_params.cc1101.textarea.freq_deviation_khz = lv_textarea_create(_registry.win.rf.setings.message_box.parameter_container);
        lv_textarea_set_accepted_chars(_registry.win.rf.setings.config_rf_params.cc1101.textarea.freq_deviation_khz, "0123456789."); // 只允许输入数字和小数点
        lv_obj_set_style_pad_top(_registry.win.rf.setings.config_rf_params.cc1101.textarea.freq_deviation_khz, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_bottom(_registry.win.rf.setings.config_rf_params.cc1101.textarea.freq_deviation_khz, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_left(_registry.win.rf.setings.config_rf_params.cc1101.textarea.freq_deviation_khz, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_right(_registry.win.rf.setings.config_rf_params.cc1101.textarea.freq_deviation_khz, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_width(_registry.win.rf.setings.config_rf_params.cc1101.textarea.freq_deviation_khz, 300);
        lv_textarea_set_one_line(_registry.win.rf.setings.config_rf_params.cc1101.textarea.freq_deviation_khz, true);
        char freq_deviation_khz_str[15];
        snprintf(freq_deviation_khz_str, sizeof(freq_deviation_khz_str), "%.6f", _device_cc1101.params.freq_deviation_khz);
        lv_textarea_set_text(_registry.win.rf.setings.config_rf_params.cc1101.textarea.freq_deviation_khz, freq_deviation_khz_str);
        lv_obj_set_style_text_font(_registry.win.rf.setings.config_rf_params.cc1101.textarea.freq_deviation_khz, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_align_to(_registry.win.rf.setings.config_rf_params.cc1101.textarea.freq_deviation_khz, msgbox_freq_deviation_khz_text, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);

        init_win_rf_setings_keyboard_position_event_cb(_registry.win.rf.setings.config_rf_params.cc1101.textarea.freq_deviation_khz);

        lv_obj_t *freq_deviation_khz_unit_text = lv_label_create(_registry.win.rf.setings.message_box.parameter_container);
        lv_label_set_text(freq_deviation_khz_unit_text, "khz");
        lv_obj_set_size(freq_deviation_khz_unit_text, 70, 40);
        lv_obj_set_style_text_font(freq_deviation_khz_unit_text, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_align_to(freq_deviation_khz_unit_text, _registry.win.rf.setings.config_rf_params.cc1101.textarea.freq_deviation_khz, LV_ALIGN_OUT_RIGHT_BOTTOM, 10, 0);

        lv_obj_t *msgbox_power_text = lv_label_create(_registry.win.rf.setings.message_box.parameter_container);
        lv_label_set_text(msgbox_power_text, "power");
        lv_obj_set_size(msgbox_power_text, 100, 40);
        lv_obj_set_style_text_font(msgbox_power_text, &lv_font_montserrat_26, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_align_to(msgbox_power_text, _registry.win.rf.setings.config_rf_params.cc1101.textarea.freq_deviation_khz, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 10);

        _registry.win.rf.setings.config_rf_params.cc1101.textarea.power = lv_textarea_create(_registry.win.rf.setings.message_box.parameter_container);
        lv_textarea_set_accepted_chars(_registry.win.rf.setings.config_rf_params.cc1101.textarea.power, "0123456789-"); // 只允许输入数字和负号
        lv_obj_set_style_pad_top(_registry.win.rf.setings.config_rf_params.cc1101.textarea.power, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_bottom(_registry.win.rf.setings.config_rf_params.cc1101.textarea.power, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_left(_registry.win.rf.setings.config_rf_params.cc1101.textarea.power, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_right(_registry.win.rf.setings.config_rf_params.cc1101.textarea.power, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_width(_registry.win.rf.setings.config_rf_params.cc1101.textarea.power, 300);
        lv_textarea_set_one_line(_registry.win.rf.setings.config_rf_params.cc1101.textarea.power, true);
        char power_str[10];
        snprintf(power_str, sizeof(power_str), "%d", _device_cc1101.params.power);
        lv_textarea_set_text(_registry.win.rf.setings.config_rf_params.cc1101.textarea.power, power_str); // 设置初始内容
        lv_obj_set_style_text_font(_registry.win.rf.setings.config_rf_params.cc1101.textarea.power, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_align_to(_registry.win.rf.setings.config_rf_params.cc1101.textarea.power, msgbox_power_text, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);

        init_win_rf_setings_keyboard_position_event_cb(_registry.win.rf.setings.config_rf_params.cc1101.textarea.power);

        lv_obj_t *power_unit_text = lv_label_create(_registry.win.rf.setings.message_box.parameter_container);
        lv_label_set_text(power_unit_text, "dbm");
        lv_obj_set_size(power_unit_text, 70, 40);
        lv_obj_set_style_text_font(power_unit_text, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_align_to(power_unit_text, _registry.win.rf.setings.config_rf_params.cc1101.textarea.power, LV_ALIGN_OUT_RIGHT_BOTTOM, 10, 0);

        lv_obj_t *msgbox_preamble_length = lv_label_create(_registry.win.rf.setings.message_box.parameter_container);
        lv_label_set_text(msgbox_preamble_length, "preamble length");
        lv_obj_set_size(msgbox_preamble_length, 300, 40);
        lv_obj_set_style_text_font(msgbox_preamble_length, &lv_font_montserrat_26, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_align_to(msgbox_preamble_length, _registry.win.rf.setings.config_rf_params.cc1101.textarea.power, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 10);

        _registry.win.rf.setings.config_rf_params.cc1101.textarea.preamble_length = lv_textarea_create(_registry.win.rf.setings.message_box.parameter_container);
        lv_textarea_set_accepted_chars(_registry.win.rf.setings.config_rf_params.cc1101.textarea.preamble_length, "0123456789"); // 只允许输入数字
        lv_obj_set_style_pad_top(_registry.win.rf.setings.config_rf_params.cc1101.textarea.preamble_length, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_bottom(_registry.win.rf.setings.config_rf_params.cc1101.textarea.preamble_length, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_left(_registry.win.rf.setings.config_rf_params.cc1101.textarea.preamble_length, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_right(_registry.win.rf.setings.config_rf_params.cc1101.textarea.preamble_length, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_width(_registry.win.rf.setings.config_rf_params.cc1101.textarea.preamble_length, 300);
        lv_textarea_set_one_line(_registry.win.rf.setings.config_rf_params.cc1101.textarea.preamble_length, true);
        char preamble_length_str[10];
        snprintf(preamble_length_str, sizeof(preamble_length_str), "%d", _device_cc1101.params.preamble_length);
        lv_textarea_set_text(_registry.win.rf.setings.config_rf_params.cc1101.textarea.preamble_length, preamble_length_str); // 设置初始内容
        lv_obj_set_style_text_font(_registry.win.rf.setings.config_rf_params.cc1101.textarea.preamble_length, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_align_to(_registry.win.rf.setings.config_rf_params.cc1101.textarea.preamble_length, msgbox_preamble_length, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);

        init_win_rf_setings_keyboard_position_event_cb(_registry.win.rf.setings.config_rf_params.cc1101.textarea.preamble_length);

        lv_obj_t *msgbox_sync_word = lv_label_create(_registry.win.rf.setings.message_box.parameter_container);
        lv_label_set_text(msgbox_sync_word, "sync word");
        lv_obj_set_size(msgbox_sync_word, 300, 40);
        lv_obj_set_style_text_font(msgbox_sync_word, &lv_font_montserrat_26, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_align_to(msgbox_sync_word, _registry.win.rf.setings.config_rf_params.cc1101.textarea.preamble_length, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 10);

        _registry.win.rf.setings.config_rf_params.cc1101.textarea.sync_word = lv_textarea_create(_registry.win.rf.setings.message_box.parameter_container);
        lv_textarea_set_accepted_chars(_registry.win.rf.setings.config_rf_params.cc1101.textarea.sync_word, "0123456789"); // 只允许输入数字
        lv_obj_set_style_pad_top(_registry.win.rf.setings.config_rf_params.cc1101.textarea.sync_word, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_bottom(_registry.win.rf.setings.config_rf_params.cc1101.textarea.sync_word, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_left(_registry.win.rf.setings.config_rf_params.cc1101.textarea.sync_word, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_right(_registry.win.rf.setings.config_rf_params.cc1101.textarea.sync_word, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_width(_registry.win.rf.setings.config_rf_params.cc1101.textarea.sync_word, 300);
        lv_textarea_set_one_line(_registry.win.rf.setings.config_rf_params.cc1101.textarea.sync_word, true);
        char sync_word_str[10];
        snprintf(sync_word_str, sizeof(sync_word_str), "%d", _device_cc1101.params.sync_word);
        lv_textarea_set_text(_registry.win.rf.setings.config_rf_params.cc1101.textarea.sync_word, sync_word_str); // 设置初始内容
        lv_obj_set_style_text_font(_registry.win.rf.setings.config_rf_params.cc1101.textarea.sync_word, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_align_to(_registry.win.rf.setings.config_rf_params.cc1101.textarea.sync_word, msgbox_sync_word, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);

        init_win_rf_setings_keyboard_position_event_cb(_registry.win.rf.setings.config_rf_params.cc1101.textarea.sync_word);
    }

    void System::init_win_rf_setings_config_nrf24l01_params_message_box(void)
    {
        // 创建全屏灰色透明遮罩
        _registry.win.rf.setings.message_box.root = lv_obj_create(_registry.win.rf.setings.root);
        lv_obj_set_style_pad_top(_registry.win.rf.setings.message_box.root, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_bottom(_registry.win.rf.setings.message_box.root, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_left(_registry.win.rf.setings.message_box.root, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_right(_registry.win.rf.setings.message_box.root, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_size(_registry.win.rf.setings.message_box.root, _width, _height);
        lv_obj_set_style_bg_color(_registry.win.rf.setings.message_box.root, lv_color_black(), LV_PART_MAIN);
        lv_obj_set_style_bg_opa(_registry.win.rf.setings.message_box.root, LV_OPA_30, LV_PART_MAIN);
        lv_obj_set_style_border_width(_registry.win.rf.setings.message_box.root, 0, LV_PART_MAIN);
        lv_obj_set_style_radius(_registry.win.rf.setings.message_box.root, 0, LV_PART_MAIN);
        lv_obj_align(_registry.win.rf.setings.message_box.root, LV_ALIGN_CENTER, 0, 0);
        lv_obj_add_flag(_registry.win.rf.setings.message_box.root, LV_OBJ_FLAG_CLICKABLE); // 添加触摸标志来禁止其他界面触摸

        _registry.win.rf.setings.message_box.root_container = lv_obj_create(_registry.win.rf.setings.message_box.root);
        lv_obj_set_style_pad_top(_registry.win.rf.setings.message_box.root_container, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_bottom(_registry.win.rf.setings.message_box.root_container, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_left(_registry.win.rf.setings.message_box.root_container, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_right(_registry.win.rf.setings.message_box.root_container, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_size(_registry.win.rf.setings.message_box.root_container, 450, _height * 0.7);
        lv_obj_set_style_radius(_registry.win.rf.setings.message_box.root_container, 15, LV_PART_MAIN);
        lv_obj_set_style_bg_color(_registry.win.rf.setings.message_box.root_container, lv_color_white(), LV_PART_MAIN);
        lv_obj_set_style_border_width(_registry.win.rf.setings.message_box.root_container, 0, LV_PART_MAIN);
        lv_obj_set_style_shadow_width(_registry.win.rf.setings.message_box.root_container, 16, LV_PART_MAIN);
        lv_obj_center(_registry.win.rf.setings.message_box.root_container);

        _registry.win.rf.setings.message_box.btn_container = lv_obj_create(_registry.win.rf.setings.message_box.root_container);
        lv_obj_set_style_pad_top(_registry.win.rf.setings.message_box.btn_container, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_bottom(_registry.win.rf.setings.message_box.btn_container, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_left(_registry.win.rf.setings.message_box.btn_container, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_right(_registry.win.rf.setings.message_box.btn_container, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_size(_registry.win.rf.setings.message_box.btn_container, 450, 110);
        lv_obj_set_style_border_width(_registry.win.rf.setings.message_box.btn_container, 0, (lv_style_selector_t)LV_PART_MAIN); // 移除边框
        lv_obj_align(_registry.win.rf.setings.message_box.btn_container, LV_ALIGN_BOTTOM_MID, 0, 0);

        lv_obj_t *btn_cancel = lv_button_create(_registry.win.rf.setings.message_box.btn_container);
        lv_obj_set_size(btn_cancel, 150, 60);
        lv_obj_align(btn_cancel, LV_ALIGN_BOTTOM_LEFT, 40, -30);
        lv_obj_set_style_radius(btn_cancel, 10, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_width(btn_cancel, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT); // 去除按钮阴影
        lv_obj_t *btn_cancel_label = lv_label_create(btn_cancel);
        lv_label_set_text(btn_cancel_label, "cancel");
        lv_obj_set_style_text_font(btn_cancel_label, &lv_font_montserrat_26, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_center(btn_cancel_label);

        // cancel 按钮回调
        lv_obj_add_event_cb(btn_cancel, [](lv_event_t *e)
                            {
                                System *self = static_cast<System *>(lv_event_get_user_data(e));
                                lv_obj_delete(self->_registry.win.rf.setings.message_box.root); }, LV_EVENT_CLICKED, this);

        lv_obj_t *btn_apply = lv_button_create(_registry.win.rf.setings.message_box.btn_container);
        lv_obj_set_size(btn_apply, 150, 60);
        lv_obj_align(btn_apply, LV_ALIGN_BOTTOM_RIGHT, -40, -30);
        lv_obj_set_style_radius(btn_apply, 10, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_width(btn_apply, 0, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT); // 去除按钮阴影
        lv_obj_t *btn_apply_label = lv_label_create(btn_apply);
        lv_label_set_text(btn_apply_label, "apply");
        lv_obj_set_style_text_font(btn_apply_label, &lv_font_montserrat_26, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_center(btn_apply_label);

        // apply 按钮回调
        lv_obj_add_event_cb(btn_apply, [](lv_event_t *e)
                            {
                                System *self = static_cast<System *>(lv_event_get_user_data(e));

                                Device_Nrf24l01 dn;

                                const char* freq_text = lv_textarea_get_text(self->_registry.win.rf.setings.config_rf_params.nrf24l01.textarea.freq);
                                if (freq_text != nullptr && freq_text[0] != '\0') // 同时检查NULL和空字符串
                                {  
                                    double buffer = std::stod(freq_text, nullptr);  

                                    // 限制范围
                                    if(buffer <= 2400.0)
                                    {
                                        dn.params.freq = 2400.0;
                                    }
                                    else if(buffer <= 2525.0)
                                    {
                                        dn.params.freq = buffer;
                                    }
                                    else
                                    {
                                        dn.params.freq = 2525.0;
                                    }
                                }

                                const char* bit_rate_text = lv_textarea_get_text(self->_registry.win.rf.setings.config_rf_params.nrf24l01.textarea.bit_rate);
                                if (bit_rate_text != nullptr && bit_rate_text[0] != '\0') // 同时检查NULL和空字符串
                                {  
                                    float buffer = std::stof(bit_rate_text, nullptr);  

                                    // 限制范围
                                    // 如果在范围内，选择最接近的较大允许值
                                    // 允许的值：250, 1000, 2000
                                    if(buffer <= 250) 
                                    {
                                        dn.params.bit_rate = 250;
                                    }
                                    else if(buffer <= 1000) 
                                    {
                                        dn.params.bit_rate = 1000;
                                    }
                                    else
                                    {
                                        dn.params.bit_rate = 2000;
                                    }
                                }

                                const char* power_text = lv_textarea_get_text(self->_registry.win.rf.setings.config_rf_params.nrf24l01.textarea.power);
                                if (power_text != nullptr && power_text[0] != '\0') // 同时检查NULL和空字符串
                                {  
                                    int8_t buffer = std::stoi(power_text);  

                                    // 限制范围
                                    // 如果在范围内，选择最接近的较大允许值
                                    // 允许的值：-18, -12, -6, 0
                                    if(buffer <= -18) 
                                    {
                                        dn.params.power = -18;
                                    }
                                    else if(buffer <= -12) 
                                    {
                                        dn.params.power = -12;
                                    }
                                    else if(buffer <= -6) 
                                    {
                                        dn.params.power = -6;
                                    }
                                    else 
                                    {
                                        dn.params.power = 0;
                                    }
                                }

                                const char* address_width_text = lv_textarea_get_text(self->_registry.win.rf.setings.config_rf_params.nrf24l01.textarea.address_width);
                                if (address_width_text != nullptr && address_width_text[0] != '\0') // 同时检查NULL和空字符串
                                {  
                                    uint8_t buffer = std::stoi(address_width_text);  

                                    // 限制范围
                                    // 如果在范围内，选择最接近的较大允许值
                                    // 允许的值：3, 4, 5
                                    if(buffer <= 3) 
                                    {
                                        dn.params.address_width = 3;
                                    }
                                    else if(buffer <= 4) 
                                    {
                                        dn.params.address_width = 4;
                                    }
                                    else 
                                    {
                                        dn.params.address_width = 5;
                                    }
                                }

                                const char* address_text = lv_textarea_get_text(self->_registry.win.rf.setings.config_rf_params.nrf24l01.textarea.address);
                                if (address_text != nullptr && address_text[0] != '\0') // 同时检查NULL和空字符串
                                {  
                                    dn.params.address = std::stoull(address_text);  
                                }

                                if(self->set_config_rf_params(dn) == true)
                                {
                                    self->_device_nrf24l01.params = dn.params;
                                }

                                lv_obj_delete(self->_registry.win.rf.setings.message_box.root); }, LV_EVENT_CLICKED, this);

        _registry.win.rf.setings.message_box.parameter_container = lv_obj_create(_registry.win.rf.setings.message_box.root_container);
        lv_obj_set_style_pad_top(_registry.win.rf.setings.message_box.parameter_container, 30, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_bottom(_registry.win.rf.setings.message_box.parameter_container, 30, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_left(_registry.win.rf.setings.message_box.parameter_container, 30, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_right(_registry.win.rf.setings.message_box.parameter_container, 30, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
#if defined SCREEN_ROTATION_DIRECTION_0
        lv_obj_set_size(_registry.win.rf.setings.message_box.parameter_container, 450, _height * 0.6);
#elif defined SCREEN_ROTATION_DIRECTION_90
        lv_obj_set_size(_registry.win.rf.setings.message_box.parameter_container, 450, _height * 0.5);
#else
#error "unknown macro definition, please select the correct macro definition."
#endif
        lv_obj_set_style_border_width(_registry.win.rf.setings.message_box.parameter_container, 0, (lv_style_selector_t)LV_PART_MAIN); // 移除边框
        lv_obj_set_scrollbar_mode(_registry.win.rf.setings.message_box.parameter_container, LV_SCROLLBAR_MODE_ACTIVE);
        lv_obj_align_to(_registry.win.rf.setings.message_box.parameter_container, _registry.win.rf.setings.message_box.btn_container, LV_ALIGN_OUT_TOP_MID, 0, 0);

        // 触摸设置消息框区域时隐藏键盘
        lv_obj_add_event_cb(_registry.win.rf.setings.message_box.parameter_container, [](lv_event_t *e)
                            {
                                System *self = static_cast<System *>(lv_event_get_user_data(e));
                                lv_event_code_t code = lv_event_get_code(e);

                                if (code == LV_EVENT_CLICKED)
                                {
                                    lv_group_remove_all_objs(self->_registry.keyboard_group);
                                } }, LV_EVENT_ALL, this);

        lv_obj_t *msgbox_freq_text = lv_label_create(_registry.win.rf.setings.message_box.parameter_container);
        lv_label_set_text(msgbox_freq_text, "freq");
        lv_obj_set_size(msgbox_freq_text, 100, 40);
        lv_obj_set_style_text_font(msgbox_freq_text, &lv_font_montserrat_26, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_align(msgbox_freq_text, LV_ALIGN_TOP_LEFT, 0, 0);

        _registry.win.rf.setings.config_rf_params.nrf24l01.textarea.freq = lv_textarea_create(_registry.win.rf.setings.message_box.parameter_container);
        lv_textarea_set_accepted_chars(_registry.win.rf.setings.config_rf_params.nrf24l01.textarea.freq, "0123456789."); // 只允许输入数字和小数点
        lv_obj_set_style_pad_top(_registry.win.rf.setings.config_rf_params.nrf24l01.textarea.freq, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_bottom(_registry.win.rf.setings.config_rf_params.nrf24l01.textarea.freq, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_left(_registry.win.rf.setings.config_rf_params.nrf24l01.textarea.freq, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_right(_registry.win.rf.setings.config_rf_params.nrf24l01.textarea.freq, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_width(_registry.win.rf.setings.config_rf_params.nrf24l01.textarea.freq, 300);
        lv_textarea_set_one_line(_registry.win.rf.setings.config_rf_params.nrf24l01.textarea.freq, true);
        char freq_str[15];
        snprintf(freq_str, sizeof(freq_str), "%.6f", _device_nrf24l01.params.freq);
        lv_textarea_set_text(_registry.win.rf.setings.config_rf_params.nrf24l01.textarea.freq, freq_str);
        lv_obj_set_style_text_font(_registry.win.rf.setings.config_rf_params.nrf24l01.textarea.freq, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_align_to(_registry.win.rf.setings.config_rf_params.nrf24l01.textarea.freq, msgbox_freq_text, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);

        init_win_rf_setings_keyboard_position_event_cb(_registry.win.rf.setings.config_rf_params.nrf24l01.textarea.freq);

        lv_obj_t *freq_unit_text = lv_label_create(_registry.win.rf.setings.message_box.parameter_container);
        lv_label_set_text(freq_unit_text, "mhz");
        lv_obj_set_size(freq_unit_text, 70, 40);
        lv_obj_set_style_text_font(freq_unit_text, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_align_to(freq_unit_text, _registry.win.rf.setings.config_rf_params.nrf24l01.textarea.freq, LV_ALIGN_OUT_RIGHT_BOTTOM, 10, 0);

        lv_obj_t *msgbox_current_limit_text = lv_label_create(_registry.win.rf.setings.message_box.parameter_container);
        lv_label_set_text(msgbox_current_limit_text, "bit rate");
        lv_obj_set_size(msgbox_current_limit_text, 300, 40);
        lv_obj_set_style_text_font(msgbox_current_limit_text, &lv_font_montserrat_26, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_align_to(msgbox_current_limit_text, _registry.win.rf.setings.config_rf_params.nrf24l01.textarea.freq, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 10);

        _registry.win.rf.setings.config_rf_params.nrf24l01.textarea.bit_rate = lv_textarea_create(_registry.win.rf.setings.message_box.parameter_container);
        lv_textarea_set_accepted_chars(_registry.win.rf.setings.config_rf_params.nrf24l01.textarea.bit_rate, "0123456789."); // 只允许输入数字和小数点
        lv_obj_set_style_pad_top(_registry.win.rf.setings.config_rf_params.nrf24l01.textarea.bit_rate, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_bottom(_registry.win.rf.setings.config_rf_params.nrf24l01.textarea.bit_rate, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_left(_registry.win.rf.setings.config_rf_params.nrf24l01.textarea.bit_rate, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_right(_registry.win.rf.setings.config_rf_params.nrf24l01.textarea.bit_rate, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_width(_registry.win.rf.setings.config_rf_params.nrf24l01.textarea.bit_rate, 300);
        lv_textarea_set_one_line(_registry.win.rf.setings.config_rf_params.nrf24l01.textarea.bit_rate, true);
        char bit_rate_str[10];
        snprintf(bit_rate_str, sizeof(bit_rate_str), "%.1f", _device_nrf24l01.params.bit_rate);
        lv_textarea_set_text(_registry.win.rf.setings.config_rf_params.nrf24l01.textarea.bit_rate, bit_rate_str); // 设置初始内容
        lv_obj_set_style_text_font(_registry.win.rf.setings.config_rf_params.nrf24l01.textarea.bit_rate, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_align_to(_registry.win.rf.setings.config_rf_params.nrf24l01.textarea.bit_rate, msgbox_current_limit_text, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);

        init_win_rf_setings_keyboard_position_event_cb(_registry.win.rf.setings.config_rf_params.nrf24l01.textarea.bit_rate);

        lv_obj_t *msgbox_power_text = lv_label_create(_registry.win.rf.setings.message_box.parameter_container);
        lv_label_set_text(msgbox_power_text, "power");
        lv_obj_set_size(msgbox_power_text, 100, 40);
        lv_obj_set_style_text_font(msgbox_power_text, &lv_font_montserrat_26, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_align_to(msgbox_power_text, _registry.win.rf.setings.config_rf_params.nrf24l01.textarea.bit_rate, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 10);

        _registry.win.rf.setings.config_rf_params.nrf24l01.textarea.power = lv_textarea_create(_registry.win.rf.setings.message_box.parameter_container);
        lv_textarea_set_accepted_chars(_registry.win.rf.setings.config_rf_params.nrf24l01.textarea.power, "0123456789-"); // 只允许输入数字和负号
        lv_obj_set_style_pad_top(_registry.win.rf.setings.config_rf_params.nrf24l01.textarea.power, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_bottom(_registry.win.rf.setings.config_rf_params.nrf24l01.textarea.power, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_left(_registry.win.rf.setings.config_rf_params.nrf24l01.textarea.power, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_right(_registry.win.rf.setings.config_rf_params.nrf24l01.textarea.power, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_width(_registry.win.rf.setings.config_rf_params.nrf24l01.textarea.power, 300);
        lv_textarea_set_one_line(_registry.win.rf.setings.config_rf_params.nrf24l01.textarea.power, true);
        char power_str[10];
        snprintf(power_str, sizeof(power_str), "%d", _device_nrf24l01.params.power);
        lv_textarea_set_text(_registry.win.rf.setings.config_rf_params.nrf24l01.textarea.power, power_str); // 设置初始内容
        lv_obj_set_style_text_font(_registry.win.rf.setings.config_rf_params.nrf24l01.textarea.power, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_align_to(_registry.win.rf.setings.config_rf_params.nrf24l01.textarea.power, msgbox_power_text, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);

        init_win_rf_setings_keyboard_position_event_cb(_registry.win.rf.setings.config_rf_params.nrf24l01.textarea.power);

        lv_obj_t *power_unit_text = lv_label_create(_registry.win.rf.setings.message_box.parameter_container);
        lv_label_set_text(power_unit_text, "dbm");
        lv_obj_set_size(power_unit_text, 70, 40);
        lv_obj_set_style_text_font(power_unit_text, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_align_to(power_unit_text, _registry.win.rf.setings.config_rf_params.nrf24l01.textarea.power, LV_ALIGN_OUT_RIGHT_BOTTOM, 10, 0);

        lv_obj_t *msgbox_address_width = lv_label_create(_registry.win.rf.setings.message_box.parameter_container);
        lv_label_set_text(msgbox_address_width, "address width");
        lv_obj_set_size(msgbox_address_width, 300, 40);
        lv_obj_set_style_text_font(msgbox_address_width, &lv_font_montserrat_26, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_align_to(msgbox_address_width, _registry.win.rf.setings.config_rf_params.nrf24l01.textarea.power, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 10);

        _registry.win.rf.setings.config_rf_params.nrf24l01.textarea.address_width = lv_textarea_create(_registry.win.rf.setings.message_box.parameter_container);
        lv_textarea_set_accepted_chars(_registry.win.rf.setings.config_rf_params.nrf24l01.textarea.address_width, "0123456789"); // 只允许输入数字
        lv_obj_set_style_pad_top(_registry.win.rf.setings.config_rf_params.nrf24l01.textarea.address_width, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_bottom(_registry.win.rf.setings.config_rf_params.nrf24l01.textarea.address_width, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_left(_registry.win.rf.setings.config_rf_params.nrf24l01.textarea.address_width, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_right(_registry.win.rf.setings.config_rf_params.nrf24l01.textarea.address_width, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_width(_registry.win.rf.setings.config_rf_params.nrf24l01.textarea.address_width, 300);
        lv_textarea_set_one_line(_registry.win.rf.setings.config_rf_params.nrf24l01.textarea.address_width, true);
        char address_width_str[10];
        snprintf(address_width_str, sizeof(address_width_str), "%d", _device_nrf24l01.params.address_width);
        lv_textarea_set_text(_registry.win.rf.setings.config_rf_params.nrf24l01.textarea.address_width, address_width_str); // 设置初始内容
        lv_obj_set_style_text_font(_registry.win.rf.setings.config_rf_params.nrf24l01.textarea.address_width, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_align_to(_registry.win.rf.setings.config_rf_params.nrf24l01.textarea.address_width, msgbox_address_width, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);

        init_win_rf_setings_keyboard_position_event_cb(_registry.win.rf.setings.config_rf_params.nrf24l01.textarea.address_width);

        lv_obj_t *msgbox_address = lv_label_create(_registry.win.rf.setings.message_box.parameter_container);
        lv_label_set_text(msgbox_address, "address");
        lv_obj_set_size(msgbox_address, 300, 40);
        lv_obj_set_style_text_font(msgbox_address, &lv_font_montserrat_26, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_align_to(msgbox_address, _registry.win.rf.setings.config_rf_params.nrf24l01.textarea.address_width, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 10);

        _registry.win.rf.setings.config_rf_params.nrf24l01.textarea.address = lv_textarea_create(_registry.win.rf.setings.message_box.parameter_container);
        lv_textarea_set_accepted_chars(_registry.win.rf.setings.config_rf_params.nrf24l01.textarea.address, "0123456789"); // 只允许输入数字
        lv_obj_set_style_pad_top(_registry.win.rf.setings.config_rf_params.nrf24l01.textarea.address, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_bottom(_registry.win.rf.setings.config_rf_params.nrf24l01.textarea.address, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_left(_registry.win.rf.setings.config_rf_params.nrf24l01.textarea.address, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_style_pad_right(_registry.win.rf.setings.config_rf_params.nrf24l01.textarea.address, 15, (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT);
        lv_obj_set_width(_registry.win.rf.setings.config_rf_params.nrf24l01.textarea.address, 300);
        lv_textarea_set_one_line(_registry.win.rf.setings.config_rf_params.nrf24l01.textarea.address, true);
        char address_str[20];
        snprintf(address_str, sizeof(address_str), "%lld", _device_nrf24l01.params.address);
        lv_textarea_set_text(_registry.win.rf.setings.config_rf_params.nrf24l01.textarea.address, address_str); // 设置初始内容
        lv_obj_set_style_text_font(_registry.win.rf.setings.config_rf_params.nrf24l01.textarea.address, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_align_to(_registry.win.rf.setings.config_rf_params.nrf24l01.textarea.address, msgbox_address, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);

        init_win_rf_setings_keyboard_position_event_cb(_registry.win.rf.setings.config_rf_params.nrf24l01.textarea.address);
    }

    bool System::set_config_rf_params(Device_Cc1101 device_cc1101)
    {
        if (_win_rf_config_cc1101_params_callback != nullptr)
        {
            if (_win_rf_config_cc1101_params_callback(device_cc1101) == true)
            {
                return true;
            }
        }

        return false;
    }

    bool System::set_config_rf_params(Device_Nrf24l01 device_nrf24l01)
    {
        if (_win_rf_config_nrf24l01_params_callback != nullptr)
        {
            if (_win_rf_config_nrf24l01_params_callback(device_nrf24l01) == true)
            {
                return true;
            }
        }

        return false;
    }

    void System::set_otg_switch_status(bool status)
    {
        if (_win_cit_otg_switch_callback != nullptr)
        {
            _win_cit_otg_switch_callback(status);
        }
    }

#endif
};
