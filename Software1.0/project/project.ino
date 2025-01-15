/*
 Huge thank you to Uspizig for offering this library that I have modified for TFT_eSPI: https://github.com/Uspizig/MLX90640
 Has to be Library, TFT_eSPI Rev 2.5.43
 The latest does not work

*/
#include <Arduino.h>
#include <FreeRTOS.h> 
#include <task.h>
#include <TFT_eSPI.h> 
#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>

#include "pico/stdlib.h"
#include "hardware/rtc.h"
#include "hardware/pll.h"
#include "hardware/clocks.h"
#include "hardware/xosc.h"
// #include "hardware/rosc.h"
#include "hardware/regs/io_bank0.h"
// For __wfi
#include "hardware/sync.h"
// For scb_hw so we can enable deep sleep
#include "hardware/structs/scb.h"


#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"
#include "CST816T.h"
#include "kalman_filter.h"
#include "BilinearInterpolation.h" //图像处理

#define TA_SHIFT 8    //MLX90640红外传感器的默认位移 Default shift for MLX90640 in open air 
#define MLX_VDD  11   //该宏定义了MLX90640传感器的电源引脚（VDD）为GPIO引脚11
#define MLX_SDA  12   //该宏定义了MLX90640传感器的SDA（数据线）引脚为GPIO引脚12
#define MLX_SCL  13   //该宏定义了MLX90640传感器的SCL（时钟线）引脚为GPIO引脚13

#define TOUCH_SDA 2   //该宏定义了触摸屏的SDA引脚为GPIO引脚2
#define TOUCH_SCL 3   //该宏定义了触摸屏的SCL引脚为GPIO引脚3
#define TOUCH_RST -1  //该宏定义了触摸屏的复位引脚为-1，表示触摸屏可能没有复位引脚

#define BAT_ADC  26     //该宏定义了用于电池电压读取的模拟输入引脚（ADC引脚）为GPIO引脚26。通常用于读取电池电压。
#define SCREEN_BL_PIN 4 //该宏定义了显示屏背光控制引脚为GPIO引脚4。通过控制该引脚的电平，可以调节显示屏的背光亮度。
#define SCREEN_VDD 5    //该宏定义了显示屏电源引脚（VDD）为GPIO引脚5。通过控制该引脚的电平，可以开启或关闭显示屏电源。

#define SCREEN_ROTATION 1 //SCREEN_ROTATION = 1 表示屏幕顺时针旋转 90 度
#define CURSOR_SIZE 2     //用于设置文本的显示大小（即字体大小）

#define DRAW_PIXELS_DMA  // 使用DMA来绘制

#define LVGL_UI_USER 
#if defined(LVGL_UI_USER)
#include <ui.h>
/*To use the built-in examples and demos of LVGL uncomment the includes below respectively.
 *You also need to copy `lvgl/examples` to `lvgl/src/examples`. Similarly for the demos `lvgl/demos` to `lvgl/src/demos`.
 Note that the `lv_examples` library is for LVGL v7 and you shouldn't install it for this version (since LVGL v8)
 as the examples and demos are now part of the main LVGL library. */
#include <examples/lv_examples.h>
#include <demos/lv_demos.h>
#define ROTATE 1
int model_flag = 1;
int color_flag = 0;  // 默认颜色方案是 HotMetal
uint16_t test_points[5][2];
bool freeze = false;  // 画面暂停为true
int brightness = 100;
static const uint16_t screenWidth  = 280;
static const uint16_t screenHeight = 240;
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[ screenWidth * screenHeight / 10 ];

TFT_eSPI tft = TFT_eSPI(screenHeight, screenWidth); /* TFT instance */
CST816T touch(TOUCH_SDA, TOUCH_SCL, TOUCH_RST, -1);

bool user_ui_flag = false; //是否开启用户ui

#define BTN_LONG_PUSH_T 1000
const int buttonPin1 = 24; 
unsigned long btn2_pushed_start_time =  0;
bool btn2_pushed = false;  
bool btn2_long_pushed = false;

unsigned long btn1_pushed_start_time =  0;     //用于记录按钮按下的时间戳
bool btn1_pushed = false;
bool btn1_long_pushed = false;
/* Display flushing */
void my_disp_flush( lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p )
{
    uint32_t w = ( area->x2 - area->x1 + 1 );
    uint32_t h = ( area->y2 - area->y1 + 1 );

    tft.startWrite();
    #if (ROTATE == 0 || ROTATE == 2)
    tft.setAddrWindow( area->x1, area->y1, w, h );
    #endif
    #if (ROTATE == 1 || ROTATE == 3)
    tft.setAddrWindow( area->x1, area->y1, w, h );
    #endif
    tft.pushColors( ( uint16_t * )&color_p->full, w * h, true );
    tft.endWrite();

    lv_disp_flush_ready( disp_drv );
}

/*Read the touchpad*/
void my_touchpad_read( lv_indev_drv_t * indev_drv, lv_indev_data_t * data )
{

    touch.update();
    // Serial.print( "touch called " );
    // Serial.println( touch.tp.touching );
    bool touched = touch.tp.touching;
    if( !touched )
    // if( 0!=touch.data.points )
    {
        data->state = LV_INDEV_STATE_REL;
    }
    else
    {
        data->state = LV_INDEV_STATE_PR;
        #if (ROTATE == 0)
        /*Change to your screen resolution*/
        data->point.x = touch.tp.x;
        data->point.y = touch.tp.y;
        #endif
        #if (ROTATE == 1)
        /*Change to your screen resolution*/
        data->point.x = touch.tp.y;
        data->point.y = 240-touch.tp.x;
        #endif
        #if (ROTATE == 2)
        /*Change to your screen resolution*/
        data->point.x = 240-touch.tp.x;
        data->point.y = 280-touch.tp.y;
        #endif

        #if (ROTATE == 3)
        data->point.x = 280-touch.tp.y;
        data->point.y = touch.tp.x;
        #endif
        // data->point.x = touch.tp.x;
        // data->point.y = touch.tp.y;
        // Serial.print( "Data x " );
        // Serial.println( touch.tp.x );

        // Serial.print( "Data y " );
        // Serial.println( touch.tp.y );  
    }
}

// 按键输入设备读取回调函数
void my_keypad_read(lv_indev_drv_t * drv, lv_indev_data_t * data)
{
    int button_state = digitalRead(24);  // 读取按键的状态，假设按键接地为 LOW
    if (button_state == LOW) {
        // 如果按键按下，记录开始按下的时间
        if (btn2_pushed_start_time == 0) {
            btn2_pushed_start_time = millis();
        }
        // 检测是否为长按
        if (millis() - btn2_pushed_start_time >= BTN_LONG_PUSH_T) {
            if (!btn2_long_pushed) {
                btn2_long_pushed = true;  // 标记为长按
                // 长按的处理，例如切换到不同屏幕
                
            }
        }
        data->state = LV_INDEV_STATE_PRESSED;  // 按键按下
    } else {
        // 按键松开，判断是否为短按
        if (btn2_pushed_start_time != 0) {
            if (!btn2_long_pushed) {
                btn2_pushed = true;  // 短按标记
                // 短按的处理，例如切换到不同屏幕
                if (!btn2_long_pushed){freeze = !freeze; } //切换 freeze 状态
            }
            btn2_pushed_start_time = 0;  // 重置按下时间
        }
        // 清除长按标记
        btn2_long_pushed = false;
        data->state = LV_INDEV_STATE_RELEASED;  // 按键松开
    }
}

void my_keypad_bootsel_read(lv_indev_drv_t * drv, lv_indev_data_t * data)
{
    if (BOOTSEL) {
        // 如果按键按下，记录开始按下的时间
        if (btn1_pushed_start_time == 0) {
            btn1_pushed_start_time = millis();
        }
        // 检测是否为长按
        if (millis() - btn1_pushed_start_time >= BTN_LONG_PUSH_T) {
            if (!btn1_long_pushed) {
                btn1_long_pushed = true;  // 标记为长按
                // 长按的处理，例如切换到不同屏幕
                _ui_screen_change(&ui_Screen3, LV_SCR_LOAD_ANIM_FADE_ON, 500, 0, &ui_Screen3_screen_init);
            }
        }
        data->state = LV_INDEV_STATE_PRESSED;  // 按键按下
    } else {
        // 按键松开，判断是否为短按
        if (btn1_pushed_start_time != 0) {
            if (!btn1_long_pushed) {
                btn1_pushed = true;  // 短按标记
                // 短按的处理，例如切换到不同屏幕
                user_ui_flag = !user_ui_flag;
                if(user_ui_flag){
                  test_points[0][0] = 0; // 重置测试点数据
                  test_points[0][1] = 0;
                  _ui_screen_change(&ui_Screen2, LV_SCR_LOAD_ANIM_FADE_ON, 500, 0, &ui_Screen2_screen_init);
                } else {
                  _ui_screen_change(&ui_Screen1, LV_SCR_LOAD_ANIM_FADE_ON, 0, 0, &ui_Screen1_screen_init);
                }
            }
            btn1_pushed_start_time = 0;  // 重置按下时间
        }
        // 清除长按标记
        btn1_long_pushed = false;
        data->state = LV_INDEV_STATE_RELEASED;  // 按键松开
    }
}
#endif

#define KALMAN  // 使用 卡尔曼滤波器
#if defined(KALMAN)
#include "kalman_filter.h"
int KALMAN_flag = 0;
const static float init_P = 0.1;      // 估算协方差 表示估计误差的大小
const static float init_G = 0.0;      // 卡尔曼增益 (Kalman Gain) 表示滤波器如何调整预测值和测量值之间的权重
const static float init_O = 26;       // 卡尔曼滤波器的输出
static KFPTypeS kfpVar3Array[768];  // 卡尔曼滤波器变量数组
// 初始化卡尔曼滤波器数组的函数
void KalmanArrayInit() { 
    // 循环遍历数组中的每个元素
    for (int i = 0; i < 768; ++i) {
        // 初始化每个元素
        kfpVar3Array[i] = (KFPTypeS){
         init_P,     //估算协方差. 初始化值为 0.02
         init_G,     //卡尔曼增益. 初始化值为 0
         init_O    //卡尔曼滤波器输出. 初始化值为 0
        };
    }
}
// 事件回调函数
void switch_event_handler(lv_event_t *e) {
    lv_obj_t *sw = lv_event_get_target(e);
    // 获取当前开关的状态
    // 检查开关的状态，是否处于选中（开启）状态
    bool checked = lv_obj_has_state(sw, LV_STATE_CHECKED);
    // 根据开关的状态来更新 KALMAN_flag
    if (checked) {
        KALMAN_flag = 1;  // 开关打开，KALMAN_flag 设置为 1
    } else {
        KALMAN_flag = 0;  // 开关关闭，KALMAN_flag 设置为 0
    }
}
#endif

void dropdown_event_handler(lv_event_t *e) {
    lv_obj_t *dropdown = lv_event_get_target(e);  // 获取目标对象
    int selected_option = lv_dropdown_get_selected(dropdown); // 获取当前选中的项

    // 检查选中的选项并更新 model_flag
    if (selected_option == 0) {  // "Non-interpol" 被选中
        model_flag = 0;
    } else if (selected_option == 1) {  // "Interp" 被选中
        model_flag = 1;
    }
}

#define _SCALE 9
#define TOUCH_LONG_PUSH_T 200

const byte MLX90640_address = 0x33;
static float mlx90640To[768];              // 从MLX90640读取的温度数据
static int mlx90640To_buffer[768];       // 缓存区域，复制MLX90640读取的温度数据并用于绘制热力图
static float mlx90640To_send_buffer[768];  // 缓存区域，复制MLX90640读取的温度数据，用于发送到上位机
static uint8_t* mlx90640To_Serial_buffer = (uint8_t*)mlx90640To_send_buffer;  

paramsMLX90640 mlx90640;

#if defined(DRAW_PIXELS)
static uint16_t heat_bitmap[32*_SCALE * 24*_SCALE] = {}; // rgb56556形式的内存，用于存储要渲染的图像
#endif

int R_colour, G_colour, B_colour;            
// int i, j;
// 存放 T_avg 值的数组
// #define MAX_HISTORY 10
// lv_coord_t ui_Chart2_series_1_array1[] = { 25, 10, 20, 30, 25, 50, 40, 23, 25, 21 };                             
float T_max, T_min, T_avg;                            
float T_center;  
int max_x, max_y, min_x, min_y;
float bat_v;
char buf_bat_v[20];  // 存储格式化后的字符串
bool lock = false;  // 简单的锁，防止拷贝温度数据的时候对内存的访问冲突
bool serial_cp_lock = false;  // 简单的锁，防止拷贝温度数据的时候对内存的访问冲突
bool touch_updated = false;
bool mlx_is_connected = false;
bool power_on = true;  // 是否开机
bool show_local_temp_flag = true;  // 是否显示点测温
bool clear_local_temp = false;     // 点测温清除
int diffx, diffy;

// ===============================
// ===== determine the colour ====
// ===============================
void getColour(int j)
{
  switch(color_flag) {
    case 0:  // HotMetal
        if (j >= 0 && j < 30) // 蓝色
          {
            R_colour = 0;
            G_colour = 0;
            B_colour = 20 + 4 * j;
          }
        
        if (j >= 30 && j < 60) // 蓝紫色
          {
            R_colour = 4 * (j - 30);
            G_colour = 0;
            B_colour = 140 - 2 * (j - 30);
          }

        if (j >= 60 && j < 90) // 紫色
          {
            R_colour = 120 + 4 * (j - 60);
            G_colour = 0;
            B_colour = 80 - 2 * (j - 60);
          }

        if (j >= 90 && j < 120) // 红紫色
          {
            R_colour = 255;
            G_colour = 0 + 2 * (j - 90);
            B_colour = 10 - (j - 90) / 3;
          }

        if (j >= 120 && j < 150) // 红色
          {
            R_colour = 255;
            G_colour = 60 + 175 * (j - 120) / 30;
            B_colour = 0;
          }

        if (j >= 150 && j <= 180) // 黄色
          {
            R_colour = 255;
            G_colour = 235 + (j - 150) * 20 / 30;
            B_colour = 0 + 85 * (j - 150) / 10;
          }
        break;
    case 1:  // Rainbow
        if (j >= 0 && j < 30) { // 从红色到橙色
            R_colour = 255;
            G_colour = j * 8;  // 绿色逐渐增加
            B_colour = 0;  // 蓝色保持为0
        }

        if (j >= 30 && j < 60) { // 从橙色到黄色
            R_colour = 255 - (j - 30) * 8;  // 红色逐渐减少
            G_colour = 255;  // 绿色保持为255
            B_colour = (j - 30) * 8;  // 蓝色逐渐增加
        }

        if (j >= 60 && j < 90) { // 从黄色到绿色
            R_colour = 0;  // 红色为0
            G_colour = 255 - (j - 60) * 8;  // 绿色逐渐减少
            B_colour = (j - 60) * 8;  // 蓝色逐渐增加
        }

        if (j >= 90 && j < 120) { // 从绿色到青色
            R_colour = 0;  // 红色为0
            G_colour = 0;  // 绿色为0
            B_colour = (j - 90) * 8;  // 蓝色逐渐增加
        }

        if (j >= 120 && j < 150) { // 从青色到蓝色
            R_colour = 0;  // 红色为0
            G_colour = (j - 120) * 8;  // 绿色逐渐增加
            B_colour = 255;  // 蓝色为255
        }

        if (j >= 150 && j <= 180) { // 从蓝色到紫色
            R_colour = (j - 150) * 8;  // 红色逐渐增加
            G_colour = 0;  // 绿色为0
            B_colour = 255 - (j - 150) * 8;  // 蓝色逐渐减少
        }
        break;
    case 2:  // Iron
        if (j >= 0 && j < 30) { // 暗红色
            R_colour = 50 + 5 * j;
            G_colour = 0;
            B_colour = 0;
        }

        if (j >= 30 && j < 60) { // 深橙色
            R_colour = 200;
            G_colour = 10 + 5 * (j - 30);
            B_colour = 0;
        }

        if (j >= 60 && j < 90) { // 橙色
            R_colour = 255;
            G_colour = 80 + 5 * (j - 60);
            B_colour = 0;
        }

        if (j >= 90 && j < 120) { // 黄色
            R_colour = 255;
            G_colour = 180 + 10 * (j - 90);
            B_colour = 0;
        }

        if (j >= 120 && j < 150) { // 浅黄色
            R_colour = 255;
            G_colour = 255;
            B_colour = 10 + (j - 120) * 5 / 30;
        }

        if (j >= 150 && j <= 180) { // 浅灰色
            R_colour = 240 - (j - 150) * 5 / 30;
            G_colour = 240 - (j - 150) * 5 / 30;
            B_colour = 240 - (j - 150) * 5 / 30;
        }
        break;
    case 3:  // Grayscale
        if (j >= 0 && j < 30) { // 黑色到灰色
            R_colour = 0 + (j * 8);   // 从黑色到灰色
            G_colour = 0 + (j * 8);   // 从黑色到灰色
            B_colour = 0 + (j * 8);   // 从黑色到灰色
        }

        if (j >= 30 && j < 60) { // 灰色到浅灰色
            R_colour = 240 - (j - 30) * 4;  // 由灰色过渡到浅灰色
            G_colour = 240 - (j - 30) * 4;  // 由灰色过渡到浅灰色
            B_colour = 240 - (j - 30) * 4;  // 由灰色过渡到浅灰色
        }

        if (j >= 60 && j < 90) { // 浅灰色到白色
            R_colour = 120 + (j - 60) * 5;  // 由浅灰色过渡到白色
            G_colour = 120 + (j - 60) * 5;  // 由浅灰色过渡到白色
            B_colour = 120 + (j - 60) * 5;  // 由浅灰色过渡到白色
        }

        if (j >= 90 && j < 120) { // 白色到极白
            R_colour = 255;
            G_colour = 255;
            B_colour = 255;
        }

        if (j >= 120 && j < 150) { // 极白到更亮的白
            R_colour = 255;
            G_colour = 255;
            B_colour = 255;
        }

        if (j >= 150 && j <= 180) { // 极亮白色
            R_colour = 255;
            G_colour = 255;
            B_colour = 255;
        }
        break;
    case 4:  // Bluewhite
        if (j >= 0 && j < 30) { // 从蓝色到深蓝
            R_colour = 0;
            G_colour = 0;
            B_colour = 20 + 4 * j;  // 从深蓝到蓝色过渡
        }

        if (j >= 30 && j < 60) { // 蓝色到浅蓝
            R_colour = 0;
            G_colour = 0;
            B_colour = 120 + (j - 30) * 4;  // 从蓝色到浅蓝过渡
        }

        if (j >= 60 && j < 90) { // 浅蓝到亮蓝
            R_colour = 0;
            G_colour = 40 + (j - 60) * 2;  // 增加绿色成分，形成亮蓝
            B_colour = 200 - (j - 60) * 4;  // 减少蓝色的深度
        }

        if (j >= 90 && j < 120) { // 亮蓝到浅白
            R_colour = 100 + (j - 90) * 3;  // 渐变到较亮的红色
            G_colour = 150 + (j - 90) * 3;  // 渐变到较亮的绿色
            B_colour = 255 - (j - 90) * 5;  // 蓝色成分减少
        }

        if (j >= 120 && j < 150) { // 浅白到白色
            R_colour = 200 + (j - 120) * 4;  // 红色成分增加
            G_colour = 200 + (j - 120) * 4;  // 绿色成分增加
            B_colour = 255 - (j - 120) * 3;  // 蓝色减小
        }

        if (j >= 150 && j <= 180) { // 白色
            R_colour = 255;
            G_colour = 255;
            B_colour = 255;  // 完全白色
        }
        break;
    case 5:  // Purplered
        if (j >= 0 && j < 30) { // 从紫色到红紫色
            R_colour = 60 + (j * 6);  // 红色从60渐增到180
            G_colour = 0;
            B_colour = 255 - (j * 8);  // 蓝色从255逐渐减少
        }

        if (j >= 30 && j < 60) { // 从红紫色到红色
            R_colour = 180 + (j - 30) * 3;  // 红色从180逐渐增加
            G_colour = 0;
            B_colour = 10 - (j - 30) * 2;  // 蓝色逐渐减少
        }

        if (j >= 60 && j < 90) { // 从红色到橙色
            R_colour = 255;
            G_colour = 30 + (j - 60) * 4;  // 绿色逐渐增加
            B_colour = 0;  // 蓝色为0
        }

        if (j >= 90 && j < 120) { // 从橙色到黄色
            R_colour = 255;
            G_colour = 150 + (j - 90) * 2;  // 绿色逐渐增加
            B_colour = 0;  // 蓝色保持为0
        }

        if (j >= 120 && j < 150) { // 从黄色到亮黄色
            R_colour = 255;
            G_colour = 230 + (j - 120) * 2;  // 绿色逐渐增加
            B_colour = 0;  // 蓝色保持为0
        }

        if (j >= 150 && j <= 180) { // 亮黄色到白色
            R_colour = 255;
            G_colour = 255;
            B_colour = 255;  // 完全白色
        }
        break;
    case 6:  // Ironbow
        if (j >= 0 && j < 30) { // 从铁灰色到金属蓝色
            R_colour = 80 + (j * 6);  // 红色从80渐增到180
            G_colour = 80 + (j * 4);  // 绿色从80渐增到160
            B_colour = 255 - (j * 6);  // 蓝色从255逐渐减少
        }

        if (j >= 30 && j < 60) { // 从金属蓝色到紫色
            R_colour = 180 + (j - 30) * 4;  // 红色逐渐增加
            G_colour = 160 + (j - 30) * 2;  // 绿色逐渐增加
            B_colour = 255 - (j - 30) * 5;  // 蓝色逐渐减少
        }

        if (j >= 60 && j < 90) { // 从紫色到红紫色
            R_colour = 255 - (j - 60) * 3;  // 红色逐渐减少
            G_colour = 180 - (j - 60) * 4;  // 绿色逐渐减少
            B_colour = 255 - (j - 60) * 6;  // 蓝色逐渐减少
        }

        if (j >= 90 && j < 120) { // 从红紫色到红色
            R_colour = 255;
            G_colour = 180 - (j - 90) * 5;  // 绿色逐渐减少
            B_colour = 255 - (j - 90) * 5;  // 蓝色逐渐减少
        }

        if (j >= 120 && j < 150) { // 从红色到橙色
            R_colour = 255;
            G_colour = 120 + (j - 120) * 3;  // 绿色逐渐增加
            B_colour = 0;  // 蓝色为0
        }

        if (j >= 150 && j <= 180) { // 从橙色到黄色
            R_colour = 255;
            G_colour = 200 + (j - 150) * 2;  // 绿色逐渐增加
            B_colour = 0;  // 蓝色保持为0
        }
        break;
  }
}

void roller_event_handler(lv_event_t *e) {
    lv_obj_t *roller = lv_event_get_target(e);
    int selected = lv_roller_get_selected(roller);  // 获取选中的索引
    // 更新 color_flag 根据滚轮选择的选项
    switch (selected) {
        case 0: // HotMetal
            color_flag = 0;
            break;
        case 1: // Rainbow
            color_flag = 1;
            break;
        case 2: // Iron
            color_flag = 2;
            break;
        case 3: // Grayscale
            color_flag = 3;
            break;
        case 4: // Bluewhite
            color_flag = 4;
            break;
        case 5: // Purplered
            color_flag = 5;
            break;
        case 6: // Ironbow
            color_flag = 6;
            break;
        default:
            color_flag = 0; // 默认是 HotMetal
            break;
    }
}

//Returns true if the MLX90640 is detected on the I2C bus
boolean isConnected()
   {
    Wire.beginTransmission((uint8_t)MLX90640_address);
    if (Wire.endTransmission() != 0){return (false);}
    return (true);
   }   

// 绘制十字
void draw_cross(int x, int y, int len){
   tft.drawLine(x - len/2, y, x + len/2, y, tft.color565(255, 255, 255));
   tft.drawLine(x, y-len/2, x, y+len/2,  tft.color565(255, 255, 255));

   tft.drawLine(x - len/4, y, x + len/4, y, tft.color565(0, 0, 0));
   tft.drawLine(x, y-len/4, x, y+len/4,  tft.color565(0, 0, 0));
}

// 点测温功能
void show_local_temp(int x, int y){
   draw_cross(x, y, 10);
   float temp_xy = mlx90640To[(24 - y / _SCALE) * 32 + (x / _SCALE)];
   int shift_x, shift_y;
   if (x<140){shift_x=10;} else {shift_x=-40;}
   if (y<120){shift_y=10;} else {shift_y=-10;}
   tft.setTextSize(CURSOR_SIZE);
   tft.setCursor(x+shift_x, y+shift_y);
   tft.printf("%.2f", temp_xy);
}  

#if defined(DRAW_PIXELS_DMA)
const int lines = 25;
uint16_t  lineBuffer[32 * _SCALE * lines];  // Toggle buffer for lines 当前线的像素缓冲区
uint16_t  dmaBuffer1[32 * _SCALE * lines];  // Toggle buffer for lines DMA 缓冲区 1
uint16_t  dmaBuffer2[32 * _SCALE * lines];  // Toggle buffer for lines DMA 缓冲区 2
uint16_t* dmaBufferPtr = dmaBuffer1;        // 当前使用的 DMA 缓冲区指针
bool dmaBufferSel = 0;                      // 缓冲区切换标志
// 在屏幕上绘制热力图
void draw_heat_image(bool re_mapcolor=true){  
   static int value;                        //当前像素值
   static int now_y = 0;                    //当前缓冲区中的行数计数器
   tft.setRotation(SCREEN_ROTATION);        //设置屏幕的旋转方向
   if(model_flag){ 
      tft.startWrite();
      for(int y=0; y<24 * _SCALE; y++){ 
         for(int x=0; x<32 * _SCALE; x++){
            value = bio_linear_interpolation(x, y, mlx90640To_buffer);
            getColour(value);
            lineBuffer[x + now_y*32 * _SCALE] = tft.color565(R_colour, G_colour, B_colour);
         }
         now_y ++;
         if(now_y==lines){
            if (dmaBufferSel) dmaBufferPtr = dmaBuffer2;
            else dmaBufferPtr = dmaBuffer1;
            dmaBufferSel = !dmaBufferSel; // Toggle buffer selection
            // tft.startWrite();
            tft.pushImageDMA(0, y-now_y, 32*_SCALE, lines, lineBuffer, dmaBufferPtr);
            // tft.endWrite();
            now_y = 0;
         }
      }if(now_y!=0){
         if (dmaBufferSel) dmaBufferPtr = dmaBuffer2;
         else dmaBufferPtr = dmaBuffer1;
         dmaBufferSel = !dmaBufferSel; // Toggle buffer selection
         // tft.startWrite();
         tft.pushImageDMA(0, 24*_SCALE-1-now_y, 32*_SCALE, now_y, lineBuffer, dmaBufferPtr);
         // tft.endWrite();
         now_y = 0;
      }
      tft.endWrite();
   } else { //非插值模式
      tft.setRotation(3);
      for (int i = 1 ; i < 24 ; i++){
        for (int j = 0; j < 32; j++){
          // if (re_mapcolor) {mlx90640To_buffer[i*32 + j] = 180.0 * (mlx90640To_buffer[i*32 + j] - T_min) / (T_max - T_min);}
          getColour(mlx90640To_buffer[i*32 + j]);
          tft.fillRect(280 - j * _SCALE, (240 - _SCALE * 24) + i * _SCALE, _SCALE, _SCALE, tft.color565(R_colour, G_colour, B_colour));  
        }
      }
   }

}

#endif

int status;
uint16_t eeMLX90640[832];
int mlx_setup(){
   pinMode(MLX_VDD, OUTPUT);
   digitalWrite(MLX_VDD, LOW);
   Wire.setSDA(MLX_SDA);
   Wire.setSCL(MLX_SCL);
   Wire.begin(); 
   vTaskDelay(500);
   Wire.setClock(800000); //Increase I2C clock speed to 800kHz
   Serial1.println("MLX90640 IR Array Example");
   mlx_is_connected = isConnected(); //调用 isConnected() 函数检查 MLX90640 是否成功连接
   if (mlx_is_connected == false){   
      // while(!isConnected()){
         Serial1.println("MLX90640 not detected at default I2C address. Please check wiring.");
      // }
      return 1;
   }
   Serial1.println("MLX90640 online!"); //如果传感器已连接，输出 “MLX90640 online!” 表示传感器成功连接。
   status = MLX90640_DumpEE(MLX90640_address, eeMLX90640); //读取传感器的校准数据和配置数据
   if (status != 0)
      Serial1.println("Failed to load system parameters");

   //数据中提取并解析所有的配置和校准参数，并将它们存储在一个结构体
   status = MLX90640_ExtractParameters(eeMLX90640, &mlx90640); 
   if (status != 0)
   {
      Serial1.println("Parameter extraction failed");
      Serial1.print(" status = ");
      Serial1.println(status);
   }
   //设置 MLX90640 传感器的刷新率为 4Hz
   MLX90640_SetRefreshRate(MLX90640_address, 0x04); //Set rate to 4Hz effective - Works 
   // MLX90640_I2CWrite(0x33, 0x800D, 6401);    // writes the value 1901 (HEX) = 6401 (DEC) in the register at position 0x800D to enable reading out the temperatures!!!
   //设置 MLX90640 传感器的刷新率为 8Hz
   MLX90640_SetRefreshRate(MLX90640_address, 0x05); //Set rate to 8Hz effective - Works at 800kHz
   return 0;
}

uint16_t count_retry = 0;
void mlx_loop(){
  // 检查传感器是否已连接。如果未连接 最大尝试次数为 10
   if(!mlx_is_connected && count_retry < 10){
      mlx_setup();
      count_retry++;
   }
   if (!freeze && mlx_is_connected==true){ // 如果画面被暂停会跳过这个热成像图的刷新
      lock = true;  //在数据采集过程中启用锁定，防止其他任务干扰
      //帧数据采集
      for (byte x = 0 ; x < 2 ; x++){ 
         uint16_t mlx90640Frame[834];
         int status = MLX90640_GetFrameData(MLX90640_address, mlx90640Frame);
         if (status < 0){
              Serial1.print("GetFrame Error: ");
              Serial1.println(status);
          }

         //环境参数计算
         float vdd = MLX90640_GetVdd(mlx90640Frame, &mlx90640); //通过帧数据计算传感器的电源电压
         float Ta = MLX90640_GetTa(mlx90640Frame, &mlx90640);   //计算传感器当前的环境温度
         float tr = Ta - TA_SHIFT; //Reflected temperature based on the sensor ambient temperature 计算反射温度
         float emissivity = 0.95;  //设置发射率为 0.95。
         //根据帧数据和环境参数计算每个像素点的温度。
         MLX90640_CalculateTo(mlx90640Frame, &mlx90640, emissivity, tr, mlx90640To);
      }

      // 错误像素修复
      // mlx90640To[229] = 0.5 * (mlx90640To[228] + mlx90640To[230]);    // eliminate the error-pixels
      // mlx90640To[428] = 0.5 * (mlx90640To[427] + mlx90640To[429]);    // eliminate the error-pixels
      
      //温度统计
      T_min = mlx90640To[0];
      T_max = mlx90640To[0];
      T_avg = mlx90640To[0];
      for (int i = 1; i < 768; i++){
         if((mlx90640To[i] > -41) && (mlx90640To[i] < 301)) //MLX90640测温范围通常-40°C至300°C 超出范围的值可能是异常值，需要过滤
            {
               if(mlx90640To[i] < T_min) //记录当前最低的温度值
                  {
                    T_min = mlx90640To[i];
                  }

               if(mlx90640To[i] > T_max) //更新 T_max 及其所在的像素位置
                  {
                    T_max = mlx90640To[i];
                    max_x = i / 32;
                    max_y = i % 32;
                  }
            #if defined(KALMAN) //对当前像素点的温度值进行 Kalman 滤波处理
            if(KALMAN_flag){
              mlx90640To[i] = KalmanFilter(&kfpVar3Array[i], mlx90640To[i]);
            }
            #endif
            }
         else if(i > 0){ //处理异常值 如果当前像素点的索引 i 大于 0，则用前一个像素点的温度值替代当前值。
               mlx90640To[i] = mlx90640To[i-1];
            }
         else{//如果当前像素点是第一个像素点（i = 0），则用第二个像素点的温度值替代当前值
                mlx90640To[i] = mlx90640To[i+1];
            }
            T_avg = T_avg + mlx90640To[i]; //累加温度值
         }
      T_avg = T_avg / 768; //计算平均温度

      //卡尔曼滤波器处理
      #if defined(KALMAN)
      if(KALMAN_flag){
        T_avg = KalmanFilter(&kfpVar1, T_avg);
        T_max = KalmanFilter(&kfpVar2, T_max);
        T_min = KalmanFilter(&kfpVar3, T_min);
      }
      #endif
      lock = false; //在数据采集过程中启用锁定，防止其他任务干扰
   }
}

// 热成像读取多任务
void task_mlx(void * ptr){
   mlx_setup();
   // MLX主循环
   for(;power_on==true;){
      mlx_loop();
      vTaskDelay(10);
   }
   vTaskDelete(NULL); 
}

// 背光调节,会限制输入亮度在正确范围内
void set_brightness(int _brightness){
   if (_brightness < 255 && _brightness > 5){
      analogWriteFreq(10000);
      analogWrite(SCREEN_BL_PIN, _brightness);
      brightness = _brightness;
   }else if(_brightness >= 255){analogWrite(SCREEN_BL_PIN, 255); brightness=255;
   }else if(_brightness <= 5)   {analogWrite(SCREEN_BL_PIN, 5); brightness=5;
   }
}

// 滑动条值变化时的回调函数
void slider_event_handler(lv_event_t * e) {
    lv_obj_t * slider = lv_event_get_target(e);
    int value = lv_slider_get_value(slider);  // 获取滑动条的当前值
    set_brightness(value);  // 调整背光亮度
}

void smooth_on(){
   pinMode(SCREEN_BL_PIN, OUTPUT);
   analogWrite(SCREEN_BL_PIN, 0);
   analogWriteFreq(10000);
   for(int i=0; i<brightness; i++){
      analogWrite(SCREEN_BL_PIN, i);
      vTaskDelay(2);
   }
}


uint32_t dt = millis();
void screen_setup(){
   tft.setRotation(SCREEN_ROTATION);
   tft.fillScreen(TFT_BLACK);
   // tft.fillScreen(TFT_GREEN);
  //  test_points[0][0] = 120;
  //  test_points[0][1] = 110;
   tft.setTextSize(0);
   tft.setCursor(25, 220);
   tft.printf("max: %.2f  ", T_max);
   tft.setCursor(25, 230);
   tft.printf("min: %.2f  ", T_min);

   tft.setCursor(105, 220);
   tft.printf("avg: %.2f  ", T_avg);
   tft.setCursor(105, 230);
   tft.printf("bat: %.2f v ", bat_v);

   tft.setCursor(180, 220);
   tft.printf("bright: %d  ", brightness);
   tft.setCursor(180, 230);
   tft.printf("time: %d ", dt);
   tft.printf("ms     ");
}

void screen_loop(){
   if (!freeze){ // 如果画面被暂停会跳过这个热成像图的刷新
   // 只有画面更新才会绘制一张热成像图
   dt = millis();
   while(lock && power_on){
      // 阻塞画面
      vTaskDelay(1); //阻塞刷新过程
   }
   // 温度信息的处理与热成像图更新
   for (int i = 0; i < 768; i++) {
      // mlx90640To_buffer[i] = mlx90640To[i];
      //将原始温度数据（mlx90640To）转换为 用于 绘制热成像图的数据（mlx90640To_buffer）
      mlx90640To_buffer[i] = (int)(180.0 * (mlx90640To[i] - T_min) / (T_max - T_min));
   }  // 拷贝温度信息
    draw_heat_image();  //绘制热成像图
    dt = millis() - dt; //更新刷新时间
   } else {dt = 0;} // 画面冻结时的处理

   //显示触摸点的温度
   tft.setRotation(SCREEN_ROTATION);
   if (test_points[0][0]==0 && test_points[0][1]==0 ){}else{show_local_temp(test_points[0][0], test_points[0][1]);}
   //清除局部温度信息
   if (clear_local_temp==true) {draw_heat_image(false); clear_local_temp=false;}

   //信息显示 //7yewh
   tft.setRotation(SCREEN_ROTATION);
   tft.setTextColor(TFT_WHITE, TFT_BLACK); 
   tft.setTextSize(0);
   //设置屏幕旋转方向、文字颜色（白色文本，黑色背景）、文字大小。
   if (!mlx_is_connected){
      tft.setCursor(25, 110);
      tft.printf("MLX90640 not detected at default I2C address. Please check wiring.");
   }

   //如果 mlx_is_connected 为 false，表示未检测到 MLX90640 传感器，屏幕上会显示错误信息。
   tft.setCursor(25, 220);
   tft.printf("max: %.2f  ", T_max);
   tft.setCursor(25, 230);
   tft.printf("min: %.2f  ", T_min);

   //显示温度的最大值和最小值，T_max 和 T_min。
   tft.setCursor(105, 220);
   tft.printf("avg: %.2f  ", T_avg);
   tft.setCursor(105, 230);
   tft.printf("bat: %.2f v ", bat_v);

   //显示温度的平均值 T_avg，以及当前电池电压 bat_v。
   tft.setCursor(180, 220);
   tft.printf("bright: %d  ", brightness);
   tft.setCursor(180, 230);
   tft.printf("time: %d ", dt);
   tft.printf("ms     ");
   // vTaskDelay(10);
}

void update_battery_bar(float bat_v) {
    // 将 bat_v 映射到进度条的 0-100 范围
    int progress = (int)((bat_v / 5.0) * 100);  // 将电压映射到 0-100 的范围

    // 限制进度条的最大值为 100，最小值为 0
    if (progress > 100) progress = 100;
    if (progress < 0) progress = 0;

    // 格式化电压并显示到 ui_Label1
    snprintf(buf_bat_v, sizeof(buf_bat_v), "%.2fV", bat_v);  // 格式化 bat_v 为带有两位小数的字符串
    lv_label_set_text(ui_Label1, buf_bat_v);  // 更新 ui_Label1 显示电池电压

    // 更新进度条显示
    lv_bar_set_value(ui_Bar3, progress, LV_ANIM_OFF);  // 根据电池电量更新进度条的值
}

void task_screen_draw(void * ptr){
   screen_setup();
   for(;power_on==true;){
      screen_loop();
   }
   vTaskDelete(NULL);
}

// 通过串口传输单个浮点数据
void send_float_as_uint8(float f, uint8_t *buf) {
   memcpy(buf, &f, sizeof(float));
   Serial.write(buf, sizeof(float));
}

// 通过串口把整个温度数据矩阵传输
void send_to_serial() {
   // memcpy(mlx90640To_Serial_buffer, mlx90640To_send_buffer, 768 * sizeof(float));
   Serial.write(mlx90640To_Serial_buffer, 768 * sizeof(float));
}

void task_serial_communicate(void * ptr){
   vTaskDelay(3000);
   uint8_t send_buf[4];

   for(;power_on==true;){
      // 拷贝温度信息
      while (lock == true) {vTaskDelay(1);}
      memcpy(mlx90640To_send_buffer, mlx90640To, 768 * sizeof(float));
      // for (int i = 0; i < 768; i++) {mlx90640To_send_buffer[i] = mlx90640To[i];} 
      Serial.print("BEGIN");
      send_float_as_uint8(T_max, send_buf);
      send_float_as_uint8(T_min, send_buf);
      send_float_as_uint8(T_avg, send_buf);
      // for (int i = 0; i < 768; i++){
      //    send_float_as_uint8(mlx90640To_send_buffer[i], send_buf);
      //    if(i % 5==0){vTaskDelay(1);}
      // }
      send_to_serial();
      Serial.print("END");
      vTaskDelay(30);
   }
   vTaskDelete(NULL);
}






// lv_chart_series_t * ui_Chart2_series_11;
void setup1(void)
 {
   Serial.begin(115200);              //初始化串口
   EEPROM.begin(128);                 //初始化EEPROM
   uint8_t value = EEPROM.read(0);    
   if (value != 0 && value != 1) {    
      value = 0;  
   }
   model_flag = value;              //标志是否使用上采样功能
   brightness = EEPROM.read(1);       //从EEPROM读取的亮度值
#if defined(SERIAL1_DEBUG)         //启动串口调试
   Serial1.begin(115200);
   Serial1.println("RP2040 is starting...");
#endif
#if defined(KALMAN)                //卡尔曼滤波器初始化
   KalmanArrayInit();
#endif
#if defined(LVGL_UI_USER)
  touch.begin();  
  pinMode(SCREEN_VDD, OUTPUT);         //显示屏电源控制
  digitalWrite(SCREEN_VDD, LOW);   
  lv_init();
  tft.begin(); 
  tft.setRotation( ROTATE ); /* Landscape orientation, flipped */
  touch.begin();                     //触摸屏初始化
  lv_disp_draw_buf_init( &draw_buf, buf, NULL, screenWidth * screenHeight / 10 );

  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init( &disp_drv );
  disp_drv.hor_res = screenWidth;
  disp_drv.ver_res = screenHeight;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register( &disp_drv );

  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init( &indev_drv );
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touchpad_read;
  lv_indev_drv_register( &indev_drv );
    
  static lv_indev_drv_t indev_drv_button;
  lv_indev_drv_init(&indev_drv_button);
  indev_drv_button.type = LV_INDEV_TYPE_KEYPAD;   // 设置为按键类型输入设备
  indev_drv_button.read_cb = my_keypad_read;  // 设置按键读取回调函数
  lv_indev_drv_register(&indev_drv_button);  // 注册输入设备

  static lv_indev_drv_t indev_drv_button_BOOTSEL;
  lv_indev_drv_init(&indev_drv_button_BOOTSEL);
  indev_drv_button_BOOTSEL.type = LV_INDEV_TYPE_KEYPAD;   // 设置为按键类型输入设备
  indev_drv_button_BOOTSEL.read_cb = my_keypad_bootsel_read;  // 设置按键读取回调函数
  lv_indev_drv_register(&indev_drv_button_BOOTSEL);  // 注册输入设备
  
  ui_init();
  // ui_Chart2_series_11 = lv_chart_add_series(ui_Chart2, lv_color_hex(0x808080), LV_CHART_AXIS_PRIMARY_Y);
#endif
  tft.setSwapBytes(true);            //设置字节交换，用于显示RGB图像。
  tft.initDMA();                     //启用DMA（直接内存访问），用于屏幕数据传输。//7yewh
  screen_setup();
  vTaskDelay(300);
  smooth_on();                       //调用一个名为smooth_on的函数（可能用于初始化滤波或过渡效果）。
}

void loop1() 
{
  if(!user_ui_flag && lv_scr_act() == ui_Screen1){
    screen_loop();
  } else if(user_ui_flag){
    if(lv_scr_act() == ui_Screen2){
      snprintf(buf_bat_v, sizeof(buf), "%.2fV", bat_v);  // 格式化 bat_v 为带有两位小数的字符串
      lv_label_set_text(ui_Label1, buf_bat_v);
    } else if (lv_scr_act() == ui_Screen5){
      update_battery_bar(bat_v);
    } else if (lv_scr_act() == ui_Screen4){
      // 将 T_avg_history 数组传递给图表的 Y 轴 ui_Chart2_series_11 ui_Chart2_series_1_array1
      //lv_chart_clear_series(ui_Chart2, ui_Chart2_series_11);  // 清空图表数据
      // lv_chart_set_ext_y_array(ui_Chart2, ui_Chart2_series_11, ui_Chart2_series_1_array1);
    }
  }
  lv_timer_handler(); /* let the GUI do its work */
  //delay( 5 );
}

void setup(void)
{
   pinMode(buttonPin1, INPUT_PULLUP); // 设置按钮引脚为输入并启用上拉电阻
   pinMode(BAT_ADC, INPUT);           // 设置 ADC 引脚为输入
   xTaskCreate(task_mlx, "MLX_FLASHING", 1024 * 4, NULL, 1, NULL);//7yewh
   delay(100);
   uint8_t send_buf[4];

   float r1 = 300.;             // 分压电阻1
   float r2 = 680.;             // 分压电阻2
   float coef = (r1+r2) / r2;   // 分压比例系数 分压公式计算实际电池电压。
   int adc_value = analogRead(BAT_ADC);         // 存储 ADC 转换后的值，用于计算电池电压
   TickType_t xStartTime = xTaskGetTickCount(); // 用于控制任务的时间间隔
   const TickType_t xWait = 5000;

   uint16_t x, y;                   // 用于触摸屏检测
   uint16_t start_x, start_y;
   bool long_pushed = false;
   unsigned long touch_pushed_start_time =  0;
   bool touched = false;
   int start_br = brightness;       // 当前亮度的初始值

  //  // 当前 T_avg 索引
  //  int history_index = 0;
  //  TickType_t xStartTime_T_avg = xTaskGetTickCount(); // 用于控制任务的时间间隔
  //  const TickType_t xWait_T_avg = 5000;


   for(;power_on==true;){
    // 触摸状态检测
      if(xStartTime + xWait > xTaskGetTickCount()){
        adc_value = analogRead(BAT_ADC);
        bat_v = (float)adc_value / 1024. * 3.3 * coef;
        xStartTime = xTaskGetTickCount();
      }
      
      // if(xStartTime_T_avg + xWait_T_avg > xTaskGetTickCount()){
      //   float current_T_avg = (float)(rand() % 100) / 10.0f;  // 模拟 T_avg 值
      //   if (history_index >= MAX_HISTORY) {
      //   for (int i = 1; i < MAX_HISTORY; i++) {
      //       T_avg_history[i - 1] = T_avg_history[i];
      //   }
      //     history_index = MAX_HISTORY - 1;
      //   }
      //   T_avg_history[history_index++] = current_T_avg;
        
      //   xStartTime_T_avg = xTaskGetTickCount();
      // }

      if(!user_ui_flag){
      if (!touch_updated){touch.update(); touch_updated=true;}
      if (touch_updated) {
      if( touch.tp.touching )
      {
         x= touch.tp.y;
         y = 240 - touch.tp.x;
         if (touched==false){start_x = x;  start_y = y; diffy=0; diffx=0;}  // 下降沿
         if (millis() - touch_pushed_start_time >= TOUCH_LONG_PUSH_T){
            long_pushed = true;
            diffx= start_x-x;
            diffy= start_y-y;
            set_brightness(start_br+diffy*5);
            }else{ // 短按的中间
               //NULL
            }
      }else{
         touch_pushed_start_time = millis();
         if (touched==true){  // 上升沿
            if (start_br == brightness){
               if (y < 207){test_points[0][0] = x; test_points[0][1] = y;}
            }
            if (long_pushed==false){  // 短按时
               if (y < 207){test_points[0][0] = x; test_points[0][1] = y;}
            }
            start_br = brightness;
            EEPROM.write(1, brightness);
            EEPROM.commit();
            long_pushed = false;  // 上升沿将长按检测标识符进行复位
         }  
      }
      touched = touch.tp.touching;
      touch_updated = false;
    } else {
    }
   }
   vTaskDelay(10);
   }
   vTaskDelete(NULL);
}

void loop() 
{
   vTaskDelay(3000);
}

