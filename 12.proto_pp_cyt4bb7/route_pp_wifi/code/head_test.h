/* 工程 11 · 航向测试（App「测试」页的两个实验）
 * ============================  这两个测试在验什么  ============================
 * 把"舵机转角"的来源从**摇杆/滑条**换成**航向**（就是 steer_ctrl 的航向外环），
 * 然后用两个固定工况去量外环到底行不行：
 * ① 固定航向（保持直线）
 * 后轮恒定 rps，目标航向 = 起步航向 + ht_yaw（默认 0）
 * ⇒ 看车走得直不直：蛇形、方向盘来回摆、速度越高越差？
 * 判据：横向漂移 / 航向峰峰 / 舵机输出峰峰（CSV 里都有）
 * ② 变角度台阶（模拟路表里的连续弯）
 * 段号 k=1..n，目标航向依次为 0, +step, −2·step, +3·step, −4·step …
 * （默认 step=5° ⇒ 0 / +5 / −10 / +15 / −20 …，与用户给的表一致）
 * 这三个量就是外环的"响应速度 + 稳态精度"，也是路表前视/Ld 的调参依据。
 * 时序必须确定。手机通过 WiFi 发指令的抖动是几十毫秒量级，而我们要量的是
 * "进 ±1° 用了多少毫秒" —— 指令在手机上排队，量出来的数字就全是网络抖动。
 * 所以：**段序列和计时都在车端 5 ms 节拍里跑**，手机只负责
 * `SET_PARAM(ht_*)` + 使能 + 记 CSV + 每 200 ms 重发一次（兼做链路保活）。
 * ============================  安全门（一条不减）  ============================
 * · 链路不在线（motor_link_online()=0）⇒ 立刻收车并自清 ht_mode
 * · 遥控接管 / 失效 / 急停 ⇒ 同上
 * · 与路表回放互斥：`pb_en=1` 时本模块不启动
 * · 收车顺序：先关航向外环 + 把转角目标对齐实测角，再把速度目标清 0
 * （反过来做，退出那一刻方向盘会甩一下）
 */

#ifndef _HEAD_TEST_H
#define _HEAD_TEST_H

#include "zf_common_headfile.h"

/* ---- 参数（ID 69~75）---- */
extern volatile float ht_mode;      /* 69 0=关 1=固定航向 2=变角度台阶 */
extern volatile float ht_rps;       /* 70 后轮恒定目标 rps（1.2 / 2.4 两档） */
extern volatile float ht_yaw;       /* 71 测试①的目标航向（相对起步航向）deg */
extern volatile float ht_dwell;     /* 72 测试②每段停留时间 s */
extern volatile float ht_step;      /* 73 测试②台阶幅值 deg（5 ⇒ 0/+5/−10/+15…） */
extern volatile float ht_n;         /* 74 测试②段数 */
extern volatile float ht_k;         /* 75 航向环增益 deg/deg（测试专用） */

#define HT_MODE_OFF         (0)
#define HT_MODE_FIXED       (1)
#define HT_MODE_STEP        (2)

#define HT_SEG_OFF          (0.0f)      /* 遥测 ht_seg：没在跑 */
#define HT_SEG_DONE         (254.0f)    /* 遥测 ht_seg：序列跑完了 */

void    head_test_init  (void);
void    head_test_tick  (void);
uint8   head_test_active(void);
float   head_test_tar_yaw (void);       /* 本拍目标航向 deg（遥测 CH36） */
float   head_test_seg     (void);       /* 段号（遥测 CH37） */
float   head_test_err     (void);       /* 本拍航向误差 deg */

#endif
