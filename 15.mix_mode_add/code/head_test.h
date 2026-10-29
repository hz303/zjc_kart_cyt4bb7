/*********************************************************************************************************************
 * 航向测试：把舵机转角来源切到航向外环, 用固定工况量外环性能
 * 模式 1 固定航向：后轮恒定 ht_rps, 目标航向 = 起步航向 + ht_yaw；量横向漂移 / 航向峰峰 / 舵机输出峰峰
 * 模式 2 变角度台阶：段号 k=1..n, 目标依次 0, +step, -2*step, +3*step...；每段量上升时间(进 ±1 度) / 段内里程 / 段末残差
 * 段序列与计时在车端 5 ms 节拍里跑, 手机只下发参数、使能、记 CSV 并每 200 ms 重发保活
 * 安全门：链路离线 / 遥控接管 / 失效 / 急停 / 驱动权不在 SPI 时收车并清 ht_mode；pb_en=1 时不启动
 * 收车顺序：先关航向外环并把转角目标对齐实测角, 再清速度目标
 ********************************************************************************************************************/

#ifndef _HEAD_TEST_H
#define _HEAD_TEST_H

#include "zf_common_headfile.h"

extern volatile float ht_mode;      /* 69 0=关 1=固定航向 2=变角度台阶 */
extern volatile float ht_rps;       /* 70 后轮恒定目标 rps */
extern volatile float ht_yaw;       /* 71 模式 1 目标航向, 相对起步航向 deg */
extern volatile float ht_dwell;     /* 72 每段停留时间 s */
extern volatile float ht_step;      /* 73 台阶幅值 deg, 5 即 0/+5/-10/+15 */
extern volatile float ht_n;         /* 74 段数 */
extern volatile float ht_k;         /* 75 航向环增益 deg/deg */

#define HT_MODE_OFF         (0)
#define HT_MODE_FIXED       (1)
#define HT_MODE_STEP        (2)

#define HT_SEG_OFF          (0.0f)      /* 遥测 ht_seg, 没在跑 */
#define HT_SEG_DONE         (254.0f)    /* 遥测 ht_seg, 序列跑完 */

void    head_test_init  (void);
void    head_test_tick  (void);         /* 放 5 ms 中断 */
uint8   head_test_active(void);
float   head_test_tar_yaw (void);       /* 本拍目标航向 deg, 遥测 CH36 */
float   head_test_seg     (void);       /* 段号, 遥测 CH37 */
float   head_test_err     (void);

#endif
