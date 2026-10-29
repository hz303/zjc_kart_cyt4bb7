/*********************************************************************************************************************
 * 航向台架测试: 用固定工况量转向航向外环, 段序列与计时都在车端 5 ms 节拍里跑
 *
 * 测试① 固定航向: 后轮恒定 ht_rps, 目标航向 = 起步航向 + ht_yaw, 看直线能力
 * 测试② 变角度台阶: 段号 k=1..n, 目标航向依次 0, +step, −2*step, +3*step …,
 *   每段量上升时间(进 ±1° 用时)、段内里程与段末残差
 *
 * 安全门: 链路不在线/遥控接管/急停任一成立即收车并清 ht_mode, pb_en=1 不启动;
 *   收车先关航向外环并对齐转角, 再清速度
 ********************************************************************************************************************/

#ifndef _HEAD_TEST_H
#define _HEAD_TEST_H

#include "zf_common_headfile.h"

extern volatile float ht_mode;      /* 69 0=关 1=固定航向 2=变角度台阶          */
extern volatile float ht_rps;       /* 70 后轮恒定目标 rps                       */
extern volatile float ht_yaw;       /* 71 测试① 目标航向, 相对起步航向, deg      */
extern volatile float ht_dwell;     /* 72 测试② 每段停留时间 s                   */
extern volatile float ht_step;      /* 73 测试② 台阶幅值 deg, 5 即 0/+5/−10/+15  */
extern volatile float ht_n;         /* 74 测试② 段数                             */
extern volatile float ht_k;         /* 75 航向环增益 deg/deg, 测试专用            */

#define HT_MODE_OFF         (0)
#define HT_MODE_FIXED       (1)
#define HT_MODE_STEP        (2)

#define HT_SEG_OFF          (0.0f)      /* 遥测 ht_seg: 没在跑      */
#define HT_SEG_DONE         (254.0f)    /* 遥测 ht_seg: 序列跑完了  */

void    head_test_init  (void);
void    head_test_tick  (void);         /* 放 5 ms 中断 */
uint8   head_test_active(void);
float   head_test_tar_yaw (void);
float   head_test_seg     (void);
float   head_test_err     (void);

#endif
