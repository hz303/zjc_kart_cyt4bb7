/*********************************************************************************************************************
 * 路表回放外环：纯追踪 / LQR 转向 + 速度剖面 + 最近点/前视点 + 起停判据
 * 状态 PB_ST_IDLE → PB_ST_RUN → PB_ST_DONE / PB_ST_ABORT
 * 互斥 录制与回放互斥，回放侧的另一半检查在 pb_start() 内
 ********************************************************************************************************************/

#ifndef _ROUTE_PB_H
#define _ROUTE_PB_H

#include "zf_common_headfile.h"

/* ---- 参数（ID 43~55 + 64~68）---- */
extern volatile float pb_en;        /* 43 回放使能 0/1 */
extern volatile float pb_mode;      /* 44 1 = 只给转向；2 = 转向+速度 */
extern volatile float pb_vmax;      /* 45 巡航最高车速 m/s */
extern volatile float pb_alat;      /* 46 允许的横向加速度 m/s^2 */
extern volatile float pb_abrk;      /* 47 刹车减速度 m/s^2（反向传一遍） */
extern volatile float pb_l;         /* 48 有效轴距 m */
extern volatile float pb_ld_a;      /* 49 前视基准 a, m；ld = a + b·|v| */
extern volatile float pb_ld_b;      /* 50 前视速度系数 b, s */
extern volatile float pb_ld_min;    /* 51 前视下限 m */
extern volatile float pb_ld_max;    /* 52 前视上限 m */
extern volatile float pb_ff_en;     /* 54 打角前馈使能 0/1 */
extern volatile float pb_ff_gain;   /* 55 前馈融合权重 w, 0~1 */
extern volatile float pb_vmin;      /* 64 最低速度 m/s（防爬行） */
extern volatile float pb_kwin;      /* 65 曲率中心差分半窗（点） */
extern volatile float pb_smode;     /* 66 0 = 航向外环 / 1 = 直接给 δ */
extern volatile float pb_kh;        /* 67 航向环增益；0 = 自动 2L/ld */
extern volatile float pb_fin;       /* 68 终点判定半径 m */

/* 有效轴距默认值 m，route_pb.c 初值与参数表 ID48 默认值共用 */
#define PB_L_DEFAULT        (0.66f)

/* 曲率中心差分半窗（点）：6 × 0.05 m ⇒ 差分跨度 0.6 m */
#define PB_KWIN_DEFAULT     (6.0f)

#define PB_MIN_POINTS       (10u)       /* 起跑所需最少表点数 */
#define PB_SHIFT_STOP_MPS   (0.03f)     /* 换向判定停止速度 m/s */
#define PB_SHIFT_HOLD_MS    (250u)      /* 换向静止驻留 ms */
#define PB_SHIFT_TIMEOUT_MS (5000u)     /* 换向超时 ms */
#define PB_SHIFT_CREEP      (0.08f)     /* 换向前后爬行速度 m/s */
#define PB_RECORD_DIR_EPS   (0.02f)     /* 录制速度判方向的死区 m/s */
#define PB_END_CREEP        (0.50f)     /* 末点爬行速度 m/s，高于电机死区 */
#define PB_STOP_MS          (350u)      /* 完成判据命中后停车斜坡时长 ms */
#define PB_NEAR_WIN         (40u)       /* 最近点前向搜索窗（点）= 2 m */
#define PB_ALIGN_MAX_M      (2.0f)      /* 起点对齐阈值 m */
#define PB_ALIGN_YAW_MAX    (60.0f)     /* 起跑朝向差告警阈值 deg */
#define PB_ALIGN_YAW_M      (1.0f)      /* 朝向判据可信所需的位置残差 m */

/* ---- 控制律（ID78 pb_ctrl，菜单 2.CtrlMode）----
   0 = 无 PP：最近点录制打角 + 航向 P 纠偏，无前视点
   1 = PP  ：κ = 2·sin(α)/ld，δ = atan(L·κ)
   2 = LQR ：u = kref − Ke·e − Kth·e_psi，δ = atan(L·u)；倒车取反
   打角统一按 steer_ctrl_eff_limit() 限幅 */
#define PB_CTRL_NOPP        (0u)
#define PB_CTRL_PP          (1u)
#define PB_CTRL_LQR         (2u)
#define PB_CTRL_MAX         (2u)
#define PB_CTRL_DEFAULT            (PB_CTRL_PP)

/* ID87 pb_cauto = 1 时按场景自动选控制律 */
#define PB_AUTO_LEN_M      (12.0f)   /* 倒车回放：表短于此值才允许 mode0 */
#define PB_AUTO_TIGHT      (0.25f)   /* 需求舵角超限段占比上限 */
#define PB_AUTO_NEED_DEG   (24.0f)   /* 余量判据（车极限 27deg）*/
#define PB_PRE_MIN_DEG     (2.0f)    /* °，起步需求超过它才预置 */
#define PB_PRE_TOL_DEG     (1.5f)    /* °，实测与目标差在它内就放行 */
#define PB_PRE_MS          (1500u)   /* ms，预置超时强制放行 */
#define PB_OFF_DIST        (1.00f)   /* m，超过即视为脱表 */
#define PB_OFF_TICKS       (60u)     /* 连续多少拍（20 ms/拍 ≈1.2 s）判丢 */
#define PB_DONE_DTRUE      (0.60f)   /* m，判完成时额外要求离折线这么近 */

/* ---- 无 PP 通道（ID78 = 0）的旋钮 ---- */
#define PB_NOPP_HEAD_KP     (1.50f)  /* 航向 P 纠偏 deg/deg */
/* 纠偏量钳位 deg，超过不再加权，防止一次大误差把舵角顶到限位 */
#define PB_NOPP_REVCORR        (13.5f)
/* 纠偏符号：+1 = 切向在车头左侧则左打；越纠越斜就取反 */
#define PB_NOPP_REV_SIGN       (-1.0f)
#define PB_NOPP_REVSCL      (1.00f)  /* 倒车速度倍率，1.0 = 照表 */

#define PB_REV_EPS          (0.10f)     /* 倒车判定死区 m/s */
/* 转向速率上限 deg/s，曲率变化率限速用（执行器满占空比约 100 °/s） */
#define PB_SLEW_RATE_DEG_S   (70.0f)

/* 跟丢判据：满舵 + elat 不小 + 最近点索引冻结（打转的特征） */
#define PB_LOST_MS          (1200u)     /* 满舵且索引冻结多久判跟丢 ms */
#define PB_LOST_MIN_M       (0.60f)     /* elat 小于这个不判（起步贴线时本来就小）*/
#define PB_REV_CREEP        (0.15f)     /* 换向前那一点的爬行速度 m/s */
#define PB_ESTOP_TICKS      (4u)        /* 急停去抖拍数（5 ms/拍） */
#define PB_REL_TICKS        (6u)        /* 收车回中最小驻留（拍，5 ms/拍） */
#define PB_REV_HEAD_KP      (0.0f)      /* 倒车航向 P 纠偏 deg/deg，0 = 纯开环 */

/* ---- 状态（遥测 pb_state）---- */
#define PB_ST_IDLE          (0u)
#define PB_ST_RUN           (1u)
#define PB_ST_DONE          (2u)
#define PB_ST_ABORT         (3u)

void    route_pb_init   (void);
void    route_pb_poll   (void);         /* 主循环每圈一次 */
float   route_pb_get_tanref (void);     /* 本拍参考切向 deg */
float   route_pb_get_alpha (void);      /* 上一拍前视偏角 rad */
float   route_pb_get_offdist (void);    /* 到表折线的真实距离 m */
float route_pb_get_errw (void);         /* 航向误差 deg */
float route_pb_get_kref (void);         /* 本拍前视曲率参考 1/m */
float route_pb_get_why_code (void);     /* 收车原因码 */
float   route_pb_get_kappa (void);      /* LQR 上一拍曲率指令 1/m（正 = 右转）*/
/* 5 ms 中断内调用，按用户意图重算自由轮；须在 spd_ctrl_tick() 之前 */
void    route_pb_sync_coast (void);

/* 起跑失败后允许重试一次（菜单写 Follow 时调） */
void    route_pb_rearm  (void);

/* 最近一次没跑 / 结束的原因（屏幕与串口） */
const char * route_pb_reason (void);

void    route_pb_stop   (const char *why);

float   route_pb_elat   (void);         /* 横向误差 m（正 = 车在参考线右侧）*/
float   route_pb_get_best(void);        /* 遥测：进度点索引 */
float   route_pb_get_fi  (void);        /* 遥测：前视点索引 */
uint8   route_pb_active (void);         /* 1 = 正在回放 */
uint8   route_pb_state  (void);         /* PB_ST_* */
uint8   route_pb_is_running (void);     /* 1 = PB_ST_RUN（目标转角归路线层） */
uint8   route_pb_owns_target (void);    /* 1 = 本拍 target_delta 由 route_pb 写 */
float   route_pb_ld     (void);         /* 本拍前视距离 m */
float   route_pb_tv     (void);         /* 本拍目标车速 m/s */
float   route_pb_yaw_tgt(void);         /* 本拍目标航向 deg（航向通道才有意义） */
uint16  route_pb_aim    (void);         /* 本拍前视点索引 */
uint16  route_pb_index  (void);         /* 本拍最近点索引 */

#endif
