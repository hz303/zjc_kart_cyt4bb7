/*********************************************************************************************************************
 * 路表回放（Pure Pursuit 外环）对外接口：参数、常量、状态码、读出接口
 *
 * 参数行尾的 43~55 / 64~68 为参数表 ID
 *********************************************************************************************************************/

#ifndef _ROUTE_PB_H
#define _ROUTE_PB_H

#include "zf_common_headfile.h"

/* ---- 参数（ID 43~55 + 64~68）---- */
extern volatile float pb_en;        /* 43 回放使能 0/1 */
extern volatile float pb_mode;      /* 44 1=只给转向 2=转向+速度 */
extern volatile float pb_vmax;      /* 45 巡航最高车速 m/s，相对表内最大车速只降不升 */
extern volatile float pb_alat;      /* 46 允许横向加速度 m/s^2 */
extern volatile float pb_abrk;      /* 47 刹车减速度 m/s^2，反向传一遍 */
extern volatile float pb_l;         /* 有效轴距 m，固定 PB_L_DEFAULT */
extern volatile float pb_ld_a;      /* 49 前视 a + b*|v| */
extern volatile float pb_ld_b;      /* 50 前视速度系数 */
extern volatile float pb_ld_min;    /* 51 前视下限 m */
extern volatile float pb_ld_max;    /* 52 前视上限 m */
extern volatile float pb_ff_en;     /* 54 打角前馈使能 */
extern volatile float pb_ff_gain;   /* 55 前馈融合权重 w */
extern volatile float pb_vmin;      /* 64 最低速度 m/s，防爬行 */
extern volatile float pb_kwin;      /* 65 曲率中心差分半窗 点 */
extern volatile float pb_smode;     /* 66 0=航向外环 1=直接给 delta */
extern volatile float pb_kh;        /* 67 航向环增益；0=自动 2L/ld */
extern volatile float pb_fin;       /* 68 终点判定半径 m */

/* 有效轴距默认值 m；route_pb.c 的初值与参数表 ID48 的默认值都引用它 */
#define PB_L_DEFAULT        (0.66f)

/* 曲率差分半窗默认值：6 点 x 0.05 m 点距 = 0.6 m 跨度 */
#define PB_KWIN_DEFAULT     (6.0f)

#define PB_MIN_POINTS       (10u)
#define PB_SHIFT_STOP_MPS   (0.03f)
#define PB_SHIFT_HOLD_MS    (250u)
#define PB_SHIFT_TIMEOUT_MS (5000u)
#define PB_SHIFT_CREEP      (0.08f)
#define PB_RECORD_DIR_EPS   (0.02f)
/* 末段减速：末点锚到爬行速度，减速曲线由反向刹车生成，减速度不超过 pb_abrk */
#define PB_END_CREEP        (0.50f)         /* 末点爬行速度 m/s，0.61 rps 高于电机死区 */

/* 终点停车斜坡时长 ms：完成判据命中后目标由当前值线性降到 0；须按时间不按拍数 */
#define PB_STOP_MS          (350u)
#define PB_NEAR_WIN         (40u)           /* 最近点前向搜索窗 点 = 2 m */
/* 起点对齐阈值 m：按 origin 换算的位置离表起点超过它，就改用车当前位置作基准 */
#define PB_ALIGN_MAX_M      (2.0f)
/* 起跑朝向告警阈值 deg：残差 dyaw = o.yaw - origin_yaw - 表首朝向，超限只告警 */
#define PB_ALIGN_YAW_MAX    (60.0f)
/* 朝向残差可信范围 m：位置残差小于它才认为位姿基准仍有效 */
#define PB_ALIGN_YAW_M      (1.0f)
/* ---- 回放控制律（ID78 pb_ctrl，菜单 2.CtrlMode）----
 * 0 = 无 PP：里程查表 + 录制打角回放 + 航向 P，无前视点
 * 1 = PP
 * 2 = LQR：u = kref - Ke*e - Kth*e_psi，delta = atan(L*u) */
#define PB_CTRL_NOPP        (0u)
#define PB_CTRL_PP          (1u)
#define PB_CTRL_LQR         (2u)
#define PB_CTRL_MAX         (2u)
#define PB_CTRL_DEFAULT            (PB_CTRL_PP)

/* ---- ID87 pb_cauto=1 时按场景自动选控制律 ---- */
#define PB_AUTO_LEN_M      (12.0f)   /* 倒车回放：表短于此值才允许用 mode0 */
#define PB_AUTO_TIGHT      (0.25f)   /* 需求舵角超限段占比上限 */
#define PB_AUTO_NEED_DEG   (24.0f)   /* 余量判据 deg，车极限 27 */
/* 起步预置舵角：冷启动舵机在 0 度，先把舵转到位再发车 */
#define PB_PRE_MIN_DEG     (2.0f)    /* °，起步需求超过它才预置 */
#define PB_PRE_TOL_DEG     (1.5f)    /* °，实测与目标差在它内就放行 */
#define PB_PRE_MS          (1500u)   /* ms，预置超时强制放行 */
/* 脱表判据用 s_dtrue（到表折线真实最短距离）；elat 是一维投影，沿切线跑偏时≈0 */
#define PB_OFF_DIST        (1.00f)   /* m，超过即视为脱表 */
#define PB_OFF_TICKS       (60u)     /* 连续判丢拍数，20 ms/拍约 1.2 s */
#define PB_DONE_DTRUE      (0.60f)   /* m，判完成时额外要求离折线这么近 */

/* ---- 无 PP 通道（ID78=0）三个旋钮 ----
 * 最近点 -> 录制打角 -> 航向 P 纠偏，无前视点；只有倒车段进入该通道
 * PB_NOPP_HEAD_KP 航向 P 增益 deg/deg；PB_NOPP_REVSCL 倒车速度倍率（1.0 = 照表速度）
 * PB_NOPP_REVCORR 纠偏量钳位 deg；PB_NOPP_REV_SIGN 纠偏符号 +1/-1 */
#define PB_NOPP_HEAD_KP     (1.50f)
/* 航向纠偏量钳位 deg，超过不再加权，防止一次大误差把舵角顶到限位 */
#define PB_NOPP_REVCORR        (13.5f)
/* 纠偏符号 +1/-1：切向在车头左侧时应左打、delta 减小；反了就是正反馈 */
#define PB_NOPP_REV_SIGN       (-1.0f)
#define PB_NOPP_REVSCL      (1.00f)

#define PB_REV_EPS          (0.10f)     /* 倒车判定死区 m/s */
/* 转向速率上限 deg/s；执行器满占空比约 100 deg/s，取 0.7 倍留控制余量 */
#define PB_SLEW_RATE_DEG_S   (70.0f)

/* 跟丢判据：实测舵角贴限位（0.95*limit）+ elat 偏大 + 最近点索引 PB_LOST_MS 内不前进 */
#define PB_LOST_MS          (1200u)     /* 满舵且索引冻结多久判跟丢 ms */
#define PB_LOST_MIN_M       (0.60f)     /* elat 小于此值不判，起步贴线时本来就小 */
#define PB_REV_CREEP        (0.15f)     /* 换向前一点的爬行速度 m/s */
/* 急停去抖拍数（5 ms/拍）；硬停机由 spd_safety 当拍执行 */
#define PB_ESTOP_TICKS      (4u)
/* 收车回中最小驻留拍数（5 ms/拍），只持续一拍的判据不写 0 */
#define PB_REL_TICKS        (6u)
#define PB_REV_HEAD_KP      (0.0f)      /* 倒车航向 P 纠偏 deg/deg；0 = 纯开环，开启前需确认符号 */

/* ---- 状态（遥测 pb_state）----
   0=IDLE 未跑 1=RUN 回放中 2=DONE 到达终点 3=ABORT 中止 */
#define PB_ST_IDLE          (0u)
#define PB_ST_RUN           (1u)
#define PB_ST_DONE          (2u)
#define PB_ST_ABORT         (3u)

void    route_pb_init   (void);
void    route_pb_poll   (void);         /* 主循环每圈一次 */
float   route_pb_get_tanref (void);     /* 本拍参考切向 deg，几何自检 */
float   route_pb_get_alpha (void);
float   route_pb_get_offdist (void);   /* 到表折线的真实距离 m */
float route_pb_get_errw (void);          /* 世界系航向误差 deg */
float route_pb_get_kref (void);      /* 上一拍前视偏角 rad，供 LQR 与遥测 */
float route_pb_get_why_code (void);      /* 收车原因码，遥测用 */
float   route_pb_get_kappa (void);      /* 上一拍曲率指令 1/m，正=右转 */
/* 每 5 ms 由中断调用：只按用户意图与驾驶来源决定自由轮
   调用点在 spd_ctrl_tick() 之前 */
void    route_pb_sync_coast (void);

/* 起跑失败后清 s_arm，允许重试；菜单写 Follow 时调用 */
void    route_pb_rearm  (void);

/* 最近一次停跑原因，屏幕与串口显示 */
const char * route_pb_reason (void);

void    route_pb_stop   (const char *why);

float   route_pb_elat   (void);          /* 横向误差 m，正=车在参考线右侧 */
float   route_pb_get_best(void);        /* 进度索引：正向前进用几何，倒车用里程 */
float   route_pb_get_fi  (void);        /* 前视点索引 */
uint8   route_pb_active (void);         /* 1 = 正在回放 */
uint8   route_pb_state  (void);         /* PB_ST_IDLE / RUN / DONE / ABORT */
uint8   route_pb_is_running (void);     /* 1 = PB_ST_RUN，目标转角归路线层 */
uint8   route_pb_owns_target (void);    /* 1 = 本拍 target_delta 由 route_pb 写，航向环让位 */
float   route_pb_ld     (void);         /* 本拍前视距离 m */
float   route_pb_tv     (void);         /* 本拍目标车速 m/s */
float   route_pb_yaw_tgt(void);         /* 本拍目标航向 deg，仅航向通道有效 */
uint16  route_pb_aim    (void);         /* 本拍前视点索引 */
uint16  route_pb_index  (void);         /* 本拍最近点索引 */

#endif
