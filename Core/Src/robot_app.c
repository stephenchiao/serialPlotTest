#include "robot_app.h"
#include "motion_math.h"
#include "main.h"
#include "can.h"
#include "usart.h"

#include "zdtCan.h"
#include "zdtEmm.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "mecanum_chassis.h"
#include "ops9.h"
#include "pid.h"
#include "llm_tuner.h"
#include "dm_g6220.h"
#include "rpi_protocol.h"
#include "host_rx_router.h"
#include "host_uart_tx.h"
#include "control_runtime.h"

#define POSE_POSITION_TOL_MM    2.0f
#define POSE_YAW_TOL_DEG        0.5f
#define POSE_SETTLE_CYCLES      25U
#define POSE_CONTROL_PERIOD_MS  20U
#define POSE_DT_MAX_MS          100U
#define POSE_TUNE_TIMEOUT_MS    15000U
#define POSE_WORK_TIMEOUT_MS    35000U
#define POSE_STOP_LINEAR_EPS_MPS 0.005f
#define POSE_STOP_YAW_EPS_RADPS  0.010f
/* POSE 平移规划参数：速度 m/s，加/减速度 m/s^2。 */
#define POSE_SPEED_DEFAULT_MPS       0.15f
#define POSE_SPEED_MIN_MPS           0.02f
#define POSE_SPEED_HARD_MAX_MPS      0.30f
#define POSE_ACCEL_DEFAULT_MPS2      0.20f
#define POSE_ACCEL_MIN_MPS2          0.05f
#define POSE_ACCEL_HARD_MAX_MPS2     0.80f
#define POSE_DECEL_DEFAULT_MPS2      0.40f
#define POSE_DECEL_MIN_MPS2          0.05f
#define POSE_DECEL_HARD_MAX_MPS2     1.20f
/* POSE 航向规划参数：速度 rad/s，加/减速度 rad/s^2。 */
#define POSE_YAW_SPEED_DEFAULT_RADPS     0.30f
#define POSE_YAW_SPEED_MIN_RADPS         0.02f
#define POSE_YAW_SPEED_HARD_MAX_RADPS    0.80f
#define POSE_YAW_ACCEL_DEFAULT_RADPS2    0.50f
#define POSE_YAW_ACCEL_MIN_RADPS2        0.10f
#define POSE_YAW_ACCEL_HARD_MAX_RADPS2   2.00f
#define POSE_YAW_DECEL_DEFAULT_RADPS2    0.80f
#define POSE_YAW_DECEL_MIN_RADPS2        0.10f
#define POSE_YAW_DECEL_HARD_MAX_RADPS2   3.00f
/* 单个 POSE 航点相对当前车体中心的硬行程边界，防止错误坐标导致长距离失控。 */
#define POSE_TARGET_DISTANCE_HARD_MAX_MM 10000.0f
#define POSE_RUNTIME_ERROR_HARD_MAX_MM   10500.0f
#define DEBUG_MOTOR_MAX_RPM         300.0f
#define DEBUG_MOTOR_DEFAULT_MS      2000UL
#define DEBUG_MOTOR_MAX_MS          10000UL
#define DEBUG_MOVE_DEFAULT_MPS      0.04f
#define DEBUG_MOVE_MAX_MPS          0.08f
#define DEBUG_MOVE_DEFAULT_MS       1000UL
#define DEBUG_MOVE_MAX_MS           3000UL
#define DEBUG_TURN_DEFAULT_RADPS     0.15f
#define DEBUG_TURN_MAX_RADPS         0.30f
#define TELEMETRY_PERIOD_MS          50U
#define TELEMETRY_STAGGER_MS         25U
#define TELEMETRY_MASK_WHEEL         0x01U
#define TELEMETRY_MASK_POSE          0x02U
#define TELEMETRY_MASK_BOTH          (TELEMETRY_MASK_WHEEL | TELEMETRY_MASK_POSE)
#define HOST_WAIT_TIMEOUT_MS         60000U
#define G6220_CAN_ID                 0x01U
#define G6220_MASTER_ID              0x00U
#define G6220_CAN2_FILTER_BANK       14U
#define G6220_SLAVE_FILTER_START     14U
#define G6220_STARTUP_DELAY_MS       1000U
#define G6220_COMMAND_DELAY_MS       50U
#define RPI_BINARY_GOAL_TIMEOUT_MAX_MS 60000U
#define RPI_BINARY_RESPONSE_PREFIX_SIZE 3U

/* PID state belongs to this application instance. */
static PID_Controller pid_x;
static PID_Controller pid_y;
static PID_Controller pid_yaw;
// === 主机串口命令接收状态 ===
static uint8_t pc_rx_byte;                       // 主机串口单字节接收
static HostRxRouter host_rx;
static volatile uint8_t rpi_binary_active = 0U;
static volatile uint8_t rpi_binary_armed = 0U;
static RpiTxQueue rpi_binary_tx_queue;
static RpiEncodedFrame rpi_binary_tx_loaded;
static volatile uint8_t rpi_binary_tx_dma_active = 0U;
static uint32_t rpi_binary_tx_started;
static volatile uint8_t rpi_binary_tx_loaded_ready = 0U;
static volatile uint8_t host_uart_fault_pending = 0U;
static uint8_t rpi_binary_tx_sequence = 0U;
static uint32_t rpi_binary_goal_id = 0U;
static uint32_t rpi_binary_goal_timeout_ms = POSE_WORK_TIMEOUT_MS;
static RpiPoseState_t rpi_binary_pose_state = RPI_POSE_IDLE;
static uint16_t rpi_binary_fault_reason = RPI_FAULT_UNSPECIFIED;

typedef enum {
    ROBOT_MODE_WORK = 0,
    ROBOT_MODE_TUNE
} RobotMode_t;

typedef enum {
    HOST_LINK_NONE = 0,
    HOST_LINK_COM,
    HOST_LINK_RPI
} HostLink_t;

typedef enum {
    POSE_PHASE_IDLE = 0,
    POSE_PHASE_TRANSLATE,
    POSE_PHASE_ROTATE
} PoseControlPhase_t;

typedef enum {
    CHASSIS_MOTION_NONE = 0,
    CHASSIS_MOTION_POSE,
    CHASSIS_MOTION_TUNE_ROUND,
    CHASSIS_MOTION_DEBUG_CHASSIS,
    CHASSIS_MOTION_SINGLE_MOTOR
} ChassisMotionType_t;

typedef enum {
    POSE_START_OK = 0,
    POSE_START_INVALID,
    POSE_START_OPS_NOT_READY,
    POSE_START_OUT_OF_BOUNDS,
    POSE_START_BUSY
} PoseStartResult_t;

typedef struct {
    float linear_accel_mps2;
    float linear_decel_mps2;
    float yaw_accel_radps2;
    float yaw_decel_radps2;
} PoseMotionProfile_t;

static RobotMode_t current_robot_mode = ROBOT_MODE_WORK;
static uint8_t debug_motor_active = 0U;
static uint8_t debug_motor_id = 0U;
static uint32_t debug_motor_stop_tick = 0U;
static uint8_t debug_chassis_active = 0U;
static uint32_t debug_chassis_stop_tick = 0U;
static uint8_t ops_monitor_enabled = 0U;
static uint32_t ops_monitor_last_tick = 0U;
static volatile uint32_t last_host_command_tick = 0U;
static volatile HostLink_t active_host_link = HOST_LINK_NONE;
static uint32_t host_wait_start_tick = 0U;
static uint8_t host_wait_timeout_reported = 0U;
static uint8_t chassis_motors_enabled = 0U;
static uint8_t pose_control_active = 0U;
static PoseControlPhase_t pose_control_phase = POSE_PHASE_IDLE;
static float pose_target_x_mm = 0.0f;
static float pose_target_y_mm = 0.0f;
static float pose_target_yaw_deg = 0.0f;
static float pose_translation_yaw_deg = 0.0f;
static float pose_target_center_x_mm = 0.0f;
static float pose_target_center_y_mm = 0.0f;
static float pose_output_vx = 0.0f;
static float pose_output_vy = 0.0f;
static float pose_output_vz = 0.0f;
static float pose_target_vx = 0.0f;
static float pose_target_vy = 0.0f;
static float pose_target_vz = 0.0f;
static uint16_t pose_settle_cycles = 0U;
static uint32_t pose_last_control_time = 0U;
static uint32_t pose_start_time = 0U;
static PoseMotionProfile_t pose_motion_profile;
static uint8_t telemetry_mask = 0U;
static uint8_t telemetry_next_group = TELEMETRY_MASK_WHEEL;
static uint16_t telemetry_wheel_sequence = 0U;
static uint16_t telemetry_pose_sequence = 0U;
static uint32_t telemetry_last_tick = 0U;
static uint32_t binary_stats_last_tick = 0U;
static uint32_t telemetry_tx_ok = 0U;
static uint32_t telemetry_tx_error = 0U;
static volatile uint32_t host_uart_tx_ok = 0U;
static volatile uint32_t host_uart_tx_error = 0U;
static DM_G6220_Motor_t g6220_motor;
static uint8_t g6220_initialized = 0U;
static uint8_t g6220_enable_requested = 0U;
static DM_G6220_Result_t g6220_last_result = DM_G6220_ERROR_PARAM;

static void Host_ProcessCommand(void);
static uint8_t Host_ProcessOperationalCommand(const char *command);
static void Motor_ProcessFeedback(void);
static void Host_PrintHelp(void);
static void Ops_PrintStatus(void);
static void Pose_ProcessControl(uint32_t now);
static void Telemetry_Process(uint32_t now);
static void ChassisSafety_Process(uint32_t now);
static const char *RobotMode_Name(RobotMode_t mode);
static DM_G6220_Result_t G6220_SetEnabled(uint8_t enable);
static void Pose_InitMotionProfile(void);
static void Pose_ResetPlanner(void);
static void Robot_StopAllMotion(void);
static void RpiBinary_Process(void);
static void RpiBinary_TxProcess(void);
static void RpiBinary_ProcessSessionTimeout(uint32_t now);
static void RpiBinary_SendFault(uint16_t reason);
static uint8_t RpiBinary_DequeueFrame(RpiFrame *frame);
static void RpiBinary_ResetRxQueue(void);
static void RpiBinary_ResetTxQueue(void);
static void HostLink_ProcessUartFault(uint32_t now);
static void Robot_SetMode(RobotMode_t mode);
static const char *HostLink_Name(HostLink_t link);
static uint8_t HostLink_ProcessCommand(const char *command);
static void HostLink_ProcessWait(uint32_t now);
static void HostLink_SetChassisEnabled(uint8_t enable);
static ChassisMotionType_t ChassisSafety_GetActiveMotion(void);
static uint8_t ChassisSafety_CanReady(void);
static uint8_t ChassisSafety_OpsReady(uint32_t now);

static void App_ResetHostParser(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    HostRx_ResetParser(&host_rx);
    __set_PRIMASK(primask);
}

static uint8_t Motor_IsValidId(unsigned int id)
{
    return (id >= 1U && id <= 4U) ? 1U : 0U;
}

static const char *RobotMode_Name(RobotMode_t mode)
{
    if (mode == ROBOT_MODE_TUNE) return "TUNE";
    return "WORK";
}

static const char *HostLink_Name(HostLink_t link)
{
    if (link == HOST_LINK_COM) return "COM";
    if (link == HOST_LINK_RPI) return "RPI";
    return "NONE";
}

static DM_G6220_Result_t G6220_SetEnabled(uint8_t enable)
{
    if (!g6220_initialized) {
        g6220_last_result = DM_G6220_ERROR_PARAM;
        return g6220_last_result;
    }

    g6220_last_result = DM_G6220_SendCommand(
        &g6220_motor,
        enable ? DM_G6220_CMD_ENABLE : DM_G6220_CMD_DISABLE);
    if (g6220_last_result == DM_G6220_OK) {
        g6220_enable_requested = enable ? 1U : 0U;
    }
    return g6220_last_result;
}

static void Robot_StopAllMotion(void)
{
    (void)StopAllMotors();
    debug_motor_active = 0U;
    debug_chassis_active = 0U;
    pose_control_active = 0U;
    Pose_ResetPlanner();
    LLM_TunerAbort();
    (void)G6220_SetEnabled(0U);
    /* PID_Reset 只清运行历史，不修改已经调好的 Kp/Ki/Kd 和输出限幅。 */
    PID_Reset(&pid_x);
    PID_Reset(&pid_y);
    PID_Reset(&pid_yaw);
}

static void HostLink_SetChassisEnabled(uint8_t enable)
{
    uint8_t id;
    uint8_t all_ok = 1U;
    uint8_t result;

    /* 四轮闭环驱动器按顺序切换使能状态；使能且零速时提供静止保持力矩。 */
    for (id = 1U; id <= 4U; id++) {
        result = ZDT_Emm_EnableByID(id, enable);
        Mecanum_ReportCanTxResult(result);
        if (result != 0U) all_ok = 0U;

    }
    chassis_motors_enabled = (enable && all_ok) ? 1U : 0U;
}

static uint8_t HostLink_ProcessCommand(const char *command)
{
    HostLink_t requested = HOST_LINK_NONE;

    if (strcmp(command, "CONTROL STATUS") == 0) {
        ControlRuntimeStats runtime = ControlRuntime_GetStats();
        HostUartTxStats uart = HostUartTx_GetStats();
        ZDT_CAN_Stats_t can_stats;
        ZDT_CAN_GetStats(&can_stats);
        printf("# CONTROL LOOP_MAX_MS=%lu CONTROL_MAX_MS=%lu LATE=%lu OVERRUN=%lu FAULT=%u IWDG_RESET=%u\r\n",
               (unsigned long)runtime.loop_max_ms, (unsigned long)runtime.control_max_ms,
               (unsigned long)runtime.control_late_count, (unsigned long)runtime.loop_overruns,
               runtime.fault, runtime.watchdog_reset);
        printf("# IO UART_DONE=%lu UART_DROP=%lu UART_REPLACE=%lu UART_BUSY_MAX_MS=%lu CAN_ENQUEUE=%lu CAN_DROP=%lu CAN_WAIT_MAX_MS=%lu\r\n",
               (unsigned long)uart.completed, (unsigned long)uart.dropped, (unsigned long)uart.replaced,
               (unsigned long)uart.max_busy_ms,
               (unsigned long)can_stats.enqueued, (unsigned long)can_stats.dropped, (unsigned long)can_stats.max_wait_ms);
        return 1U;
    }
    if (strcmp(command, "MOTOR STOP STATUS") == 0) {
        MotorStopMonitor stop = Mecanum_GetStopStatus();
        uint8_t mask = Mecanum_GetRequiredMotorMask();
        printf("# MOTOR STOP STATE=%s MASK=0x%02X SENT_MASK=0x%02X FRESH=%u EVIDENCE=CAN_TX_ONLY ELAPSED_MS=%lu\r\n",
               MotorStop_Name(stop.state), mask, ZDT_CAN_StopSentMask(), Mecanum_FeedbackReady(mask),
               (unsigned long)(stop.state == MOTOR_STOP_IDLE ? 0U :
                               (stop.state == MOTOR_STOP_SENT ?
                                stop.confirmed_tick - stop.requested_tick : HAL_GetTick() - stop.requested_tick)));
        return 1U;
    }
    if (strcmp(command, "MOTOR FEEDBACK") == 0) {
        MotorFeedback samples[4];
        uint8_t i;
        uint32_t now;
        ZDT_Emm_GetFeedback(samples); now = HAL_GetTick();
        for (i = 0U; i < 4U; ++i)
            printf("# MOTOR FEEDBACK ID=%u VALID=%u RPM=%.1f AGE_MS=%lu SEQ=%lu\r\n",
                   i + 1U, samples[i].valid, samples[i].rpm,
                   (unsigned long)(now - samples[i].tick), (unsigned long)samples[i].sequence);
        return 1U;
    }
    if (strcmp(command, "HOST RX STATUS") == 0) {
        printf("# HOST RX DROPPED=%lu INVALID=%lu\r\n",
               (unsigned long)HostRx_GetStats(&host_rx).text_dropped,
               (unsigned long)HostRx_GetStats(&host_rx).text_invalid);
        return 1U;
    }

    if (strcmp(command, "HOST BINARY START") == 0) {
        uint32_t now = HAL_GetTick();
        if (active_host_link != HOST_LINK_RPI ||
            current_robot_mode != ROBOT_MODE_WORK ||
            ChassisSafety_GetActiveMotion() != CHASSIS_MOTION_NONE ||
            !chassis_motors_enabled ||
            !ChassisSafety_CanReady() ||
            !ChassisSafety_OpsReady(now)) {
            rpi_binary_armed = 0U;
            printf("# ERROR HOST BINARY NOT READY\r\n");
            return 1U;
        }
        rpi_binary_armed = 1U;
        last_host_command_tick = now;
        printf("# HOST BINARY READY VERSION=%u CAPS=0x%08lX\r\n",
               RPI_PROTOCOL_VERSION, (unsigned long)RPI_CAPABILITIES);
        return 1U;
    }

    if (strcmp(command, "HOST LINK COM") == 0) requested = HOST_LINK_COM;
    else if (strcmp(command, "HOST LINK RPI") == 0) requested = HOST_LINK_RPI;
    else if (strncmp(command, "HOST LINK ", 10U) == 0) {
        printf("# ERROR HOST LINK COM|RPI\r\n");
        return 1U;
    }
    else if (strcmp(command, "HOST STATUS") == 0) {
        printf("# HOST STATUS STATE=%s OWNER=%s MOTOR_EN=%u HEARTBEAT=%s WAIT_MS=%lu TIMEOUT_MS=%lu\r\n",
               active_host_link == HOST_LINK_NONE ? "WAITING" : "LINKED",
               HostLink_Name(active_host_link),
               chassis_motors_enabled,
               active_host_link == HOST_LINK_RPI ? "REQUIRED" : "OFF",
               (unsigned long)(HAL_GetTick() - host_wait_start_tick),
               (unsigned long)HOST_WAIT_TIMEOUT_MS);
        return 1U;
    } else {
        return 0U;
    }

    /* 每次声明或切换主机都先停车、清旧状态，绝不恢复上一个主机的目标。 */
    (void)Mecanum_SetRequiredMotorMask(0x0FU);
    Robot_StopAllMotion();
    ControlRuntime_ClearFault();
    LLM_TunerResetSession();
    telemetry_mask = 0U;
    current_robot_mode = ROBOT_MODE_WORK;
    active_host_link = requested;
    rpi_binary_armed = 0U;
    rpi_binary_active = 0U;
    RpiBinary_ResetRxQueue();
    RpiBinary_ResetTxQueue();
    App_ResetHostParser();
    last_host_command_tick = HAL_GetTick();
    if (!chassis_motors_enabled) HostLink_SetChassisEnabled(1U);
    printf("# HOST LINK %s OK HEARTBEAT=%s\r\n",
           HostLink_Name(active_host_link),
           active_host_link == HOST_LINK_RPI ? "REQUIRED" : "OFF");
    return 1U;
}

static void HostLink_ProcessWait(uint32_t now)
{
    if (active_host_link != HOST_LINK_NONE || host_wait_timeout_reported) return;
    if ((uint32_t)(now - host_wait_start_tick) >= HOST_WAIT_TIMEOUT_MS) {
        host_wait_timeout_reported = 1U;
        printf("# HOST WAIT TIMEOUT STATE=WAITING\r\n");
    }
}

static void Robot_SetMode(RobotMode_t mode)
{
    if (current_robot_mode == mode) {
        if (mode != ROBOT_MODE_TUNE && Mecanum_GetRequiredMotorMask() != 0x0FU) {
            (void)Mecanum_SetRequiredMotorMask(0x0FU);
            (void)StopAllMotors();
        }
        if (mode == ROBOT_MODE_WORK) {
            (void)G6220_SetEnabled(1U);
        }
        printf("# MODE %s PLOT=%u CHANGED=0\r\n",
               RobotMode_Name(current_robot_mode), telemetry_mask != 0U);
        return;
    }

    /* 模式切换是安全边界：先停掉旧模式的一切运动，再启用新模式输出。 */
    if (mode != ROBOT_MODE_TUNE) {
        (void)Mecanum_SetRequiredMotorMask(0x0FU);
    }
    Robot_StopAllMotion();
    LLM_TunerResetSession();
    current_robot_mode = mode;

    if (mode == ROBOT_MODE_WORK) {
        (void)G6220_SetEnabled(1U);
    }

    printf("# MODE %s PLOT=%u CHANGED=1 MOTION=STOPPED\r\n",
           RobotMode_Name(current_robot_mode), telemetry_mask != 0U);
}

static ChassisMotionType_t ChassisSafety_GetActiveMotion(void)
{
    if (LLM_TunerIsRunning()) return CHASSIS_MOTION_TUNE_ROUND;
    if (pose_control_active) return CHASSIS_MOTION_POSE;
    if (debug_chassis_active) return CHASSIS_MOTION_DEBUG_CHASSIS;
    if (debug_motor_active) return CHASSIS_MOTION_SINGLE_MOTOR;
    return CHASSIS_MOTION_NONE;
}

static const char *ChassisSafety_MotionName(ChassisMotionType_t motion)
{
    if (motion == CHASSIS_MOTION_POSE) return "POSE";
    if (motion == CHASSIS_MOTION_TUNE_ROUND) return "TUNE";
    if (motion == CHASSIS_MOTION_DEBUG_CHASSIS) return "DEBUG_CHASSIS";
    if (motion == CHASSIS_MOTION_SINGLE_MOTOR) return "SINGLE_MOTOR";
    return "NONE";
}

static uint8_t ChassisSafety_CanReady(void)
{
    return ZDT_CAN_IsReady();
}

static uint8_t ChassisSafety_OpsReady(uint32_t now)
{
    const OPS9_Snapshot ops = OPS9_GetSnapshot();
    uint32_t last_ops_tick = ops.last_update_tick;
    now = HAL_GetTick();
    return (isfinite(ops.x_mm) && isfinite(ops.y_mm) && isfinite(ops.yaw_deg) &&
            ops.frame_count > 0U &&
            (uint32_t)(now - last_ops_tick) <= LLM_TUNE_OPS_TIMEOUT_MS) ? 1U : 0U;
}

/* Hardware faults stop immediately. A software fault remains latched through
 * the existing 100 ms grace; reading its notification cannot restart the timer. */
#define CAN_FAULT_GRACE_MS 100U
static uint32_t can_fault_since;
static uint8_t can_fault_active;

static void ChassisSafety_Stop(ChassisMotionType_t motion, const char *reason)
{
    uint16_t binary_reason = RPI_FAULT_INTERNAL_ERROR;

    if (strcmp(reason, "CAN FAULT") == 0 || strcmp(reason, "MOTOR FEEDBACK LOST") == 0) binary_reason = RPI_FAULT_CAN_FAULT;
    else if (strcmp(reason, "OPS LOST") == 0) binary_reason = RPI_FAULT_OPS9_LOST;
    else if (strcmp(reason, "HOST LOST") == 0) binary_reason = RPI_FAULT_HOST_LOST;
    else if (strcmp(reason, "TIMEOUT") == 0) binary_reason = RPI_FAULT_TIMEOUT;

    debug_motor_active = 0U;
    debug_chassis_active = 0U;
    pose_control_active = 0U;
    can_fault_active = 0U;
    Pose_ResetPlanner();
    (void)G6220_SetEnabled(0U);
    PID_Reset(&pid_x);
    PID_Reset(&pid_y);
    PID_Reset(&pid_yaw);

    if (motion == CHASSIS_MOTION_TUNE_ROUND && LLM_TunerIsRunning()) {
        LLM_TunerStopRound(reason);
        return;
    }

    LLM_TunerAbort();
    (void)StopAllMotors();
    if (motion == CHASSIS_MOTION_POSE) {
        RpiBinary_SendFault(binary_reason);
        printf("# POSE STOP SAFETY REASON=%s\r\n", reason);
    } else {
        printf("# MOTION STOP SAFETY TYPE=%s REASON=%s\r\n",
               ChassisSafety_MotionName(motion), reason);
    }
}

static void ChassisSafety_Process(uint32_t now)
{
    ChassisMotionType_t motion = ChassisSafety_GetActiveMotion();
    uint8_t needs_ops;
    uint8_t needs_host;
    uint8_t can_tx_fault;

    (void)Mecanum_ConsumeCanTxFault(); /* Notification only. */
    can_tx_fault = ZDT_CAN_HasFault();

    if (motion == CHASSIS_MOTION_NONE) {
        /* 空闲期不累计停车证据，否则紧接着开始的运动会被历史故障立刻打断。 */
        can_fault_active = 0U;
        return;
    }
    if (ControlRuntime_GetStats().fault) { ChassisSafety_Stop(motion, "CONTROL OVERRUN"); return; }
    /* CAN状态和四轮实际发送结果对所有运动类型都是共同的硬安全边界。 */
    if (can_tx_fault) {
        if (!can_fault_active) { can_fault_active = 1U; can_fault_since = now; }
    } else {
        can_fault_active = 0U;
    }
    if ((can_fault_active &&
         (uint32_t)(now - can_fault_since) >= CAN_FAULT_GRACE_MS) ||
        !ZDT_CAN_HardwareReady()) {
        ZDT_CAN_Stats_t stats;
        ZDT_CAN_GetStats(&stats);
        printf("# CAN SAFETY TX_FAULT=%u READY=%u ERR=0x%08lX ESR=0x%08lX "
               "TX_TIMEOUT=%lu AUTO_REC=%lu STREAK_MS=%lu\r\n",
               can_tx_fault, ZDT_CAN_IsReady(),
               (unsigned long)HAL_CAN_GetError(&hcan1), (unsigned long)stats.esr,
               (unsigned long)stats.tx_timeout, (unsigned long)stats.auto_recoveries,
               (unsigned long)(can_fault_active ?
                               (uint32_t)(now - can_fault_since) : 0U));
        ChassisSafety_Stop(motion, "CAN FAULT");
        return;
    }

    /* 单电机悬空台架测试不依赖OPS；其余底盘运动都要求位姿链路有效。 */
    needs_ops = (motion != CHASSIS_MOTION_SINGLE_MOTOR) ? 1U : 0U;
    if (needs_ops && !ChassisSafety_OpsReady(now)) {
        ChassisSafety_Stop(motion, "OPS LOST");
        return;
    }

    /* 只有 RPI 在 WORK 运动时要求心跳；COM 调试由操作者直接看护。 */
    needs_host = (active_host_link == HOST_LINK_RPI &&
                  current_robot_mode != ROBOT_MODE_TUNE) ? 1U : 0U;
    if (needs_host &&
        (uint32_t)(now - last_host_command_tick) > LLM_TUNE_HOST_TIMEOUT_MS) {
        ChassisSafety_Stop(motion, "HOST LOST");
        return;
    }

    if (motion == CHASSIS_MOTION_POSE) {
        uint32_t timeout_ms = (rpi_binary_active &&
                               rpi_binary_pose_state == RPI_POSE_MOVING) ?
                              rpi_binary_goal_timeout_ms :
                              ((current_robot_mode == ROBOT_MODE_TUNE) ?
                               POSE_TUNE_TIMEOUT_MS : POSE_WORK_TIMEOUT_MS);
        if ((uint32_t)(now - pose_start_time) > timeout_ms) {
            ChassisSafety_Stop(motion, "TIMEOUT");
        }
    }
}

static void Pose_InitMotionProfile(void)
{
    /* 默认值也经过硬边界裁剪，避免以后改宏时越过机械安全范围。 */
    pose_motion_profile.linear_accel_mps2 =
        Motion_Clamp(POSE_ACCEL_DEFAULT_MPS2,
                        POSE_ACCEL_MIN_MPS2,
                        POSE_ACCEL_HARD_MAX_MPS2);
    pose_motion_profile.linear_decel_mps2 =
        Motion_Clamp(POSE_DECEL_DEFAULT_MPS2,
                        POSE_DECEL_MIN_MPS2,
                        POSE_DECEL_HARD_MAX_MPS2);
    pose_motion_profile.yaw_accel_radps2 =
        Motion_Clamp(POSE_YAW_ACCEL_DEFAULT_RADPS2,
                        POSE_YAW_ACCEL_MIN_RADPS2,
                        POSE_YAW_ACCEL_HARD_MAX_RADPS2);
    pose_motion_profile.yaw_decel_radps2 =
        Motion_Clamp(POSE_YAW_DECEL_DEFAULT_RADPS2,
                        POSE_YAW_DECEL_MIN_RADPS2,
                        POSE_YAW_DECEL_HARD_MAX_RADPS2);
}

static void Pose_ResetPlanner(void)
{
    pose_control_phase = POSE_PHASE_IDLE;
    pose_target_vx = 0.0f;
    pose_target_vy = 0.0f;
    pose_target_vz = 0.0f;
    pose_output_vx = 0.0f;
    pose_output_vy = 0.0f;
    pose_output_vz = 0.0f;
    pose_settle_cycles = 0U;
    pose_last_control_time = HAL_GetTick();
}

static PoseStartResult_t Pose_StartTarget(float pose_x, float pose_y,
                                          float pose_yaw, uint8_t reject_if_busy)
{
    const OPS9_Snapshot ops = OPS9_GetSnapshot();
    float center_x;
    float center_y;
    float target_distance_mm;
    uint32_t now = HAL_GetTick();
    uint32_t last_ops_tick = ops.last_update_tick;

    if (reject_if_busy && pose_control_active) return POSE_START_BUSY;
    if (!ChassisSafety_CanReady()) return POSE_START_INVALID;
    if (!isfinite(pose_x) || !isfinite(pose_y) || !isfinite(pose_yaw)) {
        return POSE_START_INVALID;
    }
    if (ops.frame_count == 0U ||
        (uint32_t)(now - last_ops_tick) > LLM_TUNE_OPS_TIMEOUT_MS) {
        return POSE_START_OPS_NOT_READY;
    }

    Motion_OpsToCenter(pose_x, pose_y, pose_yaw,
                            &pose_target_center_x_mm,
                            &pose_target_center_y_mm);
    Motion_OpsToCenter(ops.x_mm, ops.y_mm, ops.yaw_deg, &center_x, &center_y);
    target_distance_mm = sqrtf(
        (pose_target_center_x_mm - center_x) *
        (pose_target_center_x_mm - center_x) +
        (pose_target_center_y_mm - center_y) *
        (pose_target_center_y_mm - center_y));
    if (!isfinite(target_distance_mm) ||
        target_distance_mm > POSE_TARGET_DISTANCE_HARD_MAX_MM) {
        return POSE_START_OUT_OF_BOUNDS;
    }

    debug_motor_active = 0U;
    debug_chassis_active = 0U;
    LLM_TunerAbort();
    StopAllMotors();
    PID_Reset(&pid_x);
    PID_Reset(&pid_y);
    PID_Reset(&pid_yaw);
    PID_SetTarget(&pid_x, pose_target_center_x_mm);
    PID_SetTarget(&pid_y, pose_target_center_y_mm);
    pose_target_x_mm = pose_x;
    pose_target_y_mm = pose_y;
    pose_target_yaw_deg = pose_yaw;
    pose_translation_yaw_deg = ops.yaw_deg;

    Pose_ResetPlanner();
    pose_last_control_time = now;
    pose_start_time = now;
    pose_control_phase = POSE_PHASE_TRANSLATE;
    pose_control_active = 1U;
    return POSE_START_OK;
}

static uint32_t RpiBinary_ReadU32(const uint8_t *data)
{
    return (uint32_t)data[0] |
           ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) |
           ((uint32_t)data[3] << 24);
}

static int32_t RpiBinary_ReadI32(const uint8_t *data)
{
    return (int32_t)RpiBinary_ReadU32(data);
}

static void RpiBinary_WriteU16(uint8_t *output, uint16_t value)
{
    output[0] = (uint8_t)(value & 0xFFU);
    output[1] = (uint8_t)(value >> 8);
}

static void RpiBinary_WriteU32(uint8_t *output, uint32_t value)
{
    output[0] = (uint8_t)(value & 0xFFU);
    output[1] = (uint8_t)((value >> 8) & 0xFFU);
    output[2] = (uint8_t)((value >> 16) & 0xFFU);
    output[3] = (uint8_t)(value >> 24);
}

static void RpiBinary_WriteI32(uint8_t *output, int32_t value)
{
    RpiBinary_WriteU32(output, (uint32_t)value);
}

static uint8_t RpiBinary_DequeueFrame(RpiFrame *frame)
{
    uint8_t available;
    if (frame == NULL) return 0U;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    available = HostRx_PopFrame(&host_rx, frame);
    __set_PRIMASK(primask);
    return available;
}

static void RpiBinary_ResetRxQueue(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    HostRx_ResetFrames(&host_rx);
    __set_PRIMASK(primask);
}

static void RpiBinary_ResetTxQueue(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    RpiProtocol_TxQueueReset(&rpi_binary_tx_queue);
    rpi_binary_tx_dma_active = 0U;
    rpi_binary_tx_loaded_ready = 0U;
    __set_PRIMASK(primask);
}

static void HostLink_ProcessUartFault(uint32_t now)
{
    uint32_t fault_primask;
    if (!host_uart_fault_pending) return;
    fault_primask = __get_PRIMASK();
    __disable_irq();
    host_uart_fault_pending = 0U;
    __set_PRIMASK(fault_primask);

    /* ISR只锁存故障；停车和状态清理在主循环完成，避免中断内阻塞。 */
    Robot_StopAllMotion();
    (void)HAL_UART_AbortTransmit(&huart1);
    HostUartTx_Reset();
    rpi_binary_active = 0U;
    rpi_binary_armed = 0U;
    rpi_binary_goal_id = 0U;
    rpi_binary_pose_state = RPI_POSE_FAULT;
    rpi_binary_fault_reason = RPI_FAULT_UART_FAULT;
    active_host_link = HOST_LINK_NONE;
    {
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        HostRx_ResetText(&host_rx);
        __set_PRIMASK(primask);
    }
    host_wait_start_tick = now;
    last_host_command_tick = now;
    RpiBinary_ResetRxQueue();
    RpiBinary_ResetTxQueue();
    App_ResetHostParser();
}

static void RpiBinary_ProcessSessionTimeout(uint32_t now)
{
    if (!rpi_binary_active ||
        (uint32_t)(now - last_host_command_tick) <= LLM_TUNE_HOST_TIMEOUT_MS) {
        return;
    }
    /* Link loss never resumes the old binary goal; return to ASCII self-check. */
    Robot_StopAllMotion();
    (void)HAL_UART_AbortTransmit(&huart1);
    HostUartTx_Reset();
    rpi_binary_active = 0U;
    rpi_binary_armed = 0U;
    rpi_binary_goal_id = 0U;
    rpi_binary_pose_state = RPI_POSE_IDLE;
    telemetry_mask = 0U;
    active_host_link = HOST_LINK_NONE;
    host_wait_start_tick = now;
    RpiBinary_ResetRxQueue();
    RpiBinary_ResetTxQueue();
    App_ResetHostParser();
}

static HAL_StatusTypeDef RpiBinary_QueueSend(uint8_t message_type,
                                             const uint8_t *payload,
                                             uint16_t payload_length,
                                             uint8_t priority)
{
    uint8_t frame[RPI_PROTOCOL_MAX_FRAME];
    uint16_t length = RpiProtocol_Encode(message_type,
                                         rpi_binary_tx_sequence++,
                                         payload, payload_length,
                                         frame, sizeof(frame));
    if (length == 0U) return HAL_ERROR;
    if (!RpiProtocol_TxQueuePush(&rpi_binary_tx_queue, frame, length, priority)) {
        host_uart_tx_error++;
        return HAL_BUSY;
    }
    return HAL_OK;
}

static HAL_StatusTypeDef RpiBinary_Send(uint8_t message_type,
                                        const uint8_t *payload,
                                        uint16_t payload_length)
{
    return RpiBinary_QueueSend(message_type, payload, payload_length, 1U);
}

static void RpiBinary_TxProcess(void)
{
    HAL_StatusTypeDef status;
    if (HostUartTx_Busy()) return;
    if (rpi_binary_tx_dma_active) {
        if ((uint32_t)(HAL_GetTick() - rpi_binary_tx_started) > 100U)
            host_uart_fault_pending = 1U;
        return;
    }
    if (!rpi_binary_tx_loaded_ready) {
        if (!RpiProtocol_TxQueuePop(&rpi_binary_tx_queue, &rpi_binary_tx_loaded)) return;
        rpi_binary_tx_loaded_ready = 1U;
    }
    /* RX interrupt长期处于BUSY_RX；HAL_UART_Transmit_DMA只检查独立gState。 */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    rpi_binary_tx_started = HAL_GetTick();
    rpi_binary_tx_dma_active = 1U;
    status = HAL_UART_Transmit_DMA(&huart1, rpi_binary_tx_loaded.bytes,
                                  rpi_binary_tx_loaded.length);
    if (status != HAL_OK) rpi_binary_tx_dma_active = 0U;
    __set_PRIMASK(primask);
    if (status != HAL_OK && status != HAL_BUSY) {
        host_uart_tx_error++;
        host_uart_fault_pending = 1U;
    }
}

static void RpiBinary_SendResponse(uint8_t request_sequence, uint8_t command,
                                   RpiResponseStatus_t status,
                                   const uint8_t *data, uint16_t data_length)
{
    uint8_t payload[RPI_PROTOCOL_MAX_PAYLOAD];
    uint16_t index;
    if (data_length > RPI_PROTOCOL_MAX_PAYLOAD - RPI_BINARY_RESPONSE_PREFIX_SIZE) return;
    payload[0] = request_sequence;
    payload[1] = command;
    payload[2] = (uint8_t)status;
    for (index = 0U; index < data_length; ++index) payload[3U + index] = data[index];
    (void)RpiBinary_QueueSend(RPI_MSG_RESPONSE, payload,
                              (uint16_t)(3U + data_length), 1U);
}

static void RpiBinary_SendGoalEvent(uint8_t event_code, uint32_t goal_id)
{
    uint8_t payload[5];
    if (!rpi_binary_active || goal_id == 0U) return;
    payload[0] = event_code;
    RpiBinary_WriteU32(&payload[1], goal_id);
    (void)RpiBinary_Send(RPI_MSG_EVENT, payload, sizeof(payload));
}

static void RpiBinary_SendReached(float x_mm, float y_mm, float yaw_deg,
                                  float position_error_mm, float yaw_error_deg)
{
    uint8_t payload[25];
    uint32_t goal_id = rpi_binary_goal_id;
    if (!rpi_binary_active || goal_id == 0U) return;
    payload[0] = RPI_EVENT_POSE_REACHED;
    RpiBinary_WriteU32(&payload[1], goal_id);
    RpiBinary_WriteI32(&payload[5], (int32_t)lroundf(x_mm));
    RpiBinary_WriteI32(&payload[9], (int32_t)lroundf(y_mm));
    RpiBinary_WriteI32(&payload[13], (int32_t)lroundf(yaw_deg * 17.45329252f));
    RpiBinary_WriteI32(&payload[17], (int32_t)lroundf(position_error_mm));
    RpiBinary_WriteI32(&payload[21], (int32_t)lroundf(yaw_error_deg * 17.45329252f));
    rpi_binary_pose_state = RPI_POSE_REACHED;
    rpi_binary_fault_reason = RPI_FAULT_UNSPECIFIED;
    (void)RpiBinary_Send(RPI_MSG_EVENT, payload, sizeof(payload));
}

static void RpiBinary_SendFault(uint16_t reason)
{
    uint8_t payload[7];
    if (!rpi_binary_active || rpi_binary_goal_id == 0U ||
        rpi_binary_pose_state != RPI_POSE_MOVING) return;
    rpi_binary_pose_state = RPI_POSE_FAULT;
    rpi_binary_fault_reason = reason;
    payload[0] = RPI_EVENT_MOTION_FAULT;
    RpiBinary_WriteU32(&payload[1], rpi_binary_goal_id);
    RpiBinary_WriteU16(&payload[5], reason);
    (void)RpiBinary_QueueSend(RPI_MSG_EVENT, payload, sizeof(payload), 2U);
}

static void RpiBinary_Process(void)
{
    const OPS9_Snapshot ops = OPS9_GetSnapshot();
    RpiFrame frame;
    uint8_t command = 0U;
    uint8_t response_data[21];
    uint16_t response_length = 0U;
    RpiResponseStatus_t status = RPI_STATUS_OK;
    uint32_t goal_id;
    uint32_t timeout_ms;
    uint8_t send_cancelled = 0U;
    PoseStartResult_t start_result;
    float linear_limit;
    float yaw_limit;

    if (!RpiBinary_DequeueFrame(&frame)) return;

    command = frame.payload_length > 0U ? frame.payload[0] : 0U;
    if ((!rpi_binary_armed || active_host_link != HOST_LINK_RPI) &&
        command != RPI_CMD_STOP_ALL && command != RPI_CMD_SESSION_PROBE) {
        RpiBinary_SendResponse(frame.sequence, command, RPI_STATUS_BUSY, NULL, 0U);
        App_ResetHostParser();
        return;
    }
    if (rpi_binary_armed) {
        HostUartTx_DiscardPending();
        rpi_binary_active = 1U;
        telemetry_mask = TELEMETRY_MASK_BOTH;
    }
    last_host_command_tick = HAL_GetTick();

    if (frame.message_type != RPI_MSG_COMMAND || frame.payload_length == 0U) {
        RpiBinary_SendResponse(frame.sequence, 0U, RPI_STATUS_INVALID_LENGTH, NULL, 0U);
        return;
    }
    command = frame.payload[0];
    switch (command) {
    case RPI_CMD_PING:
        if (frame.payload_length != 1U) status = RPI_STATUS_INVALID_LENGTH;
        break;
    case RPI_CMD_SESSION_PROBE:
        if (frame.payload_length != 1U) {
            status = RPI_STATUS_INVALID_LENGTH;
            break;
        }
        response_data[0] = rpi_binary_active;
        response_data[1] = rpi_binary_armed;
        response_data[2] = (uint8_t)active_host_link;
        response_data[3] = (uint8_t)rpi_binary_pose_state;
        RpiBinary_WriteU32(&response_data[4], rpi_binary_goal_id);
        RpiBinary_WriteU32(&response_data[8], RPI_CAPABILITIES);
        response_data[12] = RPI_PROTOCOL_VERSION;
        response_length = 13U;
        if (!rpi_binary_armed) App_ResetHostParser();
        break;
    case RPI_CMD_STOP_ALL:
        if (frame.payload_length != 1U) {
            status = RPI_STATUS_INVALID_LENGTH;
            break;
        }
        if (rpi_binary_pose_state == RPI_POSE_MOVING) {
            rpi_binary_pose_state = RPI_POSE_CANCELLED;
            send_cancelled = 1U;
        }
        Robot_StopAllMotion();
        break;
    case RPI_CMD_SET_POSE_GOAL:
    case RPI_CMD_SET_POSE_GOAL_WITH_LIMITS:
        if (frame.payload_length !=
            (command == RPI_CMD_SET_POSE_GOAL ? 21U : 29U)) {
            status = RPI_STATUS_INVALID_LENGTH;
            break;
        }
        if (current_robot_mode != ROBOT_MODE_WORK) {
            status = RPI_STATUS_INVALID_ARGUMENT;
            break;
        }
        goal_id = RpiBinary_ReadU32(&frame.payload[1]);
        timeout_ms = RpiBinary_ReadU32(&frame.payload[17]);
        if (goal_id == 0U || timeout_ms == 0U ||
            timeout_ms > RPI_BINARY_GOAL_TIMEOUT_MAX_MS) {
            status = RPI_STATUS_INVALID_ARGUMENT;
            break;
        }
        if (command == RPI_CMD_SET_POSE_GOAL_WITH_LIMITS) {
            linear_limit = (float)RpiBinary_ReadI32(&frame.payload[21]) / 1000000.0f;
            yaw_limit = (float)RpiBinary_ReadI32(&frame.payload[25]) / 1000000.0f;
            if (linear_limit < POSE_SPEED_MIN_MPS ||
                linear_limit > POSE_SPEED_HARD_MAX_MPS ||
                yaw_limit < POSE_YAW_SPEED_MIN_RADPS ||
                yaw_limit > POSE_YAW_SPEED_HARD_MAX_RADPS) {
                status = RPI_STATUS_INVALID_ARGUMENT;
                break;
            }
        }
        start_result = Pose_StartTarget(
            (float)RpiBinary_ReadI32(&frame.payload[5]),
            (float)RpiBinary_ReadI32(&frame.payload[9]),
            (float)RpiBinary_ReadI32(&frame.payload[13]) / 17.45329252f,
            1U);
        if (start_result == POSE_START_BUSY) status = RPI_STATUS_BUSY;
        else if (start_result != POSE_START_OK) status = RPI_STATUS_INVALID_ARGUMENT;
        if (status == RPI_STATUS_OK) {
            if (command == RPI_CMD_SET_POSE_GOAL_WITH_LIMITS) {
                pid_x.max_out = linear_limit;
                pid_y.max_out = linear_limit;
                pid_yaw.max_out = yaw_limit;
            }
            rpi_binary_goal_id = goal_id;
            rpi_binary_goal_timeout_ms = timeout_ms;
            rpi_binary_pose_state = RPI_POSE_MOVING;
            rpi_binary_fault_reason = RPI_FAULT_UNSPECIFIED;
        }
        break;
    case RPI_CMD_CANCEL_POSE_GOAL:
        if (frame.payload_length != 5U) {
            status = RPI_STATUS_INVALID_LENGTH;
            break;
        }
        goal_id = RpiBinary_ReadU32(&frame.payload[1]);
        if (goal_id == 0U || goal_id != rpi_binary_goal_id) {
            status = RPI_STATUS_INVALID_ARGUMENT;
            break;
        }
        if (rpi_binary_pose_state != RPI_POSE_MOVING) {
            status = RPI_STATUS_BUSY;
            break;
        }
        Robot_StopAllMotion();
        rpi_binary_pose_state = RPI_POSE_CANCELLED;
        break;
    case RPI_CMD_QUERY_POSE_GOAL:
        if (frame.payload_length != 1U) {
            status = RPI_STATUS_INVALID_LENGTH;
            break;
        }
        RpiBinary_WriteU32(&response_data[0], rpi_binary_goal_id);
        response_data[4] = (uint8_t)rpi_binary_pose_state;
        RpiBinary_WriteI32(&response_data[5], (int32_t)lroundf(ops.x_mm));
        RpiBinary_WriteI32(&response_data[9], (int32_t)lroundf(ops.y_mm));
        RpiBinary_WriteI32(&response_data[13], (int32_t)lroundf(ops.yaw_deg * 17.45329252f));
        RpiBinary_WriteU16(&response_data[17], rpi_binary_fault_reason);
        response_data[19] = (uint8_t)current_robot_mode;
        response_data[20] = (uint8_t)active_host_link;
        response_length = 21U;
        break;
    case RPI_CMD_SET_SPEED_LIMITS:
        if (frame.payload_length != 9U || pose_control_active) {
            status = frame.payload_length != 9U ?
                     RPI_STATUS_INVALID_LENGTH : RPI_STATUS_BUSY;
            break;
        }
        linear_limit = (float)RpiBinary_ReadI32(&frame.payload[1]) / 1000000.0f;
        yaw_limit = (float)RpiBinary_ReadI32(&frame.payload[5]) / 1000000.0f;
        if (linear_limit < POSE_SPEED_MIN_MPS ||
            linear_limit > POSE_SPEED_HARD_MAX_MPS ||
            yaw_limit < POSE_YAW_SPEED_MIN_RADPS ||
            yaw_limit > POSE_YAW_SPEED_HARD_MAX_RADPS) {
            status = RPI_STATUS_INVALID_ARGUMENT;
            break;
        }
        pid_x.max_out = linear_limit;
        pid_y.max_out = linear_limit;
        pid_yaw.max_out = yaw_limit;
        break;
    default:
        status = RPI_STATUS_UNKNOWN_COMMAND;
        break;
    }

    RpiBinary_SendResponse(frame.sequence, command, status,
                           response_data, response_length);
    if (status == RPI_STATUS_OK &&
        (command == RPI_CMD_SET_POSE_GOAL ||
         command == RPI_CMD_SET_POSE_GOAL_WITH_LIMITS)) {
        RpiBinary_SendGoalEvent(RPI_EVENT_POSE_STARTED, rpi_binary_goal_id);
    } else if (status == RPI_STATUS_OK && command == RPI_CMD_CANCEL_POSE_GOAL) {
        RpiBinary_SendGoalEvent(RPI_EVENT_POSE_CANCELLED, rpi_binary_goal_id);
    } else if (status == RPI_STATUS_OK && command == RPI_CMD_STOP_ALL &&
               send_cancelled) {
        RpiBinary_SendGoalEvent(RPI_EVENT_POSE_CANCELLED, rpi_binary_goal_id);
    }
}

static float Pose_BrakeLimitLinear(float distance_mm, float tolerance_mm)
{
    float remaining_m = (distance_mm - tolerance_mm) / 1000.0f;
    if (remaining_m <= 0.0f) return 0.0f;
    return sqrtf(2.0f * pose_motion_profile.linear_decel_mps2 * remaining_m);
}

static float Pose_BrakeLimitYaw(float error_deg, float tolerance_deg)
{
    float remaining_rad = (fabsf(error_deg) - tolerance_deg) *
                          (3.1415926f / 180.0f);
    if (remaining_rad <= 0.0f) return 0.0f;
    return sqrtf(2.0f * pose_motion_profile.yaw_decel_radps2 * remaining_rad);
}

static void Host_PrintHelp(void)
{
    printf("# HELP HOST LINK COM|RPI | HOST STATUS (COM no heartbeat; RPI heartbeat required)\r\n");
    printf("# HELP PROTO VERSION | MODE WORK|TUNE | MODE STATUS (default WORK; switching stops motion)\r\n");
    printf("# HELP STATUS | PING | STOP | RESET | OPS STATUS | OPS MONITOR ON|OFF | OPS ZERO\r\n");
    printf("# HELP PROTO EMM|X | CAN STATUS | MOTOR EN|DIS <id>\r\n");
    printf("# HELP CONTROL STATUS | MOTOR FEEDBACK | MOTOR STOP STATUS | HOST RX STATUS\r\n");
    printf("# HELP MOTOR MASK STATUS|0x01..0x0F (TUNE only; bit0..3 = ID1..4)\r\n");
    printf("# HELP MOTOR RUN <id> <signed_rpm> [ms] | MOTOR STOP <id>|ALL | MOTOR GET <id>\r\n");
    printf("# HELP MOVE FWD|BACK|LEFT|RIGHT [mps] [ms] | TURN CW|CCW [radps] [ms] | MOVE STOP\r\n");
    printf("# HELP POSE SET <x_mm> <y_mm> <yaw_deg> | POSE STOP | POSE STATUS\r\n");
    printf("# HELP TUNE AXIS X|Y|YAW | TUNE LIMIT <mps_or_radps> | NO PING ROUND=5S POSE=15S ROUNDS=20\r\n");
    printf("# HELP PID SET X|Y|YAW <p> <i> <d> | PID LIMIT X|Y|YAW <value> | PID STATUS ALL\r\n");
    printf("# HELP G6220 STATUS | G6220 ENABLE | G6220 DISABLE\r\n");
    printf("# HELP TELEM OFF|POSE|STATUS (OPS pose group, 20Hz)\r\n");
    printf("# HELP PLOT STATUS (retired; MOTOR GET is on-demand)\r\n");
    printf("# LIMIT motor id=1..4 rpm=+/-%.0f duration=100..%lu ms\r\n",
           DEBUG_MOTOR_MAX_RPM, DEBUG_MOTOR_MAX_MS);
    printf("# LIMIT move speed=0..%.2f mps duration=100..%lu ms; FWD=+Y LEFT=-X\r\n",
           DEBUG_MOVE_MAX_MPS, DEBUG_MOVE_MAX_MS);
    printf("# LIMIT turn speed=0..%.2f radps duration=100..%lu ms\r\n",
           DEBUG_TURN_MAX_RADPS, DEBUG_MOVE_MAX_MS);
}

static void Telemetry_Process(uint32_t now)
{
    const OPS9_Snapshot ops = OPS9_GetSnapshot();
    uint8_t group;
    uint32_t period_ms;
    int written;

    if (telemetry_mask == 0U) return;
    period_ms = (telemetry_mask == TELEMETRY_MASK_BOTH) ?
                TELEMETRY_STAGGER_MS : TELEMETRY_PERIOD_MS;
    if ((uint32_t)(now - telemetry_last_tick) < period_ms) return;
    telemetry_last_tick = now;

    if (telemetry_mask == TELEMETRY_MASK_BOTH) {
        group = telemetry_next_group;
        telemetry_next_group = (group == TELEMETRY_MASK_WHEEL) ?
                               TELEMETRY_MASK_POSE : TELEMETRY_MASK_WHEEL;
    } else {
        group = telemetry_mask;
    }

    if (rpi_binary_active) {
        uint8_t payload[39];
        uint16_t sequence;
        uint8_t index;
        payload[0] = group == TELEMETRY_MASK_WHEEL ? RPI_TELEM_WHEEL : RPI_TELEM_POSE;
        RpiBinary_WriteU32(&payload[1], now);
        sequence = group == TELEMETRY_MASK_WHEEL ?
                   telemetry_wheel_sequence++ : telemetry_pose_sequence++;
        RpiBinary_WriteU16(&payload[5], sequence);
        if (group == TELEMETRY_MASK_WHEEL) {
            for (index = 0U; index < 4U; ++index) {
                RpiBinary_WriteI32(&payload[7U + index * 4U],
                                   (int32_t)lroundf(motors[index].target_speed * 10.0f));
                RpiBinary_WriteI32(&payload[23U + index * 4U],
                                   (int32_t)lroundf(motors[index].actual_speed * 10.0f));
            }
        } else {
            float center_x;
            float center_y;
            Motion_OpsToCenter(ops.x_mm, ops.y_mm, ops.yaw_deg, &center_x, &center_y);
            RpiBinary_WriteI32(&payload[7], (int32_t)lroundf(ops.x_mm));
            RpiBinary_WriteI32(&payload[11], (int32_t)lroundf(ops.y_mm));
            RpiBinary_WriteI32(&payload[15], (int32_t)lroundf(ops.yaw_deg * 17.45329252f));
            RpiBinary_WriteI32(&payload[19], (int32_t)lroundf(center_x));
            RpiBinary_WriteI32(&payload[23], (int32_t)lroundf(center_y));
            RpiBinary_WriteI32(&payload[27], (int32_t)lroundf(
                (pose_control_active ? pose_output_vx : 0.0f) * 1000000.0f));
            RpiBinary_WriteI32(&payload[31], (int32_t)lroundf(
                (pose_control_active ? pose_output_vy : 0.0f) * 1000000.0f));
            RpiBinary_WriteI32(&payload[35], (int32_t)lroundf(
                (pose_control_active ? pose_output_vz : 0.0f) * 1000000.0f));
        }
        written = RpiBinary_QueueSend(RPI_MSG_TELEMETRY, payload, sizeof(payload), 0U) == HAL_OK ? 1 : -1;
    } else if (group == TELEMETRY_MASK_WHEEL) {
        written = printf("@W,1,%lu,%u,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f\r\n",
                         (unsigned long)now, telemetry_wheel_sequence++,
                         motors[0].target_speed, motors[1].target_speed,
                         motors[2].target_speed, motors[3].target_speed,
                         motors[0].actual_speed, motors[1].actual_speed,
                         motors[2].actual_speed, motors[3].actual_speed);
    } else {
        float center_x;
        float center_y;
        Motion_OpsToCenter(ops.x_mm, ops.y_mm, ops.yaw_deg, &center_x, &center_y);
        written = printf("@P,1,%lu,%u,%.2f,%.2f,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f\r\n",
                         (unsigned long)now, telemetry_pose_sequence++,
                         ops.x_mm, ops.y_mm, ops.yaw_deg, center_x, center_y,
                         pose_control_active ? pose_output_vx : 0.0f,
                         pose_control_active ? pose_output_vy : 0.0f,
                         pose_control_active ? pose_output_vz : 0.0f);
    }

    if (written > 0) telemetry_tx_ok++;
    else telemetry_tx_error++;

    if (rpi_binary_active && (uint32_t)(now - binary_stats_last_tick) >= 1000U) {
        uint8_t stats[25];
        binary_stats_last_tick = now;
        stats[0] = RPI_TELEM_LINK_STATS;
        RpiBinary_WriteU32(&stats[1], now);
        RpiBinary_WriteU32(&stats[5], HostRx_GetStats(&host_rx).binary_dropped);
        RpiBinary_WriteU32(&stats[9], rpi_binary_tx_queue.dropped_critical);
        RpiBinary_WriteU32(&stats[13], rpi_binary_tx_queue.replaced_telemetry);
        RpiBinary_WriteU32(&stats[17], HostRx_GetStats(&host_rx).crc_errors);
        RpiBinary_WriteU32(&stats[21], host_uart_tx_error);
        (void)RpiBinary_QueueSend(RPI_MSG_TELEMETRY, stats, sizeof(stats), 0U);
    }
}

static void Ops_PrintStatus(void)
{
    const OPS9_Snapshot ops = OPS9_GetSnapshot();
    /* 先读取中断更新的时间戳，再读取当前时间，避免无符号减法下溢。 */
    uint32_t last_byte_tick = ops9_last_byte_tick;
    uint32_t last_frame_tick = ops.last_update_tick;
    uint32_t now = HAL_GetTick();
    uint32_t byte_age = (ops9_rx_byte_count == 0U) ? 0xFFFFFFFFUL : now - last_byte_tick;
    uint32_t frame_age = (ops.frame_count == 0U) ? 0xFFFFFFFFUL : now - last_frame_tick;
    float center_x;
    float center_y;
    const char *link;

    /*
     * NO_DATA: USART2 一个字节都没收到，优先查供电、TX/RX、共地和RS-232模块。
     * BYTES_NO_FRAME: 有字节但不符合OPS帧，优先查波特率、模块方向和电平转换。
     * STALE: 曾经有完整帧，但最近停止更新。
     * OK: 最近100ms内收到过完整、有效的OPS坐标帧。
     */
    if (ops9_rx_byte_count == 0U) link = "NO_DATA";
    else if (ops.frame_count == 0U) link = "BYTES_NO_FRAME";
    else if (frame_age > LLM_TUNE_OPS_TIMEOUT_MS) link = "STALE";
    else link = "OK";

    Motion_OpsToCenter(ops.x_mm, ops.y_mm, ops.yaw_deg, &center_x, &center_y);
    printf("# OPS LINK=%s X=%.2f Y=%.2f YAW=%.2f CENTER_X=%.2f CENTER_Y=%.2f "
           "OFFSET_X=%.2f OFFSET_Y=%.2f BYTES=%lu HEADERS=%lu "
           "FRAMES=%lu INVALID=%lu FORMAT_ERR=%lu UART_ERR=%lu "
           "BYTE_AGE=%lu FRAME_AGE=%lu LAST=0x%02X\r\n",
           link, ops.x_mm, ops.y_mm, ops.yaw_deg, center_x, center_y,
           OPS_CENTER_OFFSET_X_MM, OPS_CENTER_OFFSET_Y_MM,
           (unsigned long)ops9_rx_byte_count,
           (unsigned long)ops9_header_count,
           (unsigned long)ops.frame_count,
           (unsigned long)ops9_invalid_frame_count,
           (unsigned long)ops9_format_error_count,
           (unsigned long)ops9_uart_error_count,
           (unsigned long)byte_age, (unsigned long)frame_age,
           ops9_last_raw_byte);
}

static uint8_t motor_feedback_print_mask;
/* MOTOR RUN 后打印一次 0xF6 的 ACK，用于确认电机是否真的接受了速度命令。 */
static uint8_t motor_ack_print_mask;

static void Motor_ProcessFeedback(void)
{
    ZDT_MotorEvent_t event;
    uint8_t budget = 8U;
    while (budget-- && ZDT_Emm_PollEvent(&event)) {
        if (event.function_code == 0x35U) {
            /* 后台轮询独立于遥测开关；仅打印人工 MOTOR GET 请求的速度回复。 */
            if (motor_feedback_print_mask & (1U << (event.motor_id - 1U))) {
                motor_feedback_print_mask &= (uint8_t)~(1U << (event.motor_id - 1U));
                printf("# MOTOR SPEED ID=%u RPM=%.1f\r\n",
                       event.motor_id, event.speed_rpm);
            }
        } else if (event.function_code == 0x3AU) {
            printf("# MOTOR STATE ID=%u EN=%u REACHED=%u STALL=%u PROTECT=%u RAW=0x%02X\r\n",
                   event.motor_id,
                   (event.value & 0x01U) ? 1U : 0U,
                   (event.value & 0x02U) ? 1U : 0U,
                   (event.value & 0x04U) ? 1U : 0U,
                   (event.value & 0x08U) ? 1U : 0U,
                   event.value);
        } else {
            const char *result = "OTHER";
            /*
             * 速度命令和空闲停车/使能刷新均可能产生大量正常 ACK。
             * 静默这些 0x02 应答；条件/格式等异常仍输出。
             * MOTOR RUN 之后允许每个电机打印一次 ACK，便于区分“电机没收到命令”
             * 和“电机收到命令但没有转动”。
             */
            if (event.value == 0x02U &&
                (event.function_code == 0xFEU || event.function_code == 0xF3U)) {
                continue;
            }
            if (event.function_code == 0xF6U) {
                if (motor_ack_print_mask & (uint8_t)(1U << (event.motor_id - 1U))) {
                    motor_ack_print_mask &= (uint8_t)~(1U << (event.motor_id - 1U));
                } else if (event.value == 0x02U) {
                    continue;
                }
            }
            if (event.value == 0x02U) result = "OK";
            else if (event.value == 0xE2U) result = "CONDITION";
            else if (event.value == 0xEEU) result = "FORMAT";
            else if (event.value == 0x9FU) result = "DONE";
            printf("# MOTOR ACK ID=%u CMD=0x%02X CODE=0x%02X %s\r\n",
                   event.motor_id, event.function_code, event.value, result);
        }
    }
}

static uint8_t Host_ProcessOperationalCommand(const char *command)
{
    const OPS9_Snapshot ops = OPS9_GetSnapshot();
    unsigned int id;
    unsigned int motor_mask;
    float rpm = 0.0f;
    unsigned long duration_ms = DEBUG_MOTOR_DEFAULT_MS;
    unsigned long move_duration_ms = DEBUG_MOVE_DEFAULT_MS;
    char move_direction[8];
    float move_speed = DEBUG_MOVE_DEFAULT_MPS;
    float turn_speed = DEBUG_TURN_DEFAULT_RADPS;
    float vx = 0.0f;
    float vy = 0.0f;
    float vz = 0.0f;
    float v1, v2, v3, v4;
    float pose_x, pose_y, pose_yaw;
    float center_x, center_y;
    char extra;
    int fields;
    uint8_t result_a;
    uint8_t result_b;
    DM_G6220_Feedback_t g6220_feedback;
    DM_G6220_Result_t g6220_result;

    if (strcmp(command, "PING") == 0) {
        printf("# PONG\r\n");
        return 1U;
    }

    if (strcmp(command, "HELP") == 0) {
        Host_PrintHelp();
        return 1U;
    }

    if (strcmp(command, "PROTO VERSION") == 0) {
        printf("# PROTO VERSION=%u MODES=WORK,TUNE LEGACY_POSE=1 HOST_LINK=REQUIRED\r\n",
               HOST_PROTOCOL_VERSION);
        return 1U;
    }

    if (strcmp(command, "MODE STATUS") == 0) {
        printf("# MODE %s PLOT=%u TUNE_STATE=%u POSE_ACTIVE=%u\r\n",
               RobotMode_Name(current_robot_mode), telemetry_mask != 0U,
               (unsigned int)LLM_TunerGetState(), pose_control_active);
        return 1U;
    }

    if (strcmp(command, "MODE WORK") == 0) {
        Robot_SetMode(ROBOT_MODE_WORK);
        return 1U;
    }

    if (strcmp(command, "MODE TUNE") == 0) {
        Robot_SetMode(ROBOT_MODE_TUNE);
        return 1U;
    }

    if (strcmp(command, "MODE PLOT") == 0 || strcmp(command, "PLOT ON") == 0 ||
        strcmp(command, "TELEM WHEEL") == 0 || strcmp(command, "TELEM BOTH") == 0) {
        printf("# ERROR PLOT RETIRED; USE MODE TUNE OR TELEM POSE; MOTOR GET IS ON-DEMAND\r\n");
        return 1U;
    }

    if (strcmp(command, "PLOT OFF") == 0) {
        uint32_t tx_ok = telemetry_tx_ok;
        uint32_t tx_error = telemetry_tx_error;
        telemetry_mask = 0U;
        Robot_SetMode(ROBOT_MODE_WORK);
        printf("# PLOT OFF TX_OK=%lu TX_ERR=%lu\r\n",
               (unsigned long)tx_ok, (unsigned long)tx_error);
        return 1U;
    }

    if (strcmp(command, "PLOT STATUS") == 0) {
        printf("# PLOT ENABLED=0 RETIRED=1 MOTOR_FEEDBACK=ON_DEMAND\r\n");
        return 1U;
    }

    if (strcmp(command, "TELEM STATUS") == 0) {
        printf("# TELEM MASK=%u WHEEL=%u POSE=%u PERIOD=50MS FORMAT=TAGGED_ASCII "
               "W_SEQ=%u P_SEQ=%u TX_OK=%lu TX_ERR=%lu\r\n",
               telemetry_mask,
               (telemetry_mask & TELEMETRY_MASK_WHEEL) != 0U,
               (telemetry_mask & TELEMETRY_MASK_POSE) != 0U,
               telemetry_wheel_sequence, telemetry_pose_sequence,
               (unsigned long)telemetry_tx_ok,
               (unsigned long)telemetry_tx_error);
        return 1U;
    }

    if (strcmp(command, "TELEM OFF") == 0 ||
        strcmp(command, "TELEM WHEEL") == 0 ||
        strcmp(command, "TELEM POSE") == 0 ||
        strcmp(command, "TELEM BOTH") == 0) {
        if (strcmp(command, "TELEM OFF") == 0) telemetry_mask = 0U;
        else if (strcmp(command, "TELEM WHEEL") == 0) telemetry_mask = TELEMETRY_MASK_WHEEL;
        else if (strcmp(command, "TELEM POSE") == 0) telemetry_mask = TELEMETRY_MASK_POSE;
        else telemetry_mask = TELEMETRY_MASK_BOTH;
        telemetry_last_tick = HAL_GetTick();
        telemetry_next_group = TELEMETRY_MASK_WHEEL;
        printf("# TELEM MASK=%u WHEEL=%u POSE=%u FORMAT=TAGGED_ASCII GROUPS=W8,P8\r\n",
               telemetry_mask,
               (telemetry_mask & TELEMETRY_MASK_WHEEL) != 0U,
               (telemetry_mask & TELEMETRY_MASK_POSE) != 0U);
        return 1U;
    }

    if (strcmp(command, "HELP") == 0) {
        Host_PrintHelp();
        return 1U;
    }

    if (strcmp(command, "PROTO EMM") == 0 || strcmp(command, "PROTO X") == 0) {
        StopAllMotors();
        debug_motor_active = 0U;
        debug_chassis_active = 0U;
        pose_control_active = 0U;
        Pose_ResetPlanner();
        LLM_TunerAbort();
        ZDT_Emm_SetProtocol((strcmp(command, "PROTO X") == 0) ? ZDT_PROTOCOL_X : ZDT_PROTOCOL_EMM);
        StopAllMotors();
        printf("# PROTOCOL %s\r\n", ZDT_Emm_GetProtocol() == ZDT_PROTOCOL_X ? "X" : "EMM");
        return 1U;
    }

    if (strcmp(command, "CAN STATUS") == 0) {
        ZDT_CAN_Stats_t stats;
        ZDT_CAN_GetStats(&stats);
        printf("# CAN STATE=%u ERROR=0x%08lX FREE=%lu TX_OK=%lu TX_ERR=%lu RX=%lu LAST=%u\r\n",
               (unsigned int)HAL_CAN_GetState(&hcan1),
               (unsigned long)HAL_CAN_GetError(&hcan1),
               (unsigned long)HAL_CAN_GetTxMailboxesFreeLevel(&hcan1),
               (unsigned long)stats.tx_ok, (unsigned long)stats.tx_error,
               (unsigned long)stats.rx_count, stats.last_tx_result);
        printf("# CAN TX_QUEUED=%lu TX_ABORT=%lu TX_TIMEOUT=%lu ERR_CB=%lu ERR_FATAL=%lu ERR_LATCH=0x%08lX ACK_SEEN=%u BOFF_SEEN=%u\r\n",
               (unsigned long)stats.tx_queued, (unsigned long)stats.tx_aborted,
               (unsigned long)stats.tx_timeout, (unsigned long)stats.error_callbacks,
               (unsigned long)stats.fatal_error_callbacks,
               (unsigned long)stats.error_latched,
               (unsigned int)!!(stats.error_latched & HAL_CAN_ERROR_ACK),
               (unsigned int)!!(stats.error_latched & HAL_CAN_ERROR_BOF));
        printf("# CAN READY=%u RECOVERIES=%lu AUTO_REC=%lu STALL_REC=%lu TX_FAULT=%u POLL_FAIL=%lu MASK=0x%02X GEN=%lu REC_PHASE=%u\r\n",
               ZDT_CAN_IsReady(), (unsigned long)stats.recoveries,
               (unsigned long)stats.auto_recoveries,
               (unsigned long)stats.stall_recoveries,
               stats.tx_fault,
               (unsigned long)Mecanum_GetPollFailures(),
               Mecanum_GetRequiredMotorMask(),
               (unsigned long)stats.fault_generation, stats.recovery_phase);
        printf("# CAN TX_WATCH NO_TX_REPAIR=%lu\r\n", (unsigned long)stats.no_tx_repairs);
        printf("# CAN ESR=0x%08lX TSR=0x%08lX TEC=%lu REC=%lu BOFF=%u EPVF=%u EWGF=%u LEC=%lu\r\n",
               (unsigned long)stats.esr, (unsigned long)stats.tsr,
               (unsigned long)((stats.esr >> 16) & 255U),
               (unsigned long)((stats.esr >> 24) & 255U),
               (unsigned int)!!(stats.esr & CAN_ESR_BOFF),
               (unsigned int)!!(stats.esr & CAN_ESR_EPVF),
               (unsigned int)!!(stats.esr & CAN_ESR_EWGF),
               (unsigned long)((stats.esr >> 4) & 7U));
        return 1U;
    }

    if (strcmp(command, "G6220 STATUS") == 0) {
        (void)DM_G6220_GetFeedback(&g6220_motor, &g6220_feedback);
        printf("# G6220 INIT=%u ENABLE_REQ=%u CAN_STATE=%u CAN_ERR=0x%08lX "
               "TX_OK=%lu TX_ERR=%lu RX_OK=%lu RX_IGN=%lu LAST=%u ",
               g6220_initialized, g6220_enable_requested,
               (unsigned int)HAL_CAN_GetState(&hcan2),
               (unsigned long)HAL_CAN_GetError(&hcan2),
               (unsigned long)g6220_motor.tx_ok,
               (unsigned long)g6220_motor.tx_error,
               (unsigned long)g6220_motor.rx_ok,
               (unsigned long)g6220_motor.rx_ignored,
               (unsigned int)g6220_last_result);
        if (g6220_motor.rx_ok > 0U) {
            printf("STATE=%u POS=%.4f VEL=%.4f TORQUE=%.3f MOS=%.1f ROTOR=%.1f AGE=%luMS\r\n",
                   (unsigned int)g6220_feedback.state,
                   g6220_feedback.position_rad,
                   g6220_feedback.velocity_radps,
                   g6220_feedback.torque_nm,
                   g6220_feedback.mos_temperature_c,
                   g6220_feedback.rotor_temperature_c,
                   (unsigned long)(HAL_GetTick() - g6220_feedback.update_tick_ms));
        } else {
            printf("STATE=NO_FEEDBACK\r\n");
        }
        return 1U;
    }

    if (strcmp(command, "G6220 ENABLE") == 0) {
        if (current_robot_mode != ROBOT_MODE_WORK) {
            printf("# ERROR G6220 ENABLE REQUIRES MODE WORK\r\n");
        } else {
            g6220_result = G6220_SetEnabled(1U);
            printf("# G6220 ENABLE RESULT=%u\r\n", (unsigned int)g6220_result);
        }
        return 1U;
    }

    if (strcmp(command, "G6220 DISABLE") == 0) {
        g6220_result = G6220_SetEnabled(0U);
        printf("# G6220 DISABLE RESULT=%u\r\n", (unsigned int)g6220_result);
        return 1U;
    }

    if (strcmp(command, "OPS STATUS") == 0) {
        Ops_PrintStatus();
        return 1U;
    }

    if (strcmp(command, "OPS MONITOR ON") == 0) {
        ops_monitor_enabled = 1U;
        ops_monitor_last_tick = 0U;
        printf("# OPS MONITOR ON PERIOD=1000MS\r\n");
        return 1U;
    }

    if (strcmp(command, "OPS MONITOR OFF") == 0) {
        ops_monitor_enabled = 0U;
        printf("# OPS MONITOR OFF\r\n");
        return 1U;
    }

    if (strcmp(command, "OPS ZERO") == 0) {
        if (OPS9_Reset_Zero()) printf("# OPS ZERO SENT ACT0\r\n");
        else printf("# ERROR OPS ZERO UART BUSY\r\n");
        return 1U;
    }

    if (strcmp(command, "POSE STOP") == 0) {
        pose_control_active = 0U;
        Pose_ResetPlanner();
        PID_Reset(&pid_x);
        PID_Reset(&pid_y);
        PID_Reset(&pid_yaw);
        /* 保持现有取消协议语义：收到 POSE STOP 后立即确认已经零速。 */
        StopAllMotors();
        printf("# POSE STOP\r\n");
        return 1U;
    }

    if (strcmp(command, "POSE STATUS") == 0) {
        Motion_OpsToCenter(ops.x_mm, ops.y_mm, ops.yaw_deg, &center_x, &center_y);
        printf("# POSE ACTIVE=%u TARGET_X=%.2f TARGET_Y=%.2f TARGET_YAW=%.2f "
               "TARGET_CENTER_X=%.2f TARGET_CENTER_Y=%.2f "
               "X=%.2f Y=%.2f YAW=%.2f CENTER_X=%.2f CENTER_Y=%.2f "
               "CMD_VX=%.3f CMD_VY=%.3f CMD_VZ=%.3f "
               "PLAN_VX=%.3f PLAN_VY=%.3f PLAN_VZ=%.3f\r\n",
               pose_control_active, pose_target_x_mm, pose_target_y_mm,
               pose_target_yaw_deg, pose_target_center_x_mm, pose_target_center_y_mm,
               ops.x_mm, ops.y_mm, ops.yaw_deg, center_x, center_y,
               pose_target_vx, pose_target_vy, pose_target_vz,
               pose_output_vx, pose_output_vy, pose_output_vz);
        return 1U;
    }

    if (strcmp(command, "MOTOR MASK STATUS") == 0) {
        uint8_t mask = Mecanum_GetRequiredMotorMask();
        printf("# MOTOR MASK=0x%02X ID1=%u ID2=%u ID3=%u ID4=%u MODE=%s\r\n",
               mask,
               (mask & 0x01U) != 0U, (mask & 0x02U) != 0U,
               (mask & 0x04U) != 0U, (mask & 0x08U) != 0U,
               RobotMode_Name(current_robot_mode));
        return 1U;
    }

    if (strncmp(command, "MOTOR MASK ", 11U) == 0) {
        fields = sscanf(command, "MOTOR MASK %x %c", &motor_mask, &extra);
        if (current_robot_mode != ROBOT_MODE_TUNE) {
            printf("# ERROR MOTOR MASK REQUIRES MODE=TUNE CURRENT=%s\r\n",
                   RobotMode_Name(current_robot_mode));
        } else if (fields != 1 || motor_mask == 0U || motor_mask > 0x0FU) {
            printf("# ERROR MOTOR MASK 0x01..0x0F\r\n");
        } else {
            Robot_StopAllMotion();
            (void)Mecanum_SetRequiredMotorMask((uint8_t)motor_mask);
            (void)StopAllMotors();
            printf("# MOTOR MASK=0x%02X POLL=OFF MOTION=STOPPED\r\n",
                   Mecanum_GetRequiredMotorMask());
        }
        return 1U;
    }

    if (sscanf(command, "POSE SET %f %f %f", &pose_x, &pose_y, &pose_yaw) == 3) {
        PoseStartResult_t start_result = Pose_StartTarget(
            pose_x, pose_y, pose_yaw, 0U);
        if (start_result != POSE_START_OK) {
            pose_control_active = 0U;
            Pose_ResetPlanner();
            StopAllMotors();
        }
        if (start_result == POSE_START_INVALID) {
            printf("# ERROR POSE VALUE\r\n");
        } else if (start_result == POSE_START_OPS_NOT_READY) {
            printf("# ERROR OPS NOT READY\r\n");
        } else if (start_result == POSE_START_OUT_OF_BOUNDS) {
            printf("# ERROR POSE OUT OF BOUNDS MAX_MM=%.2f\r\n",
                   POSE_TARGET_DISTANCE_HARD_MAX_MM);
        } else if (start_result == POSE_START_OK) {
            printf("# POSE START X=%.2f Y=%.2f YAW=%.2f "
                   "CENTER_X=%.2f CENTER_Y=%.2f TOL_MM=%.2f TOL_YAW=%.2f\r\n",
                   pose_x, pose_y, pose_yaw,
                   pose_target_center_x_mm, pose_target_center_y_mm,
                   POSE_POSITION_TOL_MM, POSE_YAW_TOL_DEG);
        }
        return 1U;
    }

    if (strcmp(command, "MOTOR STOP ALL") == 0) {
        StopAllMotors();
        debug_motor_active = 0U;
        debug_chassis_active = 0U;
        pose_control_active = 0U;
        Pose_ResetPlanner();
        LLM_TunerAbort();
        printf("# MOTOR STOP ALL\r\n");
        return 1U;
    }

    if (strcmp(command, "MOVE STOP") == 0) {
        StopAllMotors();
        debug_motor_active = 0U;
        debug_chassis_active = 0U;
        pose_control_active = 0U;
        Pose_ResetPlanner();
        LLM_TunerAbort();
        printf("# MOVE STOP\r\n");
        return 1U;
    }

    fields = sscanf(command, "TURN %7s %f %lu",
                    move_direction, &turn_speed, &move_duration_ms);
    if (fields >= 1) {
        if (!isfinite(turn_speed) || turn_speed <= 0.0f ||
            turn_speed > DEBUG_TURN_MAX_RADPS) {
            printf("# ERROR TURN SPEED 0..%.2f RADPS\r\n", DEBUG_TURN_MAX_RADPS);
            return 1U;
        }
        if (move_duration_ms < 100UL || move_duration_ms > DEBUG_MOVE_MAX_MS) {
            printf("# ERROR TURN DURATION 100..%lu MS\r\n", DEBUG_MOVE_MAX_MS);
            return 1U;
        }

        if (strcmp(move_direction, "CCW") == 0)      vz =  turn_speed;
        else if (strcmp(move_direction, "CW") == 0) vz = -turn_speed;
        else {
            printf("# ERROR TURN DIR CW|CCW\r\n");
            return 1U;
        }

        /* 调试运动仅限 TUNE；WORK 模式下唯一运动源是 POSE SET（受心跳与安全检查保护）。 */
        if (current_robot_mode == ROBOT_MODE_WORK) {
            printf("# ERROR MODE REQUIRED=TUNE CURRENT=WORK\r\n");
            return 1U;
        }
        if (Mecanum_GetRequiredMotorMask() != 0x0FU) {
            printf("# ERROR CHASSIS MOVE REQUIRES MOTOR MASK=0x0F\r\n");
            return 1U;
        }
        if (!ChassisSafety_CanReady() || !ChassisSafety_OpsReady(HAL_GetTick())) {
            printf("# ERROR DEBUG CHASSIS SAFETY CAN=%u OPS=%u\r\n",
                   ChassisSafety_CanReady(), ChassisSafety_OpsReady(HAL_GetTick()));
            return 1U;
        }

        /* 旋转测试只给 Vz，低速且定时自动停止，用于检查四轮旋转组合和OPS航向。 */
        StopAllMotors();
        debug_motor_active = 0U;
        pose_control_active = 0U;
        Pose_ResetPlanner();
        LLM_TunerAbort();

        Mecanum_Kinematics(0.0f, 0.0f, vz, &v1, &v2, &v3, &v4);
        SetAllMotorsSpeed(v1, v2, v3, v4);
        debug_chassis_active = 1U;
        debug_chassis_stop_tick = HAL_GetTick() + (uint32_t)move_duration_ms;
        printf("# TURN %s SPEED=%.3f RADPS MS=%lu VZ=%.3f\r\n",
               move_direction, turn_speed, move_duration_ms, vz);
        return 1U;
    }

    fields = sscanf(command, "MOVE %7s %f %lu",
                    move_direction, &move_speed, &move_duration_ms);
    if (fields >= 1) {
        if (!isfinite(move_speed) || move_speed <= 0.0f ||
            move_speed > DEBUG_MOVE_MAX_MPS) {
            printf("# ERROR MOVE SPEED 0..%.2f MPS\r\n", DEBUG_MOVE_MAX_MPS);
            return 1U;
        }
        if (move_duration_ms < 100UL || move_duration_ms > DEBUG_MOVE_MAX_MS) {
            printf("# ERROR MOVE DURATION 100..%lu MS\r\n", DEBUG_MOVE_MAX_MS);
            return 1U;
        }

        /* 调试命令使用车体方向；仅在车头对齐 OPS +Y 时与 OPS 全局轴重合。 */
        if (strcmp(move_direction, "FWD") == 0)       vy =  move_speed;
        else if (strcmp(move_direction, "BACK") == 0) vy = -move_speed;
        else if (strcmp(move_direction, "LEFT") == 0) vx = -move_speed;
        else if (strcmp(move_direction, "RIGHT") == 0) vx = move_speed;
        else {
            printf("# ERROR MOVE DIR FWD|BACK|LEFT|RIGHT\r\n");
            return 1U;
        }

        /* 调试运动仅限 TUNE；WORK 模式下唯一运动源是 POSE SET（受心跳与安全检查保护）。 */
        if (current_robot_mode == ROBOT_MODE_WORK) {
            printf("# ERROR MODE REQUIRED=TUNE CURRENT=WORK\r\n");
            return 1U;
        }
        if (Mecanum_GetRequiredMotorMask() != 0x0FU) {
            printf("# ERROR CHASSIS MOVE REQUIRES MOTOR MASK=0x0F\r\n");
            return 1U;
        }
        if (!ChassisSafety_CanReady() || !ChassisSafety_OpsReady(HAL_GetTick())) {
            printf("# ERROR DEBUG CHASSIS SAFETY CAN=%u OPS=%u\r\n",
                   ChassisSafety_CanReady(), ChassisSafety_OpsReady(HAL_GetTick()));
            return 1U;
        }

        StopAllMotors();
        debug_motor_active = 0U;
        pose_control_active = 0U;
        Pose_ResetPlanner();
        LLM_TunerAbort();

        Mecanum_Kinematics(vx, vy, 0.0f, &v1, &v2, &v3, &v4);
        SetAllMotorsSpeed(v1, v2, v3, v4);
        debug_chassis_active = 1U;
        debug_chassis_stop_tick = HAL_GetTick() + (uint32_t)move_duration_ms;
        printf("# MOVE %s SPEED=%.3f MPS MS=%lu VX=%.3f VY=%.3f\r\n",
               move_direction, move_speed, move_duration_ms, vx, vy);
        return 1U;
    }

    if (sscanf(command, "MOTOR EN %u", &id) == 1) {
        if (!Motor_IsValidId(id)) {
            printf("# ERROR MOTOR ID 1..4\r\n");
        } else {
            result_a = ZDT_Emm_EnableByID((uint8_t)id, 1U);
            printf("# MOTOR EN ID=%u TX=%u\r\n", id, result_a);
        }
        return 1U;
    }

    if (sscanf(command, "MOTOR DIS %u", &id) == 1) {
        if (!Motor_IsValidId(id)) {
            printf("# ERROR MOTOR ID 1..4\r\n");
        } else {
            ZDT_Emm_StopMask((uint8_t)(1U << (id - 1U)));
            result_a = ZDT_Emm_EnableByID((uint8_t)id, 0U);
            if (debug_motor_active && debug_motor_id == (uint8_t)id) debug_motor_active = 0U;
            printf("# MOTOR DIS ID=%u TX=%u\r\n", id, result_a);
        }
        return 1U;
    }

    if (sscanf(command, "MOTOR STOP %u", &id) == 1) {
        if (!Motor_IsValidId(id)) {
            printf("# ERROR MOTOR ID 1..4\r\n");
        } else {
            result_a = ZDT_Emm_StopMask((uint8_t)(1U << (id - 1U)));
            if (debug_motor_active && debug_motor_id == (uint8_t)id) debug_motor_active = 0U;
            printf("# MOTOR STOP ID=%u TX=%u\r\n", id, result_a);
        }
        return 1U;
    }

    if (sscanf(command, "MOTOR GET %u", &id) == 1) {
        if (!Motor_IsValidId(id)) {
            printf("# ERROR MOTOR ID 1..4\r\n");
        } else {
            motor_feedback_print_mask |= (uint8_t)(1U << (id - 1U));
            result_a = ZDT_Emm_ReadSpeedByID((uint8_t)id);

            result_b = ZDT_Emm_ReadStatusByID((uint8_t)id);
            printf("# MOTOR GET ID=%u TX_SPEED=%u TX_STATE=%u\r\n", id, result_a, result_b);
        }
        return 1U;
    }

    fields = sscanf(command, "MOTOR RUN %u %f %lu", &id, &rpm, &duration_ms);
    if (fields >= 2) {
        if (!Motor_IsValidId(id)) {
            printf("# ERROR MOTOR ID 1..4\r\n");
        } else if (!isfinite(rpm) || rpm == 0.0f || fabsf(rpm) > DEBUG_MOTOR_MAX_RPM) {
            printf("# ERROR RPM RANGE +/-%.0f NONZERO\r\n", DEBUG_MOTOR_MAX_RPM);
        } else if (duration_ms < 100UL || duration_ms > DEBUG_MOTOR_MAX_MS) {
            printf("# ERROR DURATION 100..%lu MS\r\n", DEBUG_MOTOR_MAX_MS);
        } else if (!(Mecanum_GetRequiredMotorMask() &
                     (uint8_t)(1U << (id - 1U)))) {
            printf("# ERROR MOTOR ID=%u NOT IN MASK=0x%02X\r\n",
                   id, Mecanum_GetRequiredMotorMask());
        } else {
            /* 调试运动仅限 TUNE；WORK 模式下唯一运动源是 POSE SET（受心跳与安全检查保护）。 */
            if (current_robot_mode == ROBOT_MODE_WORK) {
                printf("# ERROR MODE REQUIRED=TUNE CURRENT=WORK\r\n");
                return 1U;
            }
            /* Motion admission never clears the transport fault latch. */
            if (!ChassisSafety_CanReady()) {
                ZDT_CAN_Stats_t stats;
                ZDT_CAN_GetStats(&stats);
                printf("# ERROR MOTOR RUN CAN NOT READY STATE=%u ERR=0x%08lX "
                       "ESR=0x%08lX TX_TIMEOUT=%lu\r\n",
                       (unsigned int)HAL_CAN_GetState(&hcan1),
                       (unsigned long)HAL_CAN_GetError(&hcan1),
                       (unsigned long)stats.esr, (unsigned long)stats.tx_timeout);
                return 1U;
            }
            StopAllMotors();
            debug_chassis_active = 0U;
            pose_control_active = 0U;
            Pose_ResetPlanner();
            LLM_TunerAbort();
            /*
             * StopAllMotors() 的邮箱撤销/清队列有可能再次锁存一次瞬时发送故障；
             * 必须在真正下发速度之前再确认一次，否则刚发出去的速度会被
             * ChassisSafety_Process 立刻当成 CAN FAULT 用零速覆盖掉。
             */

            if (!ChassisSafety_CanReady()) {
                ZDT_CAN_Stats_t stats;
                ZDT_CAN_GetStats(&stats);
                printf("# ERROR MOTOR RUN CAN NOT READY AFTER STOP STATE=%u "
                       "ERR=0x%08lX ESR=0x%08lX TX_TIMEOUT=%lu\r\n",
                       (unsigned int)HAL_CAN_GetState(&hcan1),
                       (unsigned long)HAL_CAN_GetError(&hcan1),
                       (unsigned long)stats.esr, (unsigned long)stats.tx_timeout);
                return 1U;
            }
            /* 让下一次 0xF6 的 ACK 可见，便于确认电机是否真正接受了速度命令。 */
            motor_ack_print_mask |= (uint8_t)(1U << (id - 1U));
            result_a = ZDT_Emm_EnableByID((uint8_t)id, 1U);

            result_b = ZDT_Emm_SetSpeedByID((uint8_t)id, rpm);
            if (result_a == 0U && result_b == 0U) {
                debug_motor_active = 1U;
                debug_motor_id = (uint8_t)id;
                debug_motor_stop_tick = HAL_GetTick() + (uint32_t)duration_ms;
            }
            printf("# MOTOR RUN ID=%u RPM=%.1f MS=%lu PROTO=%s TX_EN=%u TX_RUN=%u\r\n",
                   id, rpm, duration_ms,
                   ZDT_Emm_GetProtocol() == ZDT_PROTOCOL_X ? "X" : "EMM",
                   result_a, result_b);
        }
        return 1U;
    }

    return 0U;
}

static void Pose_ProcessControl(uint32_t now)
{
    const OPS9_Snapshot ops = OPS9_GetSnapshot();
    float current_ops_x;
    float current_ops_y;
    float current_x;
    float current_y;
    float current_yaw;
    float error_x;
    float error_y;
    float error_yaw;
    float yaw_reference_deg;
    float distance_mm;
    float linear_speed;
    float world_vx;
    float world_vy;
    float world_speed;
    float linear_limit;
    float heading_rad;
    float body_vx;
    float body_vy;
    float desired_vz;
    float yaw_limit;
    float step_yaw;
    float yaw_rate;
    float dt_s;
    float v1, v2, v3, v4;
    uint32_t last_ops_tick;
    uint32_t elapsed_ms;

    if (!pose_control_active) {
        return;
    }

    last_ops_tick = ops.last_update_tick;
    now = HAL_GetTick();
    current_ops_x = ops.x_mm;
    current_ops_y = ops.y_mm;
    current_yaw = ops.yaw_deg;

    if (!isfinite(current_ops_x) || !isfinite(current_ops_y) || !isfinite(current_yaw) ||
        ops.frame_count == 0U ||
        (uint32_t)(now - last_ops_tick) > LLM_TUNE_OPS_TIMEOUT_MS ||
        (active_host_link == HOST_LINK_RPI &&
         current_robot_mode != ROBOT_MODE_TUNE &&
         (uint32_t)(now - last_host_command_tick) > LLM_TUNE_HOST_TIMEOUT_MS) ||
        (current_robot_mode == ROBOT_MODE_TUNE &&
         (uint32_t)(now - pose_start_time) > POSE_TUNE_TIMEOUT_MS) ||
        !ChassisSafety_CanReady()) {
        pose_control_active = 0U;
        Pose_ResetPlanner();
        StopAllMotors();
        (void)G6220_SetEnabled(0U);
        PID_Reset(&pid_x);
        PID_Reset(&pid_y);
        PID_Reset(&pid_yaw);
        RpiBinary_SendFault(RPI_FAULT_INTERNAL_ERROR);
        printf("# POSE STOP SAFETY\r\n");
        return;
    }

    Motion_OpsToCenter(current_ops_x, current_ops_y, current_yaw,
                            &current_x, &current_y);
    error_x = pose_target_center_x_mm - current_x;
    error_y = pose_target_center_y_mm - current_y;
    yaw_reference_deg = (pose_control_phase == POSE_PHASE_TRANSLATE) ?
                        pose_translation_yaw_deg : pose_target_yaw_deg;
    error_yaw = Motion_AngleDeltaDeg(yaw_reference_deg, current_yaw);
    distance_mm = sqrtf(error_x * error_x + error_y * error_y);

    if (!isfinite(current_x) || !isfinite(current_y) ||
        !isfinite(error_x) || !isfinite(error_y) || !isfinite(error_yaw) ||
        !isfinite(distance_mm) ||
        distance_mm > POSE_RUNTIME_ERROR_HARD_MAX_MM) {
        pose_control_active = 0U;
        Pose_ResetPlanner();
        StopAllMotors();
        (void)G6220_SetEnabled(0U);
        PID_Reset(&pid_x);
        PID_Reset(&pid_y);
        PID_Reset(&pid_yaw);
        RpiBinary_SendFault(RPI_FAULT_OUT_OF_BOUNDS);
        printf("# POSE STOP SAFETY\r\n");
        return;
    }

    /* 安全条件每次主循环都检查；只有正常规划计算受 20 ms 控制周期限制。 */
    elapsed_ms = (uint32_t)(now - pose_last_control_time);
    if (elapsed_ms < POSE_CONTROL_PERIOD_MS) return;
    ControlRuntime_RecordControl(elapsed_ms);
    if (elapsed_ms > POSE_DT_MAX_MS) elapsed_ms = POSE_DT_MAX_MS;
    dt_s = (float)elapsed_ms / 1000.0f;
    pose_last_control_time = now;

    /* X/Y PID先给出全局速度，再旋转到车体坐标，供树莓派直接下发全局目标位姿。 */
    world_vx = PID_CalcDt(&pid_x, current_x, dt_s);
    world_vy = PID_CalcDt(&pid_y, current_y, dt_s);
    world_speed = sqrtf(world_vx * world_vx + world_vy * world_vy);
    linear_limit = Pose_BrakeLimitLinear(distance_mm, POSE_POSITION_TOL_MM);
    /* X/Y 的 PID LIMIT 最终解释为二维平移速度矢量的模长上限。 */
    if (linear_limit > pid_x.max_out) linear_limit = pid_x.max_out;
    if (linear_limit > pid_y.max_out) linear_limit = pid_y.max_out;
    if (linear_limit > POSE_SPEED_HARD_MAX_MPS) {
        linear_limit = POSE_SPEED_HARD_MAX_MPS;
    }
    if (world_speed > linear_limit && world_speed > 0.0001f) {
        float scale = linear_limit / world_speed;
        world_vx *= scale;
        world_vy *= scale;
    }

    heading_rad = current_yaw * (3.1415926f / 180.0f);
    body_vx = cosf(heading_rad) * world_vx + sinf(heading_rad) * world_vy;
    body_vy = -sinf(heading_rad) * world_vx + cosf(heading_rad) * world_vy;
    pose_target_vx = body_vx;
    pose_target_vy = body_vy;

    desired_vz = PID_CalcErrorDt(&pid_yaw, error_yaw, dt_s);
    yaw_limit = Pose_BrakeLimitYaw(error_yaw, POSE_YAW_TOL_DEG);
    if (yaw_limit > pid_yaw.max_out) yaw_limit = pid_yaw.max_out;
    if (yaw_limit > POSE_YAW_SPEED_HARD_MAX_RADPS) {
        yaw_limit = POSE_YAW_SPEED_HARD_MAX_RADPS;
    }
    desired_vz = Motion_Clamp(desired_vz, -yaw_limit, yaw_limit);
    pose_target_vz = desired_vz;

    /* 对整个 Vx/Vy 差矢量限幅，保留 PID 给出的平移方向，避免逐轴斜坡扭曲轨迹。 */
    Motion_SlewVector2D(pose_output_vx, pose_output_vy,
                      body_vx, body_vy,
                      pose_motion_profile.linear_accel_mps2,
                      pose_motion_profile.linear_decel_mps2,
                      dt_s, &pose_output_vx, &pose_output_vy);
    yaw_rate = ((fabsf(pose_output_vz) > 0.0001f &&
                 pose_output_vz * desired_vz <= 0.0f) ||
                fabsf(desired_vz) < fabsf(pose_output_vz)) ?
               pose_motion_profile.yaw_decel_radps2 :
               pose_motion_profile.yaw_accel_radps2;
    step_yaw = yaw_rate * dt_s;
    pose_output_vz = Motion_Slew(pose_output_vz, desired_vz, step_yaw);
    linear_speed = sqrtf(pose_output_vx * pose_output_vx +
                         pose_output_vy * pose_output_vy);

    if (!isfinite(pose_output_vx) || !isfinite(pose_output_vy) ||
        !isfinite(pose_output_vz)) {
        pose_control_active = 0U;
        Pose_ResetPlanner();
        StopAllMotors();
        (void)G6220_SetEnabled(0U);
        PID_Reset(&pid_x);
        PID_Reset(&pid_y);
        PID_Reset(&pid_yaw);
        RpiBinary_SendFault(RPI_FAULT_INTERNAL_ERROR);
        printf("# POSE STOP SAFETY\r\n");
        return;
    }

    if (pose_control_phase == POSE_PHASE_TRANSLATE &&
        distance_mm <= POSE_POSITION_TOL_MM &&
        linear_speed <= POSE_STOP_LINEAR_EPS_MPS &&
        fabsf(error_yaw) <= POSE_YAW_TOL_DEG &&
        fabsf(pose_output_vz) <= POSE_STOP_YAW_EPS_RADPS) {
        /*
         * 平移阶段保持起始航向；到点且速度接近零后明确停车，再切换到
         * 原地转向阶段。协议仍只在最终位姿稳定后报告 POSE TARGET。
         */
        StopAllMotors();
        PID_Reset(&pid_x);
        PID_Reset(&pid_y);
        PID_Reset(&pid_yaw);
        Pose_ResetPlanner();
        pose_control_phase = POSE_PHASE_ROTATE;
        pose_last_control_time = now;
        return;
    }

    if (pose_control_phase == POSE_PHASE_ROTATE &&
        distance_mm <= POSE_POSITION_TOL_MM &&
        fabsf(error_yaw) <= POSE_YAW_TOL_DEG &&
        linear_speed <= POSE_STOP_LINEAR_EPS_MPS &&
        fabsf(pose_output_vz) <= POSE_STOP_YAW_EPS_RADPS) {
        pose_settle_cycles++;
        if (pose_settle_cycles >= POSE_SETTLE_CYCLES) {
            pose_control_active = 0U;
            Pose_ResetPlanner();
            StopAllMotors();
            RpiBinary_SendReached(current_ops_x, current_ops_y, current_yaw,
                                  distance_mm, error_yaw);
            printf("# POSE TARGET X=%.2f Y=%.2f YAW=%.2f "
                   "CENTER_X=%.2f CENTER_Y=%.2f ERROR_MM=%.2f ERROR_YAW=%.2f\r\n",
                   current_ops_x, current_ops_y, current_yaw,
                   current_x, current_y, distance_mm, error_yaw);
            return;
        }
    } else {
        pose_settle_cycles = 0U;
    }

    Mecanum_Kinematics(pose_output_vx, pose_output_vy, pose_output_vz,
                       &v1, &v2, &v3, &v4);
    SetAllMotorsSpeed(v1, v2, v3, v4);
    /* 规划器在车体系限幅，先还原到全局坐标再反馈给 X/Y PID。 */
    PID_ApplyOutput(&pid_x, Mecanum_GetAppliedScale() * (cosf(heading_rad) * pose_output_vx -
                            sinf(heading_rad) * pose_output_vy));
    PID_ApplyOutput(&pid_y, Mecanum_GetAppliedScale() * (sinf(heading_rad) * pose_output_vx +
                            cosf(heading_rad) * pose_output_vy));
    PID_ApplyOutput(&pid_yaw, Mecanum_GetAppliedScale() * pose_output_vz);
}

static void Host_ProcessCommand(void)
{
    const OPS9_Snapshot ops = OPS9_GetSnapshot();
    char command[HOST_COMMAND_LENGTH];
    uint8_t available;
    uint32_t primask;
    float p_val, i_val, d_val;
    float speed_limit_val;
    char axis_name[8];
    PID_Controller *pid;
    LLM_TuneAxis_t requested_axis;
    float kp_max, ki_max, kd_max, output_min, output_hard_max;

    primask = __get_PRIMASK();
    __disable_irq();
    available = HostRx_PopText(&host_rx, command);
    __set_PRIMASK(primask);
    if (!available) return;

    /* HOST LINK 在所有业务命令之前处理；抢占失败不能刷新当前主机心跳。 */
    if (HostLink_ProcessCommand(command)) {
        return;
    }

    if (active_host_link == HOST_LINK_NONE) {
        /* 等待期间只开放无运动副作用的探测和停车命令。 */
        if (strcmp(command, "PING") == 0 ||
            strcmp(command, "PROTO VERSION") == 0 ||
            strcmp(command, "CAN STATUS") == 0 ||
            strcmp(command, "HELP") == 0) {
            (void)Host_ProcessOperationalCommand(command);
        } else if (strcmp(command, "STOP") == 0) {
            Robot_StopAllMotion();
            printf("# STOP MODE=%s HOST=WAITING\r\n", RobotMode_Name(current_robot_mode));
        } else {
            printf("# ERROR HOST NOT LINKED\r\n");
        }
        return;
    }

    last_host_command_tick = HAL_GetTick();

    if (Host_ProcessOperationalCommand(command)) {
        return;
    }

    if (strcmp(command, "PID STATUS ALL") == 0) {
        printf("# PID ALL X=%.7f,%.8f,%.7f Y=%.7f,%.8f,%.7f "
               "YAW=%.7f,%.8f,%.7f\r\n",
               pid_x.Kp, pid_x.Ki, pid_x.Kd,
               pid_y.Kp, pid_y.Ki, pid_y.Kd,
               pid_yaw.Kp, pid_yaw.Ki, pid_yaw.Kd);
    } else if (sscanf(command, "PID LIMIT %7s %f", axis_name, &speed_limit_val) == 2) {
        if (strcmp(axis_name, "X") == 0) requested_axis = LLM_TUNE_AXIS_X;
        else if (strcmp(axis_name, "Y") == 0) requested_axis = LLM_TUNE_AXIS_Y;
        else if (strcmp(axis_name, "YAW") == 0) requested_axis = LLM_TUNE_AXIS_YAW;
        else {
            printf("# ERROR PID AXIS X|Y|YAW\r\n");
            return;
        }
        output_min = requested_axis == LLM_TUNE_AXIS_YAW ?
                     POSE_YAW_SPEED_MIN_RADPS : POSE_SPEED_MIN_MPS;
        output_hard_max = requested_axis == LLM_TUNE_AXIS_YAW ?
                          POSE_YAW_SPEED_HARD_MAX_RADPS :
                          POSE_SPEED_HARD_MAX_MPS;
        if (isfinite(speed_limit_val) && speed_limit_val >= output_min &&
            speed_limit_val <= output_hard_max) {
            pid = LLM_TunerGetPidForAxis(requested_axis);
            pid->max_out = speed_limit_val;
            printf("# PID LIMIT AXIS=%s OUTPUT=%.3f UNIT=%s\r\n",
                   LLM_TunerAxisName(requested_axis), speed_limit_val,
                   requested_axis == LLM_TUNE_AXIS_YAW ? "RADPS" : "MPS");
        } else {
            printf("# ERROR PID LIMIT OUTPUT %.2f..%.2f\r\n",
                   output_min, output_hard_max);
        }
    } else if (sscanf(command, "PID SET %7s %f %f %f",
                      axis_name, &p_val, &i_val, &d_val) == 4) {
        if (strcmp(axis_name, "X") == 0) requested_axis = LLM_TUNE_AXIS_X;
        else if (strcmp(axis_name, "Y") == 0) requested_axis = LLM_TUNE_AXIS_Y;
        else if (strcmp(axis_name, "YAW") == 0) requested_axis = LLM_TUNE_AXIS_YAW;
        else {
            printf("# ERROR PID AXIS X|Y|YAW\r\n");
            return;
        }
        pid = LLM_TunerGetPidForAxis(requested_axis);
        kp_max = requested_axis == LLM_TUNE_AXIS_YAW ? LLM_TUNE_YAW_KP_MAX : LLM_TUNE_KP_MAX;
        ki_max = requested_axis == LLM_TUNE_AXIS_YAW ? LLM_TUNE_YAW_KI_MAX : LLM_TUNE_KI_MAX;
        kd_max = requested_axis == LLM_TUNE_AXIS_YAW ? LLM_TUNE_YAW_KD_MAX : LLM_TUNE_KD_MAX;
        if (isfinite(p_val) && isfinite(i_val) && isfinite(d_val) &&
            p_val >= 0.0f && p_val <= kp_max &&
            i_val >= 0.0f && i_val <= ki_max &&
            d_val >= 0.0f && d_val <= kd_max) {
            pid->Kp = p_val;
            pid->Ki = i_val;
            pid->Kd = d_val;
            PID_Reset(pid);
            printf("# PID LOADED AXIS=%s P=%.7f I=%.8f D=%.7f\r\n",
                   LLM_TunerAxisName(requested_axis), p_val, i_val, d_val);
        } else {
            printf("# ERROR PID LIMIT P<=%.4f I<=%.5f D<=%.4f\r\n",
                   kp_max, ki_max, kd_max);
        }
    } else if (sscanf(command, "TUNE AXIS %7s", axis_name) == 1) {
        if (strcmp(axis_name, "X") == 0) requested_axis = LLM_TUNE_AXIS_X;
        else if (strcmp(axis_name, "Y") == 0) requested_axis = LLM_TUNE_AXIS_Y;
        else if (strcmp(axis_name, "YAW") == 0) requested_axis = LLM_TUNE_AXIS_YAW;
        else {
            printf("# ERROR TUNE AXIS X|Y|YAW\r\n");
            return;
        }
        /* 兼容现有调参器：TUNE AXIS 同时作为进入独立 TUNE 模式的入口。 */
        if (current_robot_mode != ROBOT_MODE_TUNE) {
            Robot_SetMode(ROBOT_MODE_TUNE);
        }
        pose_control_active = 0U;
        Pose_ResetPlanner();
        LLM_TunerSetAxis(requested_axis);
        LLM_TunerStopRound("AXIS");
        LLM_TunerResetSession();
        printf("# TUNE AXIS %s UNIT_IN=%s UNIT_OUT=%s\r\n",
               LLM_TunerAxisName(requested_axis),
               requested_axis == LLM_TUNE_AXIS_YAW ? "DEG" : "MM",
               requested_axis == LLM_TUNE_AXIS_YAW ? "RADPS" : "MPS");
    } else if (sscanf(command, "TUNE LIMIT %f", &speed_limit_val) == 1) {
        if (current_robot_mode != ROBOT_MODE_TUNE) {
            printf("# ERROR MODE REQUIRED=TUNE CURRENT=%s\r\n",
                   RobotMode_Name(current_robot_mode));
            return;
        }
        requested_axis = LLM_TunerGetAxis();
        pid = LLM_TunerGetPid();
        output_hard_max = requested_axis == LLM_TUNE_AXIS_YAW ?
                          LLM_TUNE_YAW_HARD_MAX_RADPS : LLM_TUNE_SPEED_HARD_MAX_MPS;
        /* 工作速度上限由上位机统一配置；这里保留独立硬上限作为最终安全边界。 */
        if (isfinite(speed_limit_val) && speed_limit_val >= 0.02f &&
            speed_limit_val <= output_hard_max) {
            pid->max_out = speed_limit_val;
            printf("# TUNE LIMIT AXIS=%s OUTPUT=%.3f UNIT=%s HARD_MAX=%.3f\r\n",
                   LLM_TunerAxisName(requested_axis), pid->max_out,
                   requested_axis == LLM_TUNE_AXIS_YAW ? "RADPS" : "MPS",
                   output_hard_max);
        } else {
            printf("# ERROR TUNE LIMIT 0.02..%.2f\r\n", output_hard_max);
        }
    } else if ((sscanf(command, "SET P:%f I:%f D:%f", &p_val, &i_val, &d_val) == 3) ||
        (sscanf(command, "SET KP:%f KI:%f KD:%f", &p_val, &i_val, &d_val) == 3) ||
        (sscanf(command, "PID %f %f %f", &p_val, &i_val, &d_val) == 3) ||
        (sscanf(command, "P:%f,I:%f,D:%f", &p_val, &i_val, &d_val) == 3)) {
        if (current_robot_mode != ROBOT_MODE_TUNE) {
            printf("# ERROR MODE REQUIRED=TUNE CURRENT=%s\r\n",
                   RobotMode_Name(current_robot_mode));
            return;
        }
        if (Mecanum_GetRequiredMotorMask() != 0x0FU) {
            /* 掩码在 TUNE 中同样生效：0x0F 是整车自动调参，其余为台架验证。 */
            printf("# WARN PID TUNE MASK=0x%02X FULL_CHASSIS=0x0F\r\n",
                   Mecanum_GetRequiredMotorMask());
        }
        requested_axis = LLM_TunerGetAxis();
        pid = LLM_TunerGetPid();
        kp_max = requested_axis == LLM_TUNE_AXIS_YAW ? LLM_TUNE_YAW_KP_MAX : LLM_TUNE_KP_MAX;
        ki_max = requested_axis == LLM_TUNE_AXIS_YAW ? LLM_TUNE_YAW_KI_MAX : LLM_TUNE_KI_MAX;
        kd_max = requested_axis == LLM_TUNE_AXIS_YAW ? LLM_TUNE_YAW_KD_MAX : LLM_TUNE_KD_MAX;
        if (isfinite(p_val) && isfinite(i_val) && isfinite(d_val) &&
            p_val >= 0.0f && p_val <= kp_max &&
            i_val >= 0.0f && i_val <= ki_max &&
            d_val >= 0.0f && d_val <= kd_max) {
            pid->Kp = p_val;
            pid->Ki = i_val;
            pid->Kd = d_val;
            printf("# PID UPDATED AXIS=%s P=%.7f I=%.8f D=%.7f\r\n",
                   LLM_TunerAxisName(requested_axis), p_val, i_val, d_val);
            debug_chassis_active = 0U;
            pose_control_active = 0U;
            Pose_ResetPlanner();

            LLM_TunerStartRound();
        } else {
            printf("# ERROR PID LIMIT P<=%.4f I<=%.5f D<=%.4f\r\n",
                   kp_max, ki_max, kd_max);
        }
    } else if (strcmp(command, "STATUS") == 0) {
        requested_axis = LLM_TunerGetAxis();
        pid = LLM_TunerGetPid();
        printf("# STATUS MODE=%s HOST_PROTO=%u HOST=%s AXIS=%s P=%.7f I=%.8f D=%.7f MAX_OUT=%.3f "
               "STATE=%u PLOT=%u MOTOR_PROTO=%s OPS_FRAMES=%lu UART_TX_OK=%lu UART_TX_ERR=%lu\r\n",
               RobotMode_Name(current_robot_mode), HOST_PROTOCOL_VERSION,
               HostLink_Name(active_host_link),
               LLM_TunerAxisName(requested_axis),
               pid->Kp, pid->Ki, pid->Kd, pid->max_out,
               (unsigned int)LLM_TunerGetState(),
               telemetry_mask != 0U,
               ZDT_Emm_GetProtocol() == ZDT_PROTOCOL_X ? "X" : "EMM",
               (unsigned long)ops.frame_count,
               (unsigned long)host_uart_tx_ok,
               (unsigned long)host_uart_tx_error);
    } else if (strcmp(command, "RESET") == 0) {
        PID_Reset(LLM_TunerGetPid());
        pose_control_active = 0U;
        Pose_ResetPlanner();
        LLM_TunerResetSession();
        LLM_TunerStopRound("RESET");
    } else if (strcmp(command, "STOP") == 0) {
        if (current_robot_mode == ROBOT_MODE_TUNE && LLM_TunerIsRunning()) {
            debug_motor_active = 0U;
            debug_chassis_active = 0U;
            pose_control_active = 0U;
            Pose_ResetPlanner();
            (void)G6220_SetEnabled(0U);
            LLM_TunerStopRound("HOST");
        } else {
            Robot_StopAllMotion();
            printf("# STOP MODE=%s\r\n", RobotMode_Name(current_robot_mode));
        }
    } else {
        printf("# ERROR UNKNOWN COMMAND\r\n");
    }
}

static void Boot_ServiceDelay(uint32_t duration)
{
    uint32_t start = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - start) < duration) {
        ZDT_CAN_Process(HAL_GetTick());
        Mecanum_ProcessFeedback(HAL_GetTick());
        HAL_Delay(1U);
    }
}

void RobotApp_Init(void)
{
  // 声明外部的接收缓存变量
  extern uint8_t ops9_rx_byte;
  DM_G6220_Result_t g6220_result;
  // 开启 USART2 单字节中断接收
  HAL_UART_Receive_IT(&huart2, &ops9_rx_byte, 1);
  // 开启 USART1 单字节中断接收（接收树莓派或 PC 发来的主机命令）
    HostRx_Init(&host_rx);
    HostRx_ResetFrames(&host_rx);
    RpiProtocol_TxQueueReset(&rpi_binary_tx_queue);
    HAL_UART_Receive_IT(&huart1, &pc_rx_byte, 1);
  // 1. 初始化 CAN 和过滤器
  ZDT_CAN_ConfigFilter();

  // 2. 注册回调
  ZDT_CAN_RegisterCallback(ZDT_Emm_RxHandler);

  // G6220 独占 CAN2 (1 Mbit/s)：初始化软件对象、过滤器和 FIFO0 中断。
  g6220_result = DM_G6220_Init(&g6220_motor, &hcan2,
                               G6220_CAN_ID, G6220_MASTER_ID);
  if (g6220_result == DM_G6220_OK) {
      g6220_result = DM_G6220_StartCan(&g6220_motor,
                                      G6220_CAN2_FILTER_BANK,
                                      G6220_SLAVE_FILTER_START);
  }
  g6220_last_result = g6220_result;
  g6220_initialized = (g6220_result == DM_G6220_OK) ? 1U : 0U;

  // 3. 初始化 4 个电机
  ZDT_Emm_InitAll();
  (void)Mecanum_SetRequiredMotorMask(0x0FU);

  // 4. 上电等待阶段四轮保持零速使能，用闭环保持力矩防止外力造成车体偏移。
  Boot_ServiceDelay(100);
  StopAllMotors();
  HostLink_SetChassisEnabled(1U);
  Boot_ServiceDelay(100);

  // 5. TIM3/TIM4 当前未使用，不启动定时器。

  // 6. 初始化里程计计时器
    last_host_command_tick = HAL_GetTick();

  //7.初始化PID参数
  // 注意：坐标单位是 mm，误差 1000mm 时，乘以 Kp=0.001，算出的速度正好是 1.0 m/s
    Pose_InitMotionProfile();
    /* 实机分轴调参并完成稳定性验证后的基线参数。 */
    PID_Init(&pid_x,   0.00495f, 0.0f,     0.0f, POSE_SPEED_DEFAULT_MPS, 5000.0f);
    PID_Init(&pid_y,   0.0018f,  0.0f,     0.0f, POSE_SPEED_DEFAULT_MPS, 5000.0f);
    PID_Init(&pid_yaw, 0.02f,    0.000015f, 0.0f, POSE_YAW_SPEED_DEFAULT_RADPS, 1000.0f);
    LLM_TunerInit(&pid_x, &pid_y, &pid_yaw);

    StopAllMotors();
    if (g6220_initialized) {
        /* 厂商建议 CAN 初始化后等待约 1 秒；等待态仍保持 G6220 失能。 */
        Boot_ServiceDelay(G6220_STARTUP_DELAY_MS);
        g6220_result = G6220_SetEnabled(0U);
        Boot_ServiceDelay(G6220_COMMAND_DELAY_MS);
    }
    host_wait_start_tick = HAL_GetTick();
    last_host_command_tick = host_wait_start_tick;
    printf("# STM32F407 MECANUM X/Y/YAW PID CONTROLLER HOST WAIT\r\n");
    printf("# PROTO VERSION=%u MODES=WORK,TUNE LEGACY_POSE=1 HOST_LINK=REQUIRED\r\n",
           HOST_PROTOCOL_VERSION);
    printf("# HOST WAIT STATE=WAITING TIMEOUT_MS=%lu ACCEPT=COM,RPI\r\n",
           (unsigned long)HOST_WAIT_TIMEOUT_MS);
    printf("# MODE WORK PLOT=0 MOTION=STOPPED HOST=WAITING\r\n");
    printf("# G6220 INIT=%u ENABLE_REQ=%u RESULT=%u CAN_ID=0x%02X MASTER_ID=0x%03X\r\n",
           g6220_initialized, g6220_enable_requested,
           (unsigned int)g6220_last_result,
           (unsigned int)G6220_CAN_ID, (unsigned int)G6220_MASTER_ID);
    printf("# CSV timestamp,setpoint,input,output,error,p,i,d,ops_x_mm,ops_y_mm,yaw_deg,"
           "cross_mm,yaw_delta_deg,hold_cross,hold_yaw,center_x_mm,center_y_mm;"
           " UNIT BY AXIS\r\n");
    printf("# MOTOR PROTOCOL DEFAULT EMM; SEND HELP FOR DEBUG COMMANDS\r\n");
    printf("# SEND OPS STATUS OR OPS MONITOR ON TO CHECK OPS-9 LINK\r\n");
    printf("# TUNE NO PING ROUND=5S POSE=15S ROUNDS=%lu; HARD LIMIT %.2fMPS; SAFETY FAULT STOPS MOTORS\r\n",
           (unsigned long)LLM_TUNE_MAX_SESSION_ROUNDS,
           LLM_TUNE_SPEED_HARD_MAX_MPS);
    ControlRuntime_Init();
  }

void RobotApp_Process(void)
{
      uint32_t now = HAL_GetTick();
      ControlRuntime_Begin(now);
      if (HostUartTx_ConsumeFault()) { host_uart_tx_error++; host_uart_fault_pending = 1U; }
      HostLink_ProcessUartFault(now);
      RpiBinary_Process();
      RpiBinary_TxProcess();
      if (HostUartTx_Busy() || (!rpi_binary_active && !rpi_binary_tx_dma_active))
          HostUartTx_Process(HAL_GetTick());
      Host_ProcessCommand();
      Motor_ProcessFeedback();
      Mecanum_ProcessFeedback(HAL_GetTick());

      /* 命令处理和串口中断可能更新时间戳，超时判断前必须刷新当前时间。 */
      now = HAL_GetTick();
      HostLink_ProcessWait(now);
      ChassisSafety_Process(now);
      RpiBinary_ProcessSessionTimeout(now);
      Pose_ProcessControl(now);

      /* 控制和超时检查优先；反馈查询与遥测安排在本轮末尾。 */
      if (current_robot_mode == ROBOT_MODE_TUNE) {
          LLM_TunerProcess(now);
      }

      if (debug_motor_active && (int32_t)(now - debug_motor_stop_tick) >= 0)
      {
          Mecanum_ReportCanTxResult(
              ZDT_Emm_StopMask((uint8_t)(1U << (debug_motor_id - 1U))));
          printf("# MOTOR AUTO STOP ID=%u\r\n", debug_motor_id);
          debug_motor_active = 0U;
      }

      if (debug_chassis_active && (int32_t)(now - debug_chassis_stop_tick) >= 0)
      {
          (void)StopAllMotors();
          printf("# MOVE AUTO STOP\r\n");
          debug_chassis_active = 0U;
      }

      if (ops_monitor_enabled && (uint32_t)(now - ops_monitor_last_tick) >= 1000U)
      {
          ops_monitor_last_tick = now;
          Ops_PrintStatus();
      }

      ZDT_CAN_Process(HAL_GetTick());
      if (ZDT_CAN_RecoverWhenIdle(
              ChassisSafety_GetActiveMotion() == CHASSIS_MOTION_NONE &&
              ZDT_CAN_StopSent(Mecanum_GetRequiredMotorMask()))) {

          printf("# CAN RECOVERED MOTION=STOPPED MASK=0x%02X\r\n",
                 Mecanum_GetRequiredMotorMask());
      }
      /* Idle refresh covers late motor power-up; never replay old motion. */
      {
          static uint32_t idle_refresh_tick;
          if (ChassisSafety_GetActiveMotion() == CHASSIS_MOTION_NONE &&
              (uint32_t)(now - idle_refresh_tick) >= 1000U) {
              idle_refresh_tick = now;
              if (ZDT_CAN_StopSent(Mecanum_GetRequiredMotorMask())) ZDT_Emm_RefreshEnables();
              (void)StopAllMotors();
          }
      }
      if (active_host_link != HOST_LINK_NONE) {
          Telemetry_Process(now);
      }

      ControlRuntime_End(HAL_GetTick());
      HAL_Delay(1);

}

int RobotApp_Write(char *ptr, int len)
{
    if (rpi_binary_active) return len;
    int result = HostUartTx_Write(ptr, len);
    if (!rpi_binary_tx_dma_active) HostUartTx_Process(HAL_GetTick());
    return result;
}

void RobotApp_HostRxComplete(void)
{
    if (HostRx_Feed(&host_rx, pc_rx_byte, rpi_binary_active) &&
        rpi_binary_armed && active_host_link == HOST_LINK_RPI)
        last_host_command_tick = HAL_GetTick();
    HAL_UART_Receive_IT(&huart1, &pc_rx_byte, 1U);
}

void RobotApp_HostError(void)
{
    /* 仅锁存；主循环在下一轮立即停车并清理会话。 */
    host_uart_fault_pending = 1U;
    HostRx_ResetText(&host_rx);
    rpi_binary_active = 0U;
    rpi_binary_armed = 0U;
    HostRx_ResetFrames(&host_rx);
    App_ResetHostParser();
    HAL_UART_Receive_IT(&huart1, &pc_rx_byte, 1U);
}

void RobotApp_HostTxComplete(void)
{
    if (HostUartTx_Busy()) HostUartTx_Complete();
    else {
        rpi_binary_tx_dma_active = 0U;
        rpi_binary_tx_loaded_ready = 0U;
    }
    host_uart_tx_ok++;
}

void RobotApp_Can2Rx(void)
{
    DM_G6220_RxFIFO0_Handler(&g6220_motor);
}
