/* CAN1 command transport: one in-flight frame, latest speed per wheel.
 * TXOK is bus delivery evidence, never motor execution/zero-speed feedback. */
#include "zdtCan.h"
#include <string.h>

extern CAN_HandleTypeDef hcan1;
#define QUERY_QUEUE_SIZE 8U
#define TX_TIMEOUT_MS 50U
#define RETRY_MS 100U
#define RESTART_MS 500U
#define NOTIFY_MASK (CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY)
#define BLOCKING_ERRORS (HAL_CAN_ERROR_TIMEOUT | HAL_CAN_ERROR_NOT_INITIALIZED | \
    HAL_CAN_ERROR_NOT_READY | HAL_CAN_ERROR_NOT_STARTED | HAL_CAN_ERROR_INTERNAL)

typedef struct {
    uint32_t id, tick;
    uint8_t bytes[8], length, pending;
} Frame;
static Frame speeds[4], stops[4], enables[4], queries[QUERY_QUEUE_SIZE], flight;
static uint8_t query_head, query_tail, query_count, stop_required, stop_sent;
static uint8_t cursor, flight_kind, flight_motor, aborting, restart_phase;
static uint32_t flight_mailbox, flight_tick, retry_at[4], enable_at[4], restart_tick;
static uint8_t retry_mask, enable_delay_mask;
static volatile uint8_t flight_done, fault_event;
static ZDT_CAN_RxCallback_t rx_callback;
static ZDT_CAN_Stats_t stats;


enum { FRAME_SPEED, FRAME_STOP, FRAME_ENABLE, FRAME_QUERY };

static void DropMotion(void)
{
    uint8_t i;
    for (i = 0U; i < 4U; ++i) {
        stats.dropped += speeds[i].pending;
        speeds[i].pending = 0U;
    }
}
static void LatchFault(void)
{
    if (!stats.tx_fault) {
        stats.tx_fault = 1U;
        stats.fault_generation++;
        uint8_t i;
        stop_sent = 0U;
        for (i = 0U; i < 4U; ++i)
            if ((stop_required & (1U << i)) &&
                !(flight_mailbox && flight_kind == FRAME_STOP && flight_motor == i))
                stops[i].pending = 1U;
    }
    fault_event = 1U;
    stats.recovery_phase = 1U;
    DropMotion();
}
static uint8_t Store(Frame *f, uint32_t id, uint8_t *data, uint8_t len)
{
    stats.replaced += f->pending;
    memset(f, 0, sizeof(*f)); /* HAL reads all eight data bytes, even for DLC < 8. */
    f->id = id; f->tick = HAL_GetTick(); f->length = len; f->pending = 1U;
    memcpy(f->bytes, data, len);
    stats.enqueued++;
    return 0U;
}
static uint8_t Valid(uint32_t id, uint8_t *data, uint8_t len)
{
    return data && len > 0U && len <= 8U && (id & 0xFFU) == 0U &&
           (id >> 8) >= 1U && (id >> 8) <= 4U;
}
void ZDT_CAN_ConfigFilter(void)
{
    CAN_FilterTypeDef f = {0};
    f.FilterBank = 0U; f.FilterMode = CAN_FILTERMODE_IDMASK;
    f.FilterScale = CAN_FILTERSCALE_32BIT;
    f.FilterIdLow = 4U; f.FilterMaskIdLow = 6U; /* extended DATA only */
    f.FilterFIFOAssignment = CAN_RX_FIFO0; f.FilterActivation = ENABLE;
    f.SlaveStartFilterBank = 14U;
    if (HAL_CAN_ConfigFilter(&hcan1, &f) != HAL_OK ||
        HAL_CAN_Start(&hcan1) != HAL_OK ||
        HAL_CAN_ActivateNotification(&hcan1, NOTIFY_MASK) != HAL_OK) Error_Handler();
}
void ZDT_CAN_RegisterCallback(ZDT_CAN_RxCallback_t callback) { rx_callback = callback; }
uint8_t ZDT_CAN_Send_ExtId(uint32_t id, uint8_t *data, uint8_t len)
{
    uint8_t motor;
    if (!Valid(id, data, len)) return 3U;
    motor = (uint8_t)((id >> 8) - 1U);
    if (data[0] == 0xF6U) {
        if (stats.tx_fault) return 4U;
        return Store(&speeds[motor], id, data, len);
    }
    if (data[0] == 0xF3U) return Store(&enables[motor], id, data, len);
    if (query_count == QUERY_QUEUE_SIZE) { stats.dropped++; return 1U; }
    Store(&queries[query_head], id, data, len);
    query_head = (uint8_t)((query_head + 1U) % QUERY_QUEUE_SIZE); query_count++;
    return 0U;
}
void ZDT_CAN_BeginStop(void)
{
    DropMotion();
    query_head = query_tail = query_count = 0U;
    /* Preserve an already requested stop; repeated STOP cannot erase TXOK evidence
     * or repeatedly abort its own safety frames. A motion frame must be cancelled. */
    if (flight_mailbox && flight_kind == FRAME_SPEED && !aborting) {
        aborting = 1U;
        (void)HAL_CAN_AbortTxRequest(&hcan1, flight_mailbox);
    }
    if (!ZDT_CAN_StopPending()) { stop_required = stop_sent = 0U; retry_mask = 0U; }
}
uint8_t ZDT_CAN_SendStop(uint32_t id, uint8_t *data, uint8_t len)
{
    uint8_t motor, bit;
    if (!Valid(id, data, len)) return 3U;
    motor = (uint8_t)((id >> 8) - 1U); bit = (uint8_t)(1U << motor);
    stop_required |= bit;
    if ((stop_sent & bit) || (flight_mailbox && flight_kind == FRAME_STOP && flight_motor == motor)) return 0U;
    return Store(&stops[motor], id, data, len);
}
uint8_t ZDT_CAN_StopSent(uint8_t mask)
{
    return mask && (stop_sent & mask) == mask;
}
uint8_t ZDT_CAN_StopSentMask(void) { return stop_sent; }
uint8_t ZDT_CAN_StopPending(void)
{
    return (stop_required & stop_sent) != stop_required;
}
static void FinishFlight(uint32_t now)
{
    uint8_t bit = (uint8_t)(1U << flight_motor);
    uint8_t success = flight_done == 1U && !aborting;
    if (success) {
        if (flight_kind == FRAME_STOP) stop_sent |= bit;
        if (flight_kind == FRAME_ENABLE) {
            enable_at[flight_motor] = now; enable_delay_mask |= bit;
        }
    } else if (flight_kind == FRAME_STOP || flight_kind == FRAME_ENABLE) {
        Frame *slot = flight_kind == FRAME_STOP ? &stops[flight_motor] : &enables[flight_motor];
        /* A newer enable/disable request supersedes the failed old one. */
        if (!slot->pending) { *slot = flight; slot->pending = 1U; }
        retry_at[flight_motor] = now; retry_mask |= bit;
        LatchFault();
    } else if (flight_kind == FRAME_SPEED && !aborting) LatchFault();
    flight_mailbox = 0U; flight_done = aborting = 0U;
}
/* Only a stuck abort/HAL state needs software restart. ABOM handles bus-off.
 * Each HAL step runs in the main loop with interrupts enabled. CAN2 is untouched. */
static uint8_t Restart(uint32_t now)
{
    HAL_StatusTypeDef result;
    if (hcan1.Instance->ESR & CAN_ESR_BOFF) return 0U;
    if (!restart_phase) {
        if ((uint32_t)(now - restart_tick) < RESTART_MS) return 0U;
        restart_tick = now; restart_phase = 1U;
    }
    stats.error_latched |= HAL_CAN_GetError(&hcan1);
    if (restart_phase == 1U) {
        result = HAL_CAN_GetState(&hcan1) == HAL_CAN_STATE_LISTENING
                 ? HAL_CAN_Stop(&hcan1) : HAL_CAN_Init(&hcan1);
        if (result == HAL_OK) restart_phase = 2U;
    } else {
        if (restart_phase == 3U && (uint32_t)(now - restart_tick) < RESTART_MS) return 0U;
        /* Stop/Start alone does not clear bxCAN transmit requests. Do not
         * declare the old frame gone until hardware confirms all mailboxes free. */
        (void)HAL_CAN_AbortTxRequest(&hcan1,
            CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
        if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 3U) {
            restart_phase = 3U; restart_tick = now; LatchFault(); return 1U;
        }
        result = HAL_CAN_GetState(&hcan1) == HAL_CAN_STATE_LISTENING
                 ? HAL_OK : HAL_CAN_Start(&hcan1);
        if (result == HAL_OK) result = HAL_CAN_ActivateNotification(&hcan1, NOTIFY_MASK);
        if (result == HAL_OK) {
            /* Abort old mailboxes before restart; never replay an old nonzero target. */
            flight_done = 0U;
            if (flight_mailbox) { aborting = 1U; FinishFlight(now); }
            (void)HAL_CAN_ResetError(&hcan1);
            restart_phase = 0U; stats.stall_recoveries++;
        }
    }
    if (result != HAL_OK) {
        stats.error_latched |= HAL_CAN_GetError(&hcan1);
        restart_phase = 0U; LatchFault();
    }
    return 1U;
}
void ZDT_CAN_Process(uint32_t now)
{
    Frame *slot = NULL;
    uint8_t kind = FRAME_QUERY, motor = 0U, i;
    CAN_TxHeaderTypeDef header = {0};
    uint32_t saved;
    if (hcan1.Instance->ESR & CAN_ESR_BOFF) {
        LatchFault();
        /* ABOM must never retransmit the pre-fault nonzero target on recovery. */
        if (flight_mailbox && !aborting) {
            aborting = 1U;
            (void)HAL_CAN_AbortTxRequest(&hcan1, flight_mailbox);
        }
        return;
    }
    if (restart_phase) { (void)Restart(now); return; }
    if (flight_mailbox) {
        if (!HAL_CAN_IsTxMessagePending(&hcan1, flight_mailbox)) FinishFlight(now);
        else if ((uint32_t)(now - flight_tick) >= TX_TIMEOUT_MS) {
            if (!aborting) {
                stats.tx_timeout++; stats.last_tx_result = 4U; LatchFault();
                aborting = 1U;
                (void)HAL_CAN_AbortTxRequest(&hcan1, flight_mailbox);
            }
            if ((uint32_t)(now - flight_tick) >= RESTART_MS) (void)Restart(now);
            return;
        } else return;
    }
    if (!ZDT_CAN_HardwareReady()) { LatchFault(); (void)Restart(now); return; }
    /* Round robin avoids one absent motor starving the other three stops. */
    for (i = 0U; i < 4U; ++i) {
        uint8_t j = (uint8_t)((cursor + i) % 4U), bit = (uint8_t)(1U << j);
        if ((retry_mask & bit) && (uint32_t)(now - retry_at[j]) < RETRY_MS) continue;
        retry_mask &= (uint8_t)~bit;
        if (stops[j].pending) { slot = &stops[j]; kind = FRAME_STOP; motor = j; break; }
    }
    /* Wait until all STOP frames completed before enabling or moving. */
    if (!slot && ZDT_CAN_StopPending()) return;
    if (!slot) for (i = 0U; i < 4U; ++i) {
        uint8_t j = (uint8_t)((cursor + i) % 4U), bit = (uint8_t)(1U << j);
        if (enables[j].pending && (!(retry_mask & bit) || (uint32_t)(now - retry_at[j]) >= RETRY_MS)) {
            slot = &enables[j]; kind = FRAME_ENABLE; motor = j; break;
        }
    }
    if (!slot && !stats.tx_fault) for (i = 0U; i < 4U; ++i) {
        uint8_t j = (uint8_t)((cursor + i) % 4U), bit = (uint8_t)(1U << j);
        if (!speeds[j].pending || enables[j].pending) continue;
        if ((uint32_t)(now - speeds[j].tick) >= TX_TIMEOUT_MS) { stats.dropped++; LatchFault(); return; }
        if ((enable_delay_mask & bit) && (uint32_t)(now - enable_at[j]) < 5U) continue;
        enable_delay_mask &= (uint8_t)~bit;
        slot = &speeds[j]; kind = FRAME_SPEED; motor = j; break;
    }
    if (!slot && query_count) { slot = &queries[query_tail]; motor = (uint8_t)((slot->id >> 8) - 1U); }
    if (!slot || !HAL_CAN_GetTxMailboxesFreeLevel(&hcan1)) return;
    if ((uint32_t)(now - slot->tick) > stats.max_wait_ms) stats.max_wait_ms = now - slot->tick;
    header.ExtId = slot->id; header.IDE = CAN_ID_EXT; header.RTR = CAN_RTR_DATA; header.DLC = slot->length;
    /* Publish ownership atomically with mailbox submission: completion can interrupt immediately. */
    saved = __get_PRIMASK(); __disable_irq();
    flight = *slot; flight_kind = kind; flight_motor = motor; flight_done = 0U; aborting = 0U;
    if (HAL_CAN_AddTxMessage(&hcan1, &header, flight.bytes, &flight_mailbox) != HAL_OK) {
        flight_mailbox = 0U; __set_PRIMASK(saved);
        stats.tx_error++; stats.last_tx_result = 2U;
        retry_at[motor] = now; retry_mask |= (uint8_t)(1U << motor);
        if (kind != FRAME_QUERY) LatchFault();
        else { slot->pending = 0U; query_tail = (uint8_t)((query_tail + 1U) % QUERY_QUEUE_SIZE); query_count--; }
        return;
    }
    flight_tick = now; slot->pending = 0U; stats.tx_queued++;
    if (kind == FRAME_QUERY) { query_tail = (uint8_t)((query_tail + 1U) % QUERY_QUEUE_SIZE); query_count--; }
    cursor = (uint8_t)((motor + 1U) % 4U);
    __set_PRIMASK(saved);
}
uint8_t ZDT_CAN_HardwareReady(void)
{
    return HAL_CAN_GetState(&hcan1) == HAL_CAN_STATE_LISTENING &&
           !(HAL_CAN_GetError(&hcan1) & BLOCKING_ERRORS) &&
           !(hcan1.Instance->ESR & CAN_ESR_BOFF) && !restart_phase;
}
uint8_t ZDT_CAN_IsReady(void) { return ZDT_CAN_HardwareReady() && !stats.tx_fault; }
uint8_t ZDT_CAN_HasFault(void) { return stats.tx_fault; }
void ZDT_CAN_RaiseFault(void) { LatchFault(); }
uint8_t ZDT_CAN_ConsumeFault(void) { uint8_t value = fault_event; fault_event = 0U; return value; }
uint8_t ZDT_CAN_RecoverWhenIdle(uint8_t eligible)
{
    if (!eligible || !stats.tx_fault || !ZDT_CAN_HardwareReady() || flight_mailbox ||
        !stop_required || !ZDT_CAN_StopSent(stop_required)) return 0U;
    stats.error_latched |= HAL_CAN_GetError(&hcan1);
    if (HAL_CAN_ResetError(&hcan1) != HAL_OK) return 0U;
    stats.tx_fault = 0U; stats.recovery_phase = 0U; stats.recoveries++;
    return 1U;
}
static void Complete(CAN_HandleTypeDef *hcan, uint32_t mailbox, uint8_t success)
{
    if (hcan != &hcan1) return;
    if (success) { stats.tx_ok++; stats.last_tx_result = 0U; }
    else stats.tx_aborted++;
    if (flight_mailbox == mailbox) flight_done = success ? 1U : 2U;
}
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *h) { Complete(h, CAN_TX_MAILBOX0, 1U); }
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *h) { Complete(h, CAN_TX_MAILBOX1, 1U); }
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *h) { Complete(h, CAN_TX_MAILBOX2, 1U); }
void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *h) { Complete(h, CAN_TX_MAILBOX0, 0U); }
void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef *h) { Complete(h, CAN_TX_MAILBOX1, 0U); }
void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef *h) { Complete(h, CAN_TX_MAILBOX2, 0U); }
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *h)
{
    if (h != &hcan1) return;
    stats.error_callbacks++; stats.error_latched |= HAL_CAN_GetError(h);
    /* Main-loop timeout/current hardware state decides faults; no ISR queue mutation. */
}
void ZDT_CAN_RxFIFO0_Handler(CAN_HandleTypeDef *h)
{
    uint8_t budget = 3U, data[8];
    CAN_RxHeaderTypeDef header;
    if (h != &hcan1) return;
    while (budget-- && HAL_CAN_GetRxFifoFillLevel(h, CAN_RX_FIFO0)) {
        if (HAL_CAN_GetRxMessage(h, CAN_RX_FIFO0, &header, data) != HAL_OK) break;
        stats.rx_count++;
        if (rx_callback && header.IDE == CAN_ID_EXT && header.RTR == CAN_RTR_DATA && header.DLC <= 8U)
            rx_callback(header.ExtId, data, (uint8_t)header.DLC);
    }
}
void ZDT_CAN_GetStats(ZDT_CAN_Stats_t *output)
{
    uint32_t saved;
    if (!output) return;
    saved = __get_PRIMASK(); __disable_irq();
    *output = stats; output->esr = hcan1.Instance->ESR; output->tsr = hcan1.Instance->TSR;
    __set_PRIMASK(saved);
}
#ifdef CONTROL_HOST_TEST
void ZDT_CAN_TestResetFault(void)
{
    memset(speeds, 0, sizeof(speeds)); memset(stops, 0, sizeof(stops));
    memset(enables, 0, sizeof(enables)); memset(queries, 0, sizeof(queries));
    memset(&stats, 0, sizeof(stats));
    query_head = query_tail = query_count = stop_required = stop_sent = cursor = 0U;
    flight_mailbox = 0U; flight_done = aborting = restart_phase = fault_event = 0U;
    retry_mask = enable_delay_mask = 0U; restart_tick = 0U;
}
#endif
