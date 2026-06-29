/**
 *    @file    port_rtos.c
 *
 *    @brief    header file for port_rtos
 *
 *
 *
 *    @author    RJAM
 *    @created   Fri 12 Jun 00:44:40 BST 2026
 */

/** Includes **/
#include "include/port_rtos.h"
/** Private Data **/

/** Function Prototypes **/

/** Static Functions **/

/** Tasks **/

/** Public Functions **/

status_t __attribute__((weak)) port_lock_resource(locktype_t lock, uint32_t timeout)
{
    return STATUS_NOT_IMPLEMENTED;
}

status_t __attribute__((weak)) port_unlock_resource(locktype_t lock)
{
    return STATUS_NOT_IMPLEMENTED;
}

status_t __attribute__((weak))
port_queue_get(queuetype_t queue, uint8_t *const buffer, uint32_t timeout)
{
    return STATUS_NOT_IMPLEMENTED;
}

status_t __attribute__((weak))
port_queue_put(queuetype_t queue, const uint8_t *const buffer, uint32_t timeout)
{
    return STATUS_NOT_IMPLEMENTED;
}

status_t __attribute__((weak)) port_queue_get_len(queuetype_t queue, uint32_t *const len)
{
    return STATUS_NOT_IMPLEMENTED;
}

status_t __attribute__((weak))
port_task_notify(tasktype_t task, const uint32_t notify_value, const uint32_t notify_type)
{
    return STATUS_NOT_IMPLEMENTED;
}

/** END **/
