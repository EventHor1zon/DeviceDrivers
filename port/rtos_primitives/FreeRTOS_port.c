/**
 *    @file    FreeRTOS_port.c
 *
 *    @brief   The FreeRTOS implentation of the port file
 *
 *    @author    RJAM
 *    @created   Wed 30 Jun 00:44:40 BST 2026
 */

/** Includes **/
#include "FreeRTOS.h"
#include "include/port_rtos.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
/** Private Data **/

/** Function Prototypes **/

/** Static Functions **/

/** Tasks **/

/** Public Functions **/

status_t port_lock_resource(locktype_t lock, uint32_t timeout)
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

status_t port_task_notify(tasktype_t task, const uint32_t notify_value, const uint32_t notify_type)
{
    return xTaskNotify(task, notify_value, notify_type);
}

status_t port_task_dispatch_message(
    tasktype_t task,
    const void *const msg,
    uint32_t *const length,
    const uint32_t timeout)
{
    return STATUS_NOT_IMPLEMENTED;
}

tasktype_t port_task_create(
    task_prototype_t task,
    const char *taskname,
    const uint32_t stacksize,
    void *args,
    uint32_t priority,
    const uint32_t *task_memory,
    const void *task_control)
{
    return xTaskCreateStatic(task, taskname, stacksize, args, priority, task_memory, task_control);
}