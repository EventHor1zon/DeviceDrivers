/**
 *    @file    port_rtos.h
 *
 *    @brief    header file for port_rtos
 *
 *
 *
 *    @author    RJAM
 *    @created   Fri 12 Jun 00:44:51 BST 2026
 */

#ifndef PORT_RTOS_H
#define PORT_RTOS_H

/** Includes **/
#include "port/error_types.h"
#include "stdint.h"

/** Defines **/

/** Typedefs **/

typedef void *locktype_t;
typedef void *queuetype_t;
typedef void *tasktype_t;
typedef void (*task_prototype_t)(void *);

/** Function Declarations **/

/** @brief lock resource - take the semaphore
 *  @param lock [in] void pointer to lock handle
 *  @param timeout [in] time to wait for operation
 *  @return status_t status
 */
status_t port_lock_resource(locktype_t lock, uint32_t timeout);

/** @brief unlock resource - give the semaphore
 *  @param lock [in] void pointer to lock handle
 *  @return status_t status
 */
status_t port_unlock_resource(locktype_t lock);

/** @brief get an item from the queue
 *         the user is responsible for correct item sizing
 *  @param queue [in] void pointer to queue handle
 *  @param buffer [in] pointer to data buffer
 *  @param timeout [in] time to wait for operation
 *  @return status_t status
 */
status_t port_queue_get(queuetype_t queue, uint8_t *const buffer, uint32_t timeout);

/** @brief put an item into the queue
 *         the user is responsible for correct item sizing
 *  @param queue [in] void pointer to queue handle
 *  @param buffer [in] pointer to data
 *  @param timeout [in] time to wait for operation
 *  @return status_t status
 */
status_t port_queue_put(queuetype_t queue, const uint8_t *const buffer, uint32_t timeout);

/** @brief get the current queue length
 *  @param queue [in] void pointer to queue handle
 *  @param len [out] length of the queue - 0 if error
 *  @return status_t status
 */
status_t port_queue_get_len(queuetype_t queue, uint32_t *const len);

/** @brief send a notification to a task
 *  @param task [in] void pointer to task handle
 *  @param notify_value [in] notification value to send
 *  @param notify_type [in] type of notification to send
 *  @return status_t status
 */
status_t port_task_notify(tasktype_t task, const uint32_t notify_value, const uint32_t notify_type);

tasktype_t port_task_create(
    task_prototype_t task,
    const char *taskname,
    const uint32_t stacksize,
    void *args,
    uint32_t priority,
    const uint32_t *task_memory,
    const void *task_control);

/** END **/
#endif /** PORT_RTOS_H **/
