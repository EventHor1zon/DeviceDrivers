
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "port/error_type.h"
#include "port/malloc.h"
#include "TestComponent.h"
#include "port/log.h"


#ifdef CONFIG_USE_PERIPH_MANAGER
#include "CommandAPI.h"

const parameter_t test_param_map[0] = {};

static TaskHandle_t th = NULL;
static QueueHandle_t q = NULL;

const peripheral_t test_periph_template = {
    .handle = NULL,
    .param_len = 0,
    .params = test_param_map,
    .peripheral_name = "Test component",
    .peripheral_id = 0,
    .periph_type = PTYPE_NONE,
};
#endif

static void testcomponent_driver_task(void *args) {

    log_info("TC", "Task running");
 
    msg_t msg;

    while(1) {
        if(xQueueReceive(q, &msg, pdMS_TO_TICKS(1000)) == pdPASS) {
            log_info("Received message %u\n", msg.id);
        
            log_info("Device uid = %u\n", msg.handle->uid);
        }
        else {
            log_info("No msgs\n");
        }
    }
   /** here be dragons **/
}


TC_h tc_init(testcomponent_init_t *init) {

    testcomponent_t *tc = NULL;

    tc = (testcomponent_t *)heap_caps_calloc(1, sizeof(testcomponent_t), MALLOC_CAP_8BIT);

    if(tc == NULL) {
        log_error("testcomponent", "Error alocating memory for handle!");
        return NULL;
    }
    else {
        tc->wait = init->wait;
        tc->uid = init->uid;
    }

    if(th == NULL && xTaskCreate(testcomponent_driver_task, "testcomponent_driver_task", 2048, tc, 3, &th) != pdTRUE) {
        log_error("testcomponent", "Error creating task!");
        return NULL;
    }

    if(q == NULL) {
        q = xQueueCreate(sizeof(msg_t), 10);
        if(q == NULL) {
            log_error("testcomponent", "Error creating queue!");
            return NULL;
        }
    }

    return tc;
}


void notify_task(testcomponent_t *t) {

    msg_t msg = {
        .handle = t,
        .id = 10,
    };

    xQueueSend(q, &msg, pdMS_TO_TICKS(100));

}