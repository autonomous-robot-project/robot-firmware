#include "session.h"

#include <esp_log.h>
#include <rmw_microros/rmw_microros.h>

#include "config.h"

static const char *TAG = "session";

Session::Session()
{
    support_ok_ = rclc_support_init(&support_, 0, nullptr, &allocator_) == RCL_RET_OK;
    if (!support_ok_)
    {
        ESP_LOGE(TAG, "support init failed");
        return;
    }
    node_ok_ = rclc_node_init_default(&node_, NODE_NAME, "", &support_) == RCL_RET_OK;
    if (!node_ok_)
    {
        ESP_LOGE(TAG, "node init failed");
    }
}

Session::~Session()
{
    if (!support_ok_)
    {
        return;
    }
    // A session ends when the agent is lost: destroy without waiting for acknowledgements that
    // will never come. Errors are expected here.
    rmw_uros_set_context_entity_destroy_session_timeout(
        rcl_context_get_rmw_context(&support_.context), 0
    );
    if (node_ok_)
    {
        const rcl_ret_t node_ret = rcl_node_fini(&node_);
        ESP_LOGD(TAG, "node fini: %d", static_cast<int>(node_ret));
    }
    const rcl_ret_t support_ret = rclc_support_fini(&support_);
    ESP_LOGD(TAG, "support fini: %d", static_cast<int>(support_ret));
}
