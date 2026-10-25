#include "session.h"

#include <optional>

#include <esp_log.h>
#include <rmw_microros/rmw_microros.h>

#include "config.h"
#include "controller.h"
#include "message_conversion.h"

static const char *TAG = "session";

// QoS of every topic of the link (ADR 025): the next message always carries the full state, so
// a lost one is not retransmitted, and only the latest is kept.
static rmw_qos_profile_t link_qos()
{
    rmw_qos_profile_t qos = rmw_qos_profile_default;
    qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
    qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    qos.depth = 1;
    return qos;
}

// Logs a failed creation step and returns whether it succeeded.
static bool succeeded(rcl_ret_t ret, const char *step)
{
    if (ret != RCL_RET_OK)
    {
        ESP_LOGE(TAG, "%s failed: %d", step, static_cast<int>(ret));
        return false;
    }
    return true;
}

Session::Session(Controller &controller) : controller_(controller)
{
    // Stops at the first failure; the destructor destroys whatever was created.
    ready_ = create_support() && create_node() && create_command_subscription() &&
             create_state_publisher() && create_executor();
}

Session::~Session()
{
    if (support_ok_)
    {
        // A session ends when the agent is lost: destroy without waiting for acknowledgements
        // that will never come.
        rmw_uros_set_context_entity_destroy_session_timeout(
            rcl_context_get_rmw_context(&support_.context), 0
        );
    }
    destroy_executor();
    destroy_state_publisher();
    destroy_command_subscription();
    destroy_node();
    destroy_support();
}

void Session::receive_commands()
{
    constexpr uint64_t no_wait_ns = 0;
    const rcl_ret_t ret = rclc_executor_spin_some(&executor_, no_wait_ns);
    // RCL_RET_TIMEOUT only means that no command arrived.
    if (ret != RCL_RET_OK && ret != RCL_RET_TIMEOUT)
    {
        ESP_LOGD(TAG, "spin failed: %d", static_cast<int>(ret));
    }
}

void Session::publish_state()
{
    const std::optional<State> state = controller_.latest_state();
    if (!state)
    {
        return;
    }
    state_message_ = to_message(*state);
    // Best effort: a lost state is superseded by the next one, 10 ms later.
    const rcl_ret_t ret = rcl_publish(&state_publisher_, &state_message_, nullptr);
    if (ret != RCL_RET_OK)
    {
        ESP_LOGD(TAG, "state publish failed: %d", static_cast<int>(ret));
    }
}

bool Session::create_support()
{
    const rcl_ret_t ret = rclc_support_init(&support_, 0, nullptr, &allocator_);
    support_ok_ = succeeded(ret, "support init");
    return support_ok_;
}

bool Session::create_node()
{
    const rcl_ret_t ret = rclc_node_init_default(&node_, NODE_NAME, "", &support_);
    node_ok_ = succeeded(ret, "node init");
    return node_ok_;
}

bool Session::create_command_subscription()
{
    const rmw_qos_profile_t qos = link_qos();
    const rcl_ret_t ret = rclc_subscription_init(
        &command_subscription_, &node_, command_type_support(), COMMAND_TOPIC, &qos
    );
    subscription_ok_ = succeeded(ret, "command subscription init");
    return subscription_ok_;
}

bool Session::create_state_publisher()
{
    const rmw_qos_profile_t qos = link_qos();
    const rcl_ret_t ret = rclc_publisher_init(
        &state_publisher_, &node_, state_type_support(), STATE_TOPIC, &qos
    );
    publisher_ok_ = succeeded(ret, "state publisher init");
    return publisher_ok_;
}

bool Session::create_executor()
{
    constexpr size_t handles = 1;  // the command subscription
    const rcl_ret_t init_ret = rclc_executor_init(
        &executor_, &support_.context, handles, &allocator_
    );
    executor_ok_ = succeeded(init_ret, "executor init");
    if (!executor_ok_)
    {
        return false;
    }
    const rcl_ret_t handler_ret = rclc_executor_add_subscription_with_context(
        &executor_,
        &command_subscription_,
        &command_message_,
        &Session::on_command,
        this,
        ON_NEW_DATA
    );
    return succeeded(handler_ret, "command handler registration");
}

// The destroy functions run when the agent is gone, so errors are expected; they are only logged
// at debug level. Each call stays outside the log macro, which is compiled out below the debug
// level together with its arguments; the result is [[maybe_unused]] for the same reason.

void Session::destroy_executor()
{
    if (executor_ok_)
    {
        [[maybe_unused]] const rcl_ret_t ret = rclc_executor_fini(&executor_);
        ESP_LOGD(TAG, "executor fini: %d", static_cast<int>(ret));
    }
}

void Session::destroy_state_publisher()
{
    if (publisher_ok_)
    {
        [[maybe_unused]] const rcl_ret_t ret = rcl_publisher_fini(&state_publisher_, &node_);
        ESP_LOGD(TAG, "publisher fini: %d", static_cast<int>(ret));
    }
}

void Session::destroy_command_subscription()
{
    if (subscription_ok_)
    {
        [[maybe_unused]] const rcl_ret_t ret = rcl_subscription_fini(
            &command_subscription_, &node_
        );
        ESP_LOGD(TAG, "subscription fini: %d", static_cast<int>(ret));
    }
}

void Session::destroy_node()
{
    if (node_ok_)
    {
        [[maybe_unused]] const rcl_ret_t ret = rcl_node_fini(&node_);
        ESP_LOGD(TAG, "node fini: %d", static_cast<int>(ret));
    }
}

void Session::destroy_support()
{
    if (support_ok_)
    {
        [[maybe_unused]] const rcl_ret_t ret = rclc_support_fini(&support_);
        ESP_LOGD(TAG, "support fini: %d", static_cast<int>(ret));
    }
}

// The signature is fixed by rclc, so the two pointers cannot be made distinct types.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
void Session::on_command(const void *message, void *session)
{
    const auto &command_message = *static_cast<const CommandMessage *>(message);
    const std::optional<Command> command = to_command(command_message);
    if (!command)
    {
        ESP_LOGW(TAG, "unknown behavior %u, command dropped", command_message.behavior);
        return;
    }
    static_cast<Session *>(session)->controller_.submit_command(*command);
}
