#pragma once

#include <cstdint>
#include <optional>
#include <type_traits>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

// Latest-value channel between contexts (ADR 017): a FreeRTOS queue of one element, statically
// allocated inside the object. Last-value semantics, one producer, and no operation ever blocks.
template <typename T> class LatestValueChannel
{
        // The queue copies the bytes of the value.
        static_assert(std::is_trivially_copyable_v<T>);

    public:
        LatestValueChannel()
        {
            constexpr UBaseType_t length = 1;
            handle_ = xQueueCreateStatic(length, sizeof(T), storage_, &queue_);
            configASSERT(handle_ != nullptr);
        }

        // Not copyable nor movable: the queue lives inside the object.
        LatestValueChannel(const LatestValueChannel &) = delete;
        LatestValueChannel &operator=(const LatestValueChannel &) = delete;

        // Producer: replaces the value in the channel.
        void write(const T &value) { xQueueOverwrite(handle_, &value); }

        // Producer in an interrupt: replaces the value in the channel.
        void write_from_isr(const T &value)
        {
            BaseType_t higher_priority_task_woken = pdFALSE;
            xQueueOverwriteFromISR(handle_, &value, &higher_priority_task_woken);
            portYIELD_FROM_ISR(higher_priority_task_woken);
        }

        // Consumer: the latest value, which stays in the channel; empty if never written.
        [[nodiscard]] std::optional<T> read() const
        {
            constexpr TickType_t no_wait = 0;
            T value;
            if (xQueuePeek(handle_, &value, no_wait) != pdTRUE)
            {
                return std::nullopt;
            }
            return value;
        }

    private:
        StaticQueue_t queue_;
        uint8_t storage_[sizeof(T)];
        QueueHandle_t handle_;
};
