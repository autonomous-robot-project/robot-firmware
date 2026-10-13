#include "uros_task.h"
#include "uros_transport.h"

extern "C" void app_main(void)
{
    uros_transport_register();
    uros_task_start();
}
