#include "link.h"

// Entry point: only starts the components (ADR 018).
extern "C" void app_main()
{
    link_start();
}
