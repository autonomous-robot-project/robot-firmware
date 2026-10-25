#include "controller.h"
#include "link.h"

// Components live for the whole run, in static storage: nothing is allocated after boot.
static Controller controller;
static Link link_to_sequencer{controller};

// Entry point: only starts the components (ADR 018).
extern "C" void app_main()
{
    controller.start();
    link_to_sequencer.start();
}
