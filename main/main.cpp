#include "link.h"

// Components live for the whole run, in static storage: nothing is allocated after boot.
static Link link_to_sequencer;

// Entry point: only starts the components (ADR 018).
extern "C" void app_main()
{
    link_to_sequencer.start();
}
