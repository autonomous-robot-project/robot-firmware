#include "edge_reporter.h"

#include <cinttypes>
#include <cstdio>
#include <optional>

void EdgeReporter::report_new_edge()
{
    const std::optional<EncoderEdges> edges = encoder_.latest_edges();
    const bool new_edge = edges && edges->count != reported_count_;
    if (!new_edge)
    {
        return;
    }
    const uint32_t missed_edges = edges->count - reported_count_ - 1;
    if (missed_edges > 0)
    {
        std::printf("# %c: missed %" PRIu32 " edges\n", wheel_, missed_edges);
    }
    std::printf("edge,%lld,%c,%" PRIu32 "\n", edges->last.count(), wheel_, edges->count);
    reported_count_ = edges->count;
}
