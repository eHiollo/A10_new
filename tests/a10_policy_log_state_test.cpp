#include "a10_policy_log_state.hpp"

#include <cassert>
#include <iostream>

int main()
{
    a10_tcp::PolicyLogState state;
    assert(!state.snapshot().running);
    const auto first = state.prepare(true);
    assert(!state.snapshot().running); // prepare/queued plan is not running yet
    state.start(first);
    const auto first_snapshot = state.snapshot();
    assert(first_snapshot.running && first_snapshot.enabled);
    state.stop();
    assert(!state.snapshot().running && !state.snapshot().enabled);

    const auto second = state.prepare(false);
    state.start(second);
    assert(state.snapshot().session != first_snapshot.session);
    assert(state.snapshot().running && !state.snapshot().enabled);
    state.finish(first); // delayed cleanup must not affect the next policy
    assert(state.snapshot().running);
    state.finish(second);
    assert(!state.snapshot().running);

    const auto third = state.prepare(true);
    state.start(third);
    assert(state.snapshot().enabled);
    state.finish(third); // non-stop exit, e.g. controller error
    assert(!state.snapshot().running && !state.snapshot().enabled);
    std::cout << "policy log lifecycle passed\n";
}
