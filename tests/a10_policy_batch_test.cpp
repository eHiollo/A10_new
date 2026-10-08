#include "a10_policy_batch.hpp"

#include <cassert>
#include <future>
#include <iostream>
#include <limits>
#include <thread>

using a10_tcp::PolicyBatchQueue;

PolicyBatchQueue::Actions actions(std::size_t count, double value)
{
    return PolicyBatchQueue::Actions(count, std::vector<double>(7, value));
}

void test_replacement_during_execution()
{
    PolicyBatchQueue queue;
    const auto old_seq = queue.replace(actions(3, 1.0), 0.0);
    std::vector<double> goal;
    assert(queue.peek(goal, old_seq));
    std::promise<void> replaced;
    std::thread producer([&]() {
        queue.replace(actions(4, 2.0), 0.0);
        replaced.set_value();
    });
    replaced.get_future().wait();
    assert(!queue.pop(old_seq));
    assert(!queue.peek(goal, old_seq));
    assert(queue.align(old_seq, 1.0 / 30.0, nullptr) == -1);
    auto status = queue.status();
    assert(status.remaining == 4);
    assert(queue.peek(goal, status.sequence));
    assert(goal[0] == 2.0);
    assert(queue.pop(status.sequence));
    assert(queue.status().remaining == 3);
    producer.join();
    queue.clear();
    assert(!queue.pop(status.sequence));
    assert(queue.status().remaining == 0);
}

void test_expiration_and_receive_delay()
{
    PolicyBatchQueue queue;
    const auto received = PolicyBatchQueue::Clock::now();
    auto sequence = queue.replace(actions(30, 0.0), 0.1, received);
    double age = 0.0;
    assert(queue.align(sequence, 1.0 / 30.0, &age, received + std::chrono::milliseconds(100)) == 6);
    assert(std::abs(age - 0.2) < 1e-9);
    assert(queue.status().remaining == 24);
    assert(queue.align(sequence, 1.0 / 30.0, &age, received + std::chrono::milliseconds(200)) == 6);
    assert(queue.status().remaining == 24);
    sequence = queue.replace(actions(30, 0.0), 1.0, received);
    assert(queue.align(sequence, 1.0 / 30.0, &age, received) == 30);
    assert(queue.status().expired);
    assert(queue.status().remaining == 0);
    sequence = queue.replace(actions(10, 0.0), 31.5, received);
    assert(queue.align(sequence, 1.0 / 30.0, &age, received) == 10);
    assert(age == 31.5);
    assert(queue.status().expired);
    sequence = queue.replace(actions(1, 0.0), 0.04, received);
    assert(queue.align(sequence, 1.0 / 30.0, nullptr, received) == 1);
    sequence = queue.replace(actions(1, 0.0), 1e100, received);
    assert(queue.align(sequence, 1.0 / 30.0, nullptr, received) == 1);
}

void test_invalid_batches_preserve_queue()
{
    PolicyBatchQueue queue;
    const auto sequence = queue.replace(actions(3, 1.0), 0.0);
    auto rejected = [&](PolicyBatchQueue::Actions batch, double age) {
        bool caught = false;
        try { queue.replace(std::move(batch), age); }
        catch (const std::invalid_argument&) { caught = true; }
        assert(caught);
        assert(queue.status().sequence == sequence);
        assert(queue.status().remaining == 3);
    };
    rejected(actions(1, 0.0), -1.0);
    rejected(actions(1, 0.0), std::numeric_limits<double>::infinity());
    rejected(actions(1, std::numeric_limits<double>::quiet_NaN()), 0.0);
    rejected({}, 0.0);
    rejected({std::vector<double>(6, 0.0)}, 0.0);
}

void test_remaining_execution_time_and_guarded_handoff()
{
    PolicyBatchQueue queue;
    const auto now = PolicyBatchQueue::Clock::now();
    std::vector<double> start(7, 0.0);
    std::vector<double> first(7, 0.0);
    first[0] = 0.01;
    auto second = first;
    second[6] = 8.0;
    const auto sequence = queue.replace({first, second}, 0.0, now);
    assert(!queue.update_remaining_time(sequence, start, 0.05, 80.0));
    assert(queue.align(sequence, 1.0 / 30.0, nullptr, now) == 0);
    assert(queue.update_remaining_time(sequence, start, 0.05, 80.0));
    const double expected = (a10_tcp::policy_blend_steps(start, first, 0.05, 80.0)
        + a10_tcp::policy_blend_steps(first, second, 0.05, 80.0)) * 0.002;
    assert(std::abs(queue.status().remaining_time_s - expected) < 0.01);
    bool rejected = false;
    try { queue.replace(actions(3, 0.0), 0.0, now, sequence); }
    catch (const std::invalid_argument&) { rejected = true; }
    assert(rejected);
    assert(queue.pop(sequence));
    assert(queue.pop(sequence));
    assert(queue.status().remaining_time_s == 0.0);
    const auto next_sequence = queue.replace(actions(3, 0.0), 0.0, now, sequence);
    assert(next_sequence == sequence + 1);
    assert(!queue.update_remaining_time(sequence, start, 0.05, 80.0));
    queue.clear();
    rejected = false;
    try { queue.replace(actions(3, 0.0), 0.0, now, next_sequence); }
    catch (const std::invalid_argument&) { rejected = true; }
    assert(rejected);
    assert(queue.status().remaining == 0);
    assert(a10_tcp::policy_blend_steps(start, std::vector<double>(7, 1e100), 0.05, 80.0) == std::numeric_limits<int>::max());
}

int main()
{
    test_replacement_during_execution();
    test_expiration_and_receive_delay();
    test_invalid_batches_preserve_queue();
    test_remaining_execution_time_and_guarded_handoff();
    std::cout << "policy batch tests passed\n";
}
