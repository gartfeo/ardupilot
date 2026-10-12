/*
  AP_Scheduler::loops_completed32(): a main loop counts as completed only
  once all of its tasks have run. The HIL simulator backend
  (libraries/SITL/SIM_HIL.cpp) sends the outputs once per completed loop,
  so it must never see the counter advance while a loop's tasks still
  run, unlike ticks32(), which advances before them.
 */
#include <AP_gtest.h>

#include <AP_Scheduler/AP_Scheduler.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// records what the scheduler's counters read while a task runs
class TaskRecorder {
public:
    void record(void);

    uint32_t calls;
    uint32_t loops_seen;
    uint32_t ticks_seen;
};

static AP_Scheduler scheduler;
static TaskRecorder recorder;

void TaskRecorder::record(void)
{
    calls++;
    loops_seen = scheduler.loops_completed32();
    ticks_seen = scheduler.ticks32();
}

// one fast task: it runs in every loop, whatever the time budget
static const AP_Scheduler::Task tasks[] = {
    FAST_TASK_CLASS(TaskRecorder, &recorder, record),
};

// time budget passed to run(), one 400 Hz loop period
static constexpr uint32_t TIME_AVAILABLE_US = 2500;
// loops run by the test
static constexpr uint32_t LOOPS = 3;

TEST(AP_Scheduler, loops_completed_advances_only_after_the_tasks)
{
    scheduler.init(tasks, ARRAY_SIZE(tasks), UINT32_MAX);
    const uint32_t loops0 = scheduler.loops_completed32();
    const uint32_t ticks0 = scheduler.ticks32();

    for (uint32_t loop = 1; loop <= LOOPS; loop++) {
        // a loop starts with a tick: the tick counter already counts it
        scheduler.tick();
        EXPECT_EQ(scheduler.ticks32(), ticks0 + loop);
        EXPECT_EQ(scheduler.loops_completed32(), loops0 + loop - 1);

        scheduler.run(TIME_AVAILABLE_US);
        ASSERT_EQ(recorder.calls, loop);
        // while the loop's task ran, the loop did not count as completed
        EXPECT_EQ(recorder.ticks_seen, ticks0 + loop);
        EXPECT_EQ(recorder.loops_seen, loops0 + loop - 1);
        // once every task has run, it does
        EXPECT_EQ(scheduler.loops_completed32(), loops0 + loop);
    }
}

AP_GTEST_MAIN()
