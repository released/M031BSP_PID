# M031BSP_PID

update @ 2019/04/29

Add PID sample code for M031

- use random API to generate input data for PID calculate (check USE_RANDOM_SEED_GENERATE_DATA)

- use timer (1ms) to calculate each PID calculate spend time (check USE_TIMER_CALCULATE_TIMING)

- add log to check each PID calculate process (check PID_LOG_APPROACH_PROCESS)

- add log to check PID calculate times and spend time(check PID_LOG_RESULT_SPEND_TIME)