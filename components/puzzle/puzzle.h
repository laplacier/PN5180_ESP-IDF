#ifndef puzzle
#define puzzle

typedef enum {
    SET_STATE,
    SEND_STATE
} puzzle_task_action_t;

// FreeRTOS task priorities
#define PUZZLE_TASK_PRIO                7  // Puzzle_Task
#define GENERIC_TASK_PRIO               1  // Any unspecified task

void puzzle_init(void);
void send_States(uint8_t target_id);
#endif