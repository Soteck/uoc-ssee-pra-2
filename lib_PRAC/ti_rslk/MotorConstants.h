/*----------------------------------------------------------------------------*/


#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

/*----------------------------------------------------------------------------*/

typedef enum {
    MOTOR_NONE = 0,
    MOTOR_LEFT = 1,
    MOTOR_RIGHT = 2,
    MOTOR_BOTH = 3
} motor_e;

typedef enum {
    MOTOR_DIR_NONE = 0,
    MOTOR_DIR_FORWARD = 1,
    MOTOR_DIR_BACKWARD = 2
} motor_dir_e;
