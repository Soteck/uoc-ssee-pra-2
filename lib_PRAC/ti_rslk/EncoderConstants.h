
#define WHEEL_DIAMETER_MM           ( 60.0f )
#define DISTANCIA_ENTRE_EJES_MM     ( 150 )
#define MATH_PI                     ( 3.14159f )

typedef enum {
    ENCODER_NONE = 0,
    ENCODER_LEFT = 1,
    ENCODER_RIGHT = 2
} encoder_e;


typedef uint32_t encoder_counts_t;


typedef enum {
    ENCODER_DIR_NONE = 0,
    ENCODER_DIR_FORWARD = 1,
    ENCODER_DIR_BACKWARD = 2,
} encoder_dir_e;


