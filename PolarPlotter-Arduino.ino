/*
 * PolarPlotter-Arduino
 *
 * Copyright 2018 by Thomas Buck <xythobuz@xythobuz.de>
 *
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * <xythobuz@xythobuz.de> wrote this file.  As long as you retain this notice
 * you can do whatever you want with this stuff. If we meet some day, and you
 * think this stuff is worth it, you can buy me a beer in return.   Thomas Buck
 * ----------------------------------------------------------------------------
 */

/*
 * This sketch is designed to control a modification of the popular
 * Polar-Plotter projects. These consist of a piece of plywood-board,
 * with a standard NEMA17 stepper motor in each of the two top corners.
 * They are connected with a belt to the carriage, transporting four
 * micro-servos that control the movement of four pens or other drawing devices.
 * In this case, it's designed to use paintbrushes that are dipped into an
 * ink reservoir at the corner of the board between strokes.
 * 
 * The serial control interface replicates the classic G-Code used by CNC machines
 * or 3D printers. But in our case, only a very limited number of commands are supported:
 * 
 *  Move to coordinates:
 *  - G0 Xnn.n Ynn.n Znn
 *  - G1 Xnn.n Ynn.n Znn
 *  
 *  Move current tool to ink-reservoir:
 *  - G6
 *  
 *  Set current position to zero / Home:
 *  - G28
 *  
 *  Select Tool (also sets all servos to home)
 *  - Tn
 *  
 * X and Y are the coordinates on the paper.
 * Z is the tool pressure. 0 is home pos, 1 - 10 is on paper with diff. strength.
 * 
 * Response is single line with "OK" or "ERROR" if ready for next command.
 * Everything else is status info for human operator.
 */

// https://www.arduino.cc/en/Reference/Servo
#include <Servo.h>

// http://www.airspayce.com/mikem/arduino/AccelStepper/
#include <MultiStepper.h>
#include <AccelStepper.h>

// ---------------------------------------------------------------------------

// Measured from center of stepper motors, in mm
#define MACHINE_WIDTH 800.0f
#define MACHINE_HEIGHT 600.0f

#define MOTOR_STEPS_PER_REV 200.0f // 1.8deg / rev
#define BELT_MM_PER_REV (2.0f * 16.0f) // GT2 2mm, 16tooth pulley

#define MAX_FEEDRATE 1000.0f // mm/s

#define STEPS_PER_MM (((float)MOTOR_STEPS_PER_REV) / ((float)BELT_MM_PER_REV))
#define MAX_SPEED (((float)MAX_FEEDRATE) * ((float)STEPS_PER_MM))

// all in degree (0 - 180)
#define SERVO_HOME_POS 20
#define SERVO_DRAW_POS_MIN 50
#define SERVO_DRAW_POS_MAX 110
#define SERVO_REFILL_POS 160

// X-Y coordinates of refill (for first brush)
#define REFILL_COORDINATES 10.0f, 100.0f

// Distance between ink reservoirs
#define REFILL_OFFSET 50.0f

// in ms
#define SERVO_MOVEMENT_DELAY 250

#define MOTOR_COUNT 2
#define SERVO_COUNT 4

const static float tool_offset_x[SERVO_COUNT] = {
    0.0f, 10.0f, 0.0f, 10.0f
};
const static float tool_offset_y[SERVO_COUNT] = {
    0.0f, 0.0f, 10.0f, 10.0f
};

const static uint8_t motor_step_pins[MOTOR_COUNT] = { 4, 6 };
const static uint8_t motor_dir_pins[MOTOR_COUNT] = { 5, 7 };

const static uint8_t servo_pins[SERVO_COUNT] = { 8, 9, 10, 11 };

#define LINE_BUFF_LEN 100

#define BRUSH_DIP_COUNT 3

// ---------------------------------------------------------------------------

static AccelStepper *stepper[MOTOR_COUNT];
static MultiStepper steppers;

static Servo servos[SERVO_COUNT];

struct Point {
    float x;
    float y;
    Point(float _x, float _y) : x(_x), y(_x) { }
};

static char line_buff[LINE_BUFF_LEN] = "";
static uint8_t current_tool = 0;
static Point *ink_reservoirs[SERVO_COUNT];
static Point current_pos(0.0f, 0.0f);

// ---------------------------------------------------------------------------

// set servo[id] to pos and all others to home position
static void set_servos(uint8_t id, uint8_t pos) {

    /*
     * TODO
     * Does the Arduino Servo library really work like this?
     * No regular calling needed?!?!
     */
  
    for (int i = 0; i < SERVO_COUNT; i++) {
        if (i == id) {
            servos[i].write(pos);
        } else {
            servos[i].write(SERVO_HOME_POS);
        }
    }

    // wait for servos to reach position
    delay(SERVO_MOVEMENT_DELAY);
}

static void move_to_z(uint8_t z) {
    // z can be 0 to 10 (and others for dip pos)
    if (z == 0) {
        set_servos(current_tool, SERVO_HOME_POS);
    } else if (z <= 10) {
        uint16_t off = (SERVO_DRAW_POS_MAX - SERVO_DRAW_POS_MIN) * z / 10;
        set_servos(current_tool, SERVO_DRAW_POS_MIN + off);
    } else {
        set_servos(current_tool, SERVO_REFILL_POS);
    }
}

// ---------------------------------------------------------------------------

static Point xy_to_polar(Point p) {
    // our coordinate system begins at bottom-left
    p.y = MACHINE_HEIGHT - p.y;

    // calculate radius for first motor, left
    float a = sqrt((p.x * p.x) + (p.y * p.y));

    // right motor is on the other side of x-axis
    p.x = MACHINE_WIDTH - p.x;

    // calculate radius for second motor, right
    float b = sqrt((p.x * p.x) + (p.y * p.y));
    
    return Point(a, b);
}

static inline long mm_to_steps(float mm) {
    return (long)(mm * STEPS_PER_MM);
}

static void move_to_xy(float x, float y) {
    Point polar = xy_to_polar(Point(x - tool_offset_x[current_tool], y - tool_offset_y[current_tool]));

    long absolute[MOTOR_COUNT];
    for (int i = 0; i < MOTOR_COUNT; i++) {
        absolute[i] = stepper[i]->currentPosition();
        absolute[i] += mm_to_steps((i == 0) ? polar.x : polar.y);
    }

    steppers.moveTo(absolute);
    steppers.runSpeedToPosition();
}

// ---------------------------------------------------------------------------

static bool string_begins_with(const char * a, const char * b) {
    int i;
    for (i = 0; (i < strlen(a)) && (i < strlen(b)); i++) {
        if (a[i] != b[i]) {
            return false;
        }
    }

    // a only begins with b if a didn't end before b!
    return ((i - 1) <= strlen(a)) ? true : false;
}

static inline float string_to_float(const char * s) {
    return String(s).toFloat();
}

static inline int string_to_int(const char * s) {
    return String(s).toInt();
}

void setup() {
    Serial.begin(115200);
    Serial.write("Initializing PolarPlotter-Arduino\n");

    Serial.write("Dimensions: ");
    Serial.print(MACHINE_WIDTH);
    Serial.write("x");
    Serial.print(MACHINE_HEIGHT);
    Serial.print("mm\n");

    Serial.write("Maximum Feedrate: ");
    Serial.print(MAX_FEEDRATE);
    Serial.write("mm/s\n");

    Serial.write("Steps per mm: ");
    Serial.print(STEPS_PER_MM);
    Serial.write("\n");

    Serial.write("Ink reservoirs:\n");
    Point ink(REFILL_COORDINATES);
    for (int i = 0; i < SERVO_COUNT; i++) {
        ink_reservoirs[i] = new Point(ink.x + (i * REFILL_OFFSET), ink.y);
        Serial.write("  x=");
        Serial.print(ink_reservoirs[i]->x);
        Serial.write(" y=");
        Serial.print(ink_reservoirs[i]->y);
        Serial.write("\n");
    }

    for (int i = 0; i < MOTOR_COUNT; i++) {
        stepper[i] = new AccelStepper(AccelStepper::DRIVER, motor_step_pins[i], motor_dir_pins[i]);
        stepper[i]->setMaxSpeed(MAX_SPEED);
        steppers.addStepper(*stepper[i]);
    }

    for (int i = 0; i < SERVO_COUNT; i++) {
        servos[i].attach(servo_pins[i]);
        servos[i].write(SERVO_HOME_POS);
    }
    delay(SERVO_MOVEMENT_DELAY); // wait for servos to reach home position

    Serial.write("OK\n");
}

void loop() {
    size_t len = Serial.readBytesUntil('\n', line_buff, LINE_BUFF_LEN - 1);
    line_buff[len] = '\0';

    // line_buff now contains a valid c-string with our command
    if (string_begins_with(line_buff, "G0 ") || string_begins_with(line_buff, "G1 ")) {
        Point pos = current_pos;
        char *p = strtok(line_buff + 3, " ");
        while (p != NULL) {
            if (string_begins_with(p, "X")) {
                pos.x = string_to_float(p + 1);
                if ((pos.x < 0) || (pos.x > MACHINE_WIDTH)) {
                    Serial.write("ERROR\n");
                    Serial.write("Invalid parameter: \"");
                    Serial.write(p);
                    Serial.write("\"\n");
                    return;
                }
            } else if (string_begins_with(p, "Y")) {
                pos.y = string_to_float(p + 1);
                if ((pos.y < 0) || (pos.y > MACHINE_HEIGHT)) {
                    Serial.write("ERROR\n");
                    Serial.write("Invalid parameter: \"");
                    Serial.write(p);
                    Serial.write("\"\n");
                    return;
                }
            } else if (string_begins_with(p, "Z")) {
                int n = string_to_int(p + 1);
                if ((n >= 0) && (n <= 11)) {
                    move_to_z(n);
                } else {
                    Serial.write("ERROR\n");
                    Serial.write("Invalid parameter: \"");
                    Serial.write(p);
                    Serial.write("\"\n");
                    return;
                }
            } else {
                Serial.write("ERROR\n");
                Serial.write("Invalid parameter: \"");
                Serial.write(p);
                Serial.write("\"\n");
                return;
            }
            p = strtok(NULL, " ");
        }
        if ((pos.x != current_pos.x) || (pos.y != current_pos.y)) {
            move_to_xy(pos.x, pos.y);
        }
    } else if (strcmp(line_buff, "G6") == 0) {
        set_servos(0, SERVO_HOME_POS); // all to home_pos
        Point p = current_pos;
        move_to_xy(ink_reservoirs[current_tool]->x, ink_reservoirs[current_tool]->y);
        for (int i = 0; i < BRUSH_DIP_COUNT; i++) {
            move_to_z(11); // dip brush
            move_to_z(0); // remove brush
        }
        move_to_xy(p.x, p.y); // back to starting position
    } else if (string_begins_with(line_buff, "T")) {
        current_tool = line_buff[1] - '0';
        if (current_tool >= SERVO_COUNT) {
            Serial.write("ERROR\n");
            Serial.write("Invalid tool: ");
            Serial.print(current_tool);
            Serial.write("\n");
            current_tool = 0; // invalid: fall-back to zero
        }
        set_servos(0, SERVO_HOME_POS); // all to home_pos
    } else if (strcmp(line_buff, "G28") == 0) {
        current_pos.x = 0.0f;
        current_pos.y = 0.0f;

        // reset absolute motor steps to 0
        for (int i = 0; i < MOTOR_COUNT; i++) {
            stepper[i]->setCurrentPosition(0);
        }
        Serial.write("OK\n");
    } else {
        Serial.write("ERROR\n");
        Serial.write("Unknown command: \"");
        Serial.write(line_buff);
        Serial.write("\"\n");
    }
}

