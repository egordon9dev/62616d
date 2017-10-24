#include "main.h"
#include "pid.h"
#include "setup.h"
/*
 * The purpose of this function is solely to set the default pin modes (pinMode()) and port
 * states (digitalWrite()) of limit switches, push buttons, and solenoids. It can also safely
 * configure a UART port (usartOpen()) but cannot set up an LCD (lcdInit()).
 */
void initializeIO()
{
    pinMode(MGL_LIM, INPUT);
}

#define kButtonNone 0
#define kButtonLeft LCD_BTN_LEFT
#define kButtonCenter LCD_BTN_CENTER
#define kButtonRight LCD_BTN_RIGHT

// default auton
//autonSelect = 1;

// Little macro to keep code cleaner, masks both disable/enable and auton/driver
#define vexCompetitionState ((isEnabled() ? 0 : 2) + (isAutonomous() ? 1 : 0))

int getLcdButtons()
{
    int competitionState = vexCompetitionState;
    int buttons;

    // This function will block until either
    // 1. A button is pressed on the LCD
    //    If a button is pressed when the function starts then that button
    //    must be released before a new button is detected.
    // 2. Robot competition state changes

    // Wait for all buttons to be released
    while (lcdReadButtons(uart1) != kButtonNone)
    {
        // check competition state, bail if it changes
        if (vexCompetitionState != competitionState)
            return (kButtonNone);
        taskDelay(10);
    }

    // block until an LCD button is pressed
    do
    {
        // we use a copy of the lcd buttons to avoid their state changing
        // between the test and returning the status
        buttons = lcdReadButtons(uart1);

        // check competition state, bail if it changes
        if (vexCompetitionState != competitionState)
            return (kButtonNone);

        taskDelay(10);
    } while (buttons == kButtonNone);

    return (buttons);
}

/*-----------------------------------------------------------------------------*/
/*  Display autonomous selection                                               */
/*-----------------------------------------------------------------------------*/
void LcdSetAutonomous(int value)
{
    if (value == 1)
    {
        lcdSetText(uart1, 1, "Match");
        lcdSetText(uart1, 2, "[1]     2     X ");
    }
    if (value == 2)
    {
        lcdSetText(uart1, 1, "Skills");
        lcdSetText(uart1, 2, " 1     [2]    X ");
    }
    if (value == 3)
    {
        lcdSetText(uart1, 1, "NOTHING");
        lcdSetText(uart1, 2, " 1      2    [X]");
    }

    // Save autonomous mode for later
    autonSelect = value;
}

/*-----------------------------------------------------------------------------*/
/*  Select one of three autonomous choices                                     */
/*-----------------------------------------------------------------------------*/

void LcdAutonomousSelection()
{
    int button;

    // Clear LCD and turn on backlight
    lcdClear(uart1);
    lcdSetBacklight(uart1, true);

    // default choice
    LcdSetAutonomous(1);
    // PROS seems to need a delay
    taskDelay(2000);

    while (!isEnabled())
    {
        // this function blocks until button is pressed
        button = getLcdButtons();

        // Display and select the autonomous routine
        if (button == kButtonLeft)
            LcdSetAutonomous(1);

        if (button == kButtonCenter)
            LcdSetAutonomous(2);

        if (button == kButtonRight)
            LcdSetAutonomous(3);

        // Don't hog the cpu !
        taskDelay(20);
    }

    lcdSetText(uart1, 1, "PID PID MOOSE");
    lcdSetText(uart1, 2, "DUCK DUCK GOOSE");
}

/*
 * Runs user initialization code. This function will be started in its own task with the default
 * priority and stack size once when the robot is starting up. It is possible that the VEXnet
 * communication link may not be fully established at this time, so reading from the VEX
 * Joystick may fail.
 *
 * This function should initialize most sensors (gyro, encoders, ultrasonics), LCDs, global
 * variables, and IMEs.
 *
 * This function must exit relatively promptly, or the operatorControl() and autonomous() tasks
 * will not start. An autonomous mode selection menu like the pre_auton() in other environments
 * can be implemented in this task if desired.
 */
void initialize()
{
    setupEnc();

    lcdInit(uart1);
    LcdAutonomousSelection();
}
