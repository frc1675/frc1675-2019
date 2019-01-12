package frc.robot;

public class XBoxControllerMap {

    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;

    // left analog stick
    public static final int LEFT_X_AXIS = 0;
    public static final int LEFT_Y_AXIS = 1;

    // right analog stick
    public static final int RIGHT_X_AXIS = 4;
    public static final int RIGHT_Y_AXIS = 5;

    // Trigger axis
    // Starts at 0. Left trigger increases value, right trigger decreases value.
    // NOTE that this means you cannot "detect" both triggers being pulled at
    // any degree.
    public static final int LEFT_TRIGGER_AXIS = 2;
    public static final int RIGHT_TRIGGER_AXIS = 3;

    // Face Buttons
    public static final int A_BUTTON = 1;
    public static final int B_BUTTON = 2;
    public static final int X_BUTTON = 3;
    public static final int Y_BUTTON = 4;

    // Bumpers (shoulders)
    public static final int LEFT_BUMPER_BUTTON = 5;
    public static final int RIGHT_BUMPER_BUTTON = 6;

    // other buttons
    public static final int BACK_BUTTON = 7;
    public static final int START_BUTTON = 8;

    // Joystick buttons (clicking them in)
    public static final int LEFT_JOYSTICK_BUTTON = 9;
    public static final int RIGHT_JOYSTICK_BUTTON = 10;

    public static final double DEAD_ZONE = .1675;

}
