/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  //Drive base
  public static final int LEFT_FRONT = 7;
  public static final int LEFT_MIDDLE = 5;
  public static final int LEFT_BACK = 6;

  public static final int RIGHT_FRONT = 1;
  public static final int RIGHT_MIDDLE = 3;
  public static final int RIGHT_BACK = 2;

  public static final double MOTOR_DEADZONE = .0;
  public static final double TURNING_SCALE_FACTOR = 2;
  
  //Grabber
  public static final int ARM_ROTATOR = 6;
  
  public static final int HATCH_RELEASER = 2;

  public static final int HOOK = 7;

  //Elevator components
  public static final int ELEVATOR_MOTOR = 4;
  public static final int TILT_ELEVATOR_FORWARD = 0;
  public static final int TILT_ELEVATOR_REVERSE = 1;

  public static final int LOWER_LIMIT_SWITCH = 9;
  public static final int UPPER_LIMIT_SWITCH = 8;
  
  //Elevator positions
  public static final int MAX_POSITION = 4000;
  public static final int MIN_POSITION = 0;
  public static final int SAFE_TILT_POSITION = 300;
  public static final int BOTTOM_HATCH_POSITION = 200;
  public static final int MIDDLE_HATCH_POSITION = 1500;
  public static final int TOP_HATCH_POSITION = 3500;

  public static final double ELEVATOR_P = 0.0016;
  public static final double ELEVATOR_I = 0;
  public static final double ELEVATOR_D = 0;
  public static final double ELEVATOR_TOLERANCE = 100;
  public static final double HOLD_POWER = 0.15;

  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;

}
