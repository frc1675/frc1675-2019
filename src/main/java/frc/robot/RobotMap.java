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
  public static final int LEFT_FRONT = 1;
  public static final int LEFT_MIDDLE = 2;
  public static final int LEFT_BACK = 3;

  public static final int RIGHT_FRONT = 4;
  public static final int RIGHT_MIDDLE = 5;
  public static final int RIGHT_BACK = 6;

  //Grabber
  public static final int ARM_ROTATOR = 7;
  
  public static final int HATCH_RELEASER_1 = 8;
  public static final int HATCH_RELEASER_2 = 9;

  //Elevator
  public static final int ELEVATOR = 10;

  public static final int TILT_ELEVATOR_FORWARD = 11;
  public static final int TILT_ELEVATOR_REVERSE = 12;

  public static final int LIMIT_SWITCH = 13;

  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;
}
