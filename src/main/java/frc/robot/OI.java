/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  Joystick driverController = new Joystick(XBoxControllerMap.DRIVER_CONTROLLER_PORT);
  Joystick operatorController = new Joystick(XBoxControllerMap.OPERATOR_CONTROLLER_PORT);

  //Driver controller buttons
  JoystickButton driverAButton = new JoystickButton(driverController, XBoxControllerMap.A_BUTTON);
  JoystickButton driverBButton = new JoystickButton(driverController, XBoxControllerMap.B_BUTTON);
  JoystickButton driverYButton = new JoystickButton(driverController, XBoxControllerMap.Y_BUTTON);
  JoystickButton driverXButton = new JoystickButton(driverController, XBoxControllerMap.X_BUTTON);

  //Driver controller bumpers
  JoystickButton driverLeftBumper = new JoystickButton(driverController, XBoxControllerMap.LEFT_BUMPER_BUTTON);  
  JoystickButton driverRightBumper = new JoystickButton(driverController, XBoxControllerMap.RIGHT_BUMPER_BUTTON);
  
  //Operator controller buttons
  JoystickButton operatorAButton = new JoystickButton(operatorController, XBoxControllerMap.A_BUTTON); 
  JoystickButton operatorBButton = new JoystickButton(operatorController, XBoxControllerMap.B_BUTTON);
  JoystickButton operatorYButton = new JoystickButton(operatorController, XBoxControllerMap.Y_BUTTON); 
  JoystickButton operatorXButton = new JoystickButton(operatorController, XBoxControllerMap.X_BUTTON); 

  //Operator controller bumpers
  JoystickButton operatorLeftBumper = new JoystickButton(operatorController, XBoxControllerMap.LEFT_BUMPER_BUTTON);
  JoystickButton operatorRightBumper = new JoystickButton(operatorController, XBoxControllerMap.RIGHT_BUMPER_BUTTON);

  //Driver controller joysticks
  public double getDriverLeftYAxis(){
    double value = -driverController.getRawAxis(XBoxControllerMap.LEFT_Y_AXIS);
    return value;
  }
  public double getDriverLeftXAxis(){
    double value = driverController.getRawAxis(XBoxControllerMap.LEFT_X_AXIS);
    return value;
  }
  public double getDriverRightYAxis(){
    double value = -driverController.getRawAxis(XBoxControllerMap.RIGHT_Y_AXIS);
    return value;
    }
  public double getDriverRightXAxis(){
    double value = driverController.getRawAxis(XBoxControllerMap.RIGHT_X_AXIS);
    return value;
  }

  //Operator controller joysticks
  public double getOperatorLeftYAxis(){
    return -operatorController.getRawAxis(XBoxControllerMap.LEFT_Y_AXIS);
  } 
  public double getOperatorLeftXAxis(){
    return operatorController.getRawAxis(XBoxControllerMap.LEFT_X_AXIS);
  }
  public double getOperatorRightYAxis(){
    return -operatorController.getRawAxis(XBoxControllerMap.RIGHT_Y_AXIS);
  }
  public double getOperatorRightXAxis(){
    return operatorController.getRawAxis(XBoxControllerMap.RIGHT_X_AXIS);
  }
  

  

  // Button button = new JoystickButton(stick, buttonNumber);

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());

  //jerry is here now k thanks
  //test
}
