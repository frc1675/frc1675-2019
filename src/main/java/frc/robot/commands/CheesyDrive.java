/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;


/**
 * CheesyDrive is an algorithm for turning joystick inputs into commands for the drivebase.
 *  */
public class CheesyDrive extends Command {
  public CheesyDrive() {
    requires(Robot.driveBase);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double turnPower = Robot.m_oi.getDriverRightXAxis();
    double forwardPower = Robot.m_oi.getDriverLeftYAxis();
    
    double leftPower = forwardPower + turnPower;
    double rightPower = forwardPower - turnPower;

    if (Math.abs(leftPower) > 1.0 || Math.abs(rightPower) > 1.0) {
      double Scaler = Math.max( Math.abs(rightPower),Math.abs(leftPower));
      rightPower = rightPower / Scaler;
      leftPower = leftPower / Scaler;
    } 

    

    Robot.driveBase.setLeftMotors(leftPower);
    Robot.driveBase.setRightMotors(rightPower);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.driveBase.setLeftMotors(0);
    Robot.driveBase.setRightMotors(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
