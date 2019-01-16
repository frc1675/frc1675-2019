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
 * This is the Chessey drive comand. It is a command assigning the left Joystick to backwards and forwards and the left to forward and backwards.
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
    double turnpower = Robot.m_oi.getRightXAxis();
    double forwardPower = Robot.m_oi.getLeftYAxis();
    double rightPower = forwardPower - turnpower;
    double leftPower = forwardPower + turnpower;
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
