/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;
import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class DriveForTime extends Command {
  
  double time = 0;
  double power = 0;

  public DriveForTime(double time, double power){
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveBasePID);
    this.time = 1;
    this.power = 0.5;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.driveBasePID.setLeftMotors(power);
    Robot.driveBasePID.setRightMotors(power);
    setTimeout(time);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return isTimedOut();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.driveBasePID.setLeftMotors(0);
    Robot.driveBasePID.setRightMotors(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
