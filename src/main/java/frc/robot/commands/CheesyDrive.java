/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * CheesyDrive is an algorithm for turning joystick inputs into commands for the
 * drivebase.
 */
public class CheesyDrive extends Command {

  public CheesyDrive() {
    requires(Robot.driveBasePID);
    // Robot.driveBasePID.activateVisionPIDMode();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double turnPower = Robot.oi.getDriverRightXAxis();
    double forwardPower = Robot.oi.getDriverLeftYAxis();

    turnPower = Math.signum(turnPower) * Math.pow(turnPower, RobotMap.TURNING_SCALE_FACTOR);

    double leftPower = forwardPower + turnPower;
    double rightPower = forwardPower - turnPower;

    if (Math.abs(leftPower) > 1.0 || Math.abs(rightPower) > 1.0) {
      double Scaler = Math.max(Math.abs(rightPower), Math.abs(leftPower));
      rightPower = rightPower / Scaler;
      leftPower = leftPower / Scaler;
    }
    // SmartDashboard.putBoolean("IsPIDEnabled", isPIDEnabled);
    Robot.driveBasePID.setLeftMotors(leftPower);
    Robot.driveBasePID.setRightMotors(rightPower);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
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
