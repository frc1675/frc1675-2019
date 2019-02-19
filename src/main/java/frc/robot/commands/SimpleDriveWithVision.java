/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class SimpleDriveWithVision extends Command {
  double endpoint = 0;
  double tolerance = 1;
  double power = .5;
  int count = 0;
  public SimpleDriveWithVision() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.driveBase);
    requires(Robot.vision);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if(Robot.vision.HasTarget()){
    Robot.driveBase.setAllMotors(power);
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  public boolean onTarget(){
    if(Robot.vision.getYOffset() <= endpoint+tolerance && Robot.vision.getYOffset() >= endpoint-tolerance){
      count++;
    }
    else{
      count = 0;
    }
    if(count == 5){
      return true;
    }
    return false;
  }
  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return onTarget();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.driveBase.setAllMotors(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
