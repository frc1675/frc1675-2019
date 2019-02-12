/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;


import edu.wpi.first.wpilibj.command.PIDCommand;
import frc.robot.Robot;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class TurnForAngle extends PIDCommand {
  
  int count = 0;
  double angleturned;
  double timeout;
  double initialDegrees;
  double setpoint;
  
  public TurnForAngle(double angleturned, double timeout) {
    super (RobotMap.Gyro_P, RobotMap.Gyro_I, RobotMap.Gyro_D);
    requires(Robot.driveBase);
    this.angleturned = angleturned;
    this.timeout = timeout;
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    this.getPIDController().reset();
    this.getPIDController().setOutputRange(-.5, .5);
    initialDegrees = Robot.driveBase.getAngle();
    setpoint = initialDegrees + angleturned;
    this.getPIDController().setSetpoint(setpoint);
    SmartDashboard.putNumber("Setpoint", setpoint);
    this.setTimeout(timeout);
    this.getPIDController().enable();
  //  initialDegrees = Robot.driveBase.getAngle();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
  @Override
  protected double returnPIDInput(){
    return Robot.driveBase.getAngle();
  }
  @Override
  protected void usePIDOutput(double output){
    Robot.driveBase.setRightMotors(-output);
    Robot.driveBase.setLeftMotors(output);
  }
  
}
