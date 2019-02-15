/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;


import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.PIDCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;

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
    Robot.driveBase.resetGyro(); 
    this.getPIDController().reset();
    this.getPIDController().setOutputRange(-.65, .65);
    initialDegrees = Robot.driveBase.getAngle();
    setpoint = initialDegrees + angleturned;
    this.getPIDController().setSetpoint(setpoint);
    SmartDashboard.putNumber("Setpoint", setpoint);
    this.getPIDController().setAbsoluteTolerance(5);
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
    SmartDashboard.putBoolean("timeout" , this.isTimedOut());
    SmartDashboard.putBoolean("target", this.getPIDController().onTarget());
    if (this.getPIDController().onTarget() == true){
      count += 1;
    }
    if (this.getPIDController().onTarget() == false){
      count = 0;
    }
    if (this.isTimedOut()){
      
      return true;
    }
    if (count == 5){
      return true;
    } else 
      return false;
    }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.driveBase.setLeftMotors(0);
    Robot.driveBase.setRightMotors(0);
    this.getPIDController().disable();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
  @Override
  protected double returnPIDInput(){
    double angle = Robot.driveBase.getAngle();
    SmartDashboard.putNumber("Angle", angle);
    return angle;
  }
  @Override
  protected void usePIDOutput(double output){
    Robot.driveBase.setRightMotors(-output);
    SmartDashboard.putNumber("RightMotors", -output);
    Robot.driveBase.setLeftMotors(output);
    SmartDashboard.putNumber("LeftMotors", output);
  }
  
}
