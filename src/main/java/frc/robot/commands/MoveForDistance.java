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

public class MoveForDistance extends PIDCommand {
  double setpoint;
  double timeout;
  int count = 0;

  public MoveForDistance(double setpoint, double timeout) {
    super(RobotMap.DRIVE_P, RobotMap.DRIVE_I, RobotMap.DRIVE_D);
    requires(Robot.driveBase);
    this.setpoint = setpoint * RobotMap.TICKS_PER_INCH;
    this.timeout = timeout;
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.driveBase.resetEncoder();
    this.getPIDController().reset();
    this.getPIDController().setOutputRange(-.65, .65);
    this.getPIDController().setSetpoint(setpoint);
    SmartDashboard.putNumber("Distance Setpoint", setpoint);
    this.getPIDController().setAbsoluteTolerance(5);
    this.setTimeout(timeout);
    this.getPIDController().enable();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    SmartDashboard.putBoolean("Distance Timeout" , this.isTimedOut());
    SmartDashboard.putBoolean("Distance Target", this.getPIDController().onTarget());
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
    double averageEncoderValue = (Robot.driveBase.getLeftEncoderValue() + Robot.driveBase.getRightEncoderValue()) / 2;
    SmartDashboard.putNumber("Average Encoder Value", averageEncoderValue);
    return averageEncoderValue;
  }
  @Override
  protected void usePIDOutput(double output) {
    Robot.driveBase.setRightMotors(output);
    SmartDashboard.putNumber("RightMotors", output);
    Robot.driveBase.setLeftMotors(output);
    SmartDashboard.putNumber("LeftMotors", output);
  }
}
