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

public class MoveElevatorToPosition extends PIDCommand {

  double setpoint = 0;
  int count = 0;

  public MoveElevatorToPosition(double position) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    super(RobotMap.ELEVATOR_P, RobotMap.ELEVATOR_I, RobotMap.ELEVATOR_D);
    requires(Robot.elevator);
    if (position < RobotMap.MIN_POSITION) {
      setpoint = RobotMap.MIN_POSITION;
    } 
    else if (position > RobotMap.MAX_POSITION) {
      setpoint = RobotMap.MAX_POSITION;
    } 
    else {
      setpoint = position;
    }
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    this.getPIDController().reset();
    this.getPIDController().setSetpoint(setpoint);
    this.getPIDController().setOutputRange(-.50, .50);
    this.getPIDController().enable();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    this.getPIDController().disable();
    Robot.elevator.setElevatorMotor(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }

  @Override
  protected double returnPIDInput() {
    return Robot.elevator.getElevatorPosition();
  }

  @Override
  protected void usePIDOutput(double output) {
    Robot.elevator.setElevatorMotor(output);
  }
}
