/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.PIDCommand;
import edu.wpi.first.wpilibj.filters.LinearDigitalFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class TurnWithVision extends PIDCommand {
  PIDSource pst = new PIDSource() {
    PIDSourceType pidType;
    int samples = 0;

    public void setPIDSourceType(PIDSourceType pidSource) {
      pidType = pidSource;
    }

    @Override
    public PIDSourceType getPIDSourceType() {
      // TODO Auto-generated method stub
      return pidType;
    }

    @Override
    public double pidGet() {
      samples++;
      SmartDashboard.putNumber("samples", samples);
      // TODO Auto-generated method stub
      double xoffset = Robot.vision.getXOffset();
      SmartDashboard.putNumber("X offset", xoffset);
      return xoffset;
    }
  };
  int count = 0;
  double setpoint;
  double timeout;
  double initialPosition;
  LinearDigitalFilter ldf = LinearDigitalFilter.movingAverage(pst, 10);

  public TurnWithVision(double timeout) {
    super(RobotMap.GYRO_P, RobotMap.GYRO_I, RobotMap.GYRO_D);
    requires(Robot.vision);
    requires(Robot.driveBase);
    setpoint = 0;
    this.timeout = timeout;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    this.getPIDController().reset();
    if (Robot.vision.HasTarget() == true) {
      this.getPIDController().setOutputRange(-.65, .65);
      initialPosition = Robot.vision.getXOffset();
      this.getPIDController().setSetpoint(setpoint);
      SmartDashboard.putNumber("Setpoint", setpoint);
      this.setTimeout(timeout);
      this.getPIDController().enable();
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  public boolean averageOnTarget() {
    if ((Math.abs(ldf.pidGet() - setpoint)) <= 1) {

      count++;
    } else {
      count = 0;
    }
    if (count == 5) {
      return true;
    }
    return false;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    boolean onTarget = averageOnTarget();
    SmartDashboard.putBoolean("on target", onTarget);
    if (onTarget || this.isTimedOut()) {
      return true;
    } else
      return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    this.getPIDController().disable();
    Robot.driveBase.setAllMotors(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }

  protected double returnPIDInput() {
    return (ldf.pidGet());
  }

  @Override
  protected void usePIDOutput(double output) {
    //SmartDashboard.putNumber("GyroPIDAngle", Robot.driveBase.getAngle());
    SmartDashboard.putNumber("out", output);
    if(Robot.vision.getXOffset() > 0){
      Robot.driveBase.setRightMotors(output);
      Robot.driveBase.setLeftMotors(-output);
    }
    else if (Robot.vision.getXOffset() < 0){
    Robot.driveBase.setRightMotors(-output);
    Robot.driveBase.setLeftMotors(output);
    }
  }
}
