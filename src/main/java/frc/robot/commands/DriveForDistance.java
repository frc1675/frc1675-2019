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

public class DriveForDistance extends PIDCommand {
  private static final double P = 0.7;
  private static final double I = 0;
  private static final double D = 0.3;

  private double setpoint;
  
  PIDSource pst = new PIDSource() {
    PIDSourceType pidType;

    public void setPIDSourceType(PIDSourceType pidSource) {
        pidType = pidSource;
    }

    @Override
    public PIDSourceType getPIDSourceType() {
        return pidType;
    }

    @Override
    public double pidGet() {
        double encoderval = (Robot.driveBase.getLeftEncoderPosition() + Robot.driveBase.getRightEncoderPosition()) / 2;
        SmartDashboard.putNumber("encoder value", encoderval);
        System.out.println("encoderval: "+encoderval);
        return encoderval;
    }
};
LinearDigitalFilter ldf = LinearDigitalFilter.movingAverage(pst, 10);;

  public DriveForDistance() {
    super("DriveForDistance", P, I, D);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    setpoint = 50000;
    Robot.driveBase.resetEncoder();
    this.getPIDController().reset();
    this.getPIDController().setSetpoint(setpoint);
    this.setTimeout(8);
    this.getPIDController().enable();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    double testVal = (Math.abs(ldf.pidGet()-setpoint));

    if(this.isTimedOut() || testVal <= 100){
      System.out.println("true");
      System.out.println("testVal: "+testVal);
      return true;
    }
    else{
      return false;
    }
    
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    this.getPIDController().disable();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
    
  }

  @Override
  protected double returnPIDInput(){
    System.out.println("ldf: "+ldf.pidGet());
    return ldf.pidGet();
  }

  @Override
  protected void usePIDOutput(double output){
    Robot.driveBase.setLeftMotors(output);
    Robot.driveBase.setRightMotors(output);
  }
}
