/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.CheesyDrive;

/**
 * Add your docs here.
 */
public class PIDDriveBase extends PIDSubsystem {
  private VictorSPX leftFront;
  private VictorSPX leftBack;

  private VictorSPX rightFront;
  private VictorSPX rightBack;
  
  private TalonSRX leftMiddle;
  private TalonSRX rightMiddle;
  
  private boolean visionPIDEnabled;

  public PIDDriveBase() {
    super(RobotMap.GYRO_P,RobotMap.GYRO_I,RobotMap.GYRO_D);
    leftFront = new VictorSPX(RobotMap.LEFT_FRONT);
    leftBack = new VictorSPX(RobotMap.LEFT_BACK);
    
    rightFront = new VictorSPX(RobotMap.RIGHT_FRONT);
    rightBack = new VictorSPX(RobotMap.RIGHT_BACK);

    leftMiddle = new TalonSRX(RobotMap.LEFT_MIDDLE);
    rightMiddle = new TalonSRX(RobotMap.RIGHT_MIDDLE);

    rightMiddle.setInverted(true);
    rightFront.setInverted(true);
    rightBack.setInverted(true);
  }

  public void setRightMotors(double power){
    power = correctForDeadzone(power);
    rightFront.set(ControlMode.PercentOutput,power);
    rightBack.set(ControlMode.PercentOutput,power);
    rightMiddle.set(ControlMode.PercentOutput,power);
  }

  public void setLeftMotors(double power){
    power = correctForDeadzone(power);
    leftFront.set(ControlMode.PercentOutput,power);
    leftBack.set(ControlMode.PercentOutput,power);
    leftMiddle.set(ControlMode.PercentOutput,power);
  }
  public void setAllMotors(double power){
    setLeftMotors(power);
    setRightMotors(power);
  }
  private enum Sign { POSITIVE, NEGATIVE }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new CheesyDrive());
    
  }
  public boolean getVisionPIDEnabled(){
    return visionPIDEnabled;
  }
  public void activateVisionPIDMode(){
    if(!this.getPIDController().isEnabled()){
      this.getPIDController().reset();
      this.getPIDController().setAbsoluteTolerance(1);
      this.getPIDController().setOutputRange(-.65, .65);
      this.getPIDController().setSetpoint(0);
      this.getPIDController().enable();
      visionPIDEnabled = true;
    }
  }
  public void disableVisionPIDMode(){
    this.getPIDController().disable();
    visionPIDEnabled = false;
  }
  private double correctForDeadzone(double power) {
    double correctedPower = 0;
    if ((RobotMap.MOTOR_DEADZONE < power) && (power <= 1)){
      // correctedPower = ((1 - RobotMap.MOTOR_DEADZONE) * (power - 1)) + 1;
      correctedPower = scalePastDeadzone(power, Sign.POSITIVE);
    }
    else if ((-1 <= power) && (power < -RobotMap.MOTOR_DEADZONE)){
      // correctedPower = (((-1  +  RobotMap.MOTOR_DEADZONE) / -1) * (power + 1)) - 1;
      correctedPower = scalePastDeadzone(power, Sign.NEGATIVE);
    }
    return correctedPower;
  }
  private double scalePastDeadzone(double power, Sign sign) {
    double signMultiplier = (sign == Sign.POSITIVE) ? 1.0 : -1.0;
    double correctedPower = 0;
    correctedPower = (((1 * signMultiplier) /((1 * signMultiplier) +  (-RobotMap.MOTOR_DEADZONE  * signMultiplier))) * (power - (1 * signMultiplier))) + (1 * signMultiplier);
    return correctedPower;
  }
  

  @Override
  protected double returnPIDInput() {
    // Return your input value for the PID loop
    // e.g. a sensor, like a potentiometer:
    // yourPot.getAverageVoltage() / kYourMaxVoltage;
    return Robot.vision.getXOffset();
  }

  @Override
  protected void usePIDOutput(double output) {
    // Use output to drive your system, like a motor
     setLeftMotors(output);
     setRightMotors(-output);
  }
}
