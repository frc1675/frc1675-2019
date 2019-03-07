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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  private boolean visionPIDEnabled = false;

  private double correction = 0;

  public PIDDriveBase() {
    super(RobotMap.TURN_P, RobotMap.TURN_I, RobotMap.TURN_D);
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

  public void setRightMotors(double power) {
    power = correctForDeadzone(power);
    power = scaleCorrection(power);
    double rightpower = (power - correction);
    rightFront.set(ControlMode.PercentOutput, rightpower);
    rightBack.set(ControlMode.PercentOutput, rightpower);
    rightMiddle.set(ControlMode.PercentOutput, rightpower);
    SmartDashboard.putNumber("right power", rightpower);
  }

  public void setLeftMotors(double power) {
    power = correctForDeadzone(power);
    power = scaleCorrection(power);
    double leftpower = (power + correction);
    leftFront.set(ControlMode.PercentOutput,leftpower);
    leftBack.set(ControlMode.PercentOutput, leftpower);
    leftMiddle.set(ControlMode.PercentOutput, leftpower);
    SmartDashboard.putNumber("left power",leftpower);
  }

  public void setAllMotors(double power) {
    power = correctForDeadzone(power);
    power = scaleCorrection(power);
    double leftpower = (power + correction);
    double rightpower = (power - correction);
    setLeftMotors(leftpower);
    setRightMotors(rightpower);
  }
  public double scaleCorrection(double power){
    double adjustedPower = power;
     if (Math.abs(adjustedPower) + Math.abs(correction) > 1) {
      if (adjustedPower > 0) {
        adjustedPower = adjustedPower - correction;
      } else {
        adjustedPower = adjustedPower + correction;
      }
    }
    return adjustedPower;
  }

  private enum Sign {
    POSITIVE, NEGATIVE
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new CheesyDrive());

  }

  public boolean getVisionPIDEnabled() {
    return visionPIDEnabled;
  }

  public void activateVisionPIDMode() {
    if (!this.getPIDController().isEnabled()) {
      if (Robot.vision.hasTarget()) {
        this.getPIDController().reset();
        this.getPIDController().setAbsoluteTolerance(1);
        this.getPIDController().setOutputRange(-1, 1);
        this.getPIDController().setSetpoint(0);
        this.getPIDController().enable();
        visionPIDEnabled = true;

      }
    }
  }

  public void disableVisionPIDMode() {
    this.getPIDController().disable();
    visionPIDEnabled = false;
  }

  private double correctForDeadzone(double power) {
    double correctedPower = 0;
    if ((RobotMap.MOTOR_DEADZONE < power) && (power <= 1)) {
      // correctedPower = ((1 - RobotMap.MOTOR_DEADZONE) * (power - 1)) + 1;
      correctedPower = scalePastDeadzone(power, Sign.POSITIVE);
    } else if ((-1 <= power) && (power < -RobotMap.MOTOR_DEADZONE)) {
      // correctedPower = (((-1 + RobotMap.MOTOR_DEADZONE) / -1) * (power + 1)) - 1;
      correctedPower = scalePastDeadzone(power, Sign.NEGATIVE);
    }
    return correctedPower;
  }

  private double scalePastDeadzone(double power, Sign sign) {
    double signMultiplier = (sign == Sign.POSITIVE) ? 1.0 : -1.0;
    double correctedPower = 0;
    correctedPower = (((1 * signMultiplier) / ((1 * signMultiplier) + (-RobotMap.MOTOR_DEADZONE * signMultiplier)))
        * (power - (1 * signMultiplier))) + (1 * signMultiplier);
    return correctedPower;
  }

  public void periodic() {
    SmartDashboard.putBoolean("pid mode on", getVisionPIDEnabled());
  }

  @Override
  protected double returnPIDInput() {
    return Robot.vision.getXOffset();
  }

  @Override
  protected void usePIDOutput(double output) {
    correction = output;
  }
}
