/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.CheesyDrive;
import edu.wpi.first.wpilibj.SerialPort;
/**
 *DriveBase is the representation of the physical drive motors and provides access to their motor controllers.
 */
public class DriveBase extends Subsystem {
  private VictorSPX leftMiddle;
  private VictorSPX leftBack;

  private VictorSPX rightMiddle;
  private VictorSPX rightBack;
  
  private TalonSRX leftFront;
  private TalonSRX rightFront;
  AHRS ahrs;
  public DriveBase() {
    super();
    leftMiddle = new VictorSPX(RobotMap.LEFT_MIDDLE);
    leftBack = new VictorSPX(RobotMap.LEFT_BACK);
    
    rightMiddle = new VictorSPX(RobotMap.RIGHT_MIDDLE);
    rightBack = new VictorSPX(RobotMap.RIGHT_BACK);

    leftFront = new TalonSRX(RobotMap.LEFT_FRONT);
    rightFront = new TalonSRX(RobotMap.RIGHT_FRONT);

    rightMiddle.setInverted(true);
    rightFront.setInverted(true);
    rightBack.setInverted(true);
    
    ahrs = new AHRS(SerialPort.Port.kMXP);

    rightFront.setSensorPhase(true);
    leftFront.setSensorPhase(true);
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
  private enum Sign { POSITIVE, NEGATIVE }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new CheesyDrive());
    
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
  public void resetGyro() {
    ahrs.zeroYaw();
  }
  public double getAngle() {
    return ahrs.getAngle();
  }
  public void resetEncoder() {
    leftFront.getSensorCollection().setQuadraturePosition(0, 0);
    rightFront.getSensorCollection().setQuadraturePosition(0, 0);
}
  public double getLeftEncoderValue() {

    return leftFront.getSelectedSensorPosition(0);
  }

  public double getRightEncoderValue() {

    return rightFront.getSelectedSensorPosition(0);
  }
}


