/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Elevator extends Subsystem {
  private TalonSRX elevatorMotor;
  private DoubleSolenoid tiltElevator;
  private DigitalInput lowerLimitSwitch;
  private DigitalInput upperLimitSwitch;
  private boolean isLowerLimitDefined = false;
  private double targetPosition = 0;
  private int count = 0;

  public Elevator(){

    elevatorMotor = new TalonSRX(RobotMap.ELEVATOR_MOTOR);
    tiltElevator = new DoubleSolenoid(RobotMap.TILT_ELEVATOR_FORWARD, RobotMap.TILT_ELEVATOR_REVERSE);
    lowerLimitSwitch = new DigitalInput(RobotMap.LOWER_LIMIT_SWITCH);
    upperLimitSwitch = new DigitalInput(RobotMap.UPPER_LIMIT_SWITCH);
    elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
    elevatorMotor.configForwardSoftLimitThreshold(RobotMap.MAX_POSITION);
    elevatorMotor.configReverseSoftLimitThreshold(RobotMap.MIN_POSITION);
    elevatorMotor.setSensorPhase(true);
  }

  // The limit switches return false when they're pressed, so this
  // makes the code more intuitive.
  public boolean isLowerLimitSwitchPressed() {
    return !lowerLimitSwitch.get();
  }

  private boolean isUpperLimitSwitchPressed() {
    return !upperLimitSwitch.get();
  }
  
  public double getElevatorPosition() {
    return elevatorMotor.getSelectedSensorPosition();
  }
  public void resetPosition() {
    elevatorMotor.setSelectedSensorPosition(0);
  }

  public void tiltElevatorForward() {
    if (isLowerLimitSwitchPressed() == true || getElevatorPosition() < RobotMap.MIN_POSITION) {
      tiltElevator.set(Value.kForward);
    }
  }

  public void tiltElevatorReverse() {
    if (isLowerLimitSwitchPressed() == true || getElevatorPosition() < RobotMap.MIN_POSITION) {
      tiltElevator.set(Value.kReverse);
    }
  }

  public void setElevatorMotor(double power) {
    SmartDashboard.putNumber("Motor power", power);
    double correctedPower = 0;
    if (isLowerLimitDefined == true) {
      if ((isLowerLimitSwitchPressed() == true && power < 0) || (isUpperLimitSwitchPressed() == true && power > 0)) {
        correctedPower = 0;
      } 
      else {
        correctedPower = power;
      }

    }
    elevatorMotor.set(ControlMode.PercentOutput,correctedPower);
  }

  public void setTargetPosition(double position) {
    targetPosition = position;
  }

  public boolean elevatorOnTarget() {
    if (Math.abs(targetPosition - getElevatorPosition()) <= RobotMap.ELEVATOR_TOLERANCE) {
      count ++;
    }
    else {
      count = 0;
    }

    if (count >= 10) {
      return true;
    } 
    else {
      return false;
    }
  }

  // will resolve before match begins
  public void periodic(){
    if (isLowerLimitSwitchPressed() == true && isLowerLimitDefined == false) {
      isLowerLimitDefined = true;
      elevatorMotor.configForwardSoftLimitEnable(true);
      elevatorMotor.configReverseSoftLimitEnable(true);
      resetPosition();
      
    }

    //if (isLowerLimitSwitchPressed() == true || isUpperLimitSwitchPressed() == true ){
     //Robot.oi.setOperatorRumble(1);
     
//}
    SmartDashboard.putBoolean("Elevator home position defined", isLowerLimitDefined);
    SmartDashboard.putBoolean("Elevator lower limit pressed", isLowerLimitSwitchPressed());
    SmartDashboard.putBoolean("Elevator upper limit pressed", isUpperLimitSwitchPressed());
    SmartDashboard.putNumber("Elevator position", getElevatorPosition());
    SmartDashboard.putBoolean("Elevator on target", elevatorOnTarget());
    SmartDashboard.putNumber("Target Position", targetPosition);
    SmartDashboard.putNumber("Count", count);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    //setDefaultCommand(new MoveElevatorWithJoystick());
  }
}