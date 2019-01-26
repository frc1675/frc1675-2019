/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
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

  public Elevator(){

    elevatorMotor = new TalonSRX(RobotMap.ELEVATOR_MOTOR);
    tiltElevator = new DoubleSolenoid(RobotMap.TILT_ELEVATOR_FORWARD, RobotMap.TILT_ELEVATOR_REVERSE);
    lowerLimitSwitch = new DigitalInput(RobotMap.LOWER_LIMIT_SWITCH);
    upperLimitSwitch = new DigitalInput(RobotMap.UPPER_LIMIT_SWITCH);
    elevatorMotor.setForwardSoftLimit(RobotMap.MAX_POSITION);
    elevatorMotor.enableForwardSoftLimit(true);
    elevatorMotor.setReverseSoftLimit(RobotMap.MIN_POSITION);
    elevatorMotor.enableReverseSoftLimit(true);
  }

  public void resetPosition() {
    elevatorMotor.setPosition(0);
  }

  public void tiltElevatorForward() {
    if (lowerLimitswitch.get() == true || elevatorMotor.getPosition() < RobotMap.TILT_MAX_POSITION) {
      tiltElevator.set(Value.kForward);
    }
  }

  public void tiltElevatorReverse() {
    if (lowerLimitswitch.get() == true || elevatorMotor.getPosition() < RobotMap.TILT_MAX_POSITION) {
      tiltElevator.set(Value.kReverse);
      resetPosition();
    }
  }

  public void moveElevator(double power) {

    double correctedPower = 0;
    if (isLowerLimitDefined == true) {
      if ((lowerLimitSwitch.get() == true && power < 0) || (upperLimitSwitch.get() == true && power > 0)) {
        correctedPower = 0;
      } 
      else {
        correctedPower = power;
      }

    }

    elevatorMotor.set(ControlMode.PercentOutput,correctedPower);
  }

  public void periodic(){
    if (lowerLimitSwitch.get() == true && isLowerLimitDefined == false) {
      isLowerLimitDefined = true;
      resetPosition();
    }
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
