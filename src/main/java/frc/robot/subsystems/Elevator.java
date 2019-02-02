/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.MoveElevatorWithJoystick;

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
    elevatorMotor.configForwardSoftLimitThreshold(RobotMap.MAX_POSITION);
    elevatorMotor.configReverseSoftLimitThreshold(RobotMap.MIN_POSITION);
    elevatorMotor.setSensorPhase(true);
  }

  public void resetPosition() {
    elevatorMotor.setSelectedSensorPosition(0);
  }

  public void tiltElevatorForward() {
    if (lowerLimitSwitch.get() == false || elevatorMotor.getSelectedSensorPosition() < RobotMap.TILT_MAX_POSITION) {
      tiltElevator.set(Value.kForward);
    }
  }

  public void tiltElevatorReverse() {
    if (lowerLimitSwitch.get() == false || elevatorMotor.getSelectedSensorPosition() < RobotMap.TILT_MAX_POSITION) {
      tiltElevator.set(Value.kReverse);
    }
  }

  public void setElevatorMotor(double power) {

    double correctedPower = 0;
    if (isLowerLimitDefined == true) {
      if ((lowerLimitSwitch.get() == false && power < 0) || (upperLimitSwitch.get() == false && power > 0)) {
        correctedPower = 0;
      } 
      else {
        correctedPower = power;
      }

    }
    elevatorMotor.set(ControlMode.PercentOutput,correctedPower);
  }

  // will resolve before match begins
  public void periodic(){
    if (lowerLimitSwitch.get() == false && isLowerLimitDefined == false) {
      isLowerLimitDefined = true;
      elevatorMotor.configForwardSoftLimitEnable(true);
      elevatorMotor.configReverseSoftLimitEnable(true);
      resetPosition();
    }
    SmartDashboard.putBoolean("Is lower limit defined?", isLowerLimitDefined);
    SmartDashboard.putBoolean("lower limit switch", !lowerLimitSwitch.get());
    SmartDashboard.putNumber("encoder position", elevatorMotor.getSelectedSensorPosition());
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new MoveElevatorWithJoystick());
  }
}
