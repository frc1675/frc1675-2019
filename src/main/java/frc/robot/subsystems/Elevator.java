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
  DigitalInput limitSwitch;

  public Elevator(){

    elevatorMotor = new TalonSRX(RobotMap.ELEVATOR);
    tiltElevator = new DoubleSolenoid(RobotMap.TILT_ELEVATOR_FORWARD, RobotMap.TILT_ELEVATOR_REVERSE);
    limitSwitch = new DigitalInput(RobotMap.LIMIT_SWITCH);
  }

  public void resetPosition() {
    while (limitSwitch.get()){

    }
  }

  public void tiltElevatorForward() {
    tiltElevator.set(Value.kForward);
  }

  public void tiltElevatorReverse() {
    tiltElevator.set(Value.kReverse);
  }

  public void moveElevator(double power) {
    elevatorMotor.set(ControlMode.PercentOutput,power);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
