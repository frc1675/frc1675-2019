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

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.CheesyDrive;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class DriveBase extends Subsystem {
  private VictorSPX leftFront = new VictorSPX(RobotMap.LEFT_FRONT);
  private VictorSPX leftBack = new VictorSPX(RobotMap.LEFT_BACK);

  private VictorSPX rightFront = new VictorSPX(RobotMap.RIGHT_FRONT);
  private VictorSPX rightBack = new VictorSPX(RobotMap.RIGHT_BACK);

  private TalonSRX leftMiddle = new TalonSRX(RobotMap.LEFT_MIDDLE);
  private TalonSRX rightMiddle = new TalonSRX(RobotMap.LEFT_MIDDLE);

  public void setRightMotors(double power){
    rightFront.set(ControlMode.PercentOutput,power);
    rightBack.set(ControlMode.PercentOutput,power);
    rightMiddle.set(ControlMode.PercentOutput,power);
    rightMiddle.setInverted(true);
    rightFront.setInverted(true);
    rightBack.setInverted(true);
  }

  public void setLeftMotors(double power){
    leftFront.set(ControlMode.PercentOutput,power);
    leftBack.set(ControlMode.PercentOutput,power);
    leftMiddle.set(ControlMode.PercentOutput,power);
  }
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new CheesyDrive());
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
