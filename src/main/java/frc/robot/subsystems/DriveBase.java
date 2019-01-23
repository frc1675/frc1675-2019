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
 *DriveBase is the representation of the physical drive motors and provides access to their motor controllers.
 */
public class DriveBase extends Subsystem {
  private VictorSPX leftFront;
  private VictorSPX leftBack;

  private VictorSPX rightFront;
  private VictorSPX rightBack;
  
  private TalonSRX leftMiddle;
  private TalonSRX rightMiddle;
  
  public DriveBase() {
    super();
    leftFront = new VictorSPX(RobotMap.LEFT_FRONT);
    leftBack = new VictorSPX(RobotMap.LEFT_BACK);
    
    rightFront = new VictorSPX(RobotMap.RIGHT_FRONT);
    rightBack = new VictorSPX(RobotMap.RIGHT_BACK);

    leftMiddle = new TalonSRX(RobotMap.LEFT_MIDDLE);
    rightMiddle = new TalonSRX(RobotMap.LEFT_MIDDLE);

    leftMiddle.setInverted(true);
    leftFront.setInverted(true);
    leftBack.setInverted(true);

  }

  public void setRightMotors(double power){
    power = correctForDeadzone(power);
    System.out.println("Right" + power);
    rightFront.set(ControlMode.PercentOutput,power);
    rightBack.set(ControlMode.PercentOutput,power);
    rightMiddle.set(ControlMode.PercentOutput,power);


  }

  public void setLeftMotors(double power){
    power = correctForDeadzone(power);
    System.out.println("Left" + power);
    leftFront.set(ControlMode.PercentOutput,power);
    leftBack.set(ControlMode.PercentOutput,power);
    leftMiddle.set(ControlMode.PercentOutput,power);
  }
  
  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new CheesyDrive());
    
  }
  private double correctForDeadzone(double power) {
    double correctedPower = 0;
    if ((RobotMap.Motor_DEADZONE < power) && (power <= 1)){
      correctedPower = ((1 - RobotMap.Motor_DEADZONE) * power + 1) - 1;
    }
    else if ((-1 <= power) && (power < -RobotMap.Motor_DEADZONE)){
      correctedPower = (((-1  +  RobotMap.Motor_DEADZONE) / -1) * power - 1) + 1;

    }
    return correctedPower;
  }
}
