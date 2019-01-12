/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.CheesyDrive;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class DriveBase extends Subsystem {
  private VictorSP left2 = new VictorSP(RobotMap.LEFT2);
  private VictorSP left3 = new VictorSP(RobotMap.LEFT3);

  private VictorSP right2 = new VictorSP(RobotMap.RIGHT2);
  private VictorSP right3 = new VictorSP(RobotMap.RIGHT3);

  public void setRightMotors(double power){
    right2.set(power);
    right3.set(power);
  }

  public void setLeftMotors(double power){
    left2.set(power);
    left3.set(power);
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
