/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Grabber extends Subsystem {
  private Solenoid armRotator;

  private Solenoid hatchReleaser;

  private Solenoid hook;
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public Grabber() {
    super();
    armRotator = new Solenoid(RobotMap.ARM_ROTATOR);

    hatchReleaser = new Solenoid(RobotMap.HATCH_RELEASER);

    hook = new Solenoid(RobotMap.HOOK);
  }

  public void wristUp() {
    armRotator.set(false);
  }

  public void wristDown() {
    armRotator.set(true);
  }

  public void releaseHatch() {
    hatchReleaser.set(true);
  }

  public void retractPistons() {
    hatchReleaser.set(false);
  }

  public void extendHook(){
    hook.set(true);
  }

  public void retractHook(){
    hook.set(false);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
