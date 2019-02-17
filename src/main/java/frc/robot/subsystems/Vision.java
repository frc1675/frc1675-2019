
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/*
NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
NetworkTableEntry tx = table.getEntry("tx");
NetworkTableEntry ty = table.getEntry("ty");
NetworkTableEntry ta = table.getEntry("ta");

//read values periodically
double x = tx.getDouble(0.0);
double y = ty.getDouble(0.0);
double area = ta.getDouble(0.0);

//post to smart dashboard periodically
SmartDashboard.putNumber("LimelightX", x);
SmartDashboard.putNumber("LimelightY", y);
SmartDashboard.putNumber("LimelightArea", area);
8/
/**
 * Add your docs here.
 */
public class Vision extends Subsystem {
 
  
  public Vision() {
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry ledmode = table.getEntry("ledMode");
  NetworkTableEntry cammode = table.getEntry("camMode");
    cammode.setNumber(1);
    table.getEntry("ledMode").setNumber(1);
    table.getEntry("camMode").setNumber(1);
    SmartDashboard.putNumber("camera mode",cammode.getDouble(0));
    ledmode.setNumber(1);
  }

  @Override
  public void initDefaultCommand() {
  }
} 