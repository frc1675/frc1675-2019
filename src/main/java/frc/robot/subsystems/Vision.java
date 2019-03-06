package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.commands.ChangeProcessingMode;

/**
 * Add your docs here.
 */
public class Vision extends Subsystem {
  NetworkTable table;

  public Vision() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    setPipeline(0);
  }

  public double getXOffset() {
    double xoffset = table.getEntry("tx").getDouble(0);
    return xoffset;
  }

  public double getYOffset() {
    double yoffset = table.getEntry("ty").getDouble(0);
    return yoffset;
  }

  public double getArea() {
    double area = table.getEntry("ta").getDouble(0);
    return area;
  }

  public double getPipeline() {
    double pipeline = table.getEntry("pipeline").getDouble(0);
    return pipeline;
  }

  public void setPipeline(double p) {
    table.getEntry("pipeline").setDouble(p);
  }

  public void setCameraMode(double mode) {
    table.getEntry("camMode").setDouble(mode);
  }

  public void setLightMode(double mode) {
    table.getEntry("ledMode").setDouble(mode);
  }

  public boolean hasTarget() {
    double targetb = table.getEntry("tv").getDouble(0);
    boolean hasTarget = false;
    if (targetb == 0) {
      hasTarget = false;
    }
    if (targetb == 1) {
      hasTarget = true;
    }
    return hasTarget;
  }

  public double findDistance(double h1, double h2) {
    double degrad = Math.toRadians(this.getYOffset());
    double b = h2 - h1;
    double b2 = Math.pow(b, 2.0);
    double hy = b / Math.sin(degrad);
    double hy2 = Math.pow(hy, 2.0);
    double pyth = hy2 - b2;
    double a = Math.sqrt(pyth);
    return a;
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new ChangeProcessingMode(1));
  }
}