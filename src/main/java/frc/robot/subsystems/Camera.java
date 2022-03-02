/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Add your docs here.
 */

public class Camera extends SubsystemBase {

  NetworkTableEntry tv, tx, ty, ta, ts, tl, tshort, tlong, thor, tvert, getpipe, camtran, pipeline, ledMode;
  public static final double ComputerVision = 0;
  public static final double HumanVision = 1;

  public Camera() {

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    tv = table.getEntry("tv");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    ts = table.getEntry("ts"); // not helpful. Measures rotation of target
    tshort = table.getEntry("tshort");
    tlong = table.getEntry("tlong");
    thor = table.getEntry("thor");
    tvert = table.getEntry("tvert");
    camtran = table.getEntry("camtran");
    getpipe = table.getEntry("getpipe");

    pipeline = table.getEntry("pipeline");
    ledMode = table.getEntry("ledMode");
    ledMode.setDouble(0);

  }

  /***
   * returns the Camera network table
   *
   * @return Camera NetworkTable
   */
  public NetworkTable getTable() {
    return NetworkTableInstance.getDefault().getTable("limelight");
  }

  public double getXAngle() {
    return tx.getDouble(0.0);
  }

  public double getYAngle() {
    return ty.getDouble(0.0);
  }

  public boolean hasTarget() {
    return (tv.getDouble(0) == 1);
  }

  public void setPipeline(double pipeline) {
    this.pipeline.setDouble(pipeline);
  }

  public double getPipeline() {
    return getpipe.getDouble(0.0);
  }

  public void updateDashboard() {

  }

}