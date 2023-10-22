// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight extends SubsystemBase {
  private final static int kDefaultPipeline = 0;
  private final static int kTargetingPipeline = 1;
  private final static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  private static NetworkTableEntry tx = table.getEntry("tx");
  private static NetworkTableEntry ty = table.getEntry("ty");
  private static NetworkTableEntry tArea = table.getEntry("ta");
  private static NetworkTableEntry ledMode = table.getEntry("ledMode");
  private static NetworkTableEntry pipeline = table.getEntry("pipeline");
  //private static NetworkTableEntry tv= table.getEntry("tv");
  private static float Kp = -0.1f;
  private static float min_command = 0.05f;

  private static double area;
  private static double x;
  private static double y;

  /**
   * Creates a new LimelightSubsystem in order to use the limelight in our code
   * package.
   */
  public Limelight() {
  }

  /**
   * Switches to the default pipeline, 0.
   */
  public static void switchToDefaultPipeline() {
    pipeline.setValue(kDefaultPipeline);
  }
  public static void switchPipeline(int index){
    pipeline.setValue(index);
  }
  public static void swapPipeline(){
    if(getPipeline()<2){
      switchPipeline(getPipeline()+2);
    }
    else{
      switchPipeline(getPipeline()-2);
    }
  }

  /**
   * Switches to the first pipeline, 1.
   */
  public static void switchToTargetingPipeline() {
    pipeline.setValue(kTargetingPipeline);
  }
  public static int getPipeline(){
    System.out.println("EEEEE " + String.valueOf(pipeline.getInteger(99)) + " EEEEEEE");
    return (int) pipeline.getInteger(99);
  } 
  
  /*public static boolean targetCheck(){
    boolean target=tv.getBoolean(false);
    return target;
  }
  */
  private static void updateTargeting() {
    x = tx.getDouble(0.0);
    y = ty.getDouble(0);
    area = tArea.getDouble(0);
    SmartDashboard.putNumber("Limelight_X_Coordinate", x);
    SmartDashboard.putNumber("Limelight_Y_Coordinate", y);
    SmartDashboard.putNumber("Limelight_Area", area);
  }
  public static int testing(){
    //tv.getInfo();
    return 0;
    
  }
  public static double getArea() {
    return area;
  }

  public static double getXCoordinate() {
    return x;
  }

  public static double getYCoordinate() {
    return y;
  }

  public static double getLedMode() {
    return ledMode.getDouble(0);
  }

  /**
   * This method is called once per scheduler run in the robot periodic
   */
  public static void limelightPeriodic() {
    updateTargeting();
  }

  public static void setLedMode(int modeId) {
    ledMode.setNumber(modeId);
  }

  public double limelighttargeting() {
    x = tx.getDouble(0.0);
    double heading_error = -x;
    double steering_adjust = 0.0f;
    if (x > 1.0) {
      steering_adjust = Kp * heading_error - min_command;
    } else if (x < 1.0) {
      steering_adjust = Kp * heading_error + min_command;
    }
    System.out.println(steering_adjust);
    return (steering_adjust);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
