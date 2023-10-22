// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import com.kauailabs.navx.frc.*;  
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.AnalogGyro;
import frc.robot.SwerveModule;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import com.kauailabs.navx.frc.AHRS;
public class SwerveDrive extends SubsystemBase {
  public static final double kMaxSpeed = 3.0;
  public static final double kMaxAngularSpeed = Math.PI;
  public final double L = 0.60166;
  public final double W = 0.60166;

  private Translation2d FLLocation = new Translation2d(L/2, W/2);
  private Translation2d FRLocation = new Translation2d(L/2, -W/2);
  private Translation2d BLLocation = new Translation2d(-L/2, W/2);
  private Translation2d BRLocation = new Translation2d(-L/2, -W/2);

  //Drive : 0 1 2 3  Turn: 4 5 6 7
  private final SwerveModule FLModule = new SwerveModule(0, 4, 0, 1);
  private final SwerveModule FRModule = new SwerveModule(1, 5, 2, 3);
  private final SwerveModule BLModule = new SwerveModule(2, 6, 4, 5);
  private final SwerveModule BRModule = new SwerveModule(3, 7, 6, 7);

  private final AHRS gyro = new AHRS(SPI.Port.kMXP);
  //private final AnalogGyro gyro = new AnalogGyro(0);

  private final SwerveDriveKinematics kinematics = 
    new SwerveDriveKinematics(FLLocation, FRLocation, BLLocation, BRLocation);

  private final SwerveDriveOdometry odometry = 
    new SwerveDriveOdometry(
      kinematics, 
      gyro.getRotation2d(), 
      new SwerveModulePosition[] { 
        FLModule.getPosition(),
        FRModule.getPosition(),
        BLModule.getPosition(),
        BRModule.getPosition()
      });



  public SwerveDrive() {
    gyro.reset();
  }
  /**
   * Drive robot using joystick info
   * 
   * @param xSpeed Speed of the robot in the forward direction
   * @param ySpeed Speed of the robot in the sideways direction
   * @param rot Angular Rate of the robot
   * @param fieldRelative Whether x and y are relative to the field
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds){
    var swerveModuleStates = 
      kinematics.toSwerveModuleStates(
        fieldRelative
          ? ChassisSpeeds.fromFieldRelativeSpeeds(
              xSpeed, ySpeed, rot, gyro.getRotation2d())
          : new ChassisSpeeds(xSpeed, ySpeed, rot)
        );
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    FLModule.setDesiredState(swerveModuleStates[0]);
    FRModule.setDesiredState(swerveModuleStates[1]);
    BLModule.setDesiredState(swerveModuleStates[2]);
    BRModule.setDesiredState(swerveModuleStates[3]);
    //ChassisSpeeds.discretize(fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d()) : new ChassisSpeeds(xSpeed, ySpeed, rot), periodSeconds);
    /*
     * Reminder of what is happening with discretize: 
     *  Discretize is a new function being tested by WPILIB, not fully implemented but will be in the future.
     *  It aims to negate the robot drifting at an angle when going straight while turning.
     *  Should work without it, but after swerve drive is confirmed to work and is tuned, try and implement it.
     *  Code can be found here: https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964/55?page=2
     *  If you want to implement it, make a fuction within the SwerveDrive.java file and call it there. :)
     */
  }
  public void updateOdometry() {
    odometry.update(gyro.getRotation2d(), new SwerveModulePosition[] {FLModule.getPosition(), FRModule.getPosition(), BLModule.getPosition(), BRModule.getPosition()});
  }
  @Override
  public void periodic() {
  }
  
}
