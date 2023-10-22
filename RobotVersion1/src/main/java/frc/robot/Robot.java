// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.Subsystems.SwerveDrive;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Subsystems.Turret;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private final XboxController controller = new XboxController(0);
  private final SwerveDrive swerve = new SwerveDrive();

  private final SlewRateLimiter xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);

  public Robot(){

  }
  
  private void driveWithJoystick(boolean fieldRelative){

    final var xSpeed = -xspeedLimiter.calculate(MathUtil.applyDeadband(controller.getLeftY(), 0.02)) * SwerveDrive.kMaxSpeed;
    
    final var ySpeed = -yspeedLimiter.calculate(MathUtil.applyDeadband(controller.getLeftX(), 0.02)) * SwerveDrive.kMaxSpeed;

    final var rot = -rotLimiter.calculate(MathUtil.applyDeadband(controller.getRightX(), 0.02)) * SwerveDrive.kMaxAngularSpeed;

    swerve.drive(xSpeed, ySpeed, rot, fieldRelative, getPeriod());
  }
  @Override
  public void robotInit() {}

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {
    driveWithJoystick(false);
    swerve.updateOdometry();
  }
  
  @Override
  public void teleopInit() {
    driveWithJoystick(true);
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
