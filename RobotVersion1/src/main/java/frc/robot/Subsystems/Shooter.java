// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.filter.SlewRateLimiter;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
public class Shooter extends SubsystemBase {
  private final CANSparkMax shooterMotor1 = new CANSparkMax(8, MotorType.kBrushless);
  private final CANSparkMax shooterMotor2 = new CANSparkMax(9, MotorType.kBrushless);
  private final CANSparkMax conveyorMotor = new CANSparkMax(10, MotorType.kBrushless);
  private final CANSparkMax intakeMotor = new CANSparkMax(11, MotorType.kBrushless);
  private final SlewRateLimiter shooter1Limiter;
  private final SlewRateLimiter shooter2Limiter;
  private final SlewRateLimiter conveyorLimiter;
  private final SlewRateLimiter intakeLimiter;

  /** Creates a new Shooter. */
  public Shooter() {
    shooter1Limiter = new SlewRateLimiter(2);
    shooter2Limiter = new SlewRateLimiter(2);
    conveyorLimiter = new SlewRateLimiter(2);
    intakeLimiter = new SlewRateLimiter(2);
  }
  public void setShooterMotor1(double v){
    shooterMotor1.set(shooter1Limiter.calculate(v));
  }
  public void setShooterMotor2(double v){
    shooterMotor2.set(shooter2Limiter.calculate(v));
  }
  public void setConveyorMotor(double v){
    conveyorMotor.set(conveyorLimiter.calculate(v));
  }
  public void setIntakeMotor(double v){
    intakeMotor.set(intakeLimiter.calculate(v));
  }
  public void stopAll(){
    shooterMotor1.set(0);
    shooterMotor2.set(0);
    intakeMotor.set(0);
    conveyorMotor.set(0);
  }
  @Override
  public void periodic() {
    
  }
}
