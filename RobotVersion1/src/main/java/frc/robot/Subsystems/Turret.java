// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
public class Turret extends SubsystemBase {
  private final int TURRET_ID = 11;
  private final int RESOLUTION = 0;
  private double POWER_MOD = 0.1;
  private final double kP = 0;
  private final double kI = 0;
  private final double kD = 0;
  private CANSparkMax rotMotor;
  private PIDController pidController;
  private RelativeEncoder encoder;
  private boolean limelightTracking = false;
  
  public Turret() {
    rotMotor = new CANSparkMax(TURRET_ID, MotorType.kBrushless);
    rotMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    pidController = new PIDController(kP, kI, kD);
    pidController.enableContinuousInput(-180, 180);
    encoder=rotMotor.getEncoder();
    encoder.setPositionConversionFactor(RESOLUTION*360);

  }
  public void setP(double p){
    pidController.setP(p);
  }
  public boolean isTrackingLimelight(){
    return limelightTracking;
  }
  public double getp(){
    return pidController.getP();
  }
  public void setLimelightTracking(boolean b){
    limelightTracking=b;
  }
  private double encoderToDegree(){
    double angle=encoder.getPosition()%360;
    if(angle>180){
      angle=-180+(angle-180);
    }
    else if(angle<-180){
      angle=180+(180+angle);
    }
    return angle;
  }
  private double encoderToDegree(double offset){
    double angle=(encoder.getPosition()+offset)%360;
    if(angle>180){
      angle=-180+(angle-180);
    }
    else if(angle<-180){
      angle=180+(180+angle);
    }
    return angle;
  }
  public double getDegree(){
    return encoderToDegree();
  }
  public double getAdjustedDegree(double offset){
    return encoderToDegree(offset);
  }
  public void pidSet(double offset){
    pidController.setSetpoint(getAdjustedDegree(offset));
    setMotor(MathUtil.clamp(pidController.calculate(getDegree()),-1,1));
  }
  public void setMotor(double p){
    if(!limelightTracking){
      rotMotor.set(POWER_MOD*p);  
    }
  }
  public void increasePowerMod(){
    POWER_MOD+=0.1;
    if(POWER_MOD>1){
      POWER_MOD=1;
    }
  }
  public void decreasePowerMod(){
    POWER_MOD-=1;
    if(POWER_MOD<0.1){
      POWER_MOD=0.1;
    }
  }
  @Override
  public void periodic() {
    if(limelightTracking){
      if(Limelight.getXCoordinate()!=0){
        pidSet(Limelight.getXCoordinate());//Might need to pass -x instead of x???
      }
    }
  }
}
