// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.Timer;
public class DriveSubsystem extends SubsystemBase {
  private boolean isDriving=false;
  private boolean isPrinting=false;
  private boolean setMode=false;
  public boolean isBrake;
  private double maxV=3;
  private CANSparkMax drive1 = new CANSparkMax(1,MotorType.kBrushless);
  private CANSparkMax drive2 = new CANSparkMax(2,MotorType.kBrushless);
  private CANSparkMax drive3 = new CANSparkMax(3,MotorType.kBrushless);
  private CANSparkMax drive4 = new CANSparkMax(4,MotorType.kBrushless);
  private CANSparkMax rot1 = new CANSparkMax(5,MotorType.kBrushless);
  private CANSparkMax rot2 = new CANSparkMax(6,MotorType.kBrushless);
  private CANSparkMax rot3 = new CANSparkMax(7,MotorType.kBrushless);
  private CANSparkMax rot4 = new CANSparkMax(8,MotorType.kBrushless);
  private RelativeEncoder encoder1 = drive1.getEncoder();
  private RelativeEncoder encoder2 = drive2.getEncoder();
  private RelativeEncoder encoder3 = drive3.getEncoder();
  private RelativeEncoder encoder4 = drive4.getEncoder();
  private RelativeEncoder encoderTurn1 = rot1.getEncoder();
  private RelativeEncoder encoderTurn2 = rot2.getEncoder();
  private RelativeEncoder encoderTurn3 = rot3.getEncoder();
  private RelativeEncoder encoderTurn4 = rot4.getEncoder();
  private final Timer timer = new Timer();
  


  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    drive1.setIdleMode(CANSparkMax.IdleMode.kBrake);
    drive2.setIdleMode(CANSparkMax.IdleMode.kBrake);
    drive3.setIdleMode(CANSparkMax.IdleMode.kBrake);
    drive4.setIdleMode(CANSparkMax.IdleMode.kBrake);
    rot1.setIdleMode(CANSparkMax.IdleMode.kBrake);
    rot2.setIdleMode(CANSparkMax.IdleMode.kBrake);
    rot3.setIdleMode(CANSparkMax.IdleMode.kBrake);
    rot4.setIdleMode(CANSparkMax.IdleMode.kBrake);
    timer.start();
  }
  public void setVoltageAllDrive(double v){
    drive1.setVoltage(v);
    drive2.setVoltage(v);
    drive3.setVoltage(v);
    drive4.setVoltage(v);
  }
  public void setSetAllDrive(double s){
    drive1.set(s);
    drive2.set(s);
    drive3.set(s);
    drive4.set(s);
  }
  public void setVoltageAllRot(double v){
    rot1.setVoltage(v);
    rot2.setVoltage(v);
    rot3.setVoltage(v);
    rot4.setVoltage(v);
  }
  public void setSetAllRot(double s){
    rot1.set(s);
    rot2.set(s);
    rot3.set(s);
    rot4.set(s);
  }
  public boolean getPrinting(){
    return isPrinting;
  }
  public boolean getSet(){
    return setMode;
  }
  public void setPrinting(boolean p){
    isPrinting=p;
  }
  public void setSet(boolean s){
    setMode=s;
  }
  public void setBrake(boolean b){
    if(b){
      drive1.setIdleMode(CANSparkMax.IdleMode.kBrake);
      drive2.setIdleMode(CANSparkMax.IdleMode.kBrake);
      drive3.setIdleMode(CANSparkMax.IdleMode.kBrake);
      drive4.setIdleMode(CANSparkMax.IdleMode.kBrake);
      rot1.setIdleMode(CANSparkMax.IdleMode.kBrake);
      rot2.setIdleMode(CANSparkMax.IdleMode.kBrake);
      rot3.setIdleMode(CANSparkMax.IdleMode.kBrake);
      rot4.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }
    else{
      drive1.setIdleMode(CANSparkMax.IdleMode.kCoast);
      drive2.setIdleMode(CANSparkMax.IdleMode.kCoast);
      drive3.setIdleMode(CANSparkMax.IdleMode.kCoast);
      drive4.setIdleMode(CANSparkMax.IdleMode.kCoast);
      rot1.setIdleMode(CANSparkMax.IdleMode.kCoast);
      rot2.setIdleMode(CANSparkMax.IdleMode.kCoast);
      rot3.setIdleMode(CANSparkMax.IdleMode.kCoast);
      rot4.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }
    isBrake=b;
  }
  public void drive(double power, double rot){
    if(setMode){
      setSetAllDrive(power);
      setSetAllRot(rot);
    }
    else{
      setVoltageAllDrive(maxV*power);
      setSetAllRot(maxV*rot);
    }
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(isPrinting){
      System.out.println("******* Drive1    Voltage: "+String.valueOf(drive1.getBusVoltage()) + "  Velocity: "+String.valueOf(encoder1.getVelocity()) + "   Time: " + String.valueOf(timer.get()));
      System.out.println("******* Drive1    Voltage: "+String.valueOf(drive2.getBusVoltage()) + "  Velocity: "+String.valueOf(encoder2.getVelocity()) + "   Time: " + String.valueOf(timer.get()));
      System.out.println("******* Drive1    Voltage: "+String.valueOf(drive3.getBusVoltage()) + "  Velocity: "+String.valueOf(encoder3.getVelocity()) + "   Time: " + String.valueOf(timer.get()));
      System.out.println("******* Drive1    Voltage: "+String.valueOf(drive4.getBusVoltage()) + "  Velocity: "+String.valueOf(encoder4.getVelocity()) + "   Time: " + String.valueOf(timer.get()));
      System.out.println("******* Turn1     Voltage: "+String.valueOf(rot1.getBusVoltage()) + "  Velocity: "+String.valueOf(encoderTurn1.getVelocity()) + "   Time: " + String.valueOf(timer.get()));
      System.out.println("******* Turn2     Voltage: "+String.valueOf(rot2.getBusVoltage()) + "  Velocity: "+String.valueOf(encoderTurn2.getVelocity()) + "   Time: " + String.valueOf(timer.get()));
      System.out.println("******* Turn3     Voltage: "+String.valueOf(rot3.getBusVoltage()) + "  Velocity: "+String.valueOf(encoderTurn3.getVelocity()) + "   Time: " + String.valueOf(timer.get()));
      System.out.println("******* Turn4     Voltage: "+String.valueOf(rot4.getBusVoltage()) + "  Velocity: "+String.valueOf(encoderTurn4.getVelocity()) + "   Time: " + String.valueOf(timer.get()));

    }
  }
}
