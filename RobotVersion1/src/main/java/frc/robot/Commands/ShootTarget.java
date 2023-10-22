// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Shooter;
import edu.wpi.first.wpilibj.Timer;
public class ShootTarget extends CommandBase {
  private final Timer timer = new Timer();
  private double power = 0;
  private final Shooter shooter;
  private boolean firstWaitDone=false;
  public ShootTarget(Shooter s) {
    addRequirements(s);
    shooter=s;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    shooter.setShooterMotor1(power);
    firstWaitDone=false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(timer.hasElapsed(0.7) && !firstWaitDone){
      timer.reset();
      shooter.setConveyorMotor(0.2);
      firstWaitDone=!firstWaitDone;
    }
    else if(timer.hasElapsed(2)){
      this.cancel();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    timer.reset();
    shooter.stopAll();
  }

  public void setPower(double p){
    power=p;
  }
  public double getPower(){
    return power;
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
