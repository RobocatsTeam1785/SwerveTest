// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Limelight;
import frc.robot.Subsystems.Turret;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.lib.TrialData;
public class PIDTrial extends CommandBase {
  /** Creates a new PIDTrial. */
  private final Timer timer = new Timer();
  private final Timer timerCrit = new Timer();
  private final Timer cancelTimer = new Timer();
  private final double degreeSuccessRadius = 0.5;//0.5 degrees in either directoin, so 1 degree deadzone
  private final double timeSuccessMin = 1.5;//Min time required to be a success
  private final double testTime = 7;//Total runtime of test
  private Turret turret;
  private TrialData trialData = null;
  private int oscillations = 0;
  private double crittime = 0;
  private boolean within = false;
  private double lastPos;
  public PIDTrial(Turret t) {
    addRequirements(t);
    turret=t;
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    timerCrit.stop();
    timerCrit.reset();
    cancelTimer.stop();
    cancelTimer.reset();
    crittime=0;
    oscillations=0;
    within=false;
    turret.setLimelightTracking(true);
    trialData=null;
    lastPos=0;
  }

  @Override
  public void execute() {
    if(Limelight.getXCoordinate()==0 && Math.abs(lastPos)>1){
      cancelTimer.start();
      if(cancelTimer.hasElapsed(1.5)){
        oscillations+=1;
        crittime=0;
        this.cancel();
      }
    }
    else{
      lastPos=Limelight.getXCoordinate();
      cancelTimer.stop();
      cancelTimer.reset();
      if(timer.hasElapsed(testTime)){
        this.cancel();
      }
      if(Math.abs(Limelight.getXCoordinate())<degreeSuccessRadius){
        within=true;
        if(timerCrit.get()==0){
          timerCrit.start();
        }
        else if(timerCrit.hasElapsed(timeSuccessMin) && crittime==0){
          crittime=timer.get()-timeSuccessMin;
        }
      }
      else if(within){
        within=false;
        oscillations+=1;
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
    timer.reset();
    turret.setLimelightTracking(false);
    if(oscillations==0 && crittime!=0){//Critically damped
      trialData = new TrialData(1,oscillations,turret.getp(),crittime);
    }
    else if(oscillations>0){//Under damped
      trialData = new TrialData(2,oscillations,turret.getp(),crittime);
    }
    else{//Over Damped
      trialData = new TrialData(3,oscillations,turret.getp(),crittime);
    }
  }
  public TrialData getTrialData(){
    return trialData;
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
