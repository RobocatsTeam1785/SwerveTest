// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.TrialData;
import frc.robot.Commands.PIDTrial;
import frc.robot.Subsystems.Turret;
import java.util.ArrayList;
public class PIDTester extends CommandBase {
  private double largestOverdampedValue=0;
  private double smallestUnderdampedValue=0.5;
  private ArrayList<TrialData> trialDatas;
  private PIDTrial pidTrial;
  private int numTrials=0;
  Turret turret;
  public PIDTester(Turret t) {
    //addRequirements(t);
    turret=t;
    pidTrial = new PIDTrial(turret);
    trialDatas=new ArrayList();
  }

  @Override
  public void initialize() {
    turret.setP((smallestUnderdampedValue-largestOverdampedValue)/2);
    pidTrial.schedule();
  }

  @Override
  public void execute() {
    if(!pidTrial.isScheduled()){
      this.cancel();
    }
  }

  @Override
  public void end(boolean interrupted) {
    trialDatas.add(pidTrial.getTrialData());
    if(trialDatas.get(trialDatas.size()-1).getOverDamped()){
      largestOverdampedValue=(smallestUnderdampedValue-largestOverdampedValue)/2;
    }
    else if(trialDatas.get(trialDatas.size()-1).getUnderDamped()){
      smallestUnderdampedValue=(smallestUnderdampedValue-largestOverdampedValue)/2;
    }
    numTrials++;
    displayData(numTrials-1);
  }
  public void displayData(){
    for(int i=0;i<trialDatas.size();i++){
      String output = "Trial num: "+ String.valueOf(i+1)+"\n     ";
      output+="Type: "+String.valueOf(trialDatas.get(i).getType())+"\n     ";
      output+="Oscillations: "+String.valueOf(trialDatas.get(i).getOscillations())+"\n     ";
      output+="Critical Time: "+String.valueOf(trialDatas.get(i).getCritTime())+"\n     ";
      output+="P value: "+String.valueOf(trialDatas.get(i).getp());
      System.out.println(output);
    }
  }
  public void displayData(int i){
    String output = "Trial num: "+ String.valueOf(i+1)+"\n     ";
    output+="Type: "+String.valueOf(trialDatas.get(i).getType())+"\n     ";
    output+="Oscillations: "+String.valueOf(trialDatas.get(i).getOscillations())+"\n     ";
    output+="Critical Time: "+String.valueOf(trialDatas.get(i).getCritTime())+"\n     ";
    output+="P value: "+String.valueOf(trialDatas.get(i).getp());
    System.out.println(output);
  }
  public PIDTrial getPidTrial(){
    return pidTrial;
  }
  @Override
  public boolean isFinished() {
    return false;
  }
}