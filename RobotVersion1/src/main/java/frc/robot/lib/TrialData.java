package frc.robot.lib;

public class TrialData{
    private boolean criticallyDamped;
    private boolean overDamped;
    private boolean underDamped;
    private int oscillations;
    private double critTime;
    private double p;
    /**
     * 
     * @param r 1 is critically damped, 2 is over damped, 3 is under damped
     * @param o number of times oscillated during trial
     * @param p the p value for the PIDController
     * @param end_time the time it took to reach the deadzone
     */
    public TrialData(int r, int o, double p, double end_time){
      criticallyDamped=false;
      overDamped=false;
      underDamped=false;
      if(r==1){
        criticallyDamped=!criticallyDamped;
      }
      else if(r==2){
        overDamped=!overDamped;
      }
      else{
        underDamped=!underDamped;
      }
      oscillations=o;
      critTime=end_time;
      this.p=p;
    }
    public String getType(){
      if(criticallyDamped){
        return "critically damped";
      }
      else if(overDamped){
        return "over damped";
      }
      else{
        return "under damped";
      }
    }
    public boolean getCriticallyDamped(){
        return criticallyDamped;
    }
    public boolean getOverDamped(){
        return overDamped;
    }
    public boolean getUnderDamped(){
        return underDamped;
    }
    public int getOscillations(){
      return oscillations;
    }
    public double getCritTime(){
      return critTime;
    }
    public double getp(){
      return p;
    }
  }