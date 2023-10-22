package frc.robot;
import frc.robot.Subsystems.*;
import frc.robot.Commands.*;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
public class RobotContainer {
    private final XboxController controller = new XboxController(0);
    private final Turret turret = new Turret();
    private final PIDTester pidTester = new PIDTester(turret);
    public RobotContainer(){
        new JoystickButton(controller, Button.kA.value).onTrue(new InstantCommand(() -> sPIDTester()));
        new JoystickButton(controller, Button.kY.value).onTrue(new InstantCommand(() -> toggleTracking()));
        new JoystickButton(controller, Button.kB.value).onTrue(new InstantCommand(() -> kill()));
        new JoystickButton(controller, Button.kX.value).onTrue(new InstantCommand(() -> pidTester.displayData()));
    }
    public void sPIDTester(){
        pidTester.schedule();
    }
    public void toggleTracking(){
        turret.setLimelightTracking(!turret.isTrackingLimelight());
    }
    public void kill(){
        pidTester.getPidTrial().cancel();
        pidTester.cancel();
        turret.setLimelightTracking(false);
    }


}
