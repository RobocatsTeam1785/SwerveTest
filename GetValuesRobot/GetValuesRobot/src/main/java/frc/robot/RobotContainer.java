// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Controllers are defined here
  private final XboxController controller = new XboxController(0);
  private final AutoCommand auto = new AutoCommand();
  // Subsystems are defined here
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureBindings();
    driveSubsystem.setDefaultCommand(
        new RunCommand(() -> driveSubsystem.drive(controller.getLeftY(), controller.getLeftX())));

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    //Driver Controller
    new JoystickButton(controller, Button.kA.value).onTrue(new InstantCommand(() -> togglePrint()));
    new JoystickButton(controller, Button.kB.value).onTrue(new InstantCommand(() -> toggleMode()));
    new JoystickButton(controller, Button.kX.value).onTrue(new InstantCommand(() -> toggleBrake()));

  }
  public void togglePrint(){
    driveSubsystem.setPrinting(!driveSubsystem.getPrinting());
  }
  public void toggleMode(){
    driveSubsystem.setSet(!driveSubsystem.getSet());
  }
  public void toggleBrake(){
    driveSubsystem.setBrake(!driveSubsystem.isBrake);
  }
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return auto;
  }
}
