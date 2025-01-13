package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Controle;
import frc.robot.commands.SwerveCommand;
import frc.robot.subsystems.SwerveSubsystem;
import java.io.File;
// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// import com.pathplanner.lib.auto.AutoBuilder;

public class RobotContainer {

  // private final SendableChooser<Command> autoChooser;

  private final XboxController xboxControle = new XboxController(Controle.xboxControle);

  private SwerveSubsystem swerveSubsystem = new SwerveSubsystem(
      new File(Filesystem.getDeployDirectory(), "swerve"));


  public RobotContainer() {

    // autoChooser = AutoBuilder.buildAutoChooser();
    // SmartDashboard.putData("Auto", autoChooser);

    setDefaultCommands();
    registerAutoCommands();
    configureBindings();
  }

  private void setDefaultCommands() {
    swerveSubsystem.setDefaultCommand(new SwerveCommand(
      swerveSubsystem, 
      () -> -xboxControle.getLeftY(), 
      () -> -xboxControle.getLeftX(), 
      () -> -xboxControle.getRightX(),
      () -> xboxControle.getRightBumperButtonPressed()));
  }
  
  private void registerAutoCommands() {

  }

  // Heading Correction
  public void setHeadingCorrection(boolean setHeadingCorrection) {
    swerveSubsystem.getSwerveDrive().setHeadingCorrection(setHeadingCorrection);
  }

  // Função onde os eventos (triggers) são configurados
  private void configureBindings() {
    new JoystickButton(xboxControle, XboxController.Button.kA.value)
        .onTrue(new InstantCommand(
          swerveSubsystem::resetGyro));

    new JoystickButton(xboxControle, XboxController.Button.kBack.value).onFalse(new SwerveCommand(
        swerveSubsystem,
        () -> -MathUtil.applyDeadband(xboxControle.getLeftY(), Controle.DEADBAND),
        () -> -MathUtil.applyDeadband(xboxControle.getLeftX(), Controle.DEADBAND),
        () -> -MathUtil.applyDeadband(xboxControle.getRightX(), Controle.DEADBAND),
        () -> xboxControle.getRightBumperPressed()));

  }

  // Função que retorna o autônomo
  public Command getAutonomousCommand() {

    return null;

  }

  // Define os motores como coast ou brake
  public void setMotorBrake(boolean brake) {
    swerveSubsystem.setMotorBrake(brake);
  }
}
