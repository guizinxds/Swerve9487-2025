package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Controle;
import frc.robot.commands.SwerveCommand;
import frc.robot.subsystems.SwerveSubsystem;
import java.io.File;

public class RobotContainer {


  private final SendableChooser<Command> autoChooser;

  private SwerveSubsystem swerve = new SwerveSubsystem(
    new File(Filesystem.getDeployDirectory(), "swerve")
  );

  private final XboxController xboxControle = new XboxController(
    Controle.xboxControle
  );

  public RobotContainer() {

    autoChooser = AutoBuilder.buildAutoChooser("a pecuaria tentara");
    SmartDashboard.putData("Auto", autoChooser);


    setDefaultCommands();
    registerAutoCommands();
    configureBindings();
  }

  private void setDefaultCommands() {
     if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      swerve.setDefaultCommand(
      new SwerveCommand(
        swerve,
        () ->
          -MathUtil.applyDeadband(xboxControle.getLeftY(), Controle.DEADBAND),
        () ->
          -MathUtil.applyDeadband(xboxControle.getLeftX(), Controle.DEADBAND),
        () ->
          -MathUtil.applyDeadband(xboxControle.getRightX(), Controle.DEADBAND),
        () -> xboxControle.getRightBumperPressed()
      )
    );
    } else {
      swerve.setDefaultCommand(
      new SwerveCommand(
        swerve,
        () ->
          MathUtil.applyDeadband(xboxControle.getLeftY(), Controle.DEADBAND),
        () ->
          MathUtil.applyDeadband(xboxControle.getLeftX(), Controle.DEADBAND),
        () ->
          MathUtil.applyDeadband(xboxControle.getRightX(), Controle.DEADBAND),
        () -> xboxControle.getRightBumperPressed()
      )
    );
    }
  }
  
  private void registerAutoCommands() {

  }

  //Heading Correction 
  public void setHeadingCorrection(boolean setHeadingCorrection){
    swerve.swerveDrive.setHeadingCorrection(setHeadingCorrection);
  }   

  // Função onde os eventos (triggers) são configurados 
  private void configureBindings() {
    new JoystickButton(xboxControle, XboxController.Button.kA.value)
      .onTrue(new InstantCommand(
        swerve::resetGyro
        ));

    new JoystickButton(xboxControle,XboxController.Button.kBack.value).onFalse(new SwerveCommand(
        swerve,
        () ->
          -MathUtil.applyDeadband(xboxControle.getLeftY(), Controle.DEADBAND),
        () ->
          -MathUtil.applyDeadband(xboxControle.getLeftX(), Controle.DEADBAND),
        () ->
          -MathUtil.applyDeadband(xboxControle.getRightX(), Controle.DEADBAND),
        () -> xboxControle.getRightBumperPressed()
      )
    );

  }

  // Função que retorna o autônomo
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  // Define os motores como coast ou brake
  public void setMotorBrake(boolean brake) {
    swerve.setMotorBrake(brake);
  }
}
