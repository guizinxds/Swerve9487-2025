// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Controle;
import frc.robot.Constants.StateStrings;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import swervelib.SwerveInputStream;

public class SwerveCommand extends Command {

  private final SwerveSubsystem swerveSubsystem;

  private final DoubleSupplier y;
  private final DoubleSupplier x;
  private final DoubleSupplier turn;
  private final BooleanSupplier toggleSpeed;
  
  private String fastSpeedMode = StateStrings.ON;

  public SwerveCommand(
    SwerveSubsystem swerveSubsystem,
    DoubleSupplier y,
    DoubleSupplier x,
    DoubleSupplier turn,
    BooleanSupplier toggleSpeed
  ) {
    this.y = y;
    this.x = x;
    this.turn = turn;
    this.swerveSubsystem = swerveSubsystem;
    this.toggleSpeed = toggleSpeed;

    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (toggleSpeed.getAsBoolean()) {
      switch (fastSpeedMode) {
        case StateStrings.ON:
          fastSpeedMode = StateStrings.OFF;
          break;
  
        case StateStrings.OFF:
          fastSpeedMode = StateStrings.ON;
          break;
      }
    }
    
    switch (fastSpeedMode) {
      case StateStrings.ON:
        drive(1.0);
        break;

      case StateStrings.OFF:
        drive(0.3);
        break;
    }
  }

  private void drive(double scaleTranslation) {
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
        swerveSubsystem.getSwerveDrive(),
        y,
        x)
        .withControllerRotationAxis(turn)
        .deadband(Controle.DEADBAND)
        .scaleTranslation(scaleTranslation)
        .scaleRotation(scaleTranslation)
        .allianceRelativeControl(true);

      swerveSubsystem.driveFieldOriented(driveAngularVelocity);
  }

  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
