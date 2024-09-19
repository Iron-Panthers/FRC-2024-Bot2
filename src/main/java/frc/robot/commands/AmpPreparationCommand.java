// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Elevator;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AmpPreparationCommand extends SequentialCommandGroup {
  /** Creates a new AmpPreparationCommand. */
  public AmpPreparationCommand(
    PivotSubsystem pivotSubsystem,
    ElevatorSubsystem elevatorSubsystem,
    ShooterSubsystem shooterSubsystem) {
    if (shooterSubsystem.isSerializerBeamBreakSensorTriggered()){
      addCommands(new ElevatorHeightCommand(elevatorSubsystem, Elevator.AMP_HEIGHT));
    }
    else{
      addCommands( 
          new PivotAndElevatorTransferPositionsCommand(pivotSubsystem, elevatorSubsystem)
        .andThen(
          new UnloadShooterCommand(shooterSubsystem, pivotSubsystem, elevatorSubsystem))
        .andThen(
          new ElevatorHeightCommand(elevatorSubsystem, Elevator.AMP_HEIGHT)));
    }
  }
}
