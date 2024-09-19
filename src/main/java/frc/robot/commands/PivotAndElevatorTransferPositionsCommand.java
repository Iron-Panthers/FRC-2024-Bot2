// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class PivotAndElevatorTransferPositionsCommand extends ParallelCommandGroup {
  /** Creates a new PivotAndElevatorTransferPositionsCommand. */
  public PivotAndElevatorTransferPositionsCommand(
    PivotSubsystem pivotSubsystem,
    ElevatorSubsystem elevatorSubsystem) {
      addCommands(new PivotAngleCommand(pivotSubsystem, 20),
        new ElevatorHeightCommand(elevatorSubsystem, 0));
  }
}
