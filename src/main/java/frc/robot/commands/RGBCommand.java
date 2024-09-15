// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.RGBSubsystem;
import frc.robot.subsystems.RGBSubsystem.RGBMessage;
import frc.robot.subsystems.ShooterSubsystem.ShooterMode;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import java.util.Optional;

public class RGBCommand extends Command {
  private ShooterSubsystem shooterSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private RGBSubsystem rgbSubsystem;
  private PivotSubsystem pivotSubsystem;
  private DrivebaseSubsystem drivebaseSubsystem;
  private VisionSubsystem visionSubsystem;
  private Optional<RGBMessage> noteInRobotMsg;
  private Optional<RGBMessage> readyToShootMsg;
  private Optional<RGBMessage> twoNoteMsg;
  private int sensorCounter;

  /** Creates a new RGBCommand. */
  public RGBCommand(
      ShooterSubsystem shooterSubsystem,
      IntakeSubsystem intakeSubsystem,
      RGBSubsystem rgbSubsystem,
      PivotSubsystem pivotSubsystem,
      DrivebaseSubsystem drivebaseSubsystem,
      VisionSubsystem visionSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.\
    addRequirements(rgbSubsystem);
    this.shooterSubsystem = shooterSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.rgbSubsystem = rgbSubsystem;
    this.pivotSubsystem = pivotSubsystem;
    this.drivebaseSubsystem = drivebaseSubsystem;
    this.visionSubsystem = visionSubsystem;
    twoNoteMsg = Optional.empty();
    readyToShootMsg = Optional.empty();
    noteInRobotMsg = Optional.empty();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // two note = yellow

    if (shooterSubsystem.isShooterBeamBreakSensorTriggered()
        && shooterSubsystem.isSerializerBeamBreakSensorTriggered()
        && twoNoteMsg.isEmpty()) {
      twoNoteMsg =
          Optional.of(
              rgbSubsystem.showMessage(
                  Constants.Lights.Colors.YELLOW,
                  RGBSubsystem.PatternTypes.PULSE,
                  RGBSubsystem.MessagePriority.C_TWO_NOTE_WARNING));
    } else if (!(shooterSubsystem.isShooterBeamBreakSensorTriggered()
        && shooterSubsystem.isSerializerBeamBreakSensorTriggered())) {
      twoNoteMsg.ifPresent(RGBMessage::expire);
      twoNoteMsg = Optional.empty();
    }

    // serializer = blue
    if ((shooterSubsystem.isShooterBeamBreakSensorTriggered()
            || shooterSubsystem.isSerializerBeamBreakSensorTriggered())
        && noteInRobotMsg.isEmpty()) {
      /*|| shooterSubsystem.isBeamBreakSensorTriggered()*/
      noteInRobotMsg =
          Optional.of(
              rgbSubsystem.showMessage(
                  Constants.Lights.Colors.WHITE,
                  RGBSubsystem.PatternTypes.STROBE,
                  RGBSubsystem.MessagePriority.F_NOTE_IN_ROBOT));
    } else if (!(shooterSubsystem.isShooterBeamBreakSensorTriggered()
        || shooterSubsystem.isSerializerBeamBreakSensorTriggered())) {
      noteInRobotMsg.ifPresent(RGBMessage::expire);
      noteInRobotMsg = Optional.empty();
    }

    // ready to shoot = red
    if (isReadyToShootInSun()
        && pivotSubsystem.isAtTargetDegrees()
        && Math.abs(drivebaseSubsystem.getAngularError()) < 2
        && readyToShootMsg.isEmpty()
        && visionSubsystem.getCanSeeSpeakerTags()) {
      readyToShootMsg =
          Optional.of(
              rgbSubsystem.showMessage(
                  Constants.Lights.Colors.RED,
                  RGBSubsystem.PatternTypes.STROBE,
                  RGBSubsystem.MessagePriority.D_READY_TO_SHOOT));
    } else if (shooterSubsystem.getMode().equals(ShooterMode.SHOOT_SPEAKER) || //if it shot the note, it is not ready to shoot instead of relying on beam break sensor input
      shooterSubsystem.getMode().equals(ShooterMode.SHOOT_SHUTTLE)||
      shooterSubsystem.getMode().equals(ShooterMode.SHOOT_VAR)||
      pivotSubsystem.isAtTargetDegrees()){
        readyToShootMsg.ifPresent(RGBMessage::expire);
        readyToShootMsg = Optional.empty();
    }
  }

  private boolean isReadyToShootInSun(){
    if(shooterSubsystem.isReadyToShoot()){
      sensorCounter +=1;
    }
    if (sensorCounter>=38){//waits 0.75 seconds to varify the note did not leave robot
      return true;
    }
    else{
      sensorCounter = 0;
      return false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
