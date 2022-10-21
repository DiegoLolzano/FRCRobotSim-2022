// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveCommand extends CommandBase {
  /** Creates a new DriveCommand. */
  DriveTrain m_drive;
  Supplier<Double> speed, turn;
  public DriveCommand(DriveTrain drive, Supplier<Double> speed, Supplier<Double> turn) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_drive = drive;
    this.speed = speed;
    this.turn = turn;

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.zeroHeading();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.setTeleopControl(speed.get(), turn.get());

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
