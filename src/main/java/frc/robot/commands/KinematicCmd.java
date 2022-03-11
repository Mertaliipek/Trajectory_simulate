// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class KinematicCmd extends CommandBase {
  private final DriveTrain m_drive;
  private final double xSpeed;
  private final double rot;

  
  /** Creates a new Kinematic. */
  public KinematicCmd(DriveTrain m_drive,double xSpeed,double rot) {
    this.m_drive = m_drive;
    this.xSpeed = xSpeed;
    this.rot = rot;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.drive(xSpeed, rot);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
