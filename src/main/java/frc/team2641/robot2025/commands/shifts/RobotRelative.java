// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team2641.robot2025.commands.shifts;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotRelative extends Command {
  BooleanPublisher robotPub;

  /** Creates a new RobotRelative. */
  public RobotRelative() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("state");

    robotPub = table.getBooleanTopic("robotRelative").publish();
    robotPub.set(false);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    robotPub.set(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    robotPub.set(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
