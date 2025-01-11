package frc.team2641.robot2025.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class Intake extends ParallelCommandGroup {
  public Intake() {
    addCommands(
        new Rev(4),
        new Feed());
  }
}
