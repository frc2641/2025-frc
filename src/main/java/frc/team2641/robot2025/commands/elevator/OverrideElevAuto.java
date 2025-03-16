// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team2641.robot2025.commands.elevator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team2641.robot2025.subsystems.elevator.Elevator;
import frc.team2641.robot2025.subsystems.elevator.ElevatorIO;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OverrideElevAuto extends InstantCommand {
  private ElevatorIO elev;
  public OverrideElevAuto() {

    elev = Elevator.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elev.override();
  }
}
