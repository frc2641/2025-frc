// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team2641.robot2025.commands.climbing;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team2641.robot2025.subsystems.climber.ClimberReal;
import frc.team2641.robot2025.subsystems.climber.Winch;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Extend extends Command {

  private ClimberReal climber;
  private Winch winch;
  /** Creates a new Extend. */
  public Extend() {
    // Use addRequirements() here to declare subsystem dependencies.
    climber = ClimberReal.getInstance();
    winch = Winch.getInstance();


    addRequirements(climber, winch);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      climber.extend();
      winch.extend();
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
