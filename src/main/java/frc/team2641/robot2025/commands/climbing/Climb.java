// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team2641.robot2025.commands.climbing;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team2641.robot2025.subsystems.Climber;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Climb extends Command {

  private Climber climber;
  private boolean forwards;
  /** Creates a new Extend. */
  public Climb(boolean forwards) {
    // Use addRequirements() here to declare subsystem dependencies.
    climber = Climber.getInstance();
    this.forwards = forwards;

    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(forwards)
      climber.extend();
    else
    climber.retract();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void   execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
