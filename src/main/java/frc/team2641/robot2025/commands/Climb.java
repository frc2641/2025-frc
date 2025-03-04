package frc.team2641.robot2025.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team2641.robot2025.subsystems.climber.ClimberReal;

public class Climb extends Command {
  private ClimberReal climber = ClimberReal.getInstance();
  private boolean forwards;

  public Climb(boolean forwards) {
    this.forwards = forwards;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if (forwards) climber.extend();
    else climber.retract();
  }

  @Override
  public void end(boolean interrupted) {
    climber.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}