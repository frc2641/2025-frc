package frc.team2641.robot2025.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.team2641.robot2025.subsystems.Climber;
import frc.team2641.robot2025.Constants;

public class Climb extends Command {
  private Climber climber;
  private boolean extended;

  public Climb() {
    this.climber = Climber.getInstance();
    addRequirements(climber);
    extended = false;
  }

  @Override
  public void initialize() {
    extended = !extended;
  }

  @Override
  public void execute() {
    if(extended)
    climber.up();
    else
    climber.down();
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    if(extended)
      return climber.isAt(Constants.ClimberPositions.extended);
      else
      return climber.isAt(Constants.ClimberPositions.start);
  }
}