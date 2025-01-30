package frc.team2641.robot2025.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team2641.robot2025.subsystems.Elevator;

public class MoveElevator extends Command {
  private Elevator elev;
  private int stage;

  public MoveElevator(int stage) {
    this.stage = stage;
    this.elev = Elevator.getInstance();
    addRequirements(elev);
  }
  

  @Override
  public void initialize() {
    elev.setPos(stage);
  }

  @Override
  public void execute() {
    
  }

  @Override
  public void end(boolean interrupted) {
    elev.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}