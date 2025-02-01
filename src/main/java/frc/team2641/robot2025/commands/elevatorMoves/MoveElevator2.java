package frc.team2641.robot2025.commands.elevatorMoves;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team2641.robot2025.subsystems.Elevator;

public class MoveElevator2 extends Command {
  private Elevator elev;
  private int stage = 2;

  public MoveElevator2() {
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