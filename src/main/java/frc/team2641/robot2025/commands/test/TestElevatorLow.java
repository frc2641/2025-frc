package frc.team2641.robot2025.commands.test;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team2641.robot2025.subsystems.superstructure.elevator.ElevatorReal;
import frc.team2641.robot2025.Constants.ElevatorConstants;

public class TestElevatorLow extends Command {
  ElevatorReal elevator = ElevatorReal.getInstance();

  public TestElevatorLow() {
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    elevator.goTo(ElevatorConstants.minPos);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return elevator.getPosition() <= ElevatorConstants.minPos;
  }
}
