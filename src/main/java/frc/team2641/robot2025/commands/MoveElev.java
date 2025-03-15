package frc.team2641.robot2025.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team2641.robot2025.Constants;
import frc.team2641.robot2025.Robot;
import frc.team2641.robot2025.helpers.ElevatorContrain;
import frc.team2641.robot2025.subsystems.elevator.Elevator;
import frc.team2641.robot2025.subsystems.elevator.ElevatorIO;

public class MoveElev extends Command {
  private ElevatorIO elev;

  public MoveElev() {
    elev = Elevator.getInstance();
    addRequirements(elev);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double rightY = -MathUtil.applyDeadband(Robot.getInstance().robotContainer.getOpRightStickY(), 0.05);

    double output = elev.getSetpoint() + rightY * Constants.ElevatorConstants.elevatorSpeed * (Robot.isReal() ? 1 : 0.1);
    // double output = rightY * Constants.ElevatorConstants.elevatorSpeed * (Robot.isReal() ? 1 : 0.1);

    output = new ElevatorContrain(output).get();
    elev.goTo(output);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
