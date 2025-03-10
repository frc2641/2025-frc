package frc.team2641.robot2025.commands.superStructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team2641.robot2025.Constants;
import frc.team2641.robot2025.Robot;
import frc.team2641.robot2025.helpers.ArmPosition;
import frc.team2641.robot2025.subsystems.superstructure.elevator.Elevator;
import frc.team2641.robot2025.subsystems.superstructure.elevator.ElevatorIO;

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
    double rightY = MathUtil.applyDeadband(Robot.getInstance().robotContainer.getOpRightStickY(), 0.05);

    double output = elev.getSetpoint() + -1 * rightY * Constants.ElevatorConstants.elevatorSpeed * (Robot.isReal() ? 1 : 0.1);

    output = new ArmPosition(0, output).getElev();
    
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
