package frc.team2641.robot2025.commands.superStructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team2641.robot2025.Constants;
import frc.team2641.robot2025.Robot;
import frc.team2641.robot2025.subsystems.superstructure.elevator.ElevatorReal;

public class MoveElev extends Command {
  private ElevatorReal elev;

  public MoveElev() {
    elev = ElevatorReal.getInstance();
    addRequirements(elev);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    double rightY = MathUtil.applyDeadband(Robot.getInstance().robotContainer.getOpRightStickY(), 0.05);
    double output = elev.getSetpoint() + rightY * Constants.ElevatorConstants.elevatorSpeed;
    // elev.goTo(output);
    if(rightY>0.5)
    elev.goTo(0);
    else elev.goTo(10);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
