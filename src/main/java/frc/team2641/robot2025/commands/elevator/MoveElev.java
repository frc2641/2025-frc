package frc.team2641.robot2025.commands.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team2641.robot2025.Constants;
import frc.team2641.robot2025.Robot;
import frc.team2641.robot2025.helpers.ElevatorConstrain;
import frc.team2641.robot2025.subsystems.Elevator;

public class MoveElev extends Command {
  private Elevator elev;

  public MoveElev() {
    elev = Elevator.getInstance();
    addRequirements(elev);
  }

  @Override
  public void initialize() {



  }

  @Override
  public void execute() {

    // if(!elev.getAuto()){
    double rightY = -MathUtil.applyDeadband(Robot.getInstance().robotContainer.getOpRightStickY(), 0.1);

    double output = elev.getSetpoint() + rightY * Constants.ElevatorConstants.elevatorSpeed; //* (Robot.isReal() ? 1 : 0.1);
    // double output = rightY * Constants.ElevatorConstants.elevatorSpeed * (Robot.isReal() ? 1 : 0.1);

    output = new ElevatorConstrain(output).get();
    elev.goTo(output);
    // }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
