package frc.team2641.robot2025.commands.elevator;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team2641.robot2025.Constants;
import frc.team2641.robot2025.subsystems.Elevator;
import frc.team2641.robot2025.Constants.ELEVNUM;
import frc.team2641.robot2025.Constants.ElevatorPositions;

public class SetElevator extends InstantCommand {
  private Elevator elevator = Elevator.getInstance();
  private double setpoint;

  public SetElevator(double setpoint) {
    this.setpoint = setpoint;
    addRequirements(elevator);
  }

  public SetElevator(Constants.ELEVNUM pos) {
    setSetpoint(pos);
    addRequirements(elevator);
  }

  public SetElevator(SendableChooser<ELEVNUM> pos){
    setSetpoint(pos.getSelected());
    pos.onChange((ELEVNUM newPos) -> {
      setSetpoint(newPos);
    });
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    elevator.goTo(setpoint);
    end(false);
  }

  private void setSetpoint(ELEVNUM pos){
    setpoint = 0;

    if (pos != null) {
      switch (pos) {
        case L1:
          setpoint = ElevatorPositions.L1;
          break;

        case L2:
          setpoint = ElevatorPositions.L2;
          break;

        case L3:
          setpoint = ElevatorPositions.L3;
          break;

        case L4:
          setpoint = ElevatorPositions.L4;
          break;

        case HP:
          setpoint = ElevatorPositions.HP;
          break;

        default:
          setpoint = 0;
          break;
      }
    }
  }
}