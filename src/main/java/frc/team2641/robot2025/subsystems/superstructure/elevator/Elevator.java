package frc.team2641.robot2025.subsystems.superstructure.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2641.robot2025.Robot;

public class Elevator extends SubsystemBase {
  public static ElevatorIO getInstance() {
    if (!Robot.isSimulation()) {
      // throw new Error("elev likey to live");
      return ElevatorReal.getInstance();
    }
    return ElevatorSimulation.getInstance();
    // throw new Error("elev no likey reality, sim work");
  }
}