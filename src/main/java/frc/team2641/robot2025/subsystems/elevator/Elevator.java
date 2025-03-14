package frc.team2641.robot2025.subsystems.elevator;

import frc.team2641.robot2025.Robot;

public class Elevator  {

  public static ElevatorIO getInstance() {
    if (!Robot.isSimulation()) {
      // throw new Error("elev likey to live");
      return ElevatorReal.getInstance();
    }
    return ElevatorSimulation.getInstance();
    // throw new Error("elev no likey reality, sim work");
  }
        
 }