package frc.team2641.robot2025.subsystems.superstructure.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2641.robot2025.Robot;

public class Elevator extends SubsystemBase implements ElevatorIO {
  public static Elevator getInstance() {
    if (!Robot.isSimulation()) {
      // throw new Error("elev likey to live");
      return ElevatorReal.getInstance();
    }
    return ElevatorSimulation.getInstance();
    // throw new Error("elev no likey reality, sim work");
  }

  @Override
  public void goTo(double pos) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'goTo'");
  }

  @Override
  public double getPosition() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getPosition'");
  }

  @Override
  public double getSetpoint() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getSetpoint'");
  }

  @Override
  public void stop() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'stop'");
  }
}