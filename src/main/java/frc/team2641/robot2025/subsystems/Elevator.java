package frc.team2641.robot2025.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2641.robot2025.Robot;

public class Elevator extends SubsystemBase {
    private static Elevator instance;

    public static Elevator getInstance() {
      if (instance == null)
        instance = new Elevator();
      return instance;
    }   

    private Elevator()
    {

    }

    
  @Override
  public void periodic() {
  }
}
