package frc.team2641.robot2025.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2641.robot2025.Robot;

public class Climber extends SubsystemBase {
  public static ClimberIO getInstance() {
    if (!Robot.isSimulation()) return ClimberReal.getInstance();
    return ClimberSim.getInstance();
  }
}