package frc.team2641.robot2025;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public class Alerts {
  public static Alert MissingDriverGamepad = new Alert("Driver gamepad is missing", AlertType.kError);
  public static Alert MissingOperatorGamepad = new Alert("Operator gamepad is missing", AlertType.kError);
  public static Alert NoStartPose = new Alert("Invalid starting pose", AlertType.kError);
  public static Alert InvalidAuto = new Alert("Autonomous invalid, falling back to Creep", AlertType.kError);
  public static Alert ValidAuto = new Alert("Autonomous ready", AlertType.kInfo);
  public static Alert AutoOff = new Alert("Autonomous disabled", AlertType.kWarning);
}
