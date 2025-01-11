package frc.team2641.robot2025.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2641.robot2025.Constants;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Indexer extends SubsystemBase {
  private static Indexer instance;

  public static Indexer getInstance() {
    if (instance == null)
      instance = new Indexer();
    return instance;
  }

  public TalonFX indexerMotor;

  public Indexer() {
    indexerMotor = new TalonFX(Constants.CAN.indexerMotor);
    indexerMotor.clearStickyFaults();
    indexerMotor.setNeutralMode(NeutralModeValue.Coast);
  }

  public void speaker() {
    indexerMotor.set(Constants.MotorSpeeds.speakerSpeed);
  }
  
  public void amp() {
    indexerMotor.set(Constants.MotorSpeeds.ampSpeed);
  }
  
  public void intake() {
    indexerMotor.set(-Constants.MotorSpeeds.intakeSpeed);
  }

  public void trap() {
    indexerMotor.set(Constants.MotorSpeeds.trapSpeed);
  }

  public void stop() {
    indexerMotor.stopMotor();
  }

  @Override
  public void periodic() {
  }
}