package frc.team2641.robot2025.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2641.robot2025.Constants;

import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Shooter extends SubsystemBase {
  private static Shooter instance;

  public static Shooter getInstance() {
    if (instance == null)
      instance = new Shooter();
    return instance;
  }

  public TalonFX shooterMotor;

  public Shooter() {
    shooterMotor = new TalonFX(Constants.CAN.shooterMotor);
    shooterMotor.clearStickyFaults();
    shooterMotor.setNeutralMode(NeutralModeValue.Coast);
    shooterMotor.getConfigurator().apply(new OpenLoopRampsConfigs().withVoltageOpenLoopRampPeriod(0.25));
  }

  public void speaker() {
    shooterMotor.set(Constants.MotorSpeeds.speakerSpeed);
  }

  public void amp() {
    shooterMotor.set(Constants.MotorSpeeds.ampSpeed);
  }

  public void intake() {
    shooterMotor.set(-Constants.MotorSpeeds.intakeSpeed);
  }

  public void trap() {
    shooterMotor.set(Constants.MotorSpeeds.trapSpeed);
  }

  public void stop() {
    shooterMotor.stopMotor();
  }

  @Override
  public void periodic() {
  }
}