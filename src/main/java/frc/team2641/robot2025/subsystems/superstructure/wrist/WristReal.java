package frc.team2641.robot2025.subsystems.superstructure.wrist;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2641.robot2025.Constants;
import frc.team2641.robot2025.Constants.IntakeConstants;

public class WristReal extends SubsystemBase implements WristIO {
  private TalonFX motor;

  private double setpoint = IntakeConstants.wristInitPos;
  private final PositionVoltage posRequest = new PositionVoltage(0).withSlot(0);

  private static WristReal instance;
  public static WristReal getInstance() {
    if (instance == null) instance = new WristReal();
    return instance;
  }

  private WristReal() {
    configMotor();
  }

  public void stop() {
    motor.stopMotor();
  }

  public void set(double speed) {
    motor.set(speed);
  }
  
  public void goTo(double pos) {
    setpoint = pos;
  }

  public double getPosition() {
    return motor.getPosition().getValueAsDouble();
  }

  public double getSetpoint() {
    return setpoint;
  }

  private void configMotor() {
    motor = new TalonFX(Constants.CAN.wrist);

    TalonFXConfigurator configer = motor.getConfigurator();

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
    configer.apply(config);

    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = Constants.IntakeConstants.wristPID.kP;
    slot0Configs.kI = Constants.IntakeConstants.wristPID.kI;
    slot0Configs.kD = Constants.IntakeConstants.wristPID.kD;
    configer.apply(slot0Configs);
  }
  
  @Override
  public void periodic() {
    if (setpoint < IntakeConstants.wristMinPos) setpoint = IntakeConstants.wristMinPos;
    if (setpoint > IntakeConstants.wristMaxPos) setpoint = IntakeConstants.wristMaxPos;

    motor.setControl(posRequest.withPosition(setpoint));

    if ((Math.abs(motor.getVelocity().getValue().baseUnitMagnitude()) < 0.1) && (motor.getTorqueCurrent().getValue().baseUnitMagnitude() > 30)){
      // stop();
      System.out.println("\n\n *** STALL DETECTED - WRIST *** \n\n");
    }
    // TODO: Move setpoint retargeting to an else statement above
  }

  public TalonFX getMotor() {
    return motor;
  }
}