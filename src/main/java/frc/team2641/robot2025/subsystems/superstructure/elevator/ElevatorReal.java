package frc.team2641.robot2025.subsystems.superstructure.elevator;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2641.robot2025.Constants;
import frc.team2641.robot2025.Constants.ElevatorConstants;

public class ElevatorReal extends SubsystemBase implements ElevatorIO {
  private TalonFX motor;

  private double setpoint = Constants.ElevatorConstants.initPos;
  private final PositionVoltage posRequest = new PositionVoltage(0).withSlot(0);

  private static ElevatorReal instance;
  public static ElevatorReal getInstance() {
    if (instance == null) instance = new ElevatorReal();
    return instance;
  }

  private ElevatorReal() {
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
    motor = new TalonFX(Constants.CAN.elevator);

    TalonFXConfigurator configer = motor.getConfigurator();

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
    configer.apply(config);

    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = ElevatorConstants.PID.kP;
    slot0Configs.kI = ElevatorConstants.PID.kI;
    slot0Configs.kD = ElevatorConstants.PID.kD;
    configer.apply(slot0Configs);
  }
  
  @Override
  public void periodic() {
    if (setpoint < ElevatorConstants.minPos) setpoint = ElevatorConstants.minPos;
    if (setpoint > ElevatorConstants.maxPos) setpoint = ElevatorConstants.maxPos;

    motor.setControl(posRequest.withPosition(setpoint));

    if ((Math.abs(motor.getVelocity().getValue().baseUnitMagnitude()) < 0.1) && (motor.getTorqueCurrent().getValue().baseUnitMagnitude() > 30)){
      // stop();
      System.out.println("\n\n *** STALL DETECTED - ELEVATOR *** \n\n");
    }
    // TODO: Move setpoint retargeting to an else statement above
  }
  
  public TalonFX getMotor() {
    return motor;
  }
}