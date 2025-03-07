package frc.team2641.robot2025.subsystems.superstructure.elevator;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2641.robot2025.Constants.CANConstants;
import frc.team2641.robot2025.Constants.ElevatorConstants;

public class ElevatorReal extends Elevator implements ElevatorIO {
  private TalonFX motor;
  private boolean stalled;
  private double setpoint = ElevatorConstants.initPos;
  private final PositionVoltage posRequest = new PositionVoltage(0).withSlot(0);

  private static ElevatorReal instance;
  public static ElevatorReal getInstance() {
    if (instance == null) instance = new ElevatorReal();
    return instance;
  }

  private ElevatorReal() {
    configMotor();
    setpoint = getPosition();
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
    motor = new TalonFX(CANConstants.elevator);

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
    if (setpoint > ElevatorConstants.minPos) setpoint = ElevatorConstants.minPos;
    if (setpoint < ElevatorConstants.maxPos) setpoint = ElevatorConstants.maxPos;

    motor.setControl(posRequest.withPosition(setpoint));

    if ((Math.abs(motor.getVelocity().getValue().baseUnitMagnitude()) < ElevatorConstants.stallV) && (motor.getTorqueCurrent().getValue().baseUnitMagnitude() > ElevatorConstants.stallI)){
      // stop();
      stalled = true;
    }
    else
    stalled = false;
    // TODO: Move setpoint retargeting to an else statement above

    SmartDashboard.putNumber("Arm Elevator Real Pos ", motor.getPosition().getValueAsDouble());
    SmartDashboard.putBoolean("Elevator Stall", stalled);
  }
  
  public TalonFX getMotor() {
    return motor;
  }
}