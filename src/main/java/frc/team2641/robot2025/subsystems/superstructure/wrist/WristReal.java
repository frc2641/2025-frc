package frc.team2641.robot2025.subsystems.superstructure.wrist;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2641.robot2025.Constants;

public class WristReal extends SubsystemBase implements WristIO {
  private TalonFX motor;

  private double setpoint = 2.5;

  private static WristReal instance;
  public static WristReal getInstance() {
    if (instance == null) instance = new WristReal();
    return instance;
  }

  private WristReal() {
    motor = new TalonFX(Constants.CAN.wrist);

    configMotors();
  }

  public void stop() {
    motor.stopMotor();
  }

  public void set(double speed) {
    motor.set(speed);
  }
  
  public void setPosition(double pos) {
    System.out.println("new setpoint: " + pos);
    // set position to 10 rotations
    setpoint = pos;
  }

  public double getPosition() {
    return motor.getPosition().getValueAsDouble();
  }

  private void configMotors() {
    TalonFXConfigurator configer = motor.getConfigurator();

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
    configer.apply(config);

    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = Constants.IntakeGains.wristGains.kP;
    slot0Configs.kI = Constants.IntakeGains.wristGains.kI;
    slot0Configs.kD = Constants.IntakeGains.wristGains.kD;
    configer.apply(slot0Configs);
  }

  public double getSetpoint() {
    return setpoint;
  }
  
  final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

  @Override
  public void periodic() {
    System.out.println("encoder raw: " + getPosition());
    
    if (setpoint < 1.25) setpoint = 1.25;
    if (setpoint > 9.5) setpoint = 9.5;

    System.out.println("setpoint: " + setpoint);

    motor.setControl(m_request.withPosition(setpoint));

    if ((Math.abs(motor.getVelocity().getValue().baseUnitMagnitude())<0.1)&&(motor.getTorqueCurrent().getValue().baseUnitMagnitude()>30)){
      // stop();
      System.out.println("\n Stall detected - Wrist Motor Stopped \n");
    }
  }
}