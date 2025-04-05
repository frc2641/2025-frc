package frc.team2641.robot2025.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2641.robot2025.Constants;
import frc.team2641.robot2025.Constants.CANConstants;
import frc.team2641.robot2025.Constants.ElevatorConstants;
import frc.team2641.robot2025.helpers.ElevatorConstrain;

public class Elevator extends SubsystemBase {
  private TalonFX motor;
  private boolean stalled;
  // private Timer timer;
  private double prev;
  private double setpoint = ElevatorConstants.initPos;
  private final PositionVoltage posRequest = new PositionVoltage(0).withSlot(0);

  private static Elevator instance;
  public static Elevator getInstance() {
    if (instance == null) instance = new Elevator();
    return instance;
  }

  private Elevator() {
    // timer = new Timer();
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
    return motor.getPosition().getValueAsDouble() * Constants.ElevatorConstants.elevConvert;
  }

  public double getSetpoint() {
    return setpoint;
  }

  private void configMotor() {
    motor = new TalonFX(CANConstants.elevator);

    TalonFXConfigurator configer = motor.getConfigurator();

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
    config.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);

    config.CurrentLimits.StatorCurrentLimit = 100;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    configer.apply(config);

    motor.setPosition(0);

    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = ElevatorConstants.PID.kP;
    slot0Configs.kI = ElevatorConstants.PID.kI;
    slot0Configs.kD = ElevatorConstants.PID.kD;
    slot0Configs.kS = ElevatorConstants.kS;
    slot0Configs.kA = ElevatorConstants.kA;
    slot0Configs.kG = ElevatorConstants.kG;
    slot0Configs.kV = ElevatorConstants.kV;

    slot0Configs.GravityType = GravityTypeValue.Elevator_Static;

    configer.apply(slot0Configs);
  }

  public void periodic() {
    setpoint = ElevatorConstrain.constrain(setpoint);
    double value = Constants.ElevatorConstants.SRL.calculate(setpoint);
    motor.setControl(posRequest.withPosition(value / Constants.ElevatorConstants.elevConvert));
        
    if ((Math.abs(motor.getVelocity().getValue().baseUnitMagnitude()) < ElevatorConstants.stallV) && (motor.getTorqueCurrent().getValue().baseUnitMagnitude() > ElevatorConstants.stallI)) {
      // stop();
      stalled = true;
      // timer.start();
    } else {stalled = false;
      // timer.reset();    
      // timer.stop();
    }
    if (MathUtil.applyDeadband(value-prev, 0.02) == 0 && stalled ){
      setpoint = ElevatorConstrain.constrain(setpoint - Constants.ElevatorConstants.elevatorSpeed * 0.3);
    }

    prev = value;
    
    SmartDashboard.putNumber("Elevator Pose", getPosition());
    SmartDashboard.putBoolean("Elevator Stall", stalled);
    SmartDashboard.putNumber("Elevator Torque Current", motor.getTorqueCurrent().getValue().baseUnitMagnitude());
    SmartDashboard.putNumber("Elevator Stator Current", motor.getStatorCurrent().getValue().baseUnitMagnitude());
    SmartDashboard.putNumber("Elevator Supply Current", motor.getSupplyCurrent().getValue().baseUnitMagnitude());
    SmartDashboard.putNumber("Elevator Motor Voltage", motor.getMotorVoltage().getValue().baseUnitMagnitude());
    SmartDashboard.putNumber("Elevator Velocity", motor.getVelocity().getValue().baseUnitMagnitude());
    SmartDashboard.putNumber("Elevator Setpoint", setpoint);
  }
  
  public TalonFX getMotor() {
    return motor;
  }

  public boolean atPosition() {
    return MathUtil.applyDeadband(getPosition()-getSetpoint(), 0.05) == 0;
  }
}