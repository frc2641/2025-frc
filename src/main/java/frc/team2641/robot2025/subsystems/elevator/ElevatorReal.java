package frc.team2641.robot2025.subsystems.elevator;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2641.robot2025.Constants;
import frc.team2641.robot2025.Constants.CANConstants;
import frc.team2641.robot2025.Constants.ElevatorConstants;
import frc.team2641.robot2025.helpers.ElevatorContrain;

public class ElevatorReal extends SubsystemBase implements ElevatorIO, AutoCloseable {
  private TalonFX motor;
  private boolean stalled;
  private double setpoint = ElevatorConstants.initPos;
  private final PositionVoltage posRequest = new PositionVoltage(0).withSlot(0);

  /*
  // This gearbox represents a gearbox containing 4 Vex 775pro motors.
  private final DCMotor m_elevatorGearbox = DCMotor.getKrakenX60(1);

  // Standard classes for controlling our elevator
  private final ProfiledPIDController m_controller =
      new ProfiledPIDController(
          Constants.ElevatorConstants.kElevatorKp,
          Constants.ElevatorConstants.kElevatorKi,
          Constants.ElevatorConstants.kElevatorKd,
          new TrapezoidProfile.Constraints(2.45, 2.45)); 

  ElevatorFeedforward m_feedforward =
      new ElevatorFeedforward(
          Constants.ElevatorConstants.kElevatorkS,
          Constants.ElevatorConstants.kElevatorkG,
          Constants.ElevatorConstants.kElevatorkV,
          Constants.ElevatorConstants.kElevatorkA);
          
  private final Encoder m_encoder = new Encoder(Constants.ElevatorConstants.kEncoderAChannel, Constants.ElevatorConstants.kEncoderBChannel);
   */

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
    // m_controller.setGoal(goal);

    // // With the setpoint value we run PID control like normal
    // double pidOutput = m_controller.calculate(m_encoder.getDistance());
    // double feedforwardOutput = m_feedforward.calculate(m_controller.getSetpoint().velocity);
    // motor.setVoltage(pidOutput + feedforwardOutput);

  }

  @Override
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

    // TODO numbers are untested
    // config.CurrentLimits.SupplyCurrentLimit = 40;
    // config.CurrentLimits.SupplyCurrentLowerLimit = 45;
    // config.CurrentLimits.SupplyCurrentLowerTime = 0.1;
    // config.CurrentLimits.StatorCurrentLimit = 60;

    configer.apply(config);

    motor.setPosition(0);

    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = ElevatorConstants.PID.kP;
    slot0Configs.kI = ElevatorConstants.PID.kI;
    slot0Configs.kD = ElevatorConstants.PID.kD;

    configer.apply(slot0Configs);
    
// motion magic, start
//     MotionMagicConfigs motionMagicConfigs = configer.;  
// motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
// motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
// motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)    
  }
  
  @Override
  public void periodic() {
    // if (setpoint > ElevatorConstants.minPos) setpoint = ElevatorConstants.minPos;
    // if (setpoint < ElevatorConstants.maxPos) setpoint = ElevatorConstants.maxPos;

    double value = new ElevatorContrain(Constants.ElevatorConstants.SRL.calculate(setpoint)).get();

    motor.setControl(posRequest.withPosition(value / Constants.ElevatorConstants.elevConvert));

    if ((Math.abs(motor.getVelocity().getValue().baseUnitMagnitude()) < ElevatorConstants.stallV) && (motor.getTorqueCurrent().getValue().baseUnitMagnitude() > ElevatorConstants.stallI)) {
      // stop();
      stalled = true;
    }
    else
    stalled = false;
    // TODO: Move setpoint retargeting to an else statement above

    SmartDashboard.putNumber("Arm Elevator Real Pos ", getPosition());
    SmartDashboard.putBoolean("Elevator Stall", stalled);
    SmartDashboard.putNumber("Current", motor.getTorqueCurrent().getValue().baseUnitMagnitude());
  }
  
  public TalonFX getMotor() {
    return motor;
  }

  @Override
  public void setDefaultCommand(Command command) {
    super.setDefaultCommand(command);
  }

  @Override
  public void close() {
    // m_encoder.close();
    // m_motor.close();
    // m_mech2d.close();
  }
}