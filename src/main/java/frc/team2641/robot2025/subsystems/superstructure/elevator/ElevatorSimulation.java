// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team2641.robot2025.subsystems.superstructure.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import frc.team2641.robot2025.Constants;
import frc.team2641.robot2025.Robot;
import frc.team2641.robot2025.subsystems.swerve.Drivetrain;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSimulation extends SubsystemBase implements AutoCloseable, ElevatorIO {

  private static ElevatorSimulation instance;
  public static ElevatorSimulation getInstance(){
    if (instance == null)
      instance = new ElevatorSimulation();
    return instance;
  }

  private final Pose3d elevPose;
  private final StructPublisher<Pose3d> publisher;
  // private final 

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
  private final PWMSparkMax m_motor = new PWMSparkMax(Constants.ElevatorConstants.kMotorPort);

  // Simulation classes help us simulate what's going on, including gravity.
  private final ElevatorSim m_elevatorSim =
      new ElevatorSim(
          m_elevatorGearbox,
          Constants.ElevatorConstants.kElevatorGearing,
          Constants.ElevatorConstants.kCarriageMass,
          Constants.ElevatorConstants.kElevatorDrumRadius,
          Constants.ElevatorConstants.kMinElevatorHeightMeters,
          Constants.ElevatorConstants.kMaxElevatorHeightMeters,
          true,
          0,
          0.01,
          0.0);
  private final EncoderSim m_encoderSim = new EncoderSim(m_encoder);
  private final PWMSim m_motorSim = new PWMSim(m_motor);

  // Create a Mechanism2d visualization of the elevator
  private final Mechanism2d m_mech2d = new Mechanism2d(20, 50);
  private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("Elevator Root", 10, 0);
  private final MechanismLigament2d m_elevatorMech2d =
      m_mech2dRoot.append(
          new MechanismLigament2d("Elevator", m_elevatorSim.getPositionMeters(), 90));
  
  /**         Subsystem constructor.        */
  public ElevatorSimulation() {
    m_encoder.setDistancePerPulse(Constants.ElevatorConstants.kElevatorEncoderDistPerPulse);

    // Publish Mechanism2d to SmartDashboard
    // To view the Elevator visualization, select Network Tables -> SmartDashboard -> Elevator Sim
    SmartDashboard.putData("Elevator Sim", m_mech2d);
    SmartDashboard.putBoolean("Initialized ElevatorSimulation", true);

    elevPose = new Pose3d(Drivetrain.getInstance().getPose());
publisher = NetworkTableInstance.getDefault()
  .getStructTopic("ElevatorPose", Pose3d.struct).publish();
  }

  /** Advance the simulation. */
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    m_elevatorSim.setInput(m_motorSim.getSpeed() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_elevatorSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_encoderSim.setDistance(m_elevatorSim.getPositionMeters());
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));

        publisher.set(elevPose);
  }

  /**
   * Run control loop to reach and maintain goal.
   *
   * @param goal the position to maintain
   */
  @Override
  public void goTo(double goal) {
        
    m_controller.setGoal(goal);

    // With the setpoint value we run PID control like normal
    double pidOutput = m_controller.calculate(m_encoder.getDistance());
    double feedforwardOutput = m_feedforward.calculate(m_controller.getSetpoint().velocity);
    m_motor.setVoltage(pidOutput + feedforwardOutput);

    
  }

  /** Stop the control loop and motor output. */
  public void stop() {
    m_controller.setGoal(0.0);
    m_motor.set(0.0);
  }

  /** Update telemetry, including the mechanism visualization. */
  public void updateTelemetry() {
    // Update elevator visualization with position
    m_elevatorMech2d.setLength(m_encoder.getDistance());
    SmartDashboard.putNumber("goal", getSetpoint());
  }

  @Override
  public void close() {
    m_encoder.close();
    m_motor.close();
    m_mech2d.close();
  }
@Override
  public double getSetpoint(){
    return m_controller.getGoal().position;
  }

  @Override
  /** might be the wrong value, not super important */
  public double getPosition()
  {
    return m_encoder.getDistance();
  }

  @Override
  public void setDefaultCommand(Command command) {
super.setDefaultCommand(command);
  }


}
