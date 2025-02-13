package frc.team2641.robot2025.subsystems.intake;

import org.dyn4j.geometry.Triangle;
import org.dyn4j.geometry.Vector2;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.team2641.robot2025.Robot;
import frc.team2641.robot2025.subsystems.Swerve;

public class IntakeIOSim implements IntakeIO {
    
  private final IntakeSimulation intakeSimulation;

  private static IntakeIOSim instance;
  public static IntakeIOSim getInstance() {
    if (instance == null)
      instance = new IntakeIOSim();
    return instance;
  }

  private BooleanSubscriber spinSub;
  
  public IntakeIOSim() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("state");
    spinSub = table.getBooleanTopic("reverseIntake").subscribe(false);

    // Here, create the intake simulation with respect to the intake on your real robot
    this.intakeSimulation = new IntakeSimulation(
    // Specify the type of game pieces that the intake can collect
    "Coral",
    // Specify the drivetrain to which this intake is attached
    Swerve.getInstance().getSwerveDrive().getMapleSimDrive().get(),
    // null,
    // Our intake has a custom shape of a triangle (shape is specified in chassis frame-of-reference)
    new Triangle(new Vector2(0, 0), new Vector2(0.2, 0), new Vector2(0, 0.2)),
    // The intake can hold up to 1 coral
    1);
    new Triangle(new Vector2(0, 0), new Vector2(0.2, 0), new Vector2(0, 0.2));
  }

  public void setRunning(boolean invert) {
    if (!invert)
      intakeSimulation.startIntake(); // Extends the intake out from the chassis frame and starts detecting contacts with game pieces
  }
    
  // @Override // Defined by IntakeIO
  public boolean isCoralInsideIntake() {
    return intakeSimulation.getGamePiecesAmount() != 0; // True if there is a game piece in the intake
  }

  @Override
  public void spin() {
    setRunning(spinSub.get());
  }

  @Override
  public void stop() {
    // TODO Auto-generated method stub
    intakeSimulation.stopIntake(); // Retracts the intake into the chassis frame, disabling game piece collection
  }
    
  /* @Override // Defined by IntakeIO
  public void launchCoral() {
    // if there is a coral in the intake, it will be removed and return true; otherwise, returns false
    if (intakeSimulation.obtainGamePieceFromIntake())
      ShooterIOSim.launchCoral(); // notify the simulated flywheels to launch a coral
  } */
}