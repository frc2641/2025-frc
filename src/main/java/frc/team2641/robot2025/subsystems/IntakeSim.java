package frc.team2641.robot2025.subsystems;

import org.dyn4j.geometry.Triangle;
import org.dyn4j.geometry.Vector2;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;


import frc.team2641.robot2025.Robot;

public class IntakeSim {
    
private final IntakeSimulation intakeSimulation;

private static IntakeSim instance;
public static IntakeSim getInstance() {
  if (instance == null)
    instance = new IntakeSim();
  return instance;
}
    

    public IntakeSim() {

        // Here, create the intake simulation with respect to the intake on your real robot
        this.intakeSimulation = new IntakeSimulation(
        // Specify the type of game pieces that the intake can collect
        "Coral",
        // Specify the drivetrain to which this intake is attached
        Swerve.getInstance().getSwerveDrive().getMapleSimDrive().get(), 
        // Our intake has a custom shape of a triangle (shape is specified in chassis frame-of-reference)
        new Triangle(new Vector2(0, 0), new Vector2(0.2, 0), new Vector2(0, 0.2)),
        // The intake can hold up to 1 note
        1);
    }
    
        // @Override // Defined by IntakeIO
        public void setRunning(boolean runIntake) {
            if (runIntake)
                intakeSimulation.startIntake(); // Extends the intake out from the chassis frame and starts detecting contacts with game pieces
            else
                intakeSimulation.stopIntake(); // Retracts the intake into the chassis frame, disabling game piece collection
        }
    
        // @Override // Defined by IntakeIO
        public boolean isNoteInsideIntake() {
            return intakeSimulation.getGamePiecesAmount() != 0; // True if there is a game piece in the intake
        }
    
        // // @Override // Defined by IntakeIO
        // public void launchNote() {
        //     // if there is a note in the intake, it will be removed and return true; otherwise, returns false
        //     if (intakeSimulation.obtainGamePieceFromIntake())
        //         ShooterIOSim.launchNote(); // notify the simulated flywheels to launch a note
        // }
    
}