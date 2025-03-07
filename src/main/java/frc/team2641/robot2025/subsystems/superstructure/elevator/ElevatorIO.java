package frc.team2641.robot2025.subsystems.superstructure.elevator;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface ElevatorIO extends Subsystem {
	void goTo(double pos);
	/** @return the actual position it thinks the elevator is in 
	 * also, why is this necessary?
	*/
	double getPosition();
	/**@return the "goal" of the elevator, where it is trying to go. */
	double getSetpoint();
	void stop();
}