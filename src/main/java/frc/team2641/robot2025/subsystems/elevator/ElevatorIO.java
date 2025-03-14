package frc.team2641.robot2025.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface ElevatorIO extends Subsystem  {
	void goTo(double pos);
	/** @return the actual height it thinks the elevator is in, 
	 * 	@see meters
	 * @see also, why is this necessary?
	*/
	double getPosition();
	/**@return the "goal" of the elevator, where it is trying to go. */
	double getSetpoint();
	void stop();
	void setDefaultCommand(Command command);
	void set(double speed);
}