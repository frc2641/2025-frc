package frc.team2641.robot2025.subsystems.superstructure.elevator;

public interface ElevatorIO {
	void goTo(double pos);
	double getPosition();
	double getSetpoint();
	void stop();
}