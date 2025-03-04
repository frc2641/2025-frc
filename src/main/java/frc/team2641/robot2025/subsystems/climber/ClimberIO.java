package frc.team2641.robot2025.subsystems.climber;

public interface ClimberIO {
	double getPosition();
	void stop();
	void extend();
	void retract();
}