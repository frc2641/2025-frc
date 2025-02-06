package frc.team2641.robot2025.helpers;

public class ArmPosition {
	private double wrist;
	private double elev;

	public ArmPosition(double wrist, double elev){
		this.wrist = wrist;
		this.elev = elev;
		constrain();
	}

	public double getWrist() {
		return wrist;
	}

	public double getElev() {
		return elev;
	}

	public String toString() {
		return "Wrist : " + wrist + " Elevator : " + elev;
	}

	private void constrain() {
		if (wrist > 0) 
			wrist = 0;
		if (wrist < 0)
			wrist = 0;
		if (elev > 0)
			elev = 0;
		if (elev < 0)
			elev = 0;
	}
}
