package frc.team2641.robot2025.helpers;

import frc.team2641.robot2025.Constants;

public class ArmPosition {
	private double wrist;
	private double elev;

	/**@param notice <h1><b><i>Wrist THEN elev</b></i></h1> */
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
		if (wrist > Constants.WristConstants.maxPos) wrist = Constants.WristConstants.maxPos;
		if (wrist < Constants.WristConstants.minPos) wrist = Constants.WristConstants.minPos;
		if (elev > Constants.ElevatorConstants.kMaxElevatorHeightMeters) elev = Constants.ElevatorConstants.kMaxElevatorHeightMeters;
		if (elev < Constants.ElevatorConstants.kMinElevatorHeightMeters) elev = Constants.ElevatorConstants.kMinElevatorHeightMeters;
	}
}