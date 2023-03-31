package frc.robot.commands.scoring;

import frc.robot.Robot;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.ScoringPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetScoringPosition extends CommandBase {
	Elevator elevator;
	Arm arm;
	double elevatorTarget_m;
	double armTarget_m;
	double elevatorTolerance_m;
	double armTolerance_m;

	public SetScoringPosition(Elevator elevator, Arm arm, ScoringPosition pos) {
		this(elevator, arm, pos, 0, 0);
	}

	/** set tolerance to 0 to ignore it */
	public SetScoringPosition(Elevator elevator, Arm arm, ScoringPosition pos,
		double elevatorTolerance_m, double armTolerance_m) {
		this(elevator, arm, pos.elevatorHeight_m, pos.armExtension_m, elevatorTolerance_m, armTolerance_m);
	}

	public SetScoringPosition(Elevator elevator, Arm arm, double elevatorPos_m, double armPos_m,
		double elevatorTolerance_m, double armTolerance_m) {
		this.elevator = elevator;
		this.arm = arm;
		this.elevatorTarget_m = elevatorPos_m;
		this.armTarget_m = armPos_m;
		this.elevatorTolerance_m = elevatorTolerance_m;
		this.armTolerance_m = armTolerance_m;
		addRequirements(elevator, arm);
	}

	private enum Mech {
		Elevator, Arm, None;
	}

	private Mech nextSet;

	@Override
	public void initialize() {
		elevator.inManualMode = false;
		arm.inManualMode = false;

		if (elevatorTarget_m > elevator.inputs.position_m) {
			// going up
			elevator.setPosition(elevatorTarget_m);
			nextSet = Mech.Arm;
		} else {
			// going down
			arm.setPosition(armTarget_m);
			nextSet = Mech.Elevator;
		}
	}

	@Override
	public void execute() {
		switch (nextSet) {
			case Arm -> {
				if (elevator.inputs.position_m < ElevatorConstants.dangerZone_m)
					return; // until we get above

				if (elevatorTolerated(AutoConstants.elevatorTolerance_m)) {
					arm.setPosition(armTarget_m);
					nextSet = Mech.None;
				}
			}

			case Elevator -> {
				if (elevatorTarget_m < ElevatorConstants.dangerZone_m && arm.getArmDist_m() < ArmConstants.dangerZone_m)
					break; // until we get out

				if (armTolerated(Elevator.isInDangerZone() ? 0.07 : AutoConstants.armTolerance_m)) {
					elevator.setPosition(elevatorTarget_m);
					nextSet = Mech.None;
				}
			}

			case None -> {
				// both sets already called
			}
		}
	}

	@Override
	public boolean isFinished() {
		if (Robot.isSimulation())
			return true; // temp: until we actually sim it, for now we want to test auto

		// operator aborted
		if (elevator.inManualMode || arm.inManualMode)
			return true;

		// keep running until we're at both positions (auto)
		if ((elevatorTolerance_m > 0 && !elevatorTolerated(elevatorTolerance_m))
			|| (armTolerance_m > 0 && !armTolerated(armTolerance_m)))
			return false;

		// normally only end after last mech set
		return nextSet == Mech.None;
	}

	boolean elevatorTolerated(double tolerance_m) {
		return Math.abs(elevator.inputs.position_m - elevatorTarget_m) < tolerance_m;
	}

	boolean armTolerated(double tolerance_m) {
		return Math.abs(arm.getArmDist_m() - armTarget_m) < tolerance_m;
	}
}
