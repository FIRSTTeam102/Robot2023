package frc.robot.commands.scoring;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

import frc.robot.commands.arm.SetArmPosition;
import frc.robot.commands.elevator.SetElevatorPosition;

import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class SetElevatorArmPosition extends ProxyCommand {
	public SetElevatorArmPosition(Elevator elevator, Arm arm, double elevatorPos_m, double armPos_m) {
		super(() -> new SetElevatorArmPositionProxied(elevator, arm, elevatorPos_m, armPos_m, 0));
	}

	public SetElevatorArmPosition(Elevator elevator, Arm arm, double elevatorPos_m, double armPos_m, double tolerance) {
		super(() -> new SetElevatorArmPositionProxied(elevator, arm, elevatorPos_m, armPos_m, tolerance));
	}

	private static class SetElevatorArmPositionProxied extends SequentialCommandGroup {
		SetElevatorArmPositionProxied(Elevator elevator, Arm arm, double elevatorTarget_m, double armTarget_m,
			double tolerance) {
			elevator.inManualMode = false;
			arm.inManualMode = false;
			if (elevatorTarget_m > elevator.inputs.position_m) {
				// going up -> elevator first
				addCommands(
					new SetElevatorPosition(elevator, elevatorTarget_m),
					new WaitUntilCommand(() -> arm.inManualMode || elevator.inputs.position_m > 0.6 * elevatorTarget_m), // Units.inchesToMeters(30)
					new SetArmPosition(arm, armTarget_m));
			} else {
				// going down -> arm first
				addCommands(
					new SetArmPosition(arm, armTarget_m),
					new WaitUntilCommand(() -> {
						if (elevator.inManualMode)
							return true;

						// Units.inchesToMeters(36)

						// arm going out
						if (armTarget_m > arm.getArmDist_m())
							return arm.getArmDist_m() > 0.6 * armTarget_m;
						// arm going in
						else
							return arm.getArmDist_m() < armTarget_m + 0.08; // when going to 0

						// return Math.abs(arm.getArmDist_m() - armTarget_m) <= 0.1;
					}),
					new SetElevatorPosition(elevator, elevatorTarget_m));
			}

			if (tolerance > 0) {
				addCommands(new WaitUntilCommand(() -> Math.abs(elevator.inputs.position_m - elevatorTarget_m) < tolerance
					&& Math.abs(arm.getArmDist_m() - armTarget_m) < tolerance));
			}
		}
	}
}
