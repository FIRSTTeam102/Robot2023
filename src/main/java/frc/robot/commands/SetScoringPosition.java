// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.constants.ArmConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

import frc.robot.commands.arm.SetArmPosition;
import frc.robot.commands.elevator.SetElevatorPosition;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class SetScoringPosition extends InstantCommand {
	private Elevator elevator;
	private Arm arm;
	private double elevatorTarget_m;
	private double armTarget_m;

	public SetScoringPosition(Elevator elevator, Arm arm, double elevatorTarget_m, double armTarget_m) {
		// Use addRequirements() here to declare subsystem dependencies.
		this.elevator = elevator;
		this.arm = arm;
		this.elevatorTarget_m = elevatorTarget_m;
		this.armTarget_m = armTarget_m;

		addRequirements(elevator, arm);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		if ((armTarget_m <= ArmConstants.gridSafeZone_m)
			&& (Arm.nutDistToArmDist(arm.inputs.nutPosition_m) > ArmConstants.gridSafeZone_m)) {
			new SequentialCommandGroup(new SetArmPosition(arm, armTarget_m),
				new WaitUntilCommand(() -> (Arm.nutDistToArmDist(arm.inputs.nutPosition_m) < ArmConstants.gridSafeZone_m)),
				new SetElevatorPosition(elevator, elevatorTarget_m))
					.schedule();
		} else if ((armTarget_m >= ArmConstants.gridSafeZone_m)
			&& (Arm.nutDistToArmDist(arm.inputs.nutPosition_m) < ArmConstants.gridSafeZone_m)) {
			new SequentialCommandGroup(new SetElevatorPosition(elevator, elevatorTarget_m),
				new WaitUntilCommand(() -> (elevator.inputs.position_m >= ElevatorConstants.gridSafeZone_m)),
				new SetArmPosition(arm, armTarget_m)).schedule();
		} else {
			new SetElevatorPosition(elevator, elevatorTarget_m).schedule();
			new SetArmPosition(arm, armTarget_m).schedule();
		}
	}
}
