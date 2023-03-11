package frc.robot.commands.arm;

import frc.robot.constants.AutoConstants;
import frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class MoveArmBy extends CommandBase {
	private Arm arm;
	private double deltaPos_m;
	private double targetPos_m;

	public MoveArmBy(Arm arm, double deltaDist_m) {
		this.arm = arm;
		this.deltaPos_m = deltaDist_m;
	}

	@Override
	public void initialize() {
		targetPos_m = arm.getArmDist_m() + deltaPos_m;
		arm.inManualMode = false;
		arm.setPosition(targetPos_m);
	}

	@Override
	public boolean isFinished() {
		return Math.abs(targetPos_m - arm.getArmDist_m()) <= AutoConstants.armTolerance_m;
	}

	@Override
	public void end(boolean interrupted) {
		// arm.stop();
	}
}
