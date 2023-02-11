package frc.robot.commands.arm;

import frc.robot.constants.Constants.OperatorConstants;
import frc.robot.subsystems.Arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ManualArmControl extends CommandBase {
	private Arm arm;
	private CommandXboxController operatorController;

	public ManualArmControl(Arm arm, CommandXboxController operatorController) {
		this.arm = arm;
		this.operatorController = operatorController;
		addRequirements(arm);
	}

	@Override
	public void initialize() {}

	@Override
	public void execute() {
		double xAxis = operatorController.getRightX();
		xAxis = MathUtil.applyDeadband(xAxis, OperatorConstants.stickDeadband);

		arm.setSpeed(xAxis);
	}

	@Override
	public void end(boolean interrupted) {
		arm.stop();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
