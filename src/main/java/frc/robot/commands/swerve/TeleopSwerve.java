package frc.robot.commands.swerve;

import frc.robot.constants.Constants.OperatorConstants;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleopSwerve extends CommandBase {
	private double rotation;
	private Translation2d translation;
	private boolean fieldRelative = false;
	private boolean openLoop = true;

	private Swerve swerve;
	private XboxController controller;

	public TeleopSwerve(Swerve swerve, XboxController controller) {
		this.swerve = swerve;
		addRequirements(swerve);
		this.controller = controller;
	}

	@Override
	public void execute() {
		double yAxis = modifyAxis(-controller.getLeftY());
		double xAxis = modifyAxis(-controller.getLeftX());
		double rAxis = modifyAxis(-controller.getRightX());

		translation = new Translation2d(yAxis, xAxis).times(SwerveConstants.maxVelocity_mps);
		rotation = rAxis * SwerveConstants.maxAngularVelocity_radps;
		swerve.drive(translation, rotation);
	}

	@Override
	public void end(boolean interrupted) {
		swerve.stop();
		super.end(interrupted);
	}

	private static final double cubicWeight = 0.4;
	private static final double weightExponent = 3.5;

	private static double modifyAxis(double value) {
		value = MathUtil.applyDeadband(value, OperatorConstants.stickDeadband);
		// return Math.copySign(value * value, value);

		// custom input scaling
		// @see https://desmos.com/calculator/7wy4gmgdpv
		return Math.copySign(cubicWeight * Math.pow(value, weightExponent) + (1 - cubicWeight) * value, value);
	}
}
