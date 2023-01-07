// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
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

	private Swerve swerveSubsystem;
	private XboxController controller;

	public TeleopSwerve(Swerve swerveSubsystem, XboxController controller) {
		this.swerveSubsystem = swerveSubsystem;
		addRequirements(swerveSubsystem);
		this.controller = controller;
	}

	@Override
	public void execute() {
		double yAxis = MathUtil.applyDeadband(-controller.getLeftY(), OperatorConstants.stickDeadband);
		double xAxis = MathUtil.applyDeadband(-controller.getLeftX(), OperatorConstants.stickDeadband);
		double rAxis = MathUtil.applyDeadband(-controller.getRightX(), OperatorConstants.stickDeadband);

		translation = new Translation2d(yAxis, xAxis).times(SwerveConstants.maxSpeed_mps);
		rotation = rAxis * SwerveConstants.maxAngularVelocity_mps;
		swerveSubsystem.drive(translation, rotation, fieldRelative, openLoop);
	}
}
