// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.ArmConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;

import java.util.Map;

public class Arm extends SubsystemBase {
	private CANSparkMax motor = new CANSparkMax(ArmConstants.motorPort, MotorType.kBrushless);

	private SparkMaxPIDController pidController = motor.getPIDController();
	private RelativeEncoder encoder = motor.getEncoder();
	private SparkMaxLimitSwitch frontLimitSwitch = motor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
	private SparkMaxLimitSwitch backLimitSwitch = motor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

	private GenericEntry ShuffleboardArmEntryRaw; // Total possible length
	private GenericEntry ShuffleboardArmEntry; // Arm length that will extend to the top node when the robot is touching
																							// the grid

	public Arm() {
		ShuffleboardArmEntryRaw = Shuffleboard.getTab("Drive")
			.add("Arm Length", 0)
			.withWidget(BuiltInWidgets.kNumberBar)
			.withProperties(Map.of("min", 0, "max", 48))
			.getEntry();

		ShuffleboardArmEntry = Shuffleboard.getTab("Drive")
			.add("Arm Length", 0)
			.withWidget(BuiltInWidgets.kNumberBar)
			.withProperties(Map.of("min", 0, "max", 40))
			.getEntry();

		frontLimitSwitch.enableLimitSwitch(true);
		backLimitSwitch.enableLimitSwitch(true);

		pidController.setP(ArmConstants.kP);
		pidController.setD(ArmConstants.kD);
		pidController.setI(ArmConstants.kI);
		pidController.setIZone(ArmConstants.kIZone);
		pidController.setFF(ArmConstants.kF);

		pidController.setOutputRange(ArmConstants.kMinOutput, ArmConstants.kMaxOutput);

		encoder.setPositionConversionFactor(ArmConstants.conversionFactor_in_per_rotation);
	}

	public void setPos(double armLength_in) {
		double nutDist = armDistToNutDist(armLength_in);
		nutDist = MathUtil.clamp(nutDist, 0, ArmConstants.maxNutDist_in - ArmConstants.minNutDist_in);

		pidController.setReference(nutDist, CANSparkMax.ControlType.kSmartMotion);
	}

	public void setSpeed(double speed) {
		pidController.setReference(speed, CANSparkMax.ControlType.kDutyCycle);
	}

	public void stop() {
		pidController.setReference(0, CANSparkMax.ControlType.kDutyCycle);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		if (backLimitSwitch.isPressed()) {
			encoder.setPosition(ArmConstants.maxNutDist_in - ArmConstants.minNutDist_in);
		}

		if (frontLimitSwitch.isPressed()) {
			encoder.setPosition(0);
		}

		ShuffleboardArmEntry.setDouble(nutDistToArmDist(encoder.getPosition()));
	}

	public static double armDistToNutDist(double armDistance) {
		return Math.sqrt(Math.pow(ArmConstants.armSectionLength_in, 2) - Math.pow(armDistance / 4, 2))
			- ArmConstants.minNutDist_in;
	}

	public static double nutDistToArmDist(double nutDistance) {
		return 4 * Math
			.sqrt(Math.pow(ArmConstants.armSectionLength_in, 2) - Math.pow(nutDistance + ArmConstants.minNutDist_in, 2));
	}
}
