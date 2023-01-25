// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.ArmConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;

public class Arm extends SubsystemBase {
	private CANSparkMax motor = new CANSparkMax(ArmConstants.motorPort, MotorType.kBrushless);

	private SparkMaxPIDController pidController = motor.getPIDController();
	private RelativeEncoder encoder = motor.getEncoder();
	private SparkMaxLimitSwitch limitSwitch = motor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

	public Arm() {
		limitSwitch.enableLimitSwitch(true);

		pidController.setP(ArmConstants.kP);
		pidController.setD(ArmConstants.kD);
		pidController.setI(ArmConstants.kI);
		pidController.setIZone(ArmConstants.kIZone);
		pidController.setFF(ArmConstants.kF);

		pidController.setOutputRange(ArmConstants.kMinOutput, ArmConstants.kMaxOutput);

		encoder.setPositionConversionFactor(ArmConstants.conversionFactor);
	}

	/**
	 * Set arm to a specific position
	 * 
	 * @param position position in percentage to complete extension in the range [0, 1] (0 is fully inward, 1 is max extension)
	 */
	public void setPos(double position) {
		pidController.setReference(position, CANSparkMax.ControlType.kPosition);
	}

	public void stop() {
		pidController.setReference(0, CANSparkMax.ControlType.kDutyCycle);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		if (limitSwitch.isPressed()) {
			encoder.setPosition(0);
		}
	}
}
