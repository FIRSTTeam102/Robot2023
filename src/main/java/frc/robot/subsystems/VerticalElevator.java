// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.VerticalElevatorConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;

public class VerticalElevator extends SubsystemBase {
	private CANSparkMax motor = new CANSparkMax(VerticalElevatorConstants.motorPort, MotorType.kBrushless);

	private SparkMaxPIDController pidController = motor.getPIDController();
	private RelativeEncoder encoder = motor.getEncoder();
	private SparkMaxLimitSwitch topSwitch = motor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
	private SparkMaxLimitSwitch bottomSwitch = motor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

	/** Creates a new VerticalElevator. */
	public VerticalElevator() {
		topSwitch.enableLimitSwitch(true);
		bottomSwitch.enableLimitSwitch(true);

		pidController.setP(VerticalElevatorConstants.kP);
		pidController.setD(VerticalElevatorConstants.kD);
		pidController.setI(VerticalElevatorConstants.kI);
		pidController.setIZone(VerticalElevatorConstants.kIZone);
		pidController.setFF(VerticalElevatorConstants.kF);
		pidController.setOutputRange(VerticalElevatorConstants.kMinOuput, VerticalElevatorConstants.kMaxOutput);

		encoder.setPositionConversionFactor(VerticalElevatorConstants.conversionFactor_meters_per_rotation);
	}

	public boolean getTopStatus() {
		return topSwitch.isPressed();
	}

	public boolean getBottomStatus() {
		return bottomSwitch.isPressed();
	}

	/**
	 * REMOVE AFTER TESTING
	 */
	public void setSpeed(double speed) {
		motor.set(speed);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		System.out.println(encoder.getPosition());

		if (getBottomStatus())
			encoder.setPosition(0);

		if (getTopStatus())
			encoder.setPosition(VerticalElevatorConstants.maxHeight_m);
	}
}