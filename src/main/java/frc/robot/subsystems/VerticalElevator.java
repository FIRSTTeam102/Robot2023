// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.VerticalElevatorConstants;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

public class VerticalElevator extends SubsystemBase {
	private CANSparkMax motor = new CANSparkMax(VerticalElevatorConstants.motorPort, MotorType.kBrushless);
	private SparkMaxPIDController pidController = motor.getPIDController();
	private RelativeEncoder encoder = motor.getEncoder();

	private DigitalInput topSwitch = new DigitalInput(VerticalElevatorConstants.topSwitchPort);
	private DigitalInput bottomSwitch = new DigitalInput(VerticalElevatorConstants.bottomSwitchPort);

	/** Creates a new VerticalElevator. */
	public VerticalElevator() {
		pidController.setP(VerticalElevatorConstants.kP);
		pidController.setD(VerticalElevatorConstants.kD);
		pidController.setI(VerticalElevatorConstants.kI);
		pidController.setIZone(VerticalElevatorConstants.kIZone);
		pidController.setFF(VerticalElevatorConstants.kF);
		pidController.setOutputRange(VerticalElevatorConstants.kMinOuput, VerticalElevatorConstants.kMaxOutput);

		encoder.setPositionConversionFactor(VerticalElevatorConstants.conversionFactor);
	}

	public boolean getTopStatus() {
		return topSwitch.get();
	}

	public boolean getBottomStatus() {
		return bottomSwitch.get();
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

	}
}
