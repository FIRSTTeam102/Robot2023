// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.VerticalElevatorConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class VerticalElevator extends SubsystemBase {
	private TalonFX motor = new TalonFX(VerticalElevatorConstants.motorPort);
	private static DigitalInput topSwitch = new DigitalInput(VerticalElevatorConstants.topSwitchPort);
	private static DigitalInput bottomSwitch = new DigitalInput(VerticalElevatorConstants.bottomSwitchPort);

	/** Creates a new VerticalElevator. */
	public VerticalElevator() {
		motor.configFactoryDefault();
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
		motor.set(ControlMode.PercentOutput, MathUtil.clamp(speed, -1.0, 1.0));
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		System.out.println(
			String.format("Sensor Pos: %f | % Output: %f", motor.getSelectedSensorPosition(), motor.getMotorOutputPercent()));
	}
}
