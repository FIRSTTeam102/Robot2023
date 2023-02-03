// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.VisionConstants.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
	/** Creates a new Vision. */
	public Vision() {

	}

	@Override
	public void periodic() {

		double v = tv.getDouble(0.0);
		double tx = tx.getDouble(0.0);
		double ty = ty.getDouble(0.0);
		double ta = ta.getDouble(0.0);
		double APID = ID.getDoubleArray(double[1]);
		double TX = TX.getDoubleArray(double[1]);
		double TY = TX.getDoubleArray(double[2]);
		double TZ = TX.getDoubleArray(double[3]);
		double RX = TX.getDoubleArray(double[4]);
		double RY = TX.getDoubleArray(double[5]);
		double RZ = TX.getDoubleArray(double[6]);
		double tclass = tclass.getDouble(0.0);

		SmartDashboard.putNumber("Limelight tv", v);
		SmartDashboard.putNumber("Limelight tx", tx);
		SmartDashboard.putNumber("Limelight ty", ty);
		SmartDashboard.putNumber("Limelight ta", ta);
		SmartDashboard.putNumber("Limelight APID", APID);
		SmartDashboard.putNumber("Limelight TX", TX);
		SmartDashboard.putNumber("Limelight TY", TY);
		SmartDashboard.putNumber("Limelight TZ", TZ);
		SmartDashboard.putNumber("Limelight RX", RX);
		SmartDashboard.putNumber("Limelight RY", RY);
		SmartDashboard.putNumber("Limelight RZ", RZ);
		SmartDashboard.putNumber("Limelight tclass", tclass);

		System.out.println(v);
		System.out.println(tx);
		System.out.println(ty);
		System.out.println(ta);
		System.out.println(TX);
		System.out.println(TY);
		System.out.println(TZ);
		System.out.println(RX);
		System.out.println(RY);
		System.out.println(RZ);
		System.out.println(tclass);
	}
}
