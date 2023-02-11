package frc.robot.util;

import frc.robot.Robot;
import frc.robot.constants.BuildConstants;

import com.revrobotics.CANSparkMax;

import java.io.FileWriter;
import java.nio.file.Files;
import java.nio.file.Paths;

public class BuildManager {
	private static final String filePath = "/home/lvuser/build.txt";

	private static Boolean shouldBurn = null;

	public static boolean shouldBurn() {
		if (Robot.isSimulation())
			shouldBurn = false;

		if (shouldBurn == null) {
			// read previous file
			try {
				String previous = Files.readString(Paths.get(filePath));
				shouldBurn = !previous.equals(BuildConstants.BUILD_DATE);
			} catch (Exception e) {
				shouldBurn = true;
			}

			// write new file
			try {
				FileWriter fileWriter = new FileWriter(filePath);
				fileWriter.write(BuildConstants.BUILD_DATE);
				fileWriter.close();
			} catch (Exception e) {
				e.printStackTrace();
			}
		}

		return shouldBurn;
	}

	// burns config to spark only when new build
	public static void burnSpark(CANSparkMax spark) {
		if (shouldBurn())
			spark.burnFlash();
	}
}
