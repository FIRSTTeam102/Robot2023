Swerve library based on: (with modifications/refactors)
* [HuskieRobotics/3061-lib](https://github.com/HuskieRobotics/3061-lib)
* [Mechanical-Advantage/SwerveDevelopment](https://github.com/Mechanical-Advantage/SwerveDevelopment)
* [Team364/BaseFalconSwerve](https://github.com/Team364/BaseFalconSwerve)
* [REVrobotics/MAXSwerve-Java-Template](https://github.com/REVrobotics/MAXSwerve-Java-Template)

Motor communication is abstracted through [SwerveModuleIO](./SwerveModuleIO.java) and implementations for physical/sim hardware

# Poses
* the origin of the field to the lower left corner (driver's right)
* zero is away from the driver
* angle increases in the CCW direction

# Rotations
* pitch = rotating around side-to-side axis
* yaw = rotating around vertical axis
* roll = rotating around front-to-back axis