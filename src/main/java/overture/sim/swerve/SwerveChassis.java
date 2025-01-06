package overture.sim.swerve;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.littletonrobotics.junction.Logger;
import overture.sim.NTIMU;

public class SwerveChassis extends SwerveDriveSimulation {

    public static SwerveModuleSimulationConfig ofMark4i(DCMotor driveMotor, DCMotor steerMotor, double wheelCOF) {
        return new SwerveModuleSimulationConfig(
                driveMotor,
                steerMotor,
                5.9027777,
                150.0 / 7.0,
                Volts.of(0.01),
                Volts.of(0.01),
                Inches.of(2),
                KilogramSquareMeters.of(0.03),
                wheelCOF);
    }

    static final DriveTrainSimulationConfig driveTrainSimulationConfig = DriveTrainSimulationConfig.Default()
            .withRobotMass(Kilograms.of(45))
            // Specify gyro type (for realistic gyro drifting and error simulation)
            .withGyro(COTS.ofPigeon2())
            // Specify swerve module (for realistic swerve dynamics)
            .withSwerveModule(SwerveChassis.ofMark4i(
                    DCMotor.getKrakenX60(1), // Drive motor is a Kraken X60
                    DCMotor.getFalcon500(1), // Steer motor is a Falcon 500
                    COTS.WHEELS.COLSONS.cof)) // Use the COF for Colson Wheels
            // Configures the track length and track width (spacing between swerve modules)
            .withCustomModuleTranslations(new Translation2d[] {
                new Translation2d(Inches.of(7.625), Inches.of(10.375)),
                new Translation2d(Inches.of(7.625), Inches.of(-10.375)),
                new Translation2d(Inches.of(-13.125), Inches.of(-10.375)),
                new Translation2d(Inches.of(-13.125), Inches.of(10.375)),
            })
            // Configures the bumper size (dimensions of the robot bumper)
            .withBumperSize(Inches.of(30), Inches.of(30));

    SwerveModule frontLeftModule, frontRightModule, backLeftModule, backRightModule;
    NTIMU imu;
    SwerveModuleState[] states = new SwerveModuleState[4];

    public SwerveChassis(String name, Pose2d startingPose) {
        super(driveTrainSimulationConfig, startingPose);

        frontLeftModule = new SwerveModule(name, "front_left", getModules()[0]);
        frontRightModule = new SwerveModule(name, "front_right", getModules()[1]);
        backLeftModule = new SwerveModule(name, "back_left", getModules()[2]);
        backRightModule = new SwerveModule(name, "back_right", getModules()[3]);

        imu = new NTIMU(new NTIMU.Config() {
            {
                Name = name + "/imu";
                Roll = () -> Degrees.of(0);
                Pitch = () -> Degrees.of(0);
                Yaw = () -> getSimulatedDriveTrainPose().getRotation().getMeasure();
            }
        });
    }

    public void Update() {
        frontLeftModule.Update();
        frontRightModule.Update();
        backLeftModule.Update();
        backRightModule.Update();
        imu.Update();

        for (int i = 0; i < 4; i++) {
            states[i] = getModules()[i].getCurrentState();
        }

        Logger.recordOutput("SimulatedChassis/ModuleStates", states);
    }
}
