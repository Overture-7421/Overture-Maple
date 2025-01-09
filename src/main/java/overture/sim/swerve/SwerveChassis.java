package overture.sim.swerve;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.littletonrobotics.junction.Logger;
import overture.sim.NTIMU;
import overture.sim.robots.SimBaseRobot;

public class SwerveChassis extends SwerveDriveSimulation {
    SwerveModule frontLeftModule, frontRightModule, backLeftModule, backRightModule;
    NTIMU imu;
    SwerveModuleState[] states = new SwerveModuleState[4];
    SimBaseRobot robot;
    public SwerveChassis(SimBaseRobot robot, Pose2d startingPose, DriveTrainSimulationConfig driveTrainConfig) {
        super(driveTrainConfig, startingPose);
        this.robot = robot;

        frontLeftModule = new SwerveModule(robot.GetName(), "front_left", getModules()[0]);
        frontRightModule = new SwerveModule(robot.GetName(), "front_right", getModules()[1]);
        backLeftModule = new SwerveModule(robot.GetName(), "back_left", getModules()[2]);
        backRightModule = new SwerveModule(robot.GetName(), "back_right", getModules()[3]);

        imu = new NTIMU(new NTIMU.Config() {
            {
                Name = robot.GetName() + "/imu";
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

        Logger.recordOutput(robot.GetName() + "/SimulatedChassis/ModuleStates", states);
    }
}
