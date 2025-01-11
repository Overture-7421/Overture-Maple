package overture.sim.robots;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;

import java.util.List;

import org.dyn4j.geometry.Rotation;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import overture.sim.mechanisms.SimMechanism;
import overture.sim.mechanisms.arm.Arm;
import overture.sim.mechanisms.elevator.Elevator;
import overture.sim.swerve.Constants;
import overture.sim.swerve.SwerveChassis;

public class Reefscape2025 extends SimBaseRobot {
    SwerveChassis driveTrain;
    Elevator elevator;
    Arm arm;
    Transform3d originalRobotToArm;

    List<SimMechanism> mechanisms;

    public Reefscape2025(String name, Pose2d startingPose) {
        super(name, startingPose);

        driveTrain = new SwerveChassis(this, startingPose, Constants.Swerve2024());
        elevator = new Elevator(this,
                new Transform3d(Meters.of(0), Meters.of(0), Meters.of(0), new Rotation3d()),
                new Translation3d(0, 0, 1), // Elevator moves on this axis
                "elevator",
                DCMotor.getFalcon500(1),
                9.0,
                Kilograms.of(0.5),
                Meters.of(0.1),
                Meters.of(0.0),
                Meters.of(1.0),
                Meters.of(0.0),
                false);

        originalRobotToArm = new Transform3d(Meters.of(0.1), Meters.of(0), Meters.of(0.5), new Rotation3d());
        arm = new Arm(this,
                new Transform3d(Meters.of(0.1), Meters.of(0), Meters.of(0.5), new Rotation3d()),
                new Rotation3d(1, 0, 0), // Arm rotations around this axis
                "arm",
                DCMotor.getFalcon500(1),
                40.0,
                1.0, 
                Meters.of(1),
                Degrees.of(0.0),
                Degrees.of(180.0),
                Degrees.of(0.0),
                false);

        mechanisms = List.of(elevator, arm);
    }

    @Override
    public void Update() {
        driveTrain.Update();
        mechanisms.forEach(mech -> mech.Update());

        // Arm is attached to end of elevator, so we need to update the arm's transform
        arm.SetRobotToMechanism(new Transform3d(originalRobotToArm.getTranslation(), originalRobotToArm.getRotation()).plus(new Transform3d(elevator.GetPose3d().getTranslation(), elevator.GetPose3d().getRotation())));
    }

    @Override
    public AbstractDriveTrainSimulation GetDriveTrain() {
        return driveTrain;
    }

    @Override
    public List<SimMechanism> GetMechanisms() {
        return mechanisms;
    }

}
