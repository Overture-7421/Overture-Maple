package overture.sim.robots;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;

import java.util.List;

import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
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
    Arm armCarrier, armRotator, armClimber;
    Transform3d originalRobotToArmCarrier, originalRobotToArmRotator, originalRobotToArmClimber;

    List<SimMechanism> mechanisms;

    public Reefscape2025(String name, Pose2d startingPose) {
        super(name, startingPose);

        // Drivertain
        driveTrain = new SwerveChassis(this, startingPose, Constants.Swerve2024());

        // Elevator
        elevator = new Elevator(this,
                new Transform3d(Meters.of(0), Meters.of(0), Meters.of(0), new Rotation3d()),
                new Translation3d(0, 0, 1), // Elevator moves on this axis
                "elevator",
                DCMotor.getFalcon500(2),
                5.6,
                Kilograms.of(0.5),
                Meters.of(0.1),
                Meters.of(0.0),
                Meters.of(1.05),
                Meters.of(0.0),
                2,
                false);

        // Arm Carrier
        originalRobotToArmCarrier = new Transform3d(Meters.of(0.06), Meters.of(-0.005), Meters.of(0.12), new Rotation3d());
        armCarrier = new Arm(this,
                new Transform3d(originalRobotToArmCarrier.getMeasureX(), originalRobotToArmCarrier.getMeasureY(), originalRobotToArmCarrier.getMeasureZ(), originalRobotToArmCarrier.getRotation()),
                new Rotation3d(1, 0, 0), // Arm rotations around this axis
                "arm",
                DCMotor.getFalcon500(2),
                63.0,
                1.0,
                Meters.of(1),
                Degrees.of(-999), // -999
                Degrees.of(999.0), // 999
                Degrees.of(0.0),

                false,
                false);

        // Arm Rotator
        originalRobotToArmRotator = new Transform3d(Meters.of(0.03), Meters.of(-0.00), Meters.of(0.1), new Rotation3d());
        armRotator = new Arm(this,
    
                new Transform3d(originalRobotToArmRotator.getMeasureX(), originalRobotToArmRotator.getMeasureY(), originalRobotToArmRotator.getMeasureZ(), originalRobotToArmRotator.getRotation()),
                new Rotation3d(0, 0, 1), // Arm rotations around this axis
                "arm_rotator",
                DCMotor.getFalcon500(1),
                25.0,
                1.0,
                Meters.of(1),
                Degrees.of(-999), // -999
                Degrees.of(999.0), // 999
                Degrees.of(0.0),
                false,
                false);

        // Arm Climber
        originalRobotToArmClimber = new Transform3d(Meters.of(-0.25), Meters.of(0), Meters.of(0.2), new Rotation3d());
        armClimber = new Arm(this,
            new Transform3d(originalRobotToArmClimber.getMeasureX(), originalRobotToArmClimber.getMeasureY(), originalRobotToArmClimber.getMeasureZ(), originalRobotToArmClimber.getRotation()),
            new Rotation3d(0, 1, 0),
            "climber",
            DCMotor.getFalcon500(2),
            1.0,
            1.0,
            Meters.of(1),
            Degrees.of(-999.0),
            Degrees.of(999.0),
            Degrees.of(0.0),
            false,
            true);

        // List of mechanisms
        mechanisms = List.of(elevator, armCarrier, armRotator, armClimber);
    }

    @Override
    public void Update() {
        driveTrain.Update();
        mechanisms.forEach(mech -> mech.Update());

        // Update the arm's position based on the elevator's position
        Pose3d elevatorPose = elevator.GetPoses3d().get(0);
        armCarrier.SetRobotToMechanism(
                originalRobotToArmCarrier.plus(new Transform3d(elevatorPose.getTranslation(), new Rotation3d())));

        Pose3d carrierPose = armCarrier.GetPoses3d().get(0);
        armRotator.SetRobotToMechanism(originalRobotToArmRotator.plus(new Transform3d(carrierPose.getTranslation(), carrierPose.getRotation())));

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
