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
import overture.sim.mechanisms.flywheel.Flywheel;
import overture.sim.swerve.Constants;
import overture.sim.swerve.SwerveChassis;

public class DaytonaOffseason extends SimBaseRobot {
    SwerveChassis driveTrain;
    Elevator elevator;
    Arm armRotator;
    Flywheel armWheels;
    Transform3d originalRobotToArmCarrier, originalRobotToArmRotator, originalRobotToArmWheels;

    List<SimMechanism> mechanisms;

    public DaytonaOffseason(String name, Pose2d startingPose) {
        super(name, startingPose);

        // Drivertain
        driveTrain = new SwerveChassis(this, startingPose, Constants.Swerve2024());

        // Elevator Carrier
        elevator = new Elevator(this,
                new Transform3d(Meters.of(0.105), Meters.of(0), Meters.of(0.11), new Rotation3d()),
                new Translation3d(0, 0, 1), // Elevator moves on this axis
                "elevator",
                DCMotor.getKrakenX60(2),
                5.6,
                Kilograms.of(0.5),
                Meters.of(0.1),
                Meters.of(0.0),
                Meters.of(1.35),
                Meters.of(0.0),
                2,
                false);

        // Arm Rotator
        originalRobotToArmCarrier = new Transform3d(Meters.of(-0.07), Meters.of(0.005), Meters.of(0.17), new Rotation3d());
        armRotator = new Arm(this,
                new Transform3d(originalRobotToArmCarrier.getMeasureX(), originalRobotToArmCarrier.getMeasureY(), originalRobotToArmCarrier.getMeasureZ(), originalRobotToArmCarrier.getRotation()),
                new Rotation3d(1, 0, 0), // Arm rotations around this axis
                "arm_rotator",
                DCMotor.getKrakenX60(2),
                80.88888,
                1.0,
                Meters.of(1),
                Degrees.of(-999), // -999
                Degrees.of(999.0), // 999
                Degrees.of(0.0),
                false,
                false);


        // Intake Wheels
        originalRobotToArmWheels = new Transform3d(Meters.of(0.0), Meters.of(0), Meters.of(0), new Rotation3d());
        armWheels = new Flywheel(this,
                new Transform3d(originalRobotToArmWheels.getMeasureX(), originalRobotToArmWheels.getMeasureY(), originalRobotToArmWheels.getMeasureZ(), originalRobotToArmWheels.getRotation()),
                new Rotation3d(1, 0, 0), // Flywheel rotates around this axis
                "intake",
                DCMotor.getKrakenX60(1),
                2.25,
                0.01,
                false,
                true);






        // List of mechanisms
        mechanisms = List.of(elevator, armRotator, armWheels);
    }

    @Override
    public void Update() {
        driveTrain.Update();
        mechanisms.forEach(mech -> mech.Update());

        // Actualizar la posición del brazo en base en el elevador
        Pose3d elevatorPose = elevator.GetPoses3d().get(0);
        armRotator.SetRobotToMechanism(
                originalRobotToArmCarrier.plus(new Transform3d(elevatorPose.getTranslation(), new Rotation3d())));

        // Actualizar la posición de armWheels en base en el elevador
        Pose3d armWheelsPose = elevator.GetPoses3d().get(0);
        armWheels.SetRobotToMechanism(
            originalRobotToArmWheels.plus(new Transform3d(armWheelsPose.getTranslation(), new Rotation3d())));


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