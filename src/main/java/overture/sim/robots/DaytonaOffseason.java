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
    Arm armCarrier, armRotator, armClimber, intakeRotator;
    Flywheel intakeWheels;
    Transform3d originalRobotToArmCarrier, originalRobotToArmRotator, originalRobotToArmClimber, originalRobotToIntake, originalRobotToIntakeRotator;

    List<SimMechanism> mechanisms;

    public DaytonaOffseason(String name, Pose2d startingPose) {
        super(name, startingPose);

        // Drivertain
        driveTrain = new SwerveChassis(this, startingPose, Constants.Swerve2024());

        // Elevator Carrier
        elevator = new Elevator(this,
                new Transform3d(Meters.of(2), Meters.of(2), Meters.of(2), new Rotation3d()),
                new Translation3d(0, 0, 1), // Elevator moves on this axis
                "elevator",
                DCMotor.getKrakenX60(2),
                5.6,
                Kilograms.of(0.5),
                Meters.of(0.1),
                Meters.of(0.0),
                Meters.of(1.05),
                Meters.of(0.0),
                2,
                false);



        // List of mechanisms
        mechanisms = List.of(elevator);
    }

    @Override
    public void Update() {
        driveTrain.Update();
        mechanisms.forEach(mech -> mech.Update());

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