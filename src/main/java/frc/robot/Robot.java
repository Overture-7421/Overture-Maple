// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import overture.sim.SimMain;

public class Robot extends LoggedRobot {
    SimMain simMain = new SimMain();

    public Robot() {
        Logger.recordMetadata("Overture2024", "Maple"); // Set a metadata value
        Logger.addDataReceiver(new NT4Publisher()); // Save outputs to a new log
        Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.

        simMain.Initialize();
    }

    @Override
    public void robotInit() {}

    @Override
    public void robotPeriodic() {
        simMain.Periodic();

        NetworkTableInstance.getDefault().flush();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {}

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {}

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {}

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}
}
