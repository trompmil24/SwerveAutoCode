// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;

/** Add your docs here. */
public class autonomous {

static DrivetrainSubsystem m_subsystem = new DrivetrainSubsystem();

    //static SequentialCommandGroup autoPractice = new SequentialCommandGroup(new ResetTrajectory(m_subsystem, RobotContainer.getAutoPath()), RobotContainer.getFullAuto());

    public static SequentialCommandGroup autoGetter()
    {
        return null;
    }
}