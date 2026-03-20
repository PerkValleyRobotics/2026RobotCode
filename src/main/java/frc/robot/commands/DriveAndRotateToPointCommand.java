package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

import java.util.function.DoubleSupplier;

public class DriveAndRotateToPointCommand {

    /**
     * Creates a command that lets the driver control X/Y while rotating
     * automatically
     * toward a fixed field point.
     */
    public static Command create(
            Drive drive,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier) {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        double targetX = alliance == Alliance.Red ? 4.537 : 11.879;
        double targetY = 4.035;

        // Rotation supplier that always points toward the target point
        java.util.function.Supplier<Rotation2d> rotationSupplier = () -> {
            Pose2d robotPose = drive.getPose();
            double angleToTarget = Math.atan2(targetY - robotPose.getY(), targetX - robotPose.getX());
            return new Rotation2d(angleToTarget);
        };

        // Use your joystickDriveAtAngle helper
        return DriveCommands.joystickDriveAtAngle(drive, xSupplier, ySupplier, rotationSupplier);
    }
}