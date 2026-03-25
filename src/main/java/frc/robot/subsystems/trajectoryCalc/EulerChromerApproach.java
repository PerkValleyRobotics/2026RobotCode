package frc.robot.subsystems.trajectoryCalc;

import static frc.robot.FieldConstants.BLUE_HUB_TRIANGULATED_X;
import static frc.robot.FieldConstants.HUB_TRIANGULATED_Y;
import static frc.robot.FieldConstants.HUB_TRIANGULATED_Z;
import static frc.robot.FieldConstants.RED_HUB_TRIANGULATED_X;
import static frc.robot.subsystems.launcher.LauncherConstants.HOOD_ANGLE_MAX_LIMIT;
import static frc.robot.subsystems.launcher.LauncherConstants.HOOD_ANGLE_MIN_LIMIT;
import static frc.robot.subsystems.trajectoryCalc.TrajectoriesConstants.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public class EulerChromerApproach implements LauncherTrajectoryCalc {
    public double calculate(inputParameters inputs) {
        double OptimalAngle = 45;
        double range = 45;
        double tolerance = 0.05;

        Translation2d Target = DriverStation.getAlliance().get() == DriverStation.Alliance.Red
                ? new Translation2d(RED_HUB_TRIANGULATED_X, HUB_TRIANGULATED_Y)
                : new Translation2d(BLUE_HUB_TRIANGULATED_X, HUB_TRIANGULATED_Y);
        Target.plus(inputs.velocityVectors());

        double targetDistance = inputs.currentPose().getTranslation().getDistance(Target);
        double x = simulateTrajectory(OptimalAngle, inputs.launchVelocity());

        Timer taskObserver = new Timer();
        taskObserver.start();
        while ((Math.abs(x - targetDistance) > tolerance) || !taskObserver.hasElapsed(0.05)) {
            range /= 2;
            if ((x - targetDistance) > 0) {
                OptimalAngle -= range;
            } else {
                OptimalAngle += range;
            }

            if (OptimalAngle > HOOD_ANGLE_MAX_LIMIT || OptimalAngle < HOOD_ANGLE_MIN_LIMIT) {
                break;
            }
            x = simulateTrajectory(OptimalAngle, inputs.launchVelocity());
        }
        taskObserver.stop();
        return OptimalAngle;
    }

    public double simulateTrajectory(double angle, double initialVelocity) {
        double dt = 0.02;

        double vx = Math.cos(angle) * initialVelocity;
        double vy = Math.sin(angle) * initialVelocity;

        double y = 0;
        double x = 0;

        while (y >= 0) {
            double v = Math.sqrt((Math.pow(vx, 2) + Math.pow(vy, 2)));
            double ax = ((DRAG_COEFFICENT / FUEL_MASS) * v * vx);
            double ay = -9.8 - (DRAG_COEFFICENT * v * vy);

            x += (0.5 * ax * (Math.pow(dt, 2))) * vx * dt;
            y += (0.5 * ay * (Math.pow(dt, 2))) * vy * dt;

            if ((y <= HUB_TRIANGULATED_Z) && (vy < 0)) {
                break;
            }

            vx += ax;
            vy += ay;
        }
        return x;
    }
}
