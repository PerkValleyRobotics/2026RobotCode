// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems.intake;

// import static frc.robot.subsystems.launcher.LauncherConstants.TURNING_MOTOR_MAX_AMPERAGE;

// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.spark.FeedbackSensor;
// import com.revrobotics.spark.SparkBase;
// import com.revrobotics.spark.SparkClosedLoopController;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.revrobotics.spark.config.SparkMaxConfig;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import frc.robot.subsystems.intake.IntakeIOConstants;
// import static frc.robot.util.SparkUtil.*;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class IntakePID extends SubsystemBase {
//   /** Creates a new IntakePID. */

//   SparkBase leftTurnMotor = new SparkMax(50, MotorType.kBrushless);
//   SparkBase rightTurnMotor = new SparkMax(51, MotorType.kBrushless);

//   RelativeEncoder ML = leftTurnMotor.getEncoder();
//   RelativeEncoder MR = rightTurnMotor.getEncoder();

//   SparkClosedLoopController controller = leftTurnMotor.getClosedLoopController();

//   public IntakePID() {
//     SparkMaxConfig turnConf = new SparkMaxConfig();

//     turnConf
//         .idleMode(IdleMode.kBrake)
//         .smartCurrentLimit(TURNING_MOTOR_MAX_AMPERAGE)
//         .voltageCompensation(12.0)
//         .inverted(false);
//     turnConf.encoder.uvwMeasurementPeriod(10).uvwAverageDepth(2);
//     tryUntilOk(
//         leftTurnMotor,
//         5,
//         () ->
//             leftTurnMotor.configure(
//                 turnConf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

//     tryUntilOk(leftTurnMotor, 5, () -> leftTurnEncoder.setPosition(0));

//     turnConf
//         .closedLoop
//         .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
//         .pid(INTAKE_TURN_MOTOR_kP, INTAKE_TURN_MOTOR_kI, INTAKE_TURN_MOTOR_kD);

//     SparkMaxConfig followConf = new SparkMaxConfig();

//         followConf
//             .idleMode(IdleMode.kBrake)
//             .smartCurrentLimit(TURNING_MOTOR_MAX_AMPERAGE)
//             .voltageCompensation(12.0)
//             .inverted(true)
//             .follow(INTAKE_LEFT_TURN_MOTOR_ID);
//         followConf.encoder.uvwMeasurementPeriod(10).uvwAverageDepth(2);
//         //
//         tryUntilOk(
//             rightTurnMotor,
//             5,
//             () ->
//                 rightTurnMotor.configure(
//                     followConf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

//         tryUntilOk(rightTurnMotor, 5, () -> rightTurnEncoder.setPosition(0));

//   }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//   }
// }
