// ============================================================
// STAGE 3 of 3 — DRIVEBASE + ARM + INTAKE (Complete Robot)
// ============================================================
// Everything from Stage 2 is still here — driving and arm
// control work exactly the same.
//
// NEW in this stage:
//   - A second SparkMax motor controller for the intake
//   - More constants for intake power and current limits
//   - setIntakeMotor() helper method
//   - A STATE VARIABLE: lastGamePiece
//     The robot now "remembers" what it picked up and applies
//     a small holding power to keep it from dropping.
//   - Named constants CONE, CUBE, NOTHING to represent state
//     (instead of confusing numbers like 1, 2, 3)
//
// FULL CONTROLLER MAP:
//   Left  Stick  (Y-axis)  → Drive forward/backward
//   Right Stick  (X-axis)  → Turn left/right
//   Left  Trigger          → Lower the arm
//   Left  Bumper           → Raise the arm
//   Right Trigger          → Run intake (cube in / cone out)
//   Right Bumper           → Run intake other way (cone in / cube out)
// ============================================================

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.stages;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Stage3_DrivebaseArmIntake extends TimedRobot {

  // DRIVE MOTOR FIELDS (same as Stages 1 & 2)
  WPI_VictorSPX frontLeftVictor  = new WPI_VictorSPX(1);
  WPI_VictorSPX frontRightVictor = new WPI_VictorSPX(3);
  WPI_VictorSPX backLeftVictor   = new WPI_VictorSPX(2);
  WPI_VictorSPX backRightVictor  = new WPI_VictorSPX(4);

  // ARM MOTOR FIELDS (same as Stage 2)
  SparkMax arm = new SparkMax(6, MotorType.kBrushless);
  SparkMaxConfig armConfig = new SparkMaxConfig();

  // ==========================================================
  // INTAKE MOTOR FIELDS — NEW!
  // ==========================================================
  // The intake is another brushless motor (NEO 550) on a SparkMax.
  // CAN ID 5 — change to match your robot.
  SparkMax intake = new SparkMax(5, MotorType.kBrushless);
  SparkMaxConfig intakeConfig = new SparkMaxConfig();

  // This variable tracks the current limit we last sent to the intake.
  // We use it to avoid re-configuring the motor every single loop
  // (20ms is fast — unnecessary reconfigures waste time on the CAN bus).
  double currCurr = 0;

  // ==========================================================
  // CONTROLLER
  // ==========================================================
  CommandXboxController m_controller = new CommandXboxController(0);

  // ==========================================================
  // CONSTANTS (from Stage 2, plus new intake constants)
  // ==========================================================

  /** Max amps for the arm motor */
  static final int    ARM_CURRENT_LIMIT_A       = 20;
  /** Arm speed as a fraction of full power (0.0 – 1.0) */
  static final double ARM_OUTPUT_POWER          = 0.4;

  /** Max amps while the intake is actively picking up a game piece */
  static final int    INTAKE_CURRENT_LIMIT_A    = 25;

  /** Max amps while the intake is just HOLDING a game piece.
   *  Much lower — we only need a tiny squeeze to keep it from falling. */
  static final int    INTAKE_HOLD_CURRENT_LIMIT_A = 5;

  /** Full intake speed for picking up / ejecting */
  static final double INTAKE_OUTPUT_POWER       = 1.0;

  /** Tiny holding power to keep a game piece from slipping out */
  static final double INTAKE_HOLD_POWER         = 0.07;

  // ==========================================================
  // STATE CONSTANTS & STATE VARIABLE — NEW CONCEPT!
  // ==========================================================
  // A STATE is a way to describe "what situation is the robot in
  // right now?" We represent state with an integer variable.
  //
  // We COULD write:
  //   if (lastGamePiece == 1) { ... }   ← What does 1 mean??
  //
  // Instead we give the numbers NAMES, making the code readable:
  //   if (lastGamePiece == CONE) { ... } ← Much clearer!
  //
  // These are the three possible states for the intake:
  static final int CONE    = 1;  // Last picked up a cone
  static final int CUBE    = 2;  // Last picked up a cube
  static final int NOTHING = 3;  // Haven't picked up anything yet

  // lastGamePiece REMEMBERS what the robot last picked up.
  // This is the robot's "memory" — it persists between loops.
  // Each call to teleopPeriodic() can read AND write this variable.
  int lastGamePiece;


  // ==========================================================
  // METHOD: robotInit()
  // ==========================================================
  @Override
  public void robotInit() {

    // --- Drive motor setup (same as Stage 1) -----------------
    frontLeftVictor.setInverted(false);
    frontRightVictor.setInverted(true);
    backLeftVictor.setInverted(false);
    backRightVictor.setInverted(true);

    // --- Arm motor setup (same as Stage 2) -------------------
    armConfig.inverted(true);
    armConfig.idleMode(IdleMode.kBrake);
    armConfig.smartCurrentLimit(ARM_CURRENT_LIMIT_A);
    arm.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // --- Intake motor setup — NEW! ---------------------------
    // inverted(false): positive power = intake spins inward for cubes.
    // (Reversed for cones — we'll handle that in teleopPeriodic.)
    intakeConfig.inverted(false);

    // Brake mode keeps the intake from spinning freely.
    // This helps hold the game piece even without motor power.
    intakeConfig.idleMode(IdleMode.kBrake);

    // Start with a current limit of 0 — we'll update it dynamically
    // when we know if we're picking up or holding.
    intakeConfig.smartCurrentLimit(0);
    currCurr = 0;

    intake.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }


  // METHOD: setDriveMotors(forward, turn) — same as Stage 1
  public void setDriveMotors(double forward, double turn) {
    SmartDashboard.putNumber("drive forward power (%)", forward);
    SmartDashboard.putNumber("drive turn power (%)", turn);

    double left  = forward - turn;
    double right = forward + turn;

    SmartDashboard.putNumber("drive left power (%)", left);
    SmartDashboard.putNumber("drive right power (%)", right);

    frontLeftVictor.set(left);
    frontRightVictor.set(right);
    backLeftVictor.set(left);
    backRightVictor.set(right);
  }


  // METHOD: setArmMotor(percent) — same as Stage 2
  public void setArmMotor(double percent) {
    arm.set(percent);
    SmartDashboard.putNumber("arm power (%)", percent);
    SmartDashboard.putNumber("arm motor current (amps)", arm.getOutputCurrent());
  }


  // ==========================================================
  // METHOD: setIntakeMotor(percent, amps) — NEW!
  // ==========================================================
  // Runs the intake at a given power level, using the given
  // current limit. The current limit changes depending on
  // whether we're actively intaking or just holding.
  //
  // PARAMETERS:
  //   percent — Power from -1.0 to +1.0
  //             Positive = intake spins inward (for cubes)
  //             Negative = intake spins outward (for cones)
  //   amps    — Current limit to apply to the intake motor.
  //             We change this dynamically: full amps when picking
  //             up, tiny amps when just holding.
  // ==========================================================
  public void setIntakeMotor(double percent, int amps) {
    intake.set(percent);  // Set the intake motor speed

    // Only reconfigure the current limit if it actually changed.
    // Reconfiguring every loop (50 times/sec) would flood the CAN
    // bus with unnecessary messages. This check avoids that.
    if (amps != currCurr) {
      intakeConfig.smartCurrentLimit(amps);
      // kNoResetSafeParameters: don't wipe our other settings,
      // just update the current limit.
      intake.configure(intakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
      currCurr = amps;  // Remember what we just set so we can compare next loop
    }

    SmartDashboard.putNumber("intake power (%)", percent);
    SmartDashboard.putNumber("intake motor current (amps)", intake.getOutputCurrent());
  }


  // METHOD: robotPeriodic() — same as Stage 1
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Time (seconds)", Timer.getFPGATimestamp());
  }


  // ==========================================================
  // METHOD: teleopInit()
  // ==========================================================
  @Override
  public void teleopInit() {
    // Coast mode on all drive motors
    frontLeftVictor.setNeutralMode(NeutralMode.Coast);
    frontRightVictor.setNeutralMode(NeutralMode.Coast);
    backLeftVictor.setNeutralMode(NeutralMode.Coast);
    backRightVictor.setNeutralMode(NeutralMode.Coast);

    // At the start of teleop, we haven't picked up anything yet.
    lastGamePiece = NOTHING;
  }


  // ==========================================================
  // METHOD: teleopPeriodic()
  // ==========================================================
  @Override
  public void teleopPeriodic() {

    // --- ARM CONTROL (same as Stage 2) -----------------------
    double armPower;

    if (m_controller.leftTrigger().getAsBoolean()) {
      armPower = -ARM_OUTPUT_POWER;   // Trigger held → lower arm
    } else if (m_controller.leftBumper().getAsBoolean()) {
      armPower = ARM_OUTPUT_POWER;    // Bumper held → raise arm
    } else {
      armPower = 0.0;                 // Nothing held → hold position
    }

    setArmMotor(armPower);

    // --- INTAKE CONTROL — NEW! -------------------------------
    // This is the most complex if/else chain yet. Read it carefully.
    //
    // The logic has two layers:
    //   LAYER 1: Is the driver actively pressing a button right now?
    //            If yes, run the intake at full power.
    //   LAYER 2: If no button is pressed, check what was last picked up.
    //            Apply a small holding power to keep it from falling.
    //
    // WHY HOLDING POWER?
    //   Without it, a game piece would drop as soon as the driver
    //   lets go of the trigger. A tiny squeeze keeps it secure.
    //   But we use a very low current limit (5A) so the motor
    //   doesn't overheat — it's barely working.

    double intakePower;  // Speed to send to the intake
    int    intakeAmps;   // Current limit to use

    if (m_controller.rightTrigger().getAsBoolean()) {
      // -------------------------------------------------------
      // RIGHT TRIGGER: Spin intake INWARD (positive direction)
      //   → This picks up CUBES (they get sucked in)
      //   → Or it pushes out CONES (if the arm is in cone position)
      // Use full power and full current limit for picking up.
      // Also update lastGamePiece so the robot remembers what it got.
      // -------------------------------------------------------
      intakePower   = INTAKE_OUTPUT_POWER;
      intakeAmps    = INTAKE_CURRENT_LIMIT_A;
      lastGamePiece = CUBE;  // "I just tried to pick up a cube"

    } else if (m_controller.rightBumper().getAsBoolean()) {
      // -------------------------------------------------------
      // RIGHT BUMPER: Spin intake OUTWARD (negative direction)
      //   → This picks up CONES (they get pulled in the other way)
      //   → Or it ejects CUBES
      // -------------------------------------------------------
      intakePower   = -INTAKE_OUTPUT_POWER;
      intakeAmps    = INTAKE_CURRENT_LIMIT_A;
      lastGamePiece = CONE;  // "I just tried to pick up a cone"

    } else if (lastGamePiece == CUBE) {
      // -------------------------------------------------------
      // No button pressed, but we last picked up a CUBE.
      // Apply a tiny positive squeeze to hold it.
      // Low current limit (5A) so the motor doesn't overheat.
      // -------------------------------------------------------
      intakePower = INTAKE_HOLD_POWER;
      intakeAmps  = INTAKE_HOLD_CURRENT_LIMIT_A;

    } else if (lastGamePiece == CONE) {
      // -------------------------------------------------------
      // No button pressed, but we last picked up a CONE.
      // Apply a tiny NEGATIVE squeeze to hold it (opposite direction).
      // -------------------------------------------------------
      intakePower = -INTAKE_HOLD_POWER;
      intakeAmps  = INTAKE_HOLD_CURRENT_LIMIT_A;

    } else {
      // -------------------------------------------------------
      // No button pressed and nothing has been picked up (NOTHING).
      // Just stop the intake completely.
      // -------------------------------------------------------
      intakePower = 0.0;
      intakeAmps  = 0;
    }

    setIntakeMotor(intakePower, intakeAmps);

    // --- DRIVE CONTROL (same as Stage 1) ---------------------
    setDriveMotors(
      -m_controller.getLeftY(),
      -m_controller.getRightX()
    );
  }

} // End of class Stage3_DrivebaseArmIntake
