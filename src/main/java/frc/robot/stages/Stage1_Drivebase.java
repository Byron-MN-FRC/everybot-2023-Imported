// ============================================================
// STAGE 1 of 3 — DRIVEBASE
// ============================================================
// In this stage, the robot can DRIVE using the joystick.
// The left stick moves the robot forward and backward.
// The right stick turns the robot left and right.
//
// This is the starting template. Read every comment carefully!
// By the end, you should understand:
//   - What a CLASS is
//   - What a FIELD (variable) is
//   - What a METHOD (function) is
//   - How the robot lifecycle works (init vs. periodic)
//   - How arcade drive math turns one joystick into 4 motors
// ============================================================

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// The "package" line tells Java where this file lives in the project.
// Think of it like a folder address.
package frc.robot.stages;

// "import" lines bring in code that other people have already written.
// Instead of writing motor controller code from scratch, we import
// the library that CTRE (the manufacturer) wrote for us.
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

// WPILib is the main robotics library for FRC.
// TimedRobot gives us the basic structure every FRC robot needs.
// Timer lets us read a clock. SmartDashboard lets us send data
// to the driver station screen so we can see what the robot is doing.
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// This imports the class that lets us read button and joystick inputs
// from an Xbox controller.
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

// ============================================================
// CLASS DEFINITION
// ============================================================
// A CLASS is a blueprint. Just like a blueprint for a house
// describes what rooms it has and how they connect, a class
// describes what data a robot has and what it can DO.
//
// "public" means other parts of the code can see this class.
// "class Stage1_Drivebase" is the name of our blueprint.
// "extends TimedRobot" means our robot IS a TimedRobot —
//   it inherits all the built-in FRC robot structure automatically.
// ============================================================
public class Stage1_Drivebase extends TimedRobot {

  // ==========================================================
  // FIELDS (Class-level Variables)
  // ==========================================================
  // A FIELD is a variable that belongs to the whole class.
  // Think of fields like the robot's "parts list."
  // Every method in this class can see and use these fields.
  //
  // Here we create 4 motor controller objects — one for each
  // corner of the robot's drivetrain.
  //
  // WPI_VictorSPX is the type (the brand/model of motor controller).
  // The number in parentheses is the CAN bus ID — it's like an
  // address that tells the code WHICH physical controller to talk to.
  //
  // IMPORTANT: Change these numbers to match YOUR robot's wiring!
  // ==========================================================

  WPI_VictorSPX frontLeftVictor  = new WPI_VictorSPX(1); // Front-left motor, CAN ID 1
  WPI_VictorSPX frontRightVictor = new WPI_VictorSPX(3); // Front-right motor, CAN ID 3
  WPI_VictorSPX backLeftVictor   = new WPI_VictorSPX(2); // Back-left motor, CAN ID 2
  WPI_VictorSPX backRightVictor  = new WPI_VictorSPX(4); // Back-right motor, CAN ID 4

  // This creates a controller object connected to port 0 on the driver station.
  // Port 0 is just whatever USB slot the Xbox controller is plugged into first.
  CommandXboxController m_controller = new CommandXboxController(0);


  // ==========================================================
  // METHOD: robotInit()
  // ==========================================================
  // A METHOD is a named block of code that does a specific job.
  // You can think of it like a recipe — it has a name, and
  // when you "call" it, the instructions inside run in order.
  //
  // robotInit() is special: WPILib calls it ONCE, automatically,
  // when the robot first powers on. Use it for one-time setup.
  //
  // The "@Override" above a method means: "This method already
  // exists in TimedRobot — I'm replacing it with my own version."
  // ==========================================================
  @Override
  public void robotInit() {

    // --------------------------------------------------------
    // MOTOR INVERSION SETUP
    // --------------------------------------------------------
    // On a tank/west-coast drivetrain, the motors on the LEFT
    // side and RIGHT side face opposite directions physically.
    // If we give both sides the same "positive" signal, one side
    // pushes forward and the other pushes backward — the robot spins!
    //
    // setInverted(true) flips the direction for that motor so
    // "positive" always means "move the robot forward."
    //
    // How to figure out which ones to invert:
    //   1. Comment out 3 of the 4 set() calls in setDriveMotors().
    //   2. Push the joystick forward.
    //   3. If the wheel spins backward, change false → true here.
    //   4. Repeat for each of the other 3 motors.
    // --------------------------------------------------------
    frontLeftVictor.setInverted(false);  // Left side: not inverted
    frontRightVictor.setInverted(true);  // Right side: inverted (faces opposite way)
    backLeftVictor.setInverted(false);   // Left side: not inverted
    backRightVictor.setInverted(true);   // Right side: inverted
  }


  // ==========================================================
  // METHOD: setDriveMotors(forward, turn)
  // ==========================================================
  // This is a HELPER METHOD — we wrote it ourselves to keep
  // the code organized. Instead of writing the same motor-setting
  // math in multiple places, we write it once here and call it
  // whenever we need to drive.
  //
  // PARAMETERS:
  //   forward — How fast to go forward/backward. Range: -1.0 to +1.0
  //             Positive = forward, Negative = backward
  //   turn    — How fast to turn. Range: -1.0 to +1.0
  //             Positive = counter-clockwise, Negative = clockwise
  //
  // ARCADE DRIVE MATH EXPLAINED:
  //   To turn left, the left side of the robot slows down (or goes
  //   backward) while the right side speeds up. The math below
  //   does exactly that:
  //
  //     left  = forward - turn
  //     right = forward + turn
  //
  //   Example — turning left (turn is positive):
  //     forward = 0.5,  turn = 0.3
  //     left  = 0.5 - 0.3 = 0.2  (left side slows down)
  //     right = 0.5 + 0.3 = 0.8  (right side speeds up)
  //   Result: robot curves LEFT. ✓
  // ==========================================================
  public void setDriveMotors(double forward, double turn) {

    // Send the raw values to SmartDashboard so we can see them
    // on the driver station screen. Great for debugging!
    SmartDashboard.putNumber("drive forward power (%)", forward);
    SmartDashboard.putNumber("drive turn power (%)", turn);

    // Calculate power for each side using arcade drive math
    double left  = forward - turn;
    double right = forward + turn;

    // Display the calculated left/right values too
    SmartDashboard.putNumber("drive left power (%)", left);
    SmartDashboard.putNumber("drive right power (%)", right);

    // Send the calculated power to all four motor controllers.
    // Front and back on the same side always get the same value.
    frontLeftVictor.set(left);
    frontRightVictor.set(right);
    backLeftVictor.set(left);
    backRightVictor.set(right);
  }


  // ==========================================================
  // METHOD: robotPeriodic()
  // ==========================================================
  // WPILib calls this method every 20 milliseconds (50 times
  // per second), no matter what mode the robot is in.
  // Use it for things that should always be happening —
  // like sending sensor data to the dashboard.
  // ==========================================================
  @Override
  public void robotPeriodic() {
    // Send the current timestamp to SmartDashboard.
    // FPGA = Field Programmable Gate Array — it's the onboard
    // clock chip on the roboRIO. This is the most accurate timer
    // available on the robot.
    SmartDashboard.putNumber("Time (seconds)", Timer.getFPGATimestamp());
  }


  // ==========================================================
  // METHOD: teleopInit()
  // ==========================================================
  // Called ONCE when the robot switches into Teleoperated mode
  // (driver-controlled). Use it to set up anything specific
  // to teleop before the periodic loop starts.
  // ==========================================================
  @Override
  public void teleopInit() {
    // Set all drive motors to COAST mode.
    // COAST = when power is removed, the motor spins freely until
    //         it naturally slows down (like a bike coasting).
    // BRAKE = when power is removed, the motor actively resists
    //         motion and stops quickly.
    //
    // We use Coast for driving so the robot doesn't jerk to a stop
    // when the driver lets go of the stick.
    frontLeftVictor.setNeutralMode(NeutralMode.Coast);
    frontRightVictor.setNeutralMode(NeutralMode.Coast);
    backLeftVictor.setNeutralMode(NeutralMode.Coast);
    backRightVictor.setNeutralMode(NeutralMode.Coast);
  }


  // ==========================================================
  // METHOD: teleopPeriodic()
  // ==========================================================
  // Called every 20 ms while the robot is in Teleoperated mode.
  // This is the "game loop" — it runs over and over while the
  // driver is controlling the robot.
  //
  // Everything the driver can do with the controller happens here.
  // ==========================================================
  @Override
  public void teleopPeriodic() {

    // Read the left joystick's Y-axis (up/down) for forward/backward.
    // Read the right joystick's X-axis (left/right) for turning.
    //
    // WHY THE NEGATIVE SIGNS?
    // Xbox joystick axes are "backwards" by convention —
    // pushing the stick UP returns a NEGATIVE number.
    // We flip the sign so pushing UP gives us a POSITIVE forward value,
    // which makes the math in setDriveMotors() feel natural.
    setDriveMotors(
      -m_controller.getLeftY(),   // Negate: up on stick = positive forward
      -m_controller.getRightX()   // Negate: right on stick = positive turn right
    );
  }

} // End of class Stage1_Drivebase
