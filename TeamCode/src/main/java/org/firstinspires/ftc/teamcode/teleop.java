package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;

@TeleOp
public class teleop extends LinearOpMode {

    // ========== MOTOR POWER CONSTANTS ==========
    private static final double INTAKE_FULL_POWER = 1.0;
    private static final double INTAKE_HALF_POWER = 0.5;
    private static final double LAUNCH_MOTOR_POWER = 1.0;
    private static final double RAMP_MOTOR_POWER = 1.0;
    private static final double SLOW_MODE_MULTIPLIER = 0.3;
    private static final double JOYSTICK_DEADZONE = 0.1;

    // ========== DEAD WHEEL ODOMETRY CONSTANTS ==========
    private static final double ODO_COUNTS_PER_REV = 8192.0;  // REV Through Bore Encoder
    private static final double ODO_WHEEL_DIAMETER_INCHES = 1.26;  // 32mm converted
    private static final double ODO_COUNTS_PER_INCH = ODO_COUNTS_PER_REV / (ODO_WHEEL_DIAMETER_INCHES * Math.PI);

    // Dead wheel positions relative to robot center (inches)
    private static final double FORWARD_ODO_X_OFFSET = -1.0;
    private static final double FORWARD_ODO_Y_OFFSET = -3.5;
    private static final double RIGHT_ODO_X_OFFSET = 2.5;
    private static final double RIGHT_ODO_Y_OFFSET = -3.5;

    // ========== AUTONOMOUS NAVIGATION CONSTANTS ==========
    private static final double AUTO_MAX_SPEED = 0.7;
    private static final double AUTO_MIN_SPEED = 0.2;
    private static final double SLOWDOWN_DISTANCE_FEET = 1.0;
    private static final double POSITION_TOLERANCE_INCHES = 4.0;
    private static final double ANGLE_TOLERANCE_DEGREES = 3.0;

    // ========== NAVIGATION TARGETS ==========
    // Left Trigger: (0, -4) feet
    private static final double LEFT_TRIGGER_TARGET_X = 0.0;
    private static final double LEFT_TRIGGER_TARGET_Y = -4.0;

    // Right Trigger: (0, 0) feet (field center)
    private static final double RIGHT_TRIGGER_TARGET_X = 0.0;
    private static final double RIGHT_TRIGGER_TARGET_Y = 0.0;

    // ========== APRILTAG CONFIGURATION ==========
    private static final int RED_APRILTAG_ID = 24;
    private static final int BLUE_APRILTAG_ID = 20;

    // ========== STATE VARIABLES ==========
    private double robotX = 0;  // in feet
    private double robotY = 0;  // in feet
    private double robotHeading = 0;  // in radians
    private int lastForwardOdoPos = 0;
    private int lastRightOdoPos = 0;

    private enum Alliance { NONE, RED, BLUE }
    private Alliance selectedAlliance = Alliance.NONE;
    private boolean autoNavigating = false;
    private double targetX = 0;
    private double targetY = 0;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // ========== ALIGNMENT STATE ==========
    private enum LaunchState { STOPPED, ALIGNING, RUNNING }
    private LaunchState launchState = LaunchState.STOPPED;

    @Override
    public void runOpMode() throws InterruptedException {
        // ========== HARDWARE INITIALIZATION ==========
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        DcMotor intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        DcMotor launchMotor = hardwareMap.dcMotor.get("launchMotor");
        DcMotor rampMotor = hardwareMap.dcMotor.get("rampMotor");

        // Dead wheel encoders
        DcMotor forwardOdo = hardwareMap.dcMotor.get("forwardOdo");
        DcMotor rightOdo = hardwareMap.dcMotor.get("rightOdo");

        // Reset encoders
        forwardOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        forwardOdo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightOdo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Motor modes
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rampMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set brake behavior
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rampMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reverse right side motors
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize IMU
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        // Initialize AprilTag processor
        aprilTag = new AprilTagProcessor.Builder().build();

        // Initialize vision portal
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        // ========== INIT SETUP ROUTINE ==========
        telemetry.addLine("========================================");
        telemetry.addLine("INITIALIZING ROBOT");
        telemetry.addLine("========================================");
        telemetry.addData("IMU Status", "Initialized");
        telemetry.addData("Odometry", "Reset to 0");
        telemetry.addData("Camera Status", visionPortal.getCameraState());
        telemetry.addLine("========================================");
        telemetry.addLine("✓ ROBOT READY");
        telemetry.addLine("Press START to begin");
        telemetry.addLine("========================================");
        telemetry.update();

        // Initialize gamepad state tracking
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();

        // Toggle states
        boolean slowMode = false;
        boolean dpadControlEnabled = true;  // Always active per README
        boolean intakeHalfSpeed = false;
        boolean intakeForwardActive = false;
        boolean intakeReverseActive = false;

        waitForStart();
        if (isStopRequested()) return;

        // Start ramp motor immediately
        rampMotor.setPower(RAMP_MOTOR_POWER);

        while (opModeIsActive()) {
            // Store previous gamepad state
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            // Update odometry
            updateOdometry(forwardOdo, rightOdo, imu);

            // ========== ALLIANCE SELECTION ==========
            if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
                selectedAlliance = Alliance.RED;
            } else if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
                selectedAlliance = Alliance.BLUE;
            }

            // ========== AUTONOMOUS NAVIGATION ==========
            // Left Trigger - navigate to (0, -4)
            if (currentGamepad1.left_trigger > 0.5 && previousGamepad1.left_trigger <= 0.5) {
                autoNavigating = true;
                targetX = LEFT_TRIGGER_TARGET_X;
                targetY = LEFT_TRIGGER_TARGET_Y;
            }

            // Right Trigger - navigate to (0, 0)
            if (currentGamepad1.right_trigger > 0.5 && previousGamepad1.right_trigger <= 0.5) {
                autoNavigating = true;
                targetX = RIGHT_TRIGGER_TARGET_X;
                targetY = RIGHT_TRIGGER_TARGET_Y;
            }

            // Check for driver override
            boolean driverOverride = Math.abs(currentGamepad1.left_stick_x) > JOYSTICK_DEADZONE ||
                    Math.abs(currentGamepad1.left_stick_y) > JOYSTICK_DEADZONE ||
                    Math.abs(currentGamepad1.right_stick_x) > JOYSTICK_DEADZONE;

            if (driverOverride && autoNavigating) {
                autoNavigating = false;
            }

            double y = 0, x = 0, rx = 0;

            if (autoNavigating) {
                // Autonomous navigation
                double[] navOutput = navigateToTarget(targetX, targetY, imu);
                x = navOutput[0];
                y = navOutput[1];
                rx = navOutput[2];

                // Check if reached destination
                if (navOutput[3] > 0) {
                    autoNavigating = false;
                }
            } else {
                // ========== MANUAL CONTROL ==========
                y = -currentGamepad1.left_stick_y;
                x = currentGamepad1.left_stick_x;
                rx = currentGamepad1.right_stick_x;

                // Apply deadzone
                y = applyDeadzone(y);
                x = applyDeadzone(x);
                rx = applyDeadzone(rx);

                // D-pad control (always active with slow mode)
                if (currentGamepad1.dpad_up) {
                    y = SLOW_MODE_MULTIPLIER;
                } else if (currentGamepad1.dpad_down) {
                    y = -SLOW_MODE_MULTIPLIER;
                }
                if (currentGamepad1.dpad_right) {
                    x = SLOW_MODE_MULTIPLIER;
                } else if (currentGamepad1.dpad_left) {
                    x = -SLOW_MODE_MULTIPLIER;
                }

                // Toggle slow mode with Y button
                if (currentGamepad1.y && !previousGamepad1.y) {
                    slowMode = !slowMode;
                }

                // Apply slow mode to joystick input
                if (slowMode) {
                    y *= SLOW_MODE_MULTIPLIER;
                    x *= SLOW_MODE_MULTIPLIER;
                }
            }

            // ========== IMU RESET ==========
            if (currentGamepad1.back && !previousGamepad1.back) {
                imu.resetYaw();
                robotX = 0;
                robotY = 0;
                robotHeading = 0;
            }

            // ========== FIELD-CENTRIC TRANSFORMATION ==========
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Strafe correction

            // Calculate motor powers
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // ========== INTAKE CONTROL ==========
            // Toggle intake speed with START button
            if (currentGamepad1.start && !previousGamepad1.start) {
                intakeHalfSpeed = !intakeHalfSpeed;
            }

            // B button - toggle intake forward
            if (currentGamepad1.b && !previousGamepad1.b) {
                intakeForwardActive = !intakeForwardActive;
                if (intakeForwardActive) intakeReverseActive = false;
            }

            // A button - toggle intake reverse
            if (currentGamepad1.a && !previousGamepad1.a) {
                intakeReverseActive = !intakeReverseActive;
                if (intakeReverseActive) intakeForwardActive = false;
            }

            double intakePower = 0;
            String intakeStatus = "STOPPED";
            if (intakeForwardActive) {
                double speedToUse = intakeHalfSpeed ? INTAKE_HALF_POWER : INTAKE_FULL_POWER;
                intakePower = speedToUse;
                intakeStatus = "FORWARD (" + (intakeHalfSpeed ? "50%" : "100%") + ")";
            } else if (intakeReverseActive) {
                double speedToUse = intakeHalfSpeed ? INTAKE_HALF_POWER : INTAKE_FULL_POWER;
                intakePower = -speedToUse;
                intakeStatus = "REVERSE (" + (intakeHalfSpeed ? "50%" : "100%") + ")";
            }
            intakeMotor.setPower(intakePower);

            // ========== LAUNCH MOTOR WITH APRILTAG ALIGNMENT ==========
            if (currentGamepad1.x && !previousGamepad1.x) {
                if (launchState == LaunchState.STOPPED) {
                    if (selectedAlliance != Alliance.NONE) {
                        launchState = LaunchState.ALIGNING;
                    }
                } else {
                    launchState = LaunchState.STOPPED;
                }
            }

            double launchPower = 0;
            String launchStatus = "STOPPED";

            if (launchState == LaunchState.ALIGNING) {
                int targetTag = (selectedAlliance == Alliance.RED) ? RED_APRILTAG_ID : BLUE_APRILTAG_ID;
                AprilTagDetection detection = getAprilTagDetection(targetTag);

                if (detection != null) {
                    double yawError = detection.ftcPose.yaw;
                    if (Math.abs(yawError) < ANGLE_TOLERANCE_DEGREES) {
                        launchState = LaunchState.RUNNING;
                    } else {
                        // Still aligning - rotate robot
                        rx = Math.signum(yawError) * 0.3;
                        launchStatus = "ALIGNING";
                    }
                } else {
                    launchStatus = "ALIGNING (No Tag)";
                }
            }

            if (launchState == LaunchState.RUNNING) {
                launchPower = -LAUNCH_MOTOR_POWER;  // Counter-clockwise
                launchStatus = "RUNNING";
            }

            launchMotor.setPower(launchPower);

            // ========== TELEMETRY ==========
            telemetry.addLine("========== MATCH MODE ==========");
            telemetry.addLine();

            telemetry.addData("Alliance", selectedAlliance);
            telemetry.addData("Position", "X: %.1f  Y: %.1f ft", robotX, robotY);
            telemetry.addData("Heading", "%.0f°", Math.toDegrees(robotHeading));
            telemetry.addLine();

            telemetry.addData("Intake", intakeStatus);
            telemetry.addData("Launch", launchStatus);
            telemetry.addLine();

            if (autoNavigating) {
                telemetry.addData("AUTO NAV", ">>> ACTIVE <<<");
                telemetry.addData("Target", "X: %.1f  Y: %.1f ft", targetX, targetY);
            }

            if (slowMode) {
                telemetry.addData("Drive Mode", "SLOW (30%)");
            }

            telemetry.update();
        }

        // Clean up vision
        visionPortal.close();
    }

    private void updateOdometry(DcMotor forwardOdo, DcMotor rightOdo, IMU imu) {
        int forwardPos = forwardOdo.getCurrentPosition();
        int rightPos = rightOdo.getCurrentPosition();

        int deltaForward = forwardPos - lastForwardOdoPos;
        int deltaRight = rightPos - lastRightOdoPos;

        lastForwardOdoPos = forwardPos;
        lastRightOdoPos = rightPos;

        double forwardInches = deltaForward / ODO_COUNTS_PER_INCH;
        double rightInches = deltaRight / ODO_COUNTS_PER_INCH;

        double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double deltaHeading = currentHeading - robotHeading;

        // Normalize delta heading
        while (deltaHeading > Math.PI) deltaHeading -= 2 * Math.PI;
        while (deltaHeading < -Math.PI) deltaHeading += 2 * Math.PI;

        double robotDX, robotDY;

        if (Math.abs(deltaHeading) < 0.001) {
            robotDX = rightInches;
            robotDY = forwardInches;
        } else {
            // Account for wheel offset during rotation
            double forwardWheelArcX = FORWARD_ODO_X_OFFSET * (Math.cos(deltaHeading) - 1) -
                    FORWARD_ODO_Y_OFFSET * Math.sin(deltaHeading);
            double forwardWheelArcY = FORWARD_ODO_X_OFFSET * Math.sin(deltaHeading) +
                    FORWARD_ODO_Y_OFFSET * (Math.cos(deltaHeading) - 1);

            double rightWheelArcX = RIGHT_ODO_X_OFFSET * (Math.cos(deltaHeading) - 1) -
                    RIGHT_ODO_Y_OFFSET * Math.sin(deltaHeading);
            double rightWheelArcY = RIGHT_ODO_X_OFFSET * Math.sin(deltaHeading) +
                    RIGHT_ODO_Y_OFFSET * (Math.cos(deltaHeading) - 1);

            double forwardCenterTravel = forwardInches - forwardWheelArcY;
            double rightCenterTravel = rightInches - rightWheelArcX;

            robotDX = rightCenterTravel;
            robotDY = forwardCenterTravel;
        }

        // Convert to field-centric
        double avgHeading = robotHeading + deltaHeading / 2.0;
        double fieldDX = robotDX * Math.cos(avgHeading) - robotDY * Math.sin(avgHeading);
        double fieldDY = robotDX * Math.sin(avgHeading) + robotDY * Math.cos(avgHeading);

        // Update global position (convert inches to feet)
        robotX += fieldDX / 12.0;
        robotY += fieldDY / 12.0;
        robotHeading = currentHeading;
    }

    private double[] navigateToTarget(double targetX, double targetY, IMU imu) {
        double deltaX = targetX - robotX;
        double deltaY = targetY - robotY;
        double distanceToTarget = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

        double x = 0, y = 0, rx = 0;
        double completed = 0;

        // Check if at position
        if (distanceToTarget * 12 < POSITION_TOLERANCE_INCHES) {
            completed = 1;
        } else {
            // Navigate to position
            double angleToTarget = Math.atan2(deltaX, deltaY);

            // Speed control based on distance
            double speed = AUTO_MAX_SPEED;
            if (distanceToTarget < SLOWDOWN_DISTANCE_FEET) {
                double slowdownRatio = distanceToTarget / SLOWDOWN_DISTANCE_FEET;
                speed = AUTO_MIN_SPEED + (AUTO_MAX_SPEED - AUTO_MIN_SPEED) * slowdownRatio;
            }

            // Convert to robot frame
            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            x = Math.sin(angleToTarget - heading) * speed;
            y = Math.cos(angleToTarget - heading) * speed;
        }

        return new double[]{x, y, rx, completed};
    }

    private AprilTagDetection getAprilTagDetection(int targetId) {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        for (AprilTagDetection detection : detections) {
            if (detection.id == targetId) {
                return detection;
            }
        }
        return null;
    }

    private double applyDeadzone(double value) {
        if (Math.abs(value) < JOYSTICK_DEADZONE) {
            return 0;
        }
        return value;
    }
}
