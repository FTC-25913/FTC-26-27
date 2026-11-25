package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="New BioBuzz TeleOp", group="Linear Opmode")
public class NewTeleOp extends LinearOpMode {

    // PID Stuff

    private PIDController pid_pollen;
    private PIDController pid_nectar;

    // Chassis Motors

    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightRear = null;

    // Pollen Shooter

    private DcMotorEx pollen_shooter;

    // Nectar Shooter

    private DcMotorEx nectar_shooter;

    // Intake

    private DcMotorEx intake;

    private static final double Intake_Speed = 2000;
    private boolean previousAState = false;
    private boolean isIntakeOn = false;

    @Override
    public void runOpMode() {

        // PID Stuff

        pid_pollen = new PIDController(0.002, 0.0, 0.0001);
        pid_nectar = new PIDController(0.002, 0.0, 0.001);

        double pollen_targetTicks = 3000;
        double nectar_targetTicks = 4000;

        // Important Formula for converting ticks into RPM: RPM = (ticks * 60) / (28 * Gear Reduction)

        // Pollen Shooter

        pollen_shooter = hardwareMap.get(DcMotorEx.class, "pollen_shooter");

        pollen_shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pollen_shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pollen_shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pollen_shooter.setDirection(DcMotor.Direction.FORWARD);

        // Nectar Shooter

        nectar_shooter = hardwareMap.get(DcMotorEx.class, "nectar_shooter");

        nectar_shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        nectar_shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        nectar_shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pollen_shooter.setDirection(DcMotor.Direction.FORWARD);

        // Chassis Motors

        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear   = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear  = hardwareMap.get(DcMotor.class, "rightRear");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Intake

        intake = hardwareMap.get(DcMotorEx.class, "intake");

        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setDirection(DcMotor.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        pid_pollen.reset();
        pid_nectar.reset();

        while (opModeIsActive()) {

            // =============================================================================
            // Pollen Shooter
            // =============================================================================

            double pollen_currentTicks = pollen_shooter.getCurrentPosition();
            double pollen_power = pid_pollen.update(pollen_targetTicks, pollen_currentTicks);
            pollen_shooter.setPower(pollen_power);

            // =============================================================================
            // Nectar Shooter
            // =============================================================================

            double nectar_currentTicks = nectar_shooter.getCurrentPosition();
            double nectar_power = pid_nectar.update(nectar_targetTicks, nectar_currentTicks);
            nectar_shooter.setPower(nectar_power);

            // =============================================================================
            // Chassis
            // =============================================================================

            double leftFrontPower, rightFrontPower, leftRearPower, rightRearPower;

            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            leftFrontPower = drive + strafe + turn;
            rightFrontPower = drive - strafe - turn;
            leftRearPower = drive - strafe + turn;
            rightRearPower = drive + strafe - turn;

            leftFront.setPower(leftFrontPower);
            rightFront.setPower(rightFrontPower);
            leftRear.setPower(leftRearPower);
            rightRear.setPower(rightRearPower);

            // =============================================================================
            // Intake
            // =============================================================================
            
            boolean currentAState = gamepad1.a;

            if (currentAState && !previousAState) {
                isIntakeOn = !isIntakeOn;
            }

            previousAState = currentAState;

            if (isIntakeOn) {
                intake.setVelocity(Intake_Speed);
            } else {
                intake.setVelocity(0);
            }

            // =============================================================================
            // Telemetry
            // =============================================================================

            telemetry.addData("Intake ON / OFF", isIntakeOn ? "ON" : "OFF");
            telemetry.addData("Pollen Shooter Target", pollen_targetTicks);
            telemetry.addData("Pollen Shooter Current", pollen_currentTicks);
            telemetry.addData("Pollen Shooter Error", Math.abs(pollen_targetTicks - pollen_currentTicks));
            telemetry.addData("Nectar Shooter Target", nectar_targetTicks);
            telemetry.addData("Nectar Shooter Current", nectar_currentTicks);
            telemetry.addData("Nectar Shooter Error", Math.abs(nectar_targetTicks - nectar_currentTicks));
        }
    }
}
