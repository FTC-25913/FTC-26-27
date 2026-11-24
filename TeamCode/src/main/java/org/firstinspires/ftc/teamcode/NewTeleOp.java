package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="New BioBuzz TeleOp", group="Linear Opmode")
public class NewTeleOp extends LinearOpMode {

    // Chassis Motors

    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightRear = null;

    // Intake

    private DcMotor intake = null;

    private boolean previousAState = false;
    private boolean isIntakeOn = false;

    @Override
    public void runOpMode() {

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

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotor.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {

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
                intake.setPower(1.0);
            } else {
                intake.setPower(0);
            }

            // =============================================================================
            // Telemetry
            // =============================================================================

            telemetry.addData("Intake ON / OFF", isIntakeOn ? "ON" : "OFF");

        }
    }
}
