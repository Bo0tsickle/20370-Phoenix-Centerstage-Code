package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp
public class TeleOpVeryReal extends OpMode {

    /*
     * The mecanum drivetrain involves four separate motors that spin in
     * different directions and different speeds to produce the desired
     * movement at the desired speed.
     */

    // declare and initialize four DcMotors.
    private DcMotor front_left  = null;
    private DcMotor front_right = null;
    private DcMotor back_left   = null;
    private DcMotor back_right  = null;
    private DcMotor slide_right = null;
    private DcMotor slide_left  = null;
    private DcMotor arm         = null;
    private CRServo grip_right	= null;
    private CRServo grip_left	= null;
    private CRServo grip_spin   = null;

    @Override
    public void init() {

        // Name strings must match up with the config on the Robot Controller
        // app.
        front_left   = hardwareMap.get(DcMotor.class, "FrontLeft");
        front_right  = hardwareMap.get(DcMotor.class, "FrontRight");
        back_left	 = hardwareMap.get(DcMotor.class, "BackLeft");
        back_right   = hardwareMap.get(DcMotor.class, "BackRight");
        slide_right  = hardwareMap.get(DcMotor.class, "SlideRight");
        slide_left   = hardwareMap.get(DcMotor.class, "SlideLeft");
        arm          = hardwareMap.get(DcMotor.class, "Arm");
        grip_right   = hardwareMap.get(CRServo.class, "GripRight");
        grip_left    = hardwareMap.get(CRServo.class, "GripLeft");
        grip_spin	 = hardwareMap.get(CRServo.class, "GripSpin");

        front_left.setDirection(DcMotorSimple.Direction.REVERSE);

        grip_spin.setDirection(DcMotorSimple.Direction.FORWARD);

        slide_right.setDirection(DcMotorSimple.Direction.REVERSE);
        slide_left.setDirection(DcMotorSimple.Direction.REVERSE);

        slide_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void loop() {
        // DRIVE CODE
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        front_left.setPower(frontLeftPower);
        back_left.setPower(backLeftPower);
        front_right.setPower(frontRightPower);
        back_right.setPower(backRightPower);

		/*
		 LINEAR SLIDE CODE
		*/
        slide_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //if ((slide_right.getCurrentPosition() > 0) || !(slide_right.getCurrentPosition() < 100)) {
        slide_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        boolean slide_up = gamepad2.dpad_up;
        boolean slide_down = gamepad2.dpad_down;
        if (slide_up) {
            slide_right.setPower(0.5);
            slide_left.setPower(0.5);
        } else if (slide_down) {
            slide_right.setPower(-0.5);
            slide_left.setPower(-0.5);
        }
        else {
            slide_right.setPower(0);
            slide_left.setPower(0);
        }
        //}

		/*
		GRIP CODE
		*/
        double arm_power = gamepad2.left_stick_y / 2.0;
        arm.setPower(arm_power);
        telemetry.addData("arm_power", arm_power);

        if (gamepad2.right_trigger > 0.25) {
            grip_spin.setPower(1.0);
        }
        else if (gamepad2.left_trigger > 0.25) {
            grip_spin.setPower(-1.0);
        }
        else {
            grip_spin.setPower(0.0);
        }

        double grip_power = gamepad2.right_stick_y / 2.0;

        grip_left.setPower(grip_power);
        grip_right.setPower(grip_power * -1);

        telemetry.addData("grip_left power", grip_power);
        telemetry.addData("grip_right position", grip_power * -1);
        telemetry.addData("grip_spin position", grip_spin.getPower());
        telemetry.update();
    }
}
