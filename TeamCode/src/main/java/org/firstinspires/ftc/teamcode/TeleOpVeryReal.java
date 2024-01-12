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
    private Servo arm_pivot_1   = null;
    private Servo arm_pivot_2   = null;
    private Servo grip_pivot_1	= null;
    private Servo grip_pivot_2	= null;
    private CRServo grip_spin		= null;

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
        arm_pivot_1  = hardwareMap.get(Servo.class, "ArmPivot1");
        arm_pivot_2  = hardwareMap.get(Servo.class, "ArmPivot2");
        grip_pivot_1 = hardwareMap.get(Servo.class, "GripPivot1");
        grip_pivot_2 = hardwareMap.get(Servo.class, "GripPivot2");
        grip_spin    = hardwareMap.get(CRServo.class, "GripSpin");

        slide_right.setDirection(DcMotorSimple.Direction.REVERSE);
        slide_left.setDirection(DcMotorSimple.Direction.REVERSE);

        slide_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void loop() {

        // DRIVE CODE
        double rx = gamepad1.right_stick_x; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x; // Counteract imperfect strafing
        double y = gamepad1.left_stick_y * -1;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max((Math.abs(rx) + Math.abs(x) + Math.abs(y)), 1);
        double frontLeftPower = ((rx + x + y) / denominator);
        double backLeftPower = ((rx - x + y) / denominator);
        double frontRightPower = ((rx - x - y) / denominator);
        double backRightPower = ((rx + x - y) / denominator);


        front_left.setPower(frontLeftPower);
        back_left.setPower(backLeftPower);
        front_right.setPower(frontRightPower);
        back_right.setPower(backRightPower);

        /*
         LINEAR SLIDE CODE
        */
        boolean slide_high_pos = gamepad2.dpad_up;
        boolean slide_low_pos = gamepad2.dpad_down;
        if (slide_high_pos) {
            slide_right.setTargetPosition(50);
            slide_left.setTargetPosition(50);
        }
        else if (slide_low_pos) {
            slide_right.setTargetPosition(0);
            slide_left.setTargetPosition(0);
        }

        /*
        GRIP CODE
        */
        // - harvest mode where the grip is down and spinning in
        if (gamepad2.x) {
            arm_pivot_1.setPosition(0.1);
            arm_pivot_2.setPosition(0.5);
            grip_pivot_1.setPosition(0.5);
            grip_pivot_2.setPosition(0.5);
            grip_spin.setPower(1.0);
            telemetry.addLine("Harvest Mode triggered!");
        }

        // - transport mode where the grip is up and not spinning
        else if (gamepad2.b) {
            arm_pivot_1.setPosition(0.5);
            arm_pivot_2.setPosition(0.0);
            grip_pivot_1.setPosition(0.5);
            grip_pivot_2.setPosition(0.5);
            grip_spin.setPower(0.0);
            telemetry.addLine("Transport Mode triggered!");
        }

        // - deposit mode where the grip is backwards and spinning out
        else if (gamepad2.y) {
            arm_pivot_1.setPosition(1.0);
            arm_pivot_2.setPosition(0.0);
            grip_pivot_1.setPosition(0.5);
            grip_pivot_2.setPosition(0.5);
            grip_spin.setPower(-1.0);
            telemetry.addLine("Deposit Mode triggered!");
        }
    }
}
