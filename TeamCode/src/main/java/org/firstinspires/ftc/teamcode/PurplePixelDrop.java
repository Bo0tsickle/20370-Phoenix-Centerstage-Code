/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;


/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous

public class PurplePixelDrop extends LinearOpMode {
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor front_right = null;
    private DcMotor front_left = null;
    private DcMotor back_right = null;
    private DcMotor back_left = null;

    /*
    static final double HD_COUNTS_PER_REV = 28;
    static final double DRIVE_GEAR_REDUCTION = 20.15293;
    static final double WHEEL_CIRCUMFERENCE_MM = 96 * Math.PI;
    static final double DRIVE_COUNTS_PER_MM = (HD_COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE_MM;
    static final double DRIVE_COUNTS_PER_IN = DRIVE_COUNTS_PER_MM * 25.4;
    */

    void drive(int front_left_sign, int front_right_sign, int back_left_sign, int back_right_sign, int wait_time_ms) {
        front_right.setPower(0.75 * front_right_sign);
        front_left.setPower(0.75 * front_left_sign);
        back_right.setPower(0.75 * back_right_sign);
        back_left.setPower(0.75 * back_left_sign);

        sleep(wait_time_ms);

        front_right.setPower(0);
        front_left.setPower(0);
        back_right.setPower(0);
        back_left.setPower(0);
    }

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        front_right = hardwareMap.get(DcMotor.class, "FrontRight");
        front_left = hardwareMap.get(DcMotor.class, "FrontLeft");
        back_right = hardwareMap.get(DcMotor.class, "BackRight");
        back_left = hardwareMap.get(DcMotor.class, "BackLeft");
        DistanceSensor sensor_left = hardwareMap.get(DistanceSensor.class, "DistanceSensorLeft");
        DistanceSensor sensor_right = hardwareMap.get(DistanceSensor.class, "DistanceSensorRight");

        CRServo grip_spin = hardwareMap.get(CRServo.class, "GripSpin");

        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        front_right.setDirection(DcMotorSimple.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // sensing
        boolean left_detected = false;
        boolean middle_detected = false;
        boolean right_detected = false;
        if (sensor_left.getDistance(DistanceUnit.INCH) < 48) {
            left_detected = true;
        }
        else if (sensor_right.getDistance(DistanceUnit.INCH) < 48) {
            right_detected = true;
        }
        else {
            middle_detected = true;
        }

        telemetry.addData("LeftDetect", left_detected);
        telemetry.addData("LeftDistance", sensor_left.getDistance(DistanceUnit.INCH));
        telemetry.addData("MiddleDetect", middle_detected);
        telemetry.addData("MiddleDistance", "Distance Sensor not attached");
        telemetry.addData("RightDetect", right_detected);
        telemetry.addData("RightDistance", sensor_right.getDistance(DistanceUnit.INCH));

        drive(1, 1, 1, 1, 2000);
        if (left_detected) {
            drive(-1, 1, -1, 1, 1000);
            grip_spin.setPower(-1.0);
        }
        else if (middle_detected) { // works
            grip_spin.setPower(-1.0);
        }
        else if (right_detected) {
            drive(1, -1, 1, -1, 1000);
            grip_spin.setPower(-1.0);
        }


    }
}

