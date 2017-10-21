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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 */

@Autonomous(name="AutonomousMode", group="Linear Opmode")
//@Disabled
public class AutonomousMode extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;


    private Servo leftArmServo = null;
    private Servo rightArmServo = null;
    private static final double ARM_OPENING_POSITION = 0.2;
    private static final double ARM_CLOSING_POSITION = 0.8;
    //private Servo liftServo = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        //initialize motors
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        //initialize servos
        leftArmServo = hardwareMap.servo.get("left Arm Servo");
        leftArmServo.setPosition(ARM_OPENING_POSITION);
        rightArmServo = hardwareMap.servo.get("right Arm Servo");
        rightArmServo.setPosition(ARM_OPENING_POSITION);
        setArmPosition(ARM_OPENING_POSITION);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            DriveForward(1);
            sleep(4000); //drive for 4s
            turnLeft(1);
            sleep(500);
            DriveForward(1);
            sleep(4000);
            setArmPosition(ARM_CLOSING_POSITION);
            sleep(4000);


            stopDriving();

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
        // methods
    public void DriveForward(double power)
    {
        leftDrive.setPower(power);
        rightDrive.setPower(power);
    }
    public void DriveForwardTime(double power, long time) throws InterruptedException
    {
        leftDrive.setPower(power);
        rightDrive.setPower(power);
        Thread.sleep(time);
    }

    public void stopDriving()
    {
        DriveForward(0);
    }
    public void turnLeft(double power)
    {
        leftDrive.setPower(-power);
        rightDrive.setPower(power);
    }
    public void turnRight(double power)
    {
        leftDrive.setPower(power);
        rightDrive.setPower(-power);
    }
    public void setArmPosition(double position)
    {
        leftArmServo.setPosition(position);
        rightArmServo.setPosition(position);
    }
}
