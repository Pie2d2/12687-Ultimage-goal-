/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Modified by michaudc 2017
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="16633: Teleop Quad 16633", group="16633")
//@Disabled
public class TeleopTank_Quad_2019_16633_State extends OpMode {

    BNO055IMU imu;

    /* Declare OpMode members. */
    MaristBaseRobot2019_Quad_14101 robot   = new MaristBaseRobot2019_Quad_14101(); // use the class created to define a Robot's hardware
                                                         // could also use HardwarePushbotMatrix class.
    double          clawOffset  = 0.0 ;                  // Servo mid position
    final double    CLAW_SPEED  = 0.02 ;                 // sets rate to move servo
    
    double speedControl = 1.0;
    
    private ElapsedTime runtime = new ElapsedTime();


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Robot Ready");    //

        // Set to Run without Encoder for Tele Operated
        robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double lx;
        double rx;
        double ly;
        double ry;

        // Run wheels in tank mode Left Y Axis Controls Left Motor, Right Y Axis Controls Right Motor
        ly = gamepad1.left_stick_y  / speedControl;
        ry = gamepad1.right_stick_y / speedControl;
        lx = gamepad1.left_stick_x  / speedControl;
        rx = gamepad1.right_stick_x / speedControl;
        
        robot.leftFront.setPower(-lx+ly-rx);
        robot.leftRear.setPower(lx+ly-rx);
        robot.rightFront.setPower(lx+ly+rx);
        robot.rightRear.setPower(-lx+ly+rx);

        // Alternative Method:  Single Paddle on right  (Commented out)
        //left = gamepad1.right_stick_y + gamepad1.right_stick_x;
        //right = gamepad1.right_stick_y - gamepad1.right_stick_x;
        //robot.leftMotor.setPower(left);
        //robot.rightMotor.setPower(right);

        // Use gamepad left & right Bumpers to open and close the claw
        if (gamepad1.left_bumper)
            //clawOffset += CLAW_SPEED;
            {
                robot.claw.setPosition(0.15);
            }
        else if (gamepad1.right_bumper)
            //clawOffset -= CLAW_SPEED;
            {
            robot.claw.setPosition(1);
            }
        // Move both servos to new position.  Assume servos are mirror image of each other.
        //clawOffset = Range.clip(clawOffset, -0.8, 0.8);
        //robot.leftHand.setPosition(robot.MID_SERVO + clawOffset);
        //robot.rightHand.setPosition(robot.MID_SERVO - clawOffset);

        // Use gamepad buttons to move the arm up (Y) and down (A)
        // Commented out for now
        
        if (gamepad1.b) {
            robot.liftMotorDegSet(0.5, 20, 5);
            //robot.liftMotor.setPower(0);
        }
        if (gamepad1.x) {
            robot.liftMotorDegSet(0.5, 200, 5);
        }
        if (gamepad1.y) {
            robot.liftMotorDegSet(0.5, 120, 5);
        }
        if (gamepad1.a) {
            robot.liftMotorDegSet(0.5, 0, 5);
            
        }
        
        /*
        if (gamepad1.y)
        {
            robot.rightArm.setPower(-.4);
            robot.leftArm.setPower(.4);
        }
        else if (gamepad1.a)
        {
            robot.rightArm.setPower(.4);
            robot.leftArm.setPower(-.4);
        }
        else
        {
            robot.rightArm.setPower(0);
            robot.leftArm.setPower(0);
        }
        

        */
        /*
        // Control Arm with Right and Left Triggers
        double armMotorPower = gamepad1.right_trigger - gamepad1.left_trigger;
        // Limit Power to -0.4 to 0.4
        if (armMotorPower > 0.4) {
            armMotorPower = 0.4;
        }

        if (armMotorPower < -0.4) {
            armMotorPower = -0.4;
        }
        */
        
        // Slow mode for foundation pull 
        if (gamepad1.dpad_down) {
            //goes to slow 
            speedControl = 3;
            
        }
        
        if (gamepad1.dpad_up) {
            //goes to fast
            speedControl = 1;
        }
        /*
        //Test foundation servo positions
        if (gamepad1.x) {
            robot.foundationL.setPosition(1.0);
            robot.foundationR.setPosition(0.0);
        }
        if (gamepad1.b) {
            robot.foundationL.setPosition(0.0);
            robot.foundationR.setPosition(1.0);
        }

        robot.liftMotor.setPower(armMotorPower);
        */
        /*
        //Capstone preload
        if (gamepad1.dpad_up) {     // If the dpad up control is pressed
        robot.capstoneBase.setPosition(.4);   Runs sequence 
        delay(0.5);
        robot.capstoneArm.setPosition(1);
        delay(1);
        robot.capstoneBase.setPosition(0);
        delay(0.5);
        robot.capstoneArm.setPosition(0);
        delay(0.5);
        robot.capstoneBase.setPosition(1);
        
        }
        */
        /*
        if (gamepad1.dpad_down) {       // If the dpad down control is pressed
        robot.capstoneBase.setPosition(0);  Base moves 
        }
        if (gamepad1.dpad_right) {      // If the dpad right control is pressed
        robot.capstoneArm.setPosition(0);  Arm moves 
        }
        if (gamepad1.dpad_left) {       // If the dpad left control is pressed
        robot.capstoneArm.setPosition(1);  Arm moves other way 
        
        }
        */

        

        // Send telemetry message to signify robot running;
        telemetry.addData("claw",  "Offset = %.2f", clawOffset);
        //telemetry.addData("left",  "%.2f", left);
        //telemetry.addData("right", "%.2f", right);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
    
        // Sample Delay Code
    public void delay(double t) { // Imitates the Arduino delay function
        runtime.reset();
        while ((runtime.seconds() < t)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }

}
