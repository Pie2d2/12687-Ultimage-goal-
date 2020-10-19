/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Modified by michaudc 2017
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
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

@TeleOp(name="16633: Teleop Quad 12687 IMU", group="12687")
//@Disabled
public class TeleopTank_Quad_2019_12687_IMU extends OpMode {

    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    BNO055IMU.Parameters imuParameters;
    

    /* Declare OpMode members. */
    MaristBaseRobot2019_Quad_14101 robot   = new MaristBaseRobot2019_Quad_14101(); // use the class created to define a Robot's hardware
                                                         // could also use HardwarePushbotMatrix class.
    double          clawOffset  = 0.0 ;                  // Servo mid position
    final double    CLAW_SPEED  = 0.02 ;                 // sets rate to move servo
    
    double speedControl = 10;
    double difPlateSet = 0;
    
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
        
        robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.ringGrab.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.ringGrab.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        imu = hardwareMap.get(BNO055IMU.class, "imu 1");
        imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.loggingEnabled = false;
        imu.initialize(imuParameters);
        robot.loopCount = 0;
        robot.posCalc = 0;
        

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
        robot.loopCount = robot.loopCount + 1;
        
        
        // Get IMU Data and Display
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("rot about Z", angles.firstAngle);
        telemetry.addData("right Stick", gamepad1.right_stick_x);
        double imuAngle = angles.firstAngle;
        
        // Compute Vector based on Remote Input
        double theta;
        if (Math.abs(gamepad1.left_stick_x) < 0.05)
        {
            theta = Math.atan(gamepad1.left_stick_y/0.05);
        }
        else {
            if (gamepad1.left_stick_x < 0) {
                theta = Math.atan((-1*gamepad1.left_stick_y)/(gamepad1.left_stick_x));
            }
            else {
                theta = Math.atan(gamepad1.left_stick_y/gamepad1.left_stick_x);
            }
        }
        
        double mag = Math.sqrt(Math.pow(gamepad1.left_stick_y, 2) + Math.pow(gamepad1.left_stick_x, 2));
        
        theta = theta - Math.PI/2;
        
        if (gamepad1.left_stick_x > 0) {
            theta = theta * -1;
        }
        
        double newTheta = theta - ((angles.firstAngle/360.0) * 2 * Math.PI);
        
        telemetry.addData("Theta", theta);
        telemetry.addData("mag", mag);
        
        // double robotAngle = Math.toRadians(imuAngle);
        
        // double ySpeed = gamepad1.left_stick_y;
        // double xSpeed = gamepad1.left_stick_x;
        // double distance = Math.hypot(xSpeed, ySpeed);
        // double theta = Math.atan2(ySpeed, xSpeed) - robotAngle;
        // ySpeed = distance * Math.sin(theta);
        // xSpeed = distance * Math.cos(theta);
        
        // double leftFrontPower = mag * Math.cos(newTheta + (Math.PI/4)) - gamepad1.right_stick_x;
        // double rightFrontPower = mag * Math.sin(newTheta + (Math.PI/4)) + gamepad1.right_stick_x;
        // double leftRearPower = mag * Math.sin(newTheta + (Math.PI/4)) - gamepad1.right_stick_x;
        // double rightRearPower = mag * Math.cos(newTheta + (Math.PI/4)) + gamepad1.right_stick_x;

        
        // Compute power for wheels
        double leftFrontPower = mag * Math.cos(newTheta + (Math.PI/4)) - gamepad1.right_stick_x;
        double rightFrontPower = mag * Math.sin(newTheta + (Math.PI/4)) + gamepad1.right_stick_x;
        double leftRearPower = mag * Math.sin(newTheta + (Math.PI/4)) - gamepad1.right_stick_x;
        double rightRearPower = mag * Math.cos(newTheta + (Math.PI/4)) + gamepad1.right_stick_x;
        
        robot.leftFront.setPower(leftFrontPower);
        robot.leftRear.setPower(leftRearPower);
        robot.rightFront.setPower(rightFrontPower);
        robot.rightRear.setPower(rightRearPower);
        /*
        if (gamepad1.a){
            //for Standard Drive
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
        }
        
        
        */
        // Alternative Method:  Single Paddle on right  (Commented out)
        //left = gamepad1.right_stick_y + gamepad1.right_stick_x;
        //right = gamepad1.right_stick_y - gamepad1.right_stick_x;
        //robot.leftMotor.setPower(left);
        //robot.rightMotor.setPower(right);

        // Use gamepad left & right Bumpers to open and close the claw
        
        if (gamepad1.right_bumper)
            //clawOffset -= CLAW_SPEED;
            {
            robot.claw.setPosition(.15);
            }
        else if(gamepad1.left_bumper){
            robot.claw.setPosition(1);
        }
        if (gamepad1.dpad_up){
            robot.shooter.setPower(1);
        }
        if (gamepad1.dpad_down){
            robot.shooter.setPower(0);
        }
        
        if (gamepad1.left_trigger > .9){
            robot.feed.setPosition(1);
            }
        if (gamepad1.left_trigger < .9){
            robot.feed.setPosition(.5);
        }
        
        
        // Move both servos to new position.  Assume servos are mirror image of each other.
        //clawOffset = Range.clip(clawOffset, -0.8, 0.8);
        //robot.leftHand.setPosition(robot.MID_SERVO + clawOffset);
        //robot.rightHand.setPosition(robot.MID_SERVO - clawOffset);

        // Use gamepad buttons to move the arm up (Y) and down (A)
        // Commented out for now
        
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

        
        
        // Control Arm with Right and Left Triggers
        
        
        if (gamepad1.b) {
            robot.liftMotorDegSet(0.7, 20, 5);
            //robot.liftMotor.setPower(0);
        }
        if (gamepad1.x) {
            robot.liftMotorDegSet(0.7, 230, 5);
        }
        if (gamepad1.y) {
            robot.liftMotorDegSet(0.7, 150, 5);
        }
        if (gamepad1.a) {
            robot.liftMotorDegSet(0.7, 0, 5);
            
        }
        if (gamepad1.right_trigger > .9){
            robot.ringServo.setPosition(0);
        }
        if (gamepad1.right_trigger < .9){
            robot.ringServo.setPosition(1);
        }
        
        if (gamepad1.dpad_left){
            robot.ringMotorDegSet(.5, 235, 5);
        }
        if (gamepad1.dpad_right){
            robot.ringMotorDegSet(.5, 30, 5);
        }
        
        telemetry.addData("target", robot.ringGrab.getTargetPosition());
        telemetry.addData("arm current pos", robot.ringGrab.getCurrentPosition());
        
        // Limit Power to -0.4 to 0.4
        
        /*
        // Slow mode for foundation pull 
        if (gamepad1.a) {
            //goes to slow 
            speedControl = 3;
            
        }
        
        if (gamepad1.y) {
            //goes to fast
            speedControl = 1;
        }
        */
        //Test foundation servo positions
        /*
        if (gamepad1.x) {
            robot.foundationL.setPosition(1.0);
            robot.foundationR.setPosition(0.0);
        }
        if (gamepad1.b) {
            robot.foundationL.setPosition(0.0);
            robot.foundationR.setPosition(1.0);
        }
        */
        
        //Capstone preload
        //if (gamepad1.dpad_up) {     // If the dpad up control is pressed
        //robot.capstoneBase.setPosition(.4);  /* Runs sequence */
        /*
        delay(0.5);
        robot.capstoneArm.setPosition(1);
        delay(1);
        robot.capstoneBase.setPosition(0);
        delay(0.5);
        robot.capstoneArm.setPosition(0);
        delay(0.5);
        robot.capstoneBase.setPosition(1);
        */
        // }
        // if (gamepad1.dpad_down) {       // If the dpad down control is pressed
        //robot.capstoneBase.setPosition(0); /* Base moves */
        // }
        //if (gamepad1.dpad_right) {      // If the dpad right control is pressed
        //robot.capstoneArm.setPosition(0); /* Arm moves */
        //}
        //if (gamepad1.dpad_left) {       // If the dpad left control is pressed
        //robot.capstoneArm.setPosition(1); /* Arm moves other way */
        
        //}

        

        // Send telemetry message to signify robot running;
        // telemetry.addData("claw",  "Offset = %.2f", clawOffset);
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
