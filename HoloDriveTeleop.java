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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
 
/**
 * This opMode illustrates use of Field Oriented Drive by way of the Modern
 * Robotics Gyro sensor.  Could easly be adapted to use the Rev Expansion Hub 
 * IMU or the NavX Micro as the data source.
 */

@TeleOp(name="HoloDrive Teleop with Field Orented Drive: Iterative OpMode", group="Iterative Opmode")

public class HoloDriveTeleop extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor rearLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor rearRightDrive = null;

    static final double     DRIVE_POWER = 0.2;
    static final double     TURN_POWER    = 0.3;
    static final double     GYRO_TOLERANCE  = 1;
    
    // Gyro-related declarations.
    // This us using the Modern Robotics Gyro. Can also use the NavX sensor. See
    // NavXTest.java for example of that. Note NavX gyro values are 1 to 180 for
    // CW and -1 to -180 for CCW which would change implementation below.
    
    /** In this sample, for illustration purposes we use two interfaces on the one gyro object.
     * That's likely atypical: you'll probably use one or the other in any given situation,
     * depending on what you're trying to do. {@link IntegratingGyroscope} (and it's base interface,
     * {@link Gyroscope}) are common interfaces supported by possibly several different gyro
     * implementations. {@link ModernRoboticsI2cGyro}, by contrast, provides functionality that
     * is unique to the Modern Robotics gyro sensor.
     */
    IntegratingGyroscope gyro;
    ModernRoboticsI2cGyro modernRoboticsI2cGyro;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "yellow_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "green_drive");
        rearLeftDrive  = hardwareMap.get(DcMotor.class, "white_drive");
        rearRightDrive = hardwareMap.get(DcMotor.class, "blue_drive");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        rearLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        rearRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // For Gyro
        boolean lastResetState = false;
        boolean curResetState  = false;
       
        // Get a reference to a Modern Robotics gyro object. We use several interfaces
        // on this object to illustrate which interfaces support which functionality.
        modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro = (IntegratingGyroscope)modernRoboticsI2cGyro;
        // If you're only interested int the IntegratingGyroscope interface, the following will suffice.
        // gyro = hardwareMap.get(IntegratingGyroscope.class, "gyro");
        // A similar approach will work for the Gyroscope interface, if that's all you need.
    
        // Start calibrating the gyro. This takes a few seconds and is worth performing
        // during the initialization phase at the start of each opMode.
        telemetry.log().add("Gyro Calibrating. Do Not Move!");
        modernRoboticsI2cGyro.calibrate();
    
        // Wait until the gyro calibration is complete
        //timer.reset();
        runtime.reset();
        //while (!isStopRequested() && modernRoboticsI2cGyro.isCalibrating())  {
        while (modernRoboticsI2cGyro.isCalibrating())  {
          //telemetry.addData("calibrating", "%s", Math.round(timer.seconds())%2==0 ? "|.." : "..|");
          //telemetry.update();
          //sleep(50);
        }
    
        telemetry.log().clear(); telemetry.log().add("Gyro Calibrated. Press Start.");
        telemetry.clear(); telemetry.update(); 
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
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
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double frontLeftPower;
        double rearLeftPower;
        double frontRightPower;
        double rearRightPower;

        // Holonomic Mode uses left stick to go forward (y), and strafe (x)
        // and left and right on the right stick to rotate (y).
        double drive = -gamepad1.left_stick_y; // Invert stick y axis.
        double strafe  =  gamepad1.left_stick_x;
        double rotate  =  gamepad1.right_stick_x;
        
        /* START FIELD ORIENTED DRIVE CALCULATIONS. */
        
        // Gyro heading
        int heading = modernRoboticsI2cGyro.getHeading();
        
        // ADJUST JOYSTICK X/Y INPUTS BY YAW ANGLE
        // To understand what's going on here: cartesian coordinates are
        // convereted to polar coordinates.
        // See http://thinktank.wpi.edu/article/136 
        //  https://www.mathsisfun.com/polar-cartesian-coordinates.html
        //  https://youtu.be/aSdaT62ndYE 
        double gyro_degrees = -heading; // Negate the heading, so we get the proper correction out of caclculation.
        double gyro_radians = gyro_degrees * Math.PI/180; 
        double temp = drive * Math.cos(gyro_radians) + strafe * Math.sin(gyro_radians);
        // The adjusted x
        strafe = -drive * Math.sin(gyro_radians) + strafe * Math.cos(gyro_radians);
        // The adjusted y
        drive = temp;
        // AT THIS POINT, JOYSTICK X/Y (STRAFE/DRIVE) VECTORS HAVE BEEN
        // ROTATED BY THE GYRO ANGLE, AND CAN BE SENT TO DRIVE SYSTEM
        
        /* END FIELD ORIENTED DRIVE CALCULATIONS. */

        // Use basic math to combine motions based on holonomic motor placement.
        // y + x + rotation
        frontLeftPower   = Range.clip( drive + strafe + rotate, -1.0, 1.0);
        rearLeftPower    = Range.clip( drive - strafe + rotate, -1.0, 1.0);
        frontRightPower  = Range.clip( drive - strafe - rotate, -1.0, 1.0);
        rearRightPower   = Range.clip( drive + strafe - rotate, -1.0, 1.0);

        // Send calculated power to drive motors. 
        frontLeftDrive.setPower(frontLeftPower / 3);
        frontRightDrive.setPower(frontRightPower / 3);
        rearLeftDrive.setPower(rearLeftPower / 3);
        rearRightDrive.setPower(rearRightPower / 3);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Gyro", "heading: " + heading);
        telemetry.addData("Motors", "fl (%.2f), fr (%.2f), rl (%.2f), rr (%.2f)", frontLeftPower, frontRightPower, rearLeftPower, rearRightPower);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
