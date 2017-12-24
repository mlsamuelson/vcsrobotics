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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Autonomous(name="HoloDrive Auto: Linear OpMode", group="Linear Opmode")

public class HoloDriveLinearAuto extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor rearLeftDrive = null;
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
    
    // A timer helps provide feedback while calibration is taking place
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

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
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        rearLeftDrive.setDirection(DcMotor.Direction.REVERSE);
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
        timer.reset();
        while (!isStopRequested() && modernRoboticsI2cGyro.isCalibrating())  {
          telemetry.addData("calibrating", "%s", Math.round(timer.seconds())%2==0 ? "|.." : "..|");
          telemetry.update();
          sleep(50);
        }
    
        telemetry.log().clear(); telemetry.log().add("Gyro Calibrated. Press Start.");
        telemetry.clear(); telemetry.update(); 

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        telemetry.log().clear();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double frontLeftPower = 0.0;
            double frontRightPower = 0.0;
            double rearLeftPower = 0.0;
            double rearRightPower = 0.0;

            // GYRO readings
            // The raw() methods report the angular rate of change about each of the
            // three axes directly as reported by the underlying sensor IC.
            int rawX = modernRoboticsI2cGyro.rawX();
            int rawY = modernRoboticsI2cGyro.rawY();
            int rawZ = modernRoboticsI2cGyro.rawZ();
            int heading = modernRoboticsI2cGyro.getHeading();
            int integratedZ = modernRoboticsI2cGyro.getIntegratedZValue();
    
            // Read dimensionalized data from the gyro. This gyro can report angular velocities
            // about all three axes. Additionally, it internally integrates the Z axis to
            // be able to report an absolute angular Z orientation.
            AngularVelocity rates = gyro.getAngularVelocity(AngleUnit.DEGREES);
            float zAngle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    
            // Read administrative information from the gyro
            int zAxisOffset = modernRoboticsI2cGyro.getZAxisOffset();
            int zAxisScalingCoefficient = modernRoboticsI2cGyro.getZAxisScalingCoefficient();

            // ACTIONS
/*            
            // Backward
            frontLeftPower   = -DRIVE_POWER;
            frontRightPower  = -DRIVE_POWER;
            rearLeftPower    = -DRIVE_POWER;
            rearRightPower   = -DRIVE_POWER;
            
            // Send calculated power to wheels
            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            rearLeftDrive.setPower(rearLeftPower);
            rearRightDrive.setPower(rearRightPower);
            sleep(1000);

            // Forward 
            frontLeftPower   = DRIVE_POWER;
            frontRightPower  = DRIVE_POWER;
            rearLeftPower    = DRIVE_POWER;
            rearRightPower   = DRIVE_POWER;
            
            // Send calculated power to wheels
            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            rearLeftDrive.setPower(rearLeftPower);
            rearRightDrive.setPower(rearRightPower);
            sleep(1000);

            // Right
            frontLeftPower   = DRIVE_POWER;
            frontRightPower  = -DRIVE_POWER;
            rearLeftPower    = -DRIVE_POWER;
            rearRightPower   = DRIVE_POWER;
            
            // Send calculated power to wheels
            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            rearLeftDrive.setPower(rearLeftPower);
            rearRightDrive.setPower(rearRightPower);
            sleep(1000);

            // Left
            frontLeftPower   = -DRIVE_POWER; 
            frontRightPower  = DRIVE_POWER;
            rearLeftPower    = DRIVE_POWER;
            rearRightPower   = -DRIVE_POWER;
            
            // Send calculated power to wheels
            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            rearLeftDrive.setPower(rearLeftPower);
            rearRightDrive.setPower(rearRightPower);
            sleep(1000);
*/


if (heading <= 180 && heading > 0 + GYRO_TOLERANCE) {
            // Rotate CW
            frontLeftPower   = TURN_POWER/3;
            frontRightPower  = -TURN_POWER/3;
            rearLeftPower    = TURN_POWER/3;
            rearRightPower   = -TURN_POWER/3;
}            
            // Send calculated power to wheels
//            frontLeftDrive.setPower(frontLeftPower);
//            frontRightDrive.setPower(frontRightPower);
//            rearLeftDrive.setPower(rearLeftPower);
//            rearRightDrive.setPower(rearRightPower);
//            sleep(1000);

if (heading > 180 && heading <= 360 - GYRO_TOLERANCE) {
            // Rotate CCW
            frontLeftPower   = -TURN_POWER/3;
            frontRightPower  = TURN_POWER/3;
            rearLeftPower    = -TURN_POWER/3;
            rearRightPower   = TURN_POWER/3;
}

            // Send calculated power to wheels
            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            rearLeftDrive.setPower(rearLeftPower);
            rearRightDrive.setPower(rearRightPower);
//            sleep(1000);

            // Show the elapsed game time and wheel power.
            //telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            //telemetry.update();
            
// gyro telemetry           
      
            telemetry.addLine()
              .addData("dx", formatRate(rates.xRotationRate))
              .addData("dy", formatRate(rates.yRotationRate))
              .addData("dz", "%s deg/s", formatRate(rates.zRotationRate));
            telemetry.addData("angle", "%s deg", formatFloat(zAngle));
        telemetry.addData("heading", "%3d deg", heading);
            telemetry.addData("integrated Z", "%3d", integratedZ);
            telemetry.addLine()
              .addData("rawX", formatRaw(rawX))
              .addData("rawY", formatRaw(rawY))
              .addData("rawZ", formatRaw(rawZ));
            telemetry.addLine().addData("z offset", zAxisOffset).addData("z coeff", zAxisScalingCoefficient);
            telemetry.update(); 
        }
    }
    
    String formatRaw(int rawValue) {
      return String.format("%d", rawValue);
    }

    String formatRate(float rate) {
      return String.format("%.3f", rate);
    }

    String formatFloat(float rate) {
      return String.format("%.3f", rate);
    }
    
    /*
    Void driveGyro() {
    }
    
    Void turnGyro() {
    }
    */
    
}
