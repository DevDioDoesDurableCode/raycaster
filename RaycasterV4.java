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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


/**
    This is the test code for the Raycaster.
    Built by team 11654 Plus or Minos.
    This is V4 of the Raycaster code that requires two range sensor units.
 */

@TeleOp(name="Raycaster V4", group="Raycaster")
//@Disabled
public class RaycasterV4 extends LinearOpMode {

    // Declare OpMode members.
    HardwareDrive robot           = new HardwareDrive();
    Varibles var = new Varibles();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        double baseZero = 255;
        double angle = 1;
        int angleCount = 1;
        double radians = Math.toRadians(angle);
        int bookmark = 0;
        int breaking = 0;
        int pointStoreShelf = 0;
        double servoPos = 0;
        double[] locX;
        double[] locY;
        double[] runX;
        double[] runY;
        double[] checkX;
        double[] checkY;
        int[] pointStoreA;
        int[] pointStoreB;
        locX = new double[180];
        locY = new double[180];
        runX = new double[180];
        runY = new double[180];
        checkX = new double[180];
        checkY = new double[180];
        pointStoreA = new int[50];
        pointStoreB = new int[50];
        double[] locXz;
        double[] locYz;
        double[] runXz;
        double[] runYz;
        double[] checkXz;
        double[] checkYz;
        int[] pointStoreAz;
        int[] pointStoreBz;
        locXz = new double[180];
        locYz = new double[180];
        runXz = new double[180];
        runYz = new double[180];
        checkXz = new double[180];
        checkYz = new double[180];
        pointStoreAz = new int[50];
        pointStoreBz = new int[50];
        double scan = 0;
        double scanz = 0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive())
        {
            
            while(angle<180)
            {
               scan = rangeSensor.getDistance(DistanceUnit.CM);
               scanz = rangeSensor.getDistance(DistanceUnit.CM);
               locX[angleCount] = scan * Math.sin(radians);
               locY[angleCount] = scan * Math.cos(radians);
               locXz[angleCount] = scanz * Math.sin(radians);
               locYz[angleCount] = scanz * Math.cos(radians);
               angle = angle + 1;
               angleCount = angleCount + 1;
               radians = Math.toRadians(angle);
            } 
            angleCount = 1;
            while(angleCount<180)//Checking rise/run measurements
            {
               runX[angleCount] = locX[angleCount] - locX[angleCount-1];
               runY[angleCount] = locY[angleCount] - locY[angleCount-1];
               runXz[angleCount] = locXz[angleCount] - locXz[angleCount-1];
               runYz[angleCount] = locYz[angleCount] - locYz[angleCount-1];
               angleCount = angleCount + 1;
            }
            angleCount = 0;
            while(angleCount<180)
            {
               checkX[angleCount] = runX[angleCount] - runX[angleCount-1];
               checkY[angleCount] = runY[angleCount] - runY[angleCount-1];
               checkXz[angleCount] = runXz[angleCount] - runXz[angleCount-1];
               checkYz[angleCount] = runYz[angleCount] - runYz[angleCount-1];
               if((checkX[angleCount]-checkY[angleCount])<.05)
               {
                  bookmark = angleCount-1;
                  pointStoreA[pointStoreShelf] = bookmark;
                  angleCount = angleCount + 1;
                  while(breaking == 0)//Start of first layer face checking
                  {
                     checkX[angleCount] = runX[angleCount] - runX[angleCount-1];
                     checkY[angleCount] = runY[angleCount] - runY[angleCount-1];
                     if((checkX[angleCount]-checkY[angleCount])>.05)
                     {
                        breaking = 1;
                        pointStoreB[pointStoreShelf] = angleCount-1;
                        pointStoreShelf = pointStoreShelf+1;
                     }else
                     {
                        angleCount = angleCount + 1;
                     }
                     
                  }//End of first layer face checking
                  
               }else
               {
                  angleCount = angleCount + 1;
               }
               if((checkXz[angleCount]-checkYz[angleCount])<.05)
               {
                  bookmark = angleCount-1;
                  pointStoreAz[pointStoreShelf] = bookmark;
                  angleCount = angleCount + 1;
                  while(breaking == 0)//Start of second layer face checking
                  {
                     checkXz[angleCount] = runXz[angleCount] - runXz[angleCount-1];
                     checkYz[angleCount] = runYz[angleCount] - runYz[angleCount-1];
                     if((checkXz[angleCount]-checkYz[angleCount])>.05)
                     {
                        breaking = 1;
                        pointStoreBz[pointStoreShelf] = angleCount-1;
                        pointStoreShelf = pointStoreShelf+1;
                     }else
                     {
                        angleCount = angleCount + 1;
                     }
                     
                  }//End of secound layer face checking
                  
               }else
               {
                  angleCount = angleCount + 1;
               }

            }

            if(pointStoreShelf>0)
            {
                angleCount = pointStoreA[0]//It is looking for the first face it sees
                if(locX[angleCount]<0)
                {
                    robot.Left();//Driving to the left via locX
                    sleep(0.01*locX[angleCount]);
                    robot.Stop();
                    sleep(100);
                }else{
                    robot.Right();//Driving to the right via locX
                    sleep(0.01*locX[angleCount]);
                    robot.Stop();
                    sleep(100);
                }
                robot.Forward();
                sleep(0.01*locX[angleCount]);
                robot.Stop();
            }
        }
    }
}
