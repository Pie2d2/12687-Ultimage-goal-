package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Webcam Test 1")
public class WebcamAuto extends LinearOpMode {
    
    VCam camera;
    
    @Override
    public void runOpMode() {
        //Init
        
        camera = new VCam("Webcam 1", hardwareMap);
        
        waitForStart();
        // Play
        // 93, 188, 210
        camera.startCapture();
        while (!camera.imageAvailable() && opModeIsActive()); // Wait
        VCam.VColor[][] image = camera.getLatestImage();
        VCam.VColor orange = new VCam.VColor(93,188,210);
        int orangeCount = 0;
        String finalResult = "";
        for(int i = (int) (image.length / 3d); i < (int) ((image.length / 3d) * 3); i++) {
            for (int j = 0; j < image[i].length; j++){
                VCam.VColor color = image[i][j];
                if (VCam.CVUtils.getSimilarity(color, orange) < 23) {
                    orangeCount++;
                }
            }
        }
         
        if (orangeCount > 600){
            finalResult = "Quad";
        }
        else if(orangeCount > 30 && orangeCount < 599){
            finalResult = "Single";
        }
        else if (orangeCount < 30){
            finalResult = "none";
        }
        
        
        
        telemetry.addData("# of orange", orangeCount);
        telemetry.addData("Final Result: ", finalResult);
        telemetry.update();
        
        
        while(opModeIsActive());
        
        
    }
    
}