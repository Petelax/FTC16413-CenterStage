package org.firstinspires.ftc.teamcode.drive.opmode

import android.graphics.Bitmap
import android.graphics.Canvas
import android.util.Size
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandOpMode
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.hardware.VoltageSensor
import org.firstinspires.ftc.robotcore.external.function.Consumer
import org.firstinspires.ftc.robotcore.external.function.Continuation
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibrationIdentity
import org.firstinspires.ftc.teamcode.subsystems.Arm
import org.firstinspires.ftc.teamcode.subsystems.Claw
import org.firstinspires.ftc.teamcode.subsystems.Elevator
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrivebase
import org.firstinspires.ftc.teamcode.subsystems.Wrist
import org.firstinspires.ftc.teamcode.vision.PropPipeline
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.VisionProcessor
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.Point
import org.opencv.core.Rect
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc
import java.util.Locale


@Autonomous
class VisionTest: CommandOpMode() {
    private lateinit var drivebase: SwerveDrivebase
    private lateinit var intake: Intake
    private lateinit var elevator: Elevator
    private lateinit var wrist: Wrist
    private lateinit var claw: Claw
    private lateinit var arm: Arm
    private lateinit var voltage: VoltageSensor
    private lateinit var propPipeline: PropPipeline
    private lateinit var portal: VisionPortal
    //private lateinit var redPropThreshold: RedPropThreshold
    override fun initialize() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        drivebase = SwerveDrivebase(hardwareMap)
        elevator = Elevator(hardwareMap, Elevator.Mode.RAW)
        wrist = Wrist(hardwareMap)
        claw = Claw(hardwareMap)
        arm = Arm(hardwareMap)
        intake = Intake(hardwareMap)
        voltage = hardwareMap.get(VoltageSensor::class.java, "Control Hub")

        //val camera = hardwareMap.get<WebcamName>(WebcamName::class.java, "Webcam 1")

        propPipeline = PropPipeline()
        portal = VisionPortal.Builder()
            .setCamera(hardwareMap.get(WebcamName::class.java, "Webcam 1"))
            .setCameraResolution(Size(1920, 1080))
            .setCamera(BuiltinCameraDirection.BACK)
            .addProcessor(propPipeline)
            .enableLiveView(true)
            .setAutoStopLiveView(true)
            .build()

        FtcDashboard.getInstance().startCameraStream(propPipeline, 20.0)
        //portal.saveNextFrameRaw(String.format(Locale.US, "CameraFrameCapture.png"))
        /*
        propPipeline = PropPipeline()
        portal = VisionPortal.Builder()
            .setCamera(camera)
            .setCameraResolution(Size(1920, 1080))
            .setCamera(BuiltinCameraDirection.BACK)
            .addProcessor(propPipeline) //                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
            .enableLiveView(true)
            //.setAutoStopLiveView(true)
            .build()

         */

    }

    override fun run() {
        //val location = redPropThreshold.propPosition
        val location = propPipeline.location
        telemetry.addData("location", location)
        telemetry.update()
        super.run()
    }
}


/*
@Config
class RedPropThreshold : VisionProcessor, CameraStreamSource {
    var testMat = Mat()
    var highMat = Mat()
    var lowMat = Mat()
    var finalMat = Mat()
    var redThreshold = 0.5

    //Returns postion of the prop in a String
    var propPosition = "left" //Set a default value in case vision does not work

    override fun init(width: Int, height: Int, calibration: CameraCalibration) {}
    override fun processFrame(frame: Mat, captureTimeNanos: Long): Any? {
        Imgproc.cvtColor(frame, testMat, Imgproc.COLOR_RGB2HSV)
        val lowHSVRedLower = Scalar(0.0, 100.0, 20.0) //Beginning of Color Wheel
        val lowHSVRedUpper = Scalar(10.0, 255.0, 255.0)
        val redHSVRedLower = Scalar(160.0, 100.0, 20.0) //Wraps around Color Wheel
        val highHSVRedUpper = Scalar(180.0, 255.0, 255.0)
        Core.inRange(testMat, lowHSVRedLower, lowHSVRedUpper, lowMat)
        Core.inRange(testMat, redHSVRedLower, highHSVRedUpper, highMat)
        testMat.release()
        Core.bitwise_or(lowMat, highMat, finalMat)
        lowMat.release()
        highMat.release()
        val leftBox = Core.sumElems(finalMat.submat(LEFT_RECTANGLE)).`val`[0]
        Imgproc.rectangle(finalMat, LEFT_RECTANGLE, Scalar(0.0, 255.0, 0.0), 5)
        val rightBox = Core.sumElems(finalMat.submat(CENTRE_RECTANGLE)).`val`[0]
        Imgproc.rectangle(finalMat, CENTRE_RECTANGLE, Scalar(0.0, 255.0, 0.0), 5)
        val averagedLeftBox: Double = leftBox / LEFT_RECTANGLE.area() / 255
        val averagedRightBox: Double = rightBox / CENTRE_RECTANGLE.area() / 255 //Makes value [0,1]
        if (averagedLeftBox > redThreshold) {        //Must Tune Red Threshold
            propPosition = "left"
        } else if (averagedRightBox > redThreshold) {
            propPosition = "center"
        } else {
            propPosition = "right"
        }
        finalMat.copyTo(frame) /*This line should only be added in when you want to see your custom pipeline
                                  on the driver station stream, do not use this permanently in your code as
                                  you use the "frame" mat for all of your pipelines, such as April Tags*/
        return null //You do not return the original mat anymore, instead return null
    }

    override fun onDrawFrame(
        canvas: Canvas,
        onscreenWidth: Int,
        onscreenHeight: Int,
        scaleBmpPxToCanvasPx: Float,
        scaleCanvasDensity: Float,
        userContext: Any
    ) {
    }

    companion object {
        val LEFT_RECTANGLE: Rect = Rect(
            Point(400.0, 520.0),
            Point(666.0, 666.0)
        )
        val CENTRE_RECTANGLE: Rect = Rect(
            Point(1050.0, 490.0),
            Point(1215.0, 650.0)
        )
    }

    override fun getFrameBitmap(continuation: Continuation<out Consumer<Bitmap>>?) {

    }
}

 */