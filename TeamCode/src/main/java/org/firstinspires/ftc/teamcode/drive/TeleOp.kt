package org.firstinspires.ftc.teamcode.drive

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandScheduler
import com.arcrobotics.ftclib.command.button.GamepadButton
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.commands.ftclib.drivebase.FieldCentricDrive
import org.firstinspires.ftc.teamcode.commands.ftclib.subsystems.ElevatorCommand
import org.firstinspires.ftc.teamcode.commands.ftclib.subsystems.HorizontalExtensionCommand
import org.firstinspires.ftc.teamcode.commands.ftclib.subsystems.IntakeArmExtend
import org.firstinspires.ftc.teamcode.commands.ftclib.subsystems.IntakeArmRetract
import org.firstinspires.ftc.teamcode.commands.ftclib.subsystems.PlaceSample
import org.firstinspires.ftc.teamcode.commands.ftclib.subsystems.VerticalRetract
import org.firstinspires.ftc.teamcode.commands.ftclib.subsystems.VerticalSample
import org.firstinspires.ftc.teamcode.commands.ftclib.subsystems.VerticalSpecimenPickup
import org.firstinspires.ftc.teamcode.commands.ftclib.subsystems.VerticalSpecimenPlace
import org.firstinspires.ftc.teamcode.constants.VerticalConstants
import org.firstinspires.ftc.teamcode.subsystems.ftclib.Deposit
import org.firstinspires.ftc.teamcode.subsystems.ftclib.DepositCommand
import org.firstinspires.ftc.teamcode.subsystems.ftclib.Elevator
import org.firstinspires.ftc.teamcode.subsystems.ftclib.HorizontalArm
import org.firstinspires.ftc.teamcode.subsystems.ftclib.HorizontalExtension
import org.firstinspires.ftc.teamcode.subsystems.ftclib.HorizontalWrist
import org.firstinspires.ftc.teamcode.subsystems.ftclib.Intake
import org.firstinspires.ftc.teamcode.subsystems.ftclib.VerticalArm
import org.firstinspires.ftc.teamcode.subsystems.ftclib.VerticalWrist
import org.firstinspires.ftc.teamcode.subsystems.ftclib.swerve.SwerveDrivetrain

@TeleOp(group = "test")
class TeleOp: OpMode() {
    private lateinit var hubs: List<LynxModule>
    private lateinit var drive: SwerveDrivetrain
    private lateinit var elevator: Elevator
    private lateinit var horizontalExtension: HorizontalExtension
    private lateinit var horizontalArm: HorizontalArm
    private lateinit var horizontalWrist: HorizontalWrist
    private lateinit var intake: Intake
    private lateinit var verticalArm: VerticalArm
    private lateinit var verticalWrist: VerticalWrist
    private lateinit var deposit: Deposit

    private lateinit var elapsedtime: ElapsedTime

    private lateinit var driveOp: GamepadEx
    private lateinit var toolOp: GamepadEx
    //private lateinit var voltage: PhotonLynxVoltageSensor

    override fun init() {
        CommandScheduler.getInstance().reset()
        elapsedtime = ElapsedTime()
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        // this just sets the bulk reading mode for each hub
        hubs = hardwareMap.getAll(LynxModule::class.java)
        for (hub in hubs) {
            hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        }

        //voltage = hardwareMap.getAll(PhotonLynxVoltageSensor::class.java).iterator().next()

        drive = SwerveDrivetrain(hardwareMap, 12.0)

        elevator = Elevator(hardwareMap)
        verticalArm = VerticalArm(hardwareMap)
        verticalWrist = VerticalWrist(hardwareMap)
        deposit = Deposit(hardwareMap)

        horizontalExtension = HorizontalExtension(hardwareMap)
        horizontalArm = HorizontalArm(hardwareMap)
        horizontalWrist = HorizontalWrist(hardwareMap)
        intake = Intake(hardwareMap)

        driveOp = GamepadEx(gamepad1)
        toolOp = GamepadEx(gamepad2)

        drive.defaultCommand =
            FieldCentricDrive(
                drive,
                { driveOp.leftX },
                { driveOp.leftY },
                { driveOp.rightX },
                { false },
                { true }
            )

        elevator.defaultCommand =
            ElevatorCommand(elevator) { toolOp.leftY }

        horizontalExtension.defaultCommand =
            HorizontalExtensionCommand(
                horizontalExtension
            ) { -toolOp.rightY }

        telemetry.addLine("init")
        telemetry.update()


        elapsedtime.reset()
    }

    override fun loop() {
        elapsedtime.reset()
        // clears the cache on each hub
        for (hub in hubs) {
            hub.clearBulkCache()
        }

        val cacheTime = elapsedtime.milliseconds()
        //telemetry.addData("ms cache", cacheTime)

        /*
        drive.firstOrderFieldCentricDrive(ChassisSpeeds(
            -gamepad.leftY.pow(1) *DrivebaseConstants.Measurements.MAX_VELOCITY,
            gamepad.leftX.pow(1)*DrivebaseConstants.Measurements.MAX_VELOCITY,
            gamepad.rightX.pow(1)*DrivebaseConstants.Measurements.MAX_ANGULAR_VELOCITY
        ))
         */


        val driveTime = elapsedtime.milliseconds()
        //telemetry.addData("ms drive", driveTime-cacheTime)

        //GamepadButton(toolOp, GamepadKeys.Button.A).whenPressed(IntakeRun(intake))
        //GamepadButton(toolOp, GamepadKeys.Button.B).whenPressed(IntakeStop(intake))
        GamepadButton(toolOp, GamepadKeys.Button.Y).whenPressed(
            PlaceSample(
                horizontalExtension,
                horizontalArm,
                horizontalWrist,
                intake,
                elevator,
                verticalArm,
                verticalWrist,
                deposit
            )
        )
        //GamepadButton(toolOp, GamepadKeys.Button.X).whenPressed(HorizontalExtend(horizontalExtension, horizontalArm, horizontalWrist))
        GamepadButton(toolOp, GamepadKeys.Button.X).whenPressed(
            IntakeArmRetract(
                horizontalArm,
                horizontalWrist,
                intake
            )
        )
        GamepadButton(toolOp, GamepadKeys.Button.B).whenPressed(
            IntakeArmExtend(
                horizontalArm,
                horizontalWrist,
                intake
            )
        )

        GamepadButton(toolOp, GamepadKeys.Button.LEFT_BUMPER).whenPressed(DepositCommand(deposit, VerticalConstants.DepositPositions.OUT))
        GamepadButton(toolOp, GamepadKeys.Button.RIGHT_BUMPER).whenPressed(DepositCommand(deposit, VerticalConstants.DepositPositions.IN))

        GamepadButton(toolOp, GamepadKeys.Button.DPAD_DOWN).whenPressed(
            VerticalRetract(
                elevator,
                verticalArm,
                verticalWrist,
                deposit
            )
        )
        GamepadButton(toolOp, GamepadKeys.Button.DPAD_RIGHT).whenPressed(
            VerticalSpecimenPlace(
                elevator,
                verticalArm,
                verticalWrist
            )
        )
        GamepadButton(toolOp, GamepadKeys.Button.DPAD_LEFT).whenPressed(
            VerticalSpecimenPickup(
                elevator,
                verticalArm,
                verticalWrist
            )
        )
        //GamepadButton(toolOp, GamepadKeys.Button.DPAD_UP).whenPressed(ElevatorPIDCommand(elevator, VerticalConstants.ElevatorPositions.TOP))
        GamepadButton(toolOp, GamepadKeys.Button.DPAD_UP).whenPressed(
            VerticalSample(
                elevator,
                verticalArm,
                verticalWrist
            )
        )


        telemetry.addData("horizontalExtension pos", horizontalExtension.getPosition())

        //telemetry.addData("elevator limit", elevator.atBottom())
        telemetry.addData("elevator pos", elevator.getPosition())
        //telemetry.addData("colour", intake.getGamePiece().name)

        val subsystemTime = elapsedtime.milliseconds()
        telemetry.addData("ms subsystem", subsystemTime-driveTime)

        val pose = drive.getPose()
        telemetry.addData("x", pose.x)
        telemetry.addData("y", pose.y)
        telemetry.addData("heading deg", pose.rotation.degrees)

        val poseTime = elapsedtime.milliseconds()
        telemetry.addData("ms pose", poseTime-subsystemTime)

        /*
        val vel = drive.getVelocity()
        telemetry.addData("vel x", vel.vxMetersPerSecond)
        telemetry.addData("vel y", vel.vyMetersPerSecond)
        telemetry.addData("vel heading deg", vel.omegaRadiansPerSecond)
        telemetry.addData("vel magnitude", hypot(vel.vyMetersPerSecond, vel.vxMetersPerSecond))

        //drive.test(gamepad.leftY, gamepad.rightX.49s for 30in)
        val headings = drive.getModuleHeadings()
        val voltages = drive.getModuleEncoderVoltages()
        val states = drive.getDesiredModuleStates()
        val gyro = drive.getHeading()
        //telemetry.addData("thing", drive.getDelta()[0])

        telemetry.addData("heading rad", gyro)
        telemetry.addData("lf heading", headings[0])
        //telemetry.addData("lf voltage", voltages[0])
        telemetry.addData("lf desired heading", states[0].angle.radians)

        telemetry.addData("rf heading", headings[1])
        //telemetry.addData("rf voltage", voltages[1])
        telemetry.addData("rf desired heading", states[1].angle.radians)

        telemetry.addData("lr heading", headings[2])
        //telemetry.addData("lr voltage", voltages[2])
        telemetry.addData("lr desired heading", states[2].angle.radians)

        telemetry.addData("rr heading", headings[3])
        //telemetry.addData("rr voltage", voltages[3])
        telemetry.addData("rr desired heading", states[3].angle.radians)

         */

        CommandScheduler.getInstance().run()
        //drive.periodic()


        val schedulerTime = elapsedtime.milliseconds()
        telemetry.addData("ms scheduler", schedulerTime-poseTime)

        //elevator.periodic()
        //horizontalExtension.periodic()

        //val scheduler1Time = elapsedtime.milliseconds()
        //telemetry.addData("ms scheduler 1", scheduler1Time-schedulerTime)

        telemetry.addData("ms", elapsedtime.milliseconds())

    }
}