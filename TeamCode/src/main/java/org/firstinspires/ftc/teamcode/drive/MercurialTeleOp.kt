package org.firstinspires.ftc.teamcode.drive

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.frozenmilk.dairy.pasteurized.SDKGamepad
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.bindings.BoundGamepad
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.commands.groups.Parallel
import dev.frozenmilk.mercurial.commands.groups.Race
import dev.frozenmilk.mercurial.commands.groups.Sequential
import dev.frozenmilk.mercurial.commands.util.IfElse
import dev.frozenmilk.mercurial.commands.util.Wait
import org.firstinspires.ftc.teamcode.commands.Timeout
import org.firstinspires.ftc.teamcode.commands.ftclib.subsystems.HorizontalRetract
import org.firstinspires.ftc.teamcode.constants.DrivetrainPIDCoefficients
import org.firstinspires.ftc.teamcode.constants.HorizontalConstants
import org.firstinspires.ftc.teamcode.constants.VerticalConstants
import org.firstinspires.ftc.teamcode.subsystems.Deposit
import org.firstinspires.ftc.teamcode.subsystems.Elevator
import org.firstinspires.ftc.teamcode.subsystems.HorizontalArm
import org.firstinspires.ftc.teamcode.subsystems.HorizontalExtension
import org.firstinspires.ftc.teamcode.subsystems.HorizontalWrist
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.VerticalArm
import org.firstinspires.ftc.teamcode.subsystems.VerticalWrist
import org.firstinspires.ftc.teamcode.subsystems.swerve.SwerveDrivetrain
import org.firstinspires.ftc.teamcode.utils.BulkReads
import org.firstinspires.ftc.teamcode.utils.Globals
import org.firstinspires.ftc.teamcode.utils.LoopTimes
import org.firstinspires.ftc.teamcode.utils.Telemetry

@Mercurial.Attach
@BulkReads.Attach
@LoopTimes.Attach
@Telemetry.Attach

@SwerveDrivetrain.Attach

@HorizontalExtension.Attach
@HorizontalArm.Attach
@HorizontalWrist.Attach
@Intake.Attach

@Elevator.Attach
@VerticalArm.Attach
@VerticalWrist.Attach
@Deposit.Attach

@TeleOp
class MercurialTeleOp : OpMode() {
    override fun init() {
        val mechanismGamepad = BoundGamepad(SDKGamepad(gamepad2))
        val driveGamepad = BoundGamepad(SDKGamepad(gamepad1))

        //mechanismGamepad.dpadUp.onTrue(Parallel(VerticalArm.sample(), VerticalWrist.sample(), Deposit.close()))
        //mechanismGamepad.dpadDown.onTrue(Parallel(VerticalArm.intake(), VerticalWrist.intake(), Deposit.open()))

        val verticalSample = Parallel(
            Telemetry.putCommand("help", "me"),
            Elevator.pid(VerticalConstants.ElevatorPositions.TOP),
            Elevator.waitUntilSetPoint(VerticalConstants.ElevatorPositions.TOP),
            Sequential(
                Elevator.waitUntilAboveArm(),
                Parallel(
                    VerticalArm.sample(),
                    VerticalWrist.sample(),
                ),
            ),
        )

        val verticalSpecimenPlace = Parallel(
            Elevator.waitUntilSetPoint(VerticalConstants.ElevatorPositions.SPECIMEN_PLACE),
            Elevator.pid(VerticalConstants.ElevatorPositions.SPECIMEN_PLACE+2.0),
            Sequential(
                Elevator.waitUntilAboveArm(),
                Parallel(
                    VerticalArm.specimen(),
                    VerticalWrist.specimenPlace(),
                ),
            ),
        )

        val verticalSpecimenPickup = IfElse(
            {VerticalArm.isArmIntake()},
            Sequential(
                Parallel(
                    Elevator.waitUntilAboveArm(),
                    Elevator.pid(VerticalConstants.ElevatorPositions.ARM_TARGET+0.5)
                ),
                Parallel(
                    Wait(VerticalConstants.VerticalArmConstants.intakeToSpecimen),
                    VerticalArm.specimen(),
                    VerticalWrist.specimenPickup(),
                ),
                Parallel(
                    Elevator.waitUntilSetPoint(VerticalConstants.ElevatorPositions.BOTTOM),
                    Elevator.pid(VerticalConstants.ElevatorPositions.BOTTOM),
                )
            ),
            Parallel(
                Elevator.waitUntilSetPoint(VerticalConstants.ElevatorPositions.BOTTOM),
                Elevator.pid(VerticalConstants.ElevatorPositions.BOTTOM),
                VerticalArm.specimen(),
                VerticalWrist.specimenPickup(),
            ),
        )

        val verticalRetract = IfElse( {!VerticalArm.isArmIntake()},
            Sequential(
                IfElse( {Elevator.getPosition() < VerticalConstants.ElevatorPositions.ARM},
                    Sequential(
                        Elevator.pid(VerticalConstants.ElevatorPositions.ARM_TARGET+0.5),
                        Elevator.waitUntilAboveArm(),
                    ),
                    Sequential()
                ),
                IfElse( {VerticalArm.isArmSpecimen()},
                    Sequential(
                        Parallel(
                            Wait(VerticalConstants.VerticalArmConstants.specimenToIntake),
                            VerticalArm.intake(),
                            VerticalWrist.intake(),
                            Deposit.open()
                        ),
                        Parallel(
                            Elevator.waitUntilSetPoint(VerticalConstants.ElevatorPositions.BOTTOM),
                            Elevator.pid(VerticalConstants.ElevatorPositions.BOTTOM),
                        )
                    ),
                    Sequential(
                        Deposit.open(),
                        Wait(0.100),
                        Parallel(
                            Wait(VerticalConstants.VerticalArmConstants.sampleToIntake),
                            VerticalArm.intake(),
                            VerticalWrist.intake(),
                            Deposit.open()
                        ),
                        Parallel(
                            Elevator.waitUntilSetPoint(VerticalConstants.ElevatorPositions.BOTTOM),
                            Elevator.pid(VerticalConstants.ElevatorPositions.BOTTOM),
                        )

                    )
                )
            ),
            Parallel(
                Elevator.waitUntilSetPoint(VerticalConstants.ElevatorPositions.BOTTOM),
                Elevator.pid(VerticalConstants.ElevatorPositions.BOTTOM),
                VerticalArm.intake(),
                VerticalWrist.intake(),
                Deposit.open(),
            )

        )

        val horizontalRetract = Parallel(HorizontalExtension.waitUntilSetPoint(HorizontalConstants.HorizontalExtensionPositions.BOTTOM), HorizontalExtension.pid(HorizontalConstants.HorizontalExtensionPositions.BOTTOM), HorizontalArm.inHorizontalArm(), HorizontalWrist.inHorizontalWrist(), Intake.stopIntake())

        /*
        val sample = Sequential(

            Timeout(Parallel(
                HorizontalExtension.waitUntilInsideSetPoint(HorizontalConstants.HorizontalExtensionPositions.INSIDE, HorizontalConstants.HorizontalExtensionPositions.BOTTOM),
                //HorizontalExtension.waitUntilSetPoint(HorizontalConstants.HorizontalExtensionPositions.INSIDE),
                HorizontalExtension.pid(HorizontalConstants.HorizontalExtensionPositions.BOTTOM),
                HorizontalArm.inHorizontalArm(),
                HorizontalWrist.inHorizontalWrist(),
                Intake.stopIntake()
            ), 1.0),

            IfElse(
                {HorizontalExtension.getPosition() > 0.5},
                Parallel(
                    Sequential(
                        HorizontalExtension.waitUntilSetPoint(HorizontalConstants.HorizontalExtensionPositions.BOTTOM),
                        HorizontalExtension.spin(-0.3),
                    ),
                    Wait(0.250),
                ),
                Sequential()
            ),
            HorizontalExtension.spin(-0.3),


            /*
            Timeout(HorizontalExtension.retract(), 0.500),
             */

            //Race(
                Sequential(
                    Wait(0.01),
                    Race( null,
                        Intake.spinUntilHolding(),
                        Wait(0.400),
                    ),
                    Deposit.halfClose(),
                    Intake.runIntake(),
                    Wait(0.05),
                    Intake.stopIntake(),

                    Deposit.close(),
                    Wait(0.10),
                ),
                //HorizontalExtension.retractKeep()
            //),

            verticalSample
        )

         */
        val sample = Sequential(
            //Parallel(
            Timeout(Parallel(/*HorizontalExtension.waitUntilSetPoint(HorizontalConstants.HorizontalExtensionPositions.INSIDE), */HorizontalExtension.pid(HorizontalConstants.HorizontalExtensionPositions.BOTTOM), HorizontalArm.inHorizontalArm(), HorizontalWrist.inHorizontalWrist(), Intake.stopIntake()), 1.5),
                //Wait(0.100)
            //),

            IfElse(
                {HorizontalExtension.getPosition() > 0.5},
                Parallel(
                    Sequential(
                        HorizontalExtension.waitUntilSetPoint(HorizontalConstants.HorizontalExtensionPositions.BOTTOM),
                    ),
                    Wait(0.450),
                ),
                Sequential()
            ),
            HorizontalExtension.spin(-0.3),
            Wait(0.10),
            Race( null,
                Intake.spinUntilHolding(),
                Wait(0.400),
            ),
            Deposit.halfClose(),
            Intake.runIntake(),
            Wait(0.05),
            Intake.stopIntake(),

            Deposit.close(),
            HorizontalExtension.spin(0.0),
            Wait(0.10),
            verticalSample
        )

        val climb = Sequential(
            Lambda("climb").setInit{ Elevator.limitless = true },
            Timeout(
            Lambda("disable").setInit{
                HorizontalExtension.defaultCommand = HorizontalExtension.stop()
                SwerveDrivetrain.defaultCommand = SwerveDrivetrain.stopCmd()
            }, 0.1),
            /*
            Race(
                Wait(0.1),
                SwerveDrivetrain.kill(),
                HorizontalExtension.kill(),
                HorizontalArm.kill(),
                HorizontalWrist.kill(),
                Intake.kill(),
                VerticalWrist.kill(),
                Deposit.kill(),
            ),
             */

            Parallel(
                Sequential(
                    Wait(0.100),
                    Parallel(
                        Elevator.pid(VerticalConstants.ElevatorPositions.CLIMB_ONE),
                        Race(
                            null,
                            Elevator.waitUntilSetPoint(VerticalConstants.ElevatorPositions.CLIMB_ONE),
                            Wait(1.0)
                        )
                    )
                ),
                //Wait(0.100).then(Elevator.pid(VerticalConstants.ElevatorPositions.CLIMB_ONE).with(Elevator.waitUntilSetPoint(VerticalConstants.ElevatorPositions.CLIMB_ONE))),
                VerticalArm.sample()
            ),
            Wait(0.1),
            //Lambda("delete").addRequirements(Elevator).setInit{Elevator.defaultCommand = null},
            //Elevator.cancel(),

            //Elevator.driveAndClimb({gamepad2.left_stick_y.toDouble() + if(gamepad1.right_bumper) {-1000.0} else {0.0}}, {if (gamepad1.dpad_up) {1.0} else {0.0} + if (gamepad1.dpad_down) {-1.0} else {0.0}})
            Elevator.climb{-1.0}
        )

        mechanismGamepad.dpadUp.onTrue(Parallel(Sequential(Intake.runIntake(), Wait(0.400), Intake.stopIntake()), verticalSample).with(horizontalRetract))
        mechanismGamepad.dpadDown.onTrue(verticalRetract)

        mechanismGamepad.dpadLeft.onTrue(verticalSpecimenPickup.with(horizontalRetract))
        mechanismGamepad.dpadRight.onTrue(verticalSpecimenPlace.with(horizontalRetract))

        mechanismGamepad.y.onTrue(sample)

        mechanismGamepad.x.onTrue(horizontalRetract)
        mechanismGamepad.b.onTrue(
            IfElse ( {HorizontalExtension.getPosition() < 1.0},
                Parallel(HorizontalExtension.waitUntilSetPoint(HorizontalConstants.HorizontalExtensionPositions.CLEAR), HorizontalExtension.pid(HorizontalConstants.HorizontalExtensionPositions.CLEAR), HorizontalArm.outHorizontalArm(), HorizontalWrist.outHorizontalWrist()),
                Parallel(HorizontalArm.outHorizontalArm(), HorizontalWrist.outHorizontalWrist()),
            ),
        )
        /*
        mechanismGamepad.b.onTrue(
            Parallel(
                IfElse ( {HorizontalExtension.getPosition() < 1.0},
                    Parallel(HorizontalExtension.waitUntilSetPoint(HorizontalConstants.HorizontalExtensionPositions.CLEAR), HorizontalExtension.pid(HorizontalConstants.HorizontalExtensionPositions.CLEAR), HorizontalArm.outHorizontalArm(), HorizontalWrist.outHorizontalWrist()),
                    Parallel(HorizontalArm.outHorizontalArm(), HorizontalWrist.outHorizontalWrist()),
                ),
                VerticalArm.intake(),
                VerticalWrist.intake(),
                IfElse ( {Elevator.getPosition() > VerticalConstants.ElevatorPositions.SPECIMEN_PLACE+1.0},
                    Parallel (
                        Elevator.pid(VerticalConstants.ElevatorPositions.SPECIMEN_PLACE),
                        Elevator.waitUntilSetPoint(VerticalConstants.ElevatorPositions.SPECIMEN_PLACE)
                    ),
                    Sequential()
                )
            )
        )

         */
        mechanismGamepad.b.onTrue(Wait(0.100).then(Intake.runIntakeStopping().then(Intake.backDrive())))
        mechanismGamepad.start.onTrue(Wait(0.100).then(Intake.runIntakeStoppingBackwards().then(Intake.backBackDrive())))

        mechanismGamepad.a.onTrue(Parallel(HorizontalExtension.waitUntilSetPoint(HorizontalConstants.HorizontalExtensionPositions.BOTTOM), HorizontalExtension.pid(HorizontalConstants.HorizontalExtensionPositions.BOTTOM), HorizontalArm.setPositionCommand(HorizontalConstants.HorizontalArmPositions.MID)))

        mechanismGamepad.rightBumper.onTrue(Deposit.close())
        mechanismGamepad.leftBumper.onTrue(Deposit.open())

        driveGamepad.leftBumper.onTrue(climb)

        //telemetry.addData("state", mechanismGamepad.rightTrigger.state )
        mechanismGamepad.rightTrigger.conditionalBindState().greaterThan(0.05).bind().onTrue(Lambda("clear").addRequirements(Intake))
        mechanismGamepad.leftTrigger.conditionalBindState().greaterThan(0.05).bind().onTrue(Lambda("clear").addRequirements(Intake))

        //mechanismGamepad.rightTrigger.conditionalBindState().greaterThan(0.05).bind()
        //mechanismGamepad.leftTrigger.conditionalBindState().greaterThan(0.05).bind()

        mechanismGamepad.leftStickY.conditionalBindState().lessThan(-0.05).bind().onTrue(Elevator.disableController())
        mechanismGamepad.leftStickY.conditionalBindState().greaterThan(0.05).bind().onTrue(Elevator.disableController())

        mechanismGamepad.rightStickY.conditionalBindState().lessThan(-0.05).bind().onTrue(HorizontalExtension.disableController())
        mechanismGamepad.rightStickY.conditionalBindState().greaterThan(0.05).bind().onTrue(HorizontalExtension.disableController())

        driveGamepad.leftStickButton.onTrue(SwerveDrivetrain.resetHeadingCommand())

        driveGamepad.back.onTrue(Intake.spinUntilHolding())

        //Telemetry.points.add(Pose2d(72.0, 72.0, Rotation2d(0.0)))
        //Telemetry.points.add(Pose2d(78.0, 7.375, Rotation2d.fromDegrees(90.0)))
        /*
        val path = listOf(
            CurvePoint(Pose2d(78.0, 7.375, Rotation2d.fromDegrees(90.0)), 1.0, 1.0, 5.0, 1.0, 1.0, 1.0),
            CurvePoint(Pose2d(78.0, 39.5, Rotation2d.fromDegrees(90.0)), 1.0, 1.0, 5.0, 1.0, 1.0, 1.0),
            CurvePoint(Pose2d(103.5, 16.0, Rotation2d.fromDegrees(90.0)), 1.0, 1.0, 5.0, 1.0, 1.0, 1.0),
            CurvePoint(Pose2d(108.0, 58.0, Rotation2d.fromDegrees(-85.0)), 1.0, 1.0, 5.0, 1.0, 1.0, 1.0),
            CurvePoint(Pose2d(120.0, 57.0, Rotation2d.fromDegrees(-90.0)), 1.0, 1.0, 5.0, 1.0, 1.0, 1.0),
            CurvePoint(Pose2d(120.0, 16.5, Rotation2d.fromDegrees(-90.0)), 1.0, 1.0, 5.0, 1.0, 1.0, 1.0),
            CurvePoint(Pose2d(70.0, 36.0, Rotation2d.fromDegrees(90.0)), 1.0, 1.0, 5.0, 1.0, 1.0, 1.0),
            CurvePoint(Pose2d(104.0, 25.0, Rotation2d.fromDegrees(90.0)), 1.0, 1.0, 5.0, 1.0, 1.0, 1.0),
            CurvePoint(Pose2d(120.0, 56.0, Rotation2d.fromDegrees(-90.0)), 1.0, 1.0, 5.0, 1.0, 1.0, 1.0),
            CurvePoint(Pose2d(129.0, 56.0, Rotation2d.fromDegrees(-90.0)), 1.0, 1.0, 5.0, 1.0, 1.0, 1.0),
            CurvePoint(Pose2d(129.0, 19.0, Rotation2d.fromDegrees(-90.0)), 1.0, 1.0, 5.0, 1.0, 1.0, 1.0),
            CurvePoint(Pose2d(69.0, 34.5, Rotation2d.fromDegrees(90.0)), 1.0, 1.0, 5.0, 1.0, 1.0, 1.0),
            CurvePoint(Pose2d(120.0, 12.0, Rotation2d.fromDegrees(90.0)), 1.0, 1.0, 5.0, 1.0, 1.0, 1.0),
        )

        Telemetry.path = path
         */
        Telemetry.put("alliance colour", Globals.AllianceColour.name)

    }

    override fun start() {
        //SwerveDrivetrain.resetHeading()
    }

    override fun loop() {

        //packet.put("elevator pid speed", Elevator.controller.velocity)
        //packet.put("elevator pid state", Elevator.controller.state)
        //packet.put("elevator pos", Elevator.getPosition())
        Telemetry.put("horizontal pos", HorizontalExtension.getPosition())
        //packet.put("elevator target", Elevator.targetPosition)
        //packet.put("elevator pid atSetPoint", Elevator.controller.finished())
        //packet.put("elevator pid atSetPoint 2", Elevator.atSetPoint())
        Telemetry.put("hori arm cached", HorizontalArm.getCachedPosition())
        Telemetry.put("hori arm", HorizontalArm.getPosition())

        //packet.put("pin0", Intake.getPin0())
        Telemetry.put("intake piece", Intake.getGamePiece().name)

    }

}