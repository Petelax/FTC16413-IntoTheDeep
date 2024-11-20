package org.firstinspires.ftc.teamcode.commands.ftclib.subsystems

import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
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

class PlaceSample(private val horizontalExtension: HorizontalExtension,
                  private val horizontalArm: HorizontalArm,
                  private val horizontalWrist: HorizontalWrist,
                  private val intake: Intake,
                  private val elevator: Elevator,
                  private val verticalArm: VerticalArm,
                  private val verticalWrist: VerticalWrist,
                  private val deposit: Deposit
) : SequentialCommandGroup() {
    init {
        addRequirements(horizontalExtension, horizontalArm, horizontalWrist, elevator, verticalArm, verticalWrist, deposit)

        addCommands(
            org.firstinspires.ftc.teamcode.commands.ftclib.subsystems.VerticalRetract(
                elevator,
                verticalArm,
                verticalWrist,
                deposit
            ),
                //ElevatorPIDCommand(elevator, VerticalConstants.ElevatorPositions.BOTTOM),
                //VerticalArmCommand(verticalArm, VerticalConstants.VerticalArmPositions.INTAKE),
                //VerticalWristCommand(verticalWrist, VerticalConstants.VerticalWristPositions.INTAKE),
                //HorizontalArmCommand(horizontalArm, HorizontalConstants.HorizontalArmPositions.IN),
                //HorizontalWristCommand(horizontalWrist, HorizontalConstants.HorizontalWristPositions.OUT),
                //DepositCommand(deposit, VerticalConstants.DepositPositions.OUT)
            //HorizontalExtensionPIDCommand(horizontalExtension, HorizontalConstants.HorizontalExtensionPositions.BOTTOM),
            //DepositCommand(deposit, VerticalConstants.DepositPositions.IN)
            org.firstinspires.ftc.teamcode.commands.ftclib.subsystems.HorizontalRetract(
                horizontalExtension,
                horizontalArm,
                horizontalWrist
            ),
            WaitCommand(100),
            DepositCommand(deposit, VerticalConstants.DepositPositions.IN+0.08),
            org.firstinspires.ftc.teamcode.commands.ftclib.subsystems.IntakeRelease(intake),
            DepositCommand(deposit, VerticalConstants.DepositPositions.IN),
            org.firstinspires.ftc.teamcode.commands.ftclib.subsystems.VerticalSample(
                elevator,
                verticalArm,
                verticalWrist
            )

        )
    }
}