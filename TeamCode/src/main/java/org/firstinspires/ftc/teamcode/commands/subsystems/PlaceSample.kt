package org.firstinspires.ftc.teamcode.commands.subsystems

import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import org.firstinspires.ftc.teamcode.constants.HorizontalConstants
import org.firstinspires.ftc.teamcode.constants.VerticalConstants
import org.firstinspires.ftc.teamcode.subsystems.Deposit
import org.firstinspires.ftc.teamcode.subsystems.DepositCommand
import org.firstinspires.ftc.teamcode.subsystems.Elevator
import org.firstinspires.ftc.teamcode.subsystems.HorizontalArm
import org.firstinspires.ftc.teamcode.subsystems.HorizontalArmCommand
import org.firstinspires.ftc.teamcode.subsystems.HorizontalExtension
import org.firstinspires.ftc.teamcode.subsystems.HorizontalWrist
import org.firstinspires.ftc.teamcode.subsystems.HorizontalWristCommand
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.VerticalArm
import org.firstinspires.ftc.teamcode.subsystems.VerticalArmCommand
import org.firstinspires.ftc.teamcode.subsystems.VerticalWrist
import org.firstinspires.ftc.teamcode.subsystems.VerticalWristCommand

class PlaceSample(private val horizontalExtension: HorizontalExtension,
                  private val horizontalArm: HorizontalArm,
                  private val horizontalWrist: HorizontalWrist,
                  private val intake: Intake,
                  private val elevator: Elevator,
                  private val verticalArm: VerticalArm,
                  private val verticalWrist: VerticalWrist,
                  private val deposit: Deposit) : SequentialCommandGroup() {
    init {
        addRequirements(horizontalExtension, horizontalArm, horizontalWrist, elevator, verticalArm, verticalWrist, deposit)

        addCommands(
            VerticalRetract(elevator, verticalArm, verticalWrist, deposit),
                //ElevatorPIDCommand(elevator, VerticalConstants.ElevatorPositions.BOTTOM),
                //VerticalArmCommand(verticalArm, VerticalConstants.VerticalArmPositions.INTAKE),
                //VerticalWristCommand(verticalWrist, VerticalConstants.VerticalWristPositions.INTAKE),
                //HorizontalArmCommand(horizontalArm, HorizontalConstants.HorizontalArmPositions.IN),
                //HorizontalWristCommand(horizontalWrist, HorizontalConstants.HorizontalWristPositions.OUT),
                //DepositCommand(deposit, VerticalConstants.DepositPositions.OUT)
            //HorizontalExtensionPIDCommand(horizontalExtension, HorizontalConstants.HorizontalExtensionPositions.BOTTOM),
            //DepositCommand(deposit, VerticalConstants.DepositPositions.IN)
            HorizontalRetract(horizontalExtension, horizontalArm, horizontalWrist),
            DepositCommand(deposit, VerticalConstants.DepositPositions.IN+0.08),
            IntakeRelease(intake),
            DepositCommand(deposit, VerticalConstants.DepositPositions.IN),
            VerticalSample(elevator, verticalArm, verticalWrist)

        )
    }
}