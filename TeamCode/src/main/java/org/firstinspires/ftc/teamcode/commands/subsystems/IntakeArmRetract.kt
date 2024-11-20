package org.firstinspires.ftc.teamcode.commands.subsystems

import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import org.firstinspires.ftc.teamcode.constants.HorizontalConstants
import org.firstinspires.ftc.teamcode.subsystems.HorizontalArm
import org.firstinspires.ftc.teamcode.subsystems.HorizontalArmCommand
import org.firstinspires.ftc.teamcode.subsystems.HorizontalExtension
import org.firstinspires.ftc.teamcode.subsystems.HorizontalWrist
import org.firstinspires.ftc.teamcode.subsystems.HorizontalWristCommand
import org.firstinspires.ftc.teamcode.subsystems.Intake

class IntakeArmRetract(private val horizontalArm: HorizontalArm, private val horizontalWrist: HorizontalWrist, private val intake: Intake) : SequentialCommandGroup() {
    init {
        addRequirements(horizontalArm, horizontalWrist, intake)

        addCommands(
            ParallelCommandGroup(
                HorizontalArmCommand(horizontalArm, HorizontalConstants.HorizontalArmPositions.IN),
                HorizontalWristCommand(horizontalWrist, HorizontalConstants.HorizontalWristPositions.IN),
                InstantCommand({intake.setSpeed(0.0)})
            )
        )
    }
}