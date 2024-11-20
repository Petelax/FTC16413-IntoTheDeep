package org.firstinspires.ftc.teamcode.commands.ftclib.subsystems

import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import org.firstinspires.ftc.teamcode.constants.HorizontalConstants
import org.firstinspires.ftc.teamcode.subsystems.ftclib.HorizontalArm
import org.firstinspires.ftc.teamcode.subsystems.ftclib.HorizontalArmCommand
import org.firstinspires.ftc.teamcode.subsystems.ftclib.HorizontalWrist
import org.firstinspires.ftc.teamcode.subsystems.ftclib.HorizontalWristCommand
import org.firstinspires.ftc.teamcode.subsystems.ftclib.Intake

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