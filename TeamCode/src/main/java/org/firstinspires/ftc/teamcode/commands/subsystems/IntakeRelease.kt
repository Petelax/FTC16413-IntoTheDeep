package org.firstinspires.ftc.teamcode.commands.subsystems

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.constants.HorizontalConstants
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.utils.AllianceColours
import org.firstinspires.ftc.teamcode.utils.Globals
import java.util.function.BooleanSupplier

class IntakeRelease(private var intake: Intake): SequentialCommandGroup() {
    init {
        addRequirements(intake)
        addCommands(
            InstantCommand({intake.setSpeed(HorizontalConstants.IntakeSpeeds.MAX)}),
            WaitCommand(300),
            InstantCommand({intake.setSpeed(0.0)})
        )
    }
}