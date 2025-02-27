package frc.robot.BulukLib.Dashboard.Syncro;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEvent;
import java.util.EnumSet;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.BulukLib.Dashboard.Elastic;
import frc.robot.BulukLib.Dashboard.Elastic.Notification;
import frc.robot.BulukLib.Dashboard.Elastic.Notification.NotificationLevel;

public class SyncroLinker {

    private static final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
    private static final NetworkTable table = ntInstance.getTable("Syncro");
    private static final NetworkTableEntry namedCommandEntry = table.getEntry("namedCommand");
    private static final Notification connection = new Notification(NotificationLevel.INFO, "SYNCRO: Starting...", "Conecting to networkTables");
 
    public static void listen() {

        Elastic.sendNotification(connection);

        ntInstance.addListener(
            namedCommandEntry,
            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
            event -> {
                String commandName = event.valueData.value.getString();
        
                if (SyncroCommands.has(commandName)) {
                    CommandScheduler.getInstance().schedule(SyncroCommands.get(commandName));
    
                    namedCommandEntry.setString("");  //Limpia el valor
                    ntInstance.flush();  //Fuerza la actualización en NT
                } 
            }
        );
    }
    
}
