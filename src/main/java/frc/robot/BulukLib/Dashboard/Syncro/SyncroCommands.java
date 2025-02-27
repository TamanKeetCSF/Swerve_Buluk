package frc.robot.BulukLib.Dashboard.Syncro;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class SyncroCommands{
    private static final Map<String, Command> commands = new HashMap<>();
    private static final NetworkTable table = NetworkTableInstance.getDefault().getTable("Syncro");
    private static final NetworkTableEntry commandListEntry = table.getEntry("commandList");

    private static final Timer updateTimer = new Timer();

    static {
        updateTimer.start();
    }

    public static void periodicUpdate() {
        if (updateTimer.hasElapsed(5.0)) { // Cada 5 segundos
            updateNT();
            updateTimer.reset(); 
        }
    }

    public static void add(String name, Command command) {
        commands.put(name, command);
        updateNT();
        
        // Hacer persistente la lista de comandos
        commandListEntry.setStringArray(getRegistered());
        commandListEntry.setPersistent(); 
    }

    public static Command get(String name) {

        if (has(name)) {
            System.out.println("[Syncro] Ejecutando comando: " + name);
            return commands.get(name);
        } else {
            System.out.println("[Syncro] ❌ Comando NO encontrado: " + name);
            return Commands.none();
        }

    }
    
    public static boolean has(String name) {
        return commands.containsKey(name);
    }

    // Obtener la lista de nombres de comandos registrados
    public static String[] getRegistered() {
        return commands.keySet().toArray(new String[0]);
    }

    private static void updateNT() {
        String[] registered = getRegistered();
        System.out.println("[Syncro] Actualizando commandList en NT: " + String.join(", ", registered));
        commandListEntry.setStringArray(registered);
        commandListEntry.setPersistent();
    }

}
