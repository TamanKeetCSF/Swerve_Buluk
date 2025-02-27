package frc.robot.BulukLib.Swerve;

public class ModuleMap {

    public int drive, turn, encoder;
    public double off;
    public boolean driveInv, turnInv;

    public ModuleMap(int drive, int turn, int encoder, double off, boolean driveInv, boolean turnInv){
        this.drive = drive;
        this.turn = turn;
        this.encoder = encoder;
        this.off = off;
        this.driveInv = driveInv;
        this.turnInv = turnInv;

    }
}
