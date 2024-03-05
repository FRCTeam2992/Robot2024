package frc.lib.autonomous;

public enum AutoPreloadScore {
    No_Preload("No Preload Score"),
    Mid_Cube("Mid Cube Preload");

    public String description;

    private AutoPreloadScore(String description) {
        this.description = description;
    }

}