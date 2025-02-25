package frc.robot;

import java.util.HashMap;
import java.util.Map;

public class FieldTagMap {
    private Map<String , AprilTags> redMap = new HashMap<>(); 
    private Map<String , AprilTags> blueMap = new HashMap<>(); 

    public FieldTagMap() {
        redMap.put("coral1", AprilTags.RedCoral1);
        redMap.put("coral2", AprilTags.RedCoral2);
        redMap.put("redBarge", AprilTags.RedSideRedBarge);
        redMap.put("blueBarge", AprilTags.RedSideBlueBarge);
        redMap.put("processor", AprilTags.RedProcessor);
        redMap.put("reef1", AprilTags.RedReef1);
        redMap.put("reef2", AprilTags.RedReef2);
        redMap.put("reef3", AprilTags.RedReef3);
        redMap.put("reef4", AprilTags.RedReef4);
        redMap.put("reef5", AprilTags.RedReef5);
        redMap.put("reef6", AprilTags.RedReef6);
        
        blueMap.put("coral1", AprilTags.BlueCoral1);
        blueMap.put("coral2", AprilTags.BlueCoral2);
        blueMap.put("redBarge", AprilTags.BlueSideRedBarge);
        blueMap.put("blueBarge", AprilTags.BlueSideBlueBarge);
        blueMap.put("processor", AprilTags.BlueProcessor);
        blueMap.put("reef1", AprilTags.BlueReef1);
        blueMap.put("reef2", AprilTags.BlueReef2);
        blueMap.put("reef3", AprilTags.BlueReef3);
        blueMap.put("reef4", AprilTags.BlueReef4);
        blueMap.put("reef5", AprilTags.BlueReef5);
        blueMap.put("reef6", AprilTags.BlueReef6);
    }

    public Map<String , AprilTags> getBlueMap() {
        return blueMap;
    }

    public Map<String , AprilTags> getRedMap() {
        return redMap;
    }
    
}
