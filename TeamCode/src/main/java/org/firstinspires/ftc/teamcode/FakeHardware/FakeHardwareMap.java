package org.firstinspires.ftc.teamcode.FakeHardware;

import java.util.Map;

public class FakeHardwareMap {
    public Map<String, Object> map;
    public Object get(Object object, String deviceName){
        map.put(deviceName, object);
        return map.get(object);
    }
}
