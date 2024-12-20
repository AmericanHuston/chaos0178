package org.firstinspires.ftc.teamcode.FakeHardware;

import com.sun.tools.javac.code.Attribute;

import java.util.Map;

public class FakeHardwareMap {
    public Map<String, Object> map;
    public Object get(Class object, String deviceName){
        map.put(deviceName, object);
        return map.get(deviceName);
    }
}
