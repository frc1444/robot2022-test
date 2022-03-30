package frc.robot.vision;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonIgnoreProperties;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.databind.JavaType;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.annotation.JsonDeserialize;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class VisionPacketParser {
    private final ObjectMapper mapper;
    private final JavaType visionInstantArrayListType;

    public VisionPacketParser(ObjectMapper mapper) {
        this.mapper = mapper;
        visionInstantArrayListType = mapper.getTypeFactory().constructCollectionType(ArrayList.class, CameraPacket.class);
    }
    public VisionInstant parse(double timestamp, String jsonString) throws IOException {
        List<CameraPacket> instants = mapper.readValue(jsonString, visionInstantArrayListType);

        double thetaDegreesSum = 0.0;
        int count = 0;
        Double firstThetaDebug = null;

        final List<Surrounding> surroundings = new ArrayList<>();
        for(CameraPacket instant : instants){
            for (TargetPacket packet : instant.packets) {
                Transform2d transform = new Transform2d( // target/world centric
                    new Translation2d(packet.zMeters, packet.xMeters).rotateBy(Rotation2d.fromDegrees(-packet.yawDegrees)),
                    Rotation2d.fromDegrees(-packet.yawDegrees)
                );
//                Transform2d transform = new Transform2d( // robot centric
//                        new Translation2d(packet.zMeters * Math.cos(Math.toRadians(packet.pitchDegrees)), -packet.xMeters),
//                        Rotation2d.fromDegrees(packet.yawDegrees)
//                );

                // The goal here is that transform is a top-down view with a robot centric coordinate system
                //   As of writing this we were switching between receiving data as camera centric or target centric, so this isn't actually correct right now
                Surrounding surrounding = new Surrounding(transform, timestamp);
                surroundings.add(surrounding);

                if (firstThetaDebug == null) {
                    firstThetaDebug = packet.thetaDegrees;
                }
                thetaDegreesSum += packet.thetaDegrees;
                count++;
            }
        }
        SmartDashboard.putString("first theta", "" + firstThetaDebug);
        return new VisionInstant(surroundings, count == 0 ? null : Rotation2d.fromDegrees(thetaDegreesSum / count), timestamp);
    }

    private static class CameraPacket {
        private final List<TargetPacket> packets;
        private final int cameraId;

        private CameraPacket(
                @JsonProperty(value = "packets", required = true) @JsonDeserialize(as = ArrayList.class) List<TargetPacket> packets,
                @JsonProperty(value = "cameraId", required = true) int cameraId
        ) {
            this.packets = packets;
            this.cameraId = cameraId;
        }
    }
    @JsonIgnoreProperties(ignoreUnknown = true)
    private static class TargetPacket {
        //        private final int status;
        private final double imageX, imageY;
        private final double xMeters, yMeters, zMeters;
        private final double yawDegrees, pitchDegrees, rollDegrees;
        private final double thetaDegrees;
        private final double distanceMeters;

        //        @JsonIgnoreProperties({"theta_deg", "dist_mm"})
        @JsonCreator
        private TargetPacket(
//                @JsonProperty(value = "status", required = true) int status,
                @JsonProperty(value = "imageX_px", required = true) double imageX,
                @JsonProperty(value = "imageY_px", required = true) double imageY,

                @JsonProperty(value = "x_mm", required = true) double xMm,
                @JsonProperty(value = "y_mm", required = true) double yMm,
                @JsonProperty(value = "z_mm", required = true) double zMm,

                @JsonProperty(value = "yaw_deg", required = true) double yawDegrees,
                @JsonProperty(value = "pitch_deg", required = true) double pitchDegrees,
                @JsonProperty(value = "roll_deg", required = true) double rollDegrees,

                @JsonProperty(value = "theta_deg", required = true) double thetaDegrees,
                @JsonProperty(value = "dist_mm", required = true) double distanceMm
        ) {
//            this.status = status;
            this.imageX = imageX;
            this.imageY = imageY;
            this.xMeters = xMm / 1000.0;
            this.yMeters = yMm / 1000.0;
            this.zMeters = zMm / 1000.0;
            this.yawDegrees = yawDegrees;
            this.pitchDegrees = pitchDegrees;
            this.rollDegrees = rollDegrees;
            this.thetaDegrees = thetaDegrees;
            this.distanceMeters = distanceMm / 1000.0;
        }
    }

    /**
     * More info <a href="https://github.com/frc1444/robot2020-vision/blob/master/VisionStatus.hpp">here</a>
     */
    private enum VisionStatus { // use this if we find it useful later
        TARGET_FOUND,
        NO_TARGET_FOUND,
        CAMERA_ERROR,
        PROCESSING_ERROR,
        NOT_RUNNING
    }

}
