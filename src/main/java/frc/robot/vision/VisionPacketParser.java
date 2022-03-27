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

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class VisionPacketParser {
    private final ObjectMapper mapper;
    private final JavaType visionInstantArrayListType;

    public VisionPacketParser(ObjectMapper mapper) {
        this.mapper = mapper;
        visionInstantArrayListType = mapper.getTypeFactory().constructCollectionType(ArrayList.class, VisionInstant.class);
    }

    public List<Surrounding> parseSurroundings(double timestamp, String jsonString) throws IOException {
        List<VisionInstant> instants = mapper.readValue(jsonString, visionInstantArrayListType);
        return parseSurroundings(timestamp, instants);
    }
    private List<Surrounding> parseSurroundings(double timestamp, List<VisionInstant> instants) throws IOException {
        final List<Surrounding> surroundings = new ArrayList<>();
        for(VisionInstant instant : instants){
            if (instant.cameraId != 0) {
                // In robot2020, we caught an invalid camera ID exception when using an offsetProvider, then rethrow as an IOException
                throw new IOException("Invalid camera ID!");
            }
            // if in the future we add multiple cameras, we would have different offsets for each camera here. robot2019 had this but we don't need it anymore
            final Rotation2d offset = Rotation2d.fromDegrees(0); // TODO look at robot2021 and see what the default offset is

            for (VisionPacket packet : instant.packets) {
                Transform2d transform = new Transform2d(
                        new Translation2d(packet.zMeters, -packet.xMeters).rotateBy(offset),
                        Rotation2d.fromDegrees(packet.yawDegrees).plus(offset)
                );
                Surrounding surrounding = new Surrounding(transform, timestamp);
                surroundings.add(surrounding);
            }
        }
        return surroundings;
    }
    private static class VisionInstant {
        private final List<VisionPacket> packets;
        private final int cameraId;

        private VisionInstant(
                @JsonProperty(value = "packets", required = true) @JsonDeserialize(as = ArrayList.class) List<VisionPacket> packets,
                @JsonProperty(value = "cameraId", required = true) int cameraId
        ) {
            this.packets = packets;
            this.cameraId = cameraId;
        }
    }
    @JsonIgnoreProperties(ignoreUnknown = true)
    private static class VisionPacket {
        //        private final int status;
        private final double imageX, imageY;
        private final double xMeters, yMeters, zMeters;
        private final double yawDegrees, pitchDegrees, rollDegrees;

        //        @JsonIgnoreProperties({"theta_deg", "dist_mm"})
        @JsonCreator
        private VisionPacket(
//                @JsonProperty(value = "status", required = true) int status,
                @JsonProperty(value = "imageX_px", required = true) double imageX,
                @JsonProperty(value = "imageY_px", required = true) double imageY,

                @JsonProperty(value = "x_mm", required = true) double xMeters,
                @JsonProperty(value = "y_mm", required = true) double yMeters,
                @JsonProperty(value = "z_mm", required = true) double zMeters,

                @JsonProperty(value = "yaw_deg", required = true) double yawDegrees,
                @JsonProperty(value = "pitch_deg", required = true) double pitchDegrees,
                @JsonProperty(value = "roll_deg", required = true) double rollDegrees
        ) {
//            this.status = status;
            this.imageX = imageX;
            this.imageY = imageY;
            this.xMeters = xMeters / 1000.0;
            this.yMeters = yMeters / 1000.0;
            this.zMeters = zMeters / 1000.0;
            this.yawDegrees = yawDegrees;
            this.pitchDegrees = pitchDegrees;
            this.rollDegrees = rollDegrees;
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
