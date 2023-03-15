package us.ilite.common;

import us.ilite.robot.hardware.LidarLite;

public class Field2022 {

    public enum Distances{
        TARMAC_TO_HUB(84.25),
        HUB_RING_DIAMETER(306),
        BALL_TO_HUB(126.5),
        TARGETTING_OFFSET(21.0);

        public Distance mDistance;

        Distances(double pDist) {
            mDistance = Distance.fromInches(pDist);
        }
    }

    /**
     * Any trackable field component for 2020's game.
     */
    public enum FieldElement implements IFieldComponent { // add field elements with their measurements and the pipeline that tracks them
        //Do not switch order
        NULL(0, 0, 0),
        HUB_UPPER (104,48, 1),
//        TARGET_ZOOM             (0d,0d),
        //RED_BALL(0 , 0, 3),
        //BLUE_BALL(0, 0, 4),
        CAMERA(0,0, 0),
        REFLECTIVE_TAPE(0,0, 1),
        CONE(12 + ((double)13/16), 8 + ((double) 1/2), 2), // cones have a width of 12 13/16 inches and a height of 8 1/2 inches
        //CUBE(9 + ((double)3/4),9 + ((double)3/4), 3), // cues have a width of 9 3/4 inches and a height of 9 3/4 inches
        BLUE_ROBOT_LEFT(5,32,3), // track blue robots' bumper with dimensions of 5 inches tall and 32 inches long
        BLUE_ROBOT_RIGHT(5,32,4), // track blue robots' bumper with dimensions of 5 inches tall and 32 inches long
        RED_ROBOT_LEFT(5,32,5), // track red robots' bumper with dimensions of 5 inches tall and 32 inches long
        RED_ROBOT_RIGHT(5,32,6), // track red robots' bumper with dimensions of 5 inches tall and 32 inches long
        ;


        // This is done in inches -- straight from game manual
        private final double height;
        private final double width;
        private int pipeline;

        // Limelight-based FieldElement
        FieldElement(double pHeight, double pWidth, int pPipeline){
            width = pWidth;
            height = pHeight;
            pipeline = pPipeline;
        }

        // Limelight-based FieldElement
        FieldElement(double pHeight, double pWidth){
            width = pWidth;
            height = pHeight;
            pipeline = 1;
        }

        @Override
        public double width() { return width; }

        @Override
        public double height() {
            return height;
        }

        @Override
        public int pipeline() {
            return pipeline;
        }

        public int id() { return ordinal(); }
    }

    public static void main(String[] args) {
        for(FieldElement anElement: FieldElement.values()) {
            System.out.println(anElement.name()+" id: " + anElement.id()+", pipeline: "+ anElement.pipeline());
        }
    }
}
