/*
 *  This file contains code of the GraphHopper project. See the
 *  THIRD_PARTY.md file distributed with this work for additional
 *  information regarding copyright ownership.
 */

package de.geofabrik.railway_routing.http;

import java.util.List;

import javax.inject.Inject;

import com.graphhopper.GraphHopperConfig;
import de.geofabrik.railway_routing.RailwayHopper;
import io.dropwizard.lifecycle.Managed;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class RailwayRoutingManaged implements Managed {

    private final Logger logger = LoggerFactory.getLogger(getClass());
    private final RailwayHopper graphHopper;
    
    @Inject
    public RailwayRoutingManaged(GraphHopperConfig configuration, List<FlagEncoderConfiguration> encoderConfig) {
        graphHopper = new RailwayHopper(encoderConfig);
        graphHopper.init(configuration);
    }
    
    @Override
    public void start() throws Exception {
        graphHopper.importOrLoad();
        logger.info("loaded graph at:" + graphHopper.getGraphHopperLocation()
                + ", data_reader_file:" + graphHopper.getOSMFile()
                + ", flag_encoders:" + graphHopper.getEncodingManager()
                + ", " + graphHopper.getBaseGraph().toDetailsString());
    }

    RailwayHopper getGraphHopper() {
        return graphHopper;
    }


    @Override
    public void stop() throws Exception {
        graphHopper.close();
    }

}
