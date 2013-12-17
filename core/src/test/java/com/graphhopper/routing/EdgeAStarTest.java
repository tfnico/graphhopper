/*
 *  Licensed to Peter Karich under one or more contributor license 
 *  agreements. See the NOTICE file distributed with this work for 
 *  additional information regarding copyright ownership.
 * 
 *  Peter Karich licenses this file to you under the Apache License, 
 *  Version 2.0 (the "License"); you may not use this file except 
 *  in compliance with the License. You may obtain a copy of the 
 *  License at
 * 
 *       http://www.apache.org/licenses/LICENSE-2.0
 * 
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */
package com.graphhopper.routing;

import com.graphhopper.routing.edgebased.EdgeAStar;
import com.graphhopper.routing.util.*;
import com.graphhopper.storage.Graph;
import com.graphhopper.util.Helper;
import org.junit.Test;

import java.io.IOException;

import static org.junit.Assert.assertEquals;

/**
 * 
 * @author Karl Hübner
 */
public class EdgeAStarTest extends AbstractRoutingAlgorithmTester
{

    @Override
    public AlgorithmPreparation prepareGraph( Graph g,
            final FlagEncoder encoder, final Weighting calc )
    {
        return new NoOpAlgorithmPreparation()
        {
            @Override
            public RoutingAlgorithm createAlgo()
            {
                return new EdgeAStar(_graph, encoder, calc);
            }
        }.setGraph(g);
    }

    @Test
    public void testPerformance() throws IOException
    {
        //the edge based version of astar needs more time because we have much more edges to traverse
        //TODO speed improvements
    }

    @Test
    public void testCalcWithTurnRestrictions_PTurnInShortestPath()
    {
        Graph graph = createTestGraphPTurn(createTurnCostsGraph());
        Path p1 = prepareGraph(graph, carEncoder, new FastestWeighting(carEncoder)).createAlgo()
                .calcPath(3, 0);
        assertEquals(Helper.createTList(3, 5, 8, 9, 10, 5, 6, 7, 0), p1.calcNodes());
        assertEquals(p1.toString(), 26, p1.getDistance(), 1e-6);
    }
}
