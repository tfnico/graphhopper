/*
 *  Licensed to GraphHopper and Peter Karich under one or more contributor
 *  license agreements. See the NOTICE file distributed with this work for 
 *  additional information regarding copyright ownership.
 * 
 *  GraphHopper licenses this file to you under the Apache License, 
 *  Version 2.0 (the "License"); you may not use this file except in 
 *  compliance with the License. You may obtain a copy of the License at
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

import com.graphhopper.routing.util.*;
import com.graphhopper.storage.*;
import com.graphhopper.storage.index.LocationIndex;
import com.graphhopper.storage.index.LocationIndexTree;
import com.graphhopper.storage.index.LocationIndexTreeSC;
import com.graphhopper.storage.index.QueryResult;
import com.graphhopper.util.*;
import gnu.trove.list.TIntList;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.*;

/**
 *
 * @author Peter Karich
 */
public abstract class AbstractRoutingAlgorithmTester
{
    // problem is: matrix graph is expensive to create to cache it in a static variable
    private static Graph matrixGraph;
    protected TurnCostCalculation turnIgnore = new DefaultTurnCostsCalc(DefaultTurnCostsCalc.MODE_IGNORE_RESTRICTIONS);
    protected static EncodingManager encodingManager = new EncodingManager("CAR,FOOT");
    protected CarFlagEncoder carEncoder = (CarFlagEncoder) encodingManager.getEncoder("CAR");
    protected FootFlagEncoder footEncoder = (FootFlagEncoder) encodingManager.getEncoder("FOOT");

    protected Graph createGraph()
    {
        return new GraphBuilder(encodingManager).create();
    }

    protected GraphTurnCosts createTurnCostsGraph() {
        return new GraphBuilder(encodingManager).turnCostsGraphCreate();
    }

    public AlgorithmPreparation prepareGraph( Graph g )
    {
        return prepareGraph(g, carEncoder, new ShortestWeighting());
    }

    public abstract AlgorithmPreparation prepareGraph( Graph g, FlagEncoder encoder, Weighting w );

    @Test
    public void testCalcShortestPath()
    {
        Graph graph = createTestGraph();
        Path p = prepareGraph(graph).createAlgo().calcPath(0, 7);
        assertEquals(p.toString(), 13, p.getDistance(), 1e-4);
        assertEquals(p.toString(), 5, p.calcNodes().size());
    }

    // see calc-fastest-graph.svg
    @Test
    public void testCalcFastestPath()
    {
        Graph graphShortest = createGraph();
        initDirectedAndDiffSpeed(graphShortest);
        Path p1 = prepareGraph(graphShortest, carEncoder, new ShortestWeighting()).createAlgo().calcPath(0, 3);
        assertEquals(Helper.createTList(0, 1, 5, 2, 3), p1.calcNodes());
        assertEquals(p1.toString(), 24000, p1.getDistance(), 1e-6);
        assertEquals(p1.toString(), 8640 * 1000, p1.getMillis());

        Graph graphFastest = createGraph();
        initDirectedAndDiffSpeed(graphFastest);
        Path p2 = prepareGraph(graphFastest, carEncoder, new FastestWeighting(carEncoder)).createAlgo().calcPath(0, 3);
        assertEquals(Helper.createTList(0, 4, 6, 7, 5, 3), p2.calcNodes());
        assertEquals(p2.toString(), 31000, p2.getDistance(), 1e-6);
        assertEquals(p2.toString(), 5580 * 1000, p2.getMillis());
    }

    public static void initNodes(Graph graph, int nodes) {
        for (int i = 0; i < nodes; i++) {
            graph.setNode(i, 0, 0);
        }
    }

    // 0-1-2-3
    // |/|/ /|
    // 4-5-- |
    // |/ \--7
    // 6----/
    void initDirectedAndDiffSpeed( Graph graph )
    {
        initNodes(graph, 8);
        graph.edge(0, 1).setDistance(7000).setFlags(carEncoder.setProperties(10, true, false));
        graph.edge(0, 4).setDistance(5000).setFlags(carEncoder.setProperties(20, true, false));

        graph.edge(1, 4).setDistance(7000).setFlags(carEncoder.setProperties(10, true, true));
        graph.edge(1, 5).setDistance(7000).setFlags(carEncoder.setProperties(10, true, true));
        graph.edge(1, 2).setDistance(20000).setFlags(carEncoder.setProperties(10, true, true));

        graph.edge(5, 2).setDistance(5000).setFlags(carEncoder.setProperties(10, true, false));
        graph.edge(2, 3).setDistance(5000).setFlags(carEncoder.setProperties(10, true, false));

        graph.edge(5, 3).setDistance(11000).setFlags(carEncoder.setProperties(20, true, false));
        graph.edge(3, 7).setDistance(7000).setFlags(carEncoder.setProperties(10, true, false));

        graph.edge(4, 6).setDistance(5000).setFlags(carEncoder.setProperties(20, true, false));
        graph.edge(5, 4).setDistance(7000).setFlags(carEncoder.setProperties(10, true, false));

        graph.edge(5, 6).setDistance(7000).setFlags(carEncoder.setProperties(10, true, false));
        graph.edge(7, 5).setDistance(5000).setFlags(carEncoder.setProperties(20, true, false));

        graph.edge(6, 7).setDistance(5000).setFlags(carEncoder.setProperties(20, true, true));
    }

    @Test
    public void testCalcFootPath()
    {
        Graph graphShortest = createGraph();
        initFootVsCar(graphShortest);
        Path p1 = prepareGraph(graphShortest, footEncoder, new ShortestWeighting()).createAlgo().calcPath(0, 7);
        assertEquals(p1.toString(), 17000, p1.getDistance(), 1e-6);
        assertEquals(p1.toString(), 12240 * 1000, p1.getMillis());
        assertEquals(Helper.createTList(0, 4, 5, 7), p1.calcNodes());
    }

    void initFootVsCar( Graph graph )
    {
        initNodes(graph, 8);
        graph.edge(0, 1).setDistance(7000).setFlags(footEncoder.setProperties(5, true, true) | carEncoder.setProperties(10, true, false));
        graph.edge(0, 4).setDistance(5000).setFlags(footEncoder.setProperties(5, true, true) | carEncoder.setProperties(20, true, false));

        graph.edge(1, 4).setDistance(7000).setFlags(carEncoder.setProperties(10, true, true));
        graph.edge(1, 5).setDistance(7000).setFlags(carEncoder.setProperties(10, true, true));
        graph.edge(1, 2).setDistance(20000).setFlags(footEncoder.setProperties(5, true, true) | carEncoder.setProperties(10, true, true));

        graph.edge(5, 2).setDistance(5000).setFlags(carEncoder.setProperties(10, true, false));
        graph.edge(2, 3).setDistance(5000).setFlags(footEncoder.setProperties(5, true, true) | carEncoder.setProperties(10, true, false));

        graph.edge(5, 3).setDistance(11000).setFlags(carEncoder.setProperties(20, true, false));
        graph.edge(3, 7).setDistance(7000).setFlags(footEncoder.setProperties(5, true, true) | carEncoder.setProperties(10, true, false));

        graph.edge(4, 6).setDistance(5000).setFlags(carEncoder.setProperties(20, true, false));
        graph.edge(5, 4).setDistance(7000).setFlags(footEncoder.setProperties(5, true, true) | carEncoder.setProperties(10, true, false));

        graph.edge(5, 6).setDistance(7000).setFlags(carEncoder.setProperties(10, true, false));
        graph.edge(7, 5).setDistance(5000).setFlags(footEncoder.setProperties(5, true, true) | carEncoder.setProperties(20, true, false));

        graph.edge(6, 7).setDistance(5000).setFlags(carEncoder.setProperties(20, true, true));
    }

    void initTurnRestrictionsFootVsCar(GraphTurnCosts graph) {
        graph.setNode(8, 0, 0);

        initFootVsCar(graph);

        //TODO (tfnico): Not sure how to test varying speeds any more, used to be embedded in the encoder flags
        int edge_5_8 = graph.edge(5, 8, 3000, true).getEdge();// footEncoder.flags(5, true) | carEncoder.flags(20, false)).getEdge();
        graph.edge(8, 5, 3000, true).getEdge();//footEncoder.flags(5, true));

        int edge_2_8 = graph.edge(2, 8, 4000, true).getEdge();//footEncoder.flags(5, true) | carEncoder.flags(20, false)).getEdge();
        graph.edge(8, 2, 4000, true).getEdge();//footEncoder.flags(5, true) | carEncoder.flags(20, false));

        int edge_8_3 = graph.edge(8, 3, 1000, true).getEdge();//footEncoder.flags(5, true) | carEncoder.flags(20, false)).getEdge();
        graph.edge(3, 8, 1000, true).getEdge();//footEncoder.flags(5, true) | carEncoder.flags(20, false));

        EdgeExplorer e1 = graph.createEdgeExplorer(new EdgeFilter() {
            @Override
            public boolean accept(EdgeIteratorState edgeIterState) {
                return edgeIterState.getBaseNode() == 5 && edgeIterState.getAdjNode() == 2;
            }
        });
        EdgeIterator edgeIterator1 = e1.setBaseNode(5);
        assertTrue(edgeIterator1.next());

        EdgeExplorer e2 = graph.createEdgeExplorer(new EdgeFilter() {
            @Override
            public boolean accept(EdgeIteratorState edgeIterState) {
                return edgeIterState.getBaseNode() == 2 && edgeIterState.getAdjNode() == 3;
            }
        });
        EdgeIterator edgeIterator2 = e2.setBaseNode(2);
        assertTrue(edgeIterator2.next());

        //add some restrictions
        graph.turnCosts(8, edge_5_8, edge_8_3, TurnCostEncoder.restriction());
        graph.turnCosts(8, edge_2_8, edge_8_3, TurnCostEncoder.restriction());
        graph.turnCosts(2, edgeIterator1.getEdge(), edgeIterator2.getEdge(), TurnCostEncoder.restriction());
    }

    @Test
    public void testCalcWithTurnRestrictions_CarPath_turnRestrictions() {
        GraphTurnCosts graph = createTurnCostsGraph();
        initTurnRestrictionsFootVsCar(graph);
        Path p1 = prepareGraph(graph, carEncoder, new ShortestWeighting()).createAlgo().calcPath(0, 3);

        assertEquals(Helper.createTList(0, 1, 5, 3), p1.calcNodes());
        assertEquals(p1.toString(), 7020, (int)p1.getWeight());

    }

    @Test
    public void testCalcWithTurnRestrictions_CarPath_ignoreTurnRestrictions() {
        GraphTurnCosts graph = createTurnCostsGraph();
        initTurnRestrictionsFootVsCar(graph);
        Path p1 = prepareGraph(graph, carEncoder, new ShortestWeighting()).createAlgo().turnCosts(turnIgnore).calcPath(0, 3);
        assertEquals(Helper.createTList(0, 1, 5, 8, 3), p1.calcNodes());
        assertEquals(p1.toString(), 5760, (int)p1.getWeight());
    }

    @Test
    public void testCalcWithTurnRestrictions_FootPath_noTurnRestrictions() {
        GraphTurnCosts graph = createTurnCostsGraph();
        initFootVsCar(graph);
        initTurnRestrictionsFootVsCar(graph);
        Path p1 = prepareGraph(graph, footEncoder, new ShortestWeighting()).createAlgo().calcPath(0, 3);
        assertEquals(p1.toString(), 11520, (int)p1.getWeight());
        assertEquals(Helper.createTList(0, 4, 5, 8, 3), p1.calcNodes());
    }

    protected Graph createTestGraph()
    {
        Graph graph = createGraph();
        createTestGraph(graph);
        return graph;
    }

    // see test-graph.svg !
    protected Graph createTestGraph(Graph graph) {
        initNodes(graph, 8);
        graph.edge(0, 1, 7, true);
        graph.edge(0, 4, 6, true);

        graph.edge(1, 4, 2, true);
        graph.edge(1, 5, 8, true);
        graph.edge(1, 2, 2, true);

        graph.edge(2, 5, 5, true);
        graph.edge(2, 3, 2, true);

        graph.edge(3, 5, 2, true);
        graph.edge(3, 7, 10, true);

        graph.edge(4, 6, 4, true);
        graph.edge(4, 5, 7, true);

        graph.edge(5, 6, 2, true);
        graph.edge(5, 7, 1, true);

        graph.edge(6, 7, 5, true);
        return graph;
    }

    @Test
    public void testCalcIfEmptyWay()
    {
        Graph graph = createTestGraph();
        Path p = prepareGraph(graph).createAlgo().calcPath(0, 0);
        assertEquals(p.toString(), 0, p.calcNodes().size());
        assertEquals(p.toString(), 0, p.getDistance(), 1e-4);
    }

    @Test
    public void testNoPathFound()
    {
        Graph graph = createGraph();
        assertFalse(prepareGraph(graph).createAlgo().calcPath(0, 1).isFound());

        // two disconnected areas
        graph.edge(0, 1, 7, true);

        graph.edge(5, 6, 2, true);
        graph.edge(5, 7, 1, true);
        graph.edge(5, 8, 1, true);
        graph.edge(7, 8, 1, true);
        RoutingAlgorithm algo = prepareGraph(graph).createAlgo();
        assertFalse(algo.calcPath(0, 5).isFound());
        // assertEquals(4, algo.getVisitedNodes());

        // disconnected as directed graph
        graph = createGraph();
        graph.edge(0, 1, 1, false);
        graph.edge(0, 2, 1, true);
        algo = prepareGraph(graph).createAlgo();
        assertFalse(algo.calcPath(1, 2).isFound());
    }

    @Test
    public void testWikipediaShortestPath()
    {
        Graph graph = createWikipediaTestGraph();
        Path p = prepareGraph(graph).createAlgo().calcPath(0, 4);
        assertEquals(p.toString(), 20, p.getDistance(), 1e-4);
        assertEquals(p.toString(), 4, p.calcNodes().size());
    }

    @Test
    public void testCalcIf1EdgeAway()
    {
        Graph graph = createTestGraph();
        Path p = prepareGraph(graph).createAlgo().calcPath(1, 2);
        assertEquals(Helper.createTList(1, 2), p.calcNodes());
        assertEquals(p.toString(), 2, p.getDistance(), 1e-4);
    }

//    //TODO routing algorithms currently do not support P-turns (see http://www.easts.info/on-line/journal_06/1426.pdf) 
//    @Test public void testCalcWithTurnRestrictions_PTurnInShortestPath() {
//        Graph graph = createTestGraphPTurn(createTurnCostsGraph());
//        Path p1 = prepareGraph(graph, new FastestCalc(carEncoder), carEncoder).createAlgo().calcPath(3, 0);
//        assertEquals(Helper.createTList(3, 5, 8, 9, 10, 5, 6, 7, 0), p1.calcNodes());
//        assertEquals(p1.toString(), 26, p1.distance(), 1e-6);
//    }
    /*
     * 0---------1
     *  \        |
     *   7->8--9 |
     *   |  |  | |
     *   6--5-10 |
     *      |    |
     * 4----3----2
     * 
     * Outer links are expensive
     * Inner links are cheap
     * Turn restriction: (3,5) to (5,6)
     * One way: (7,8)
     * 
     * Shortest path from 3 to 0 should be 3,5,8,9,10,5,6,7,0 (which contains a p-turn)  
     * 
     */
    protected Graph createTestGraphPTurn(GraphTurnCosts graph) {
        initNodes(graph, 11);
        //outer lanes
        graph.edge(0, 1, 10, true);
        graph.edge(1, 2, 20, true);
        graph.edge(2, 3, 5, true);
        graph.edge(3, 4, 5, true);

        //inner lanes

        graph.edge(5, 8, 3, true);
        int e56 = graph.edge(5, 6, 3, true).getEdge();
        int e510 = graph.edge(5, 10, 3, true).getEdge();
        graph.edge(8, 9, 3, true);
        graph.edge(9, 10, 3, true);
        graph.edge(6, 7, 3, false);
        graph.edge(7, 8, 3, false);

        //connections
        int e35= graph.edge(3, 5, 4, true).getEdge();
        graph.edge(7, 0, 4, true);
        
        //turn costs
        graph.turnCosts(5, e35, e56, TurnCostEncoder.restriction());
        graph.turnCosts(5, e35, e510, TurnCostEncoder.costs(5, 5));

        return graph;
    }
    
    // see wikipedia-graph.svg !
    protected Graph createWikipediaTestGraph()
    {
        Graph graph = createGraph();
        initNodes(graph, 6);
        graph.edge(0, 1, 7, true);
        graph.edge(0, 2, 9, true);
        graph.edge(0, 5, 14, true);
        graph.edge(1, 2, 10, true);
        graph.edge(1, 3, 15, true);
        graph.edge(2, 5, 2, true);
        graph.edge(2, 3, 11, true);
        graph.edge(3, 4, 6, true);
        graph.edge(4, 5, 9, true);
        return graph;
    }

    // 0-1-2-3-4
    // |     / |
    // |    8  |
    // \   /   |
    //  7-6----5
    public static void initBiGraph( Graph graph )
    {
        graph.edge(0, 1, 100, true);
        graph.edge(1, 2, 1, true);
        graph.edge(2, 3, 1, true);
        graph.edge(3, 4, 1, true);
        graph.edge(4, 5, 1, true);
        graph.edge(5, 6, 1, true);
        graph.edge(6, 7, 1, true);
        graph.edge(7, 0, 1, true);
        graph.edge(3, 8, 1, true);
        graph.edge(8, 6, 1, true);

        // we need lat,lon for edge precise queries because the distances of snapped point 
        // to adjacent nodes is calculated from lat,lon of the necessary points
        updateDistancesFor(graph, 0, 0.001, 0);
        updateDistancesFor(graph, 1, 0.100, 0.0005);
        updateDistancesFor(graph, 2, 0.010, 0.0010);
        updateDistancesFor(graph, 3, 0.001, 0.0011);
        updateDistancesFor(graph, 4, 0.001, 0.00111);

        updateDistancesFor(graph, 8, 0.0005, 0.0011);

        updateDistancesFor(graph, 7, 0, 0);
        updateDistancesFor(graph, 6, 0, 0.001);
        updateDistancesFor(graph, 5, 0, 0.004);
    }

    private static final DistanceCalc distCalc = new DistanceCalcEarth();

    private static void updateDistancesFor( Graph g, int node, double lat, double lon )
    {
        g.setNode(node, lat, lon);
        EdgeIterator iter = g.createEdgeExplorer().setBaseNode(node);
        while (iter.next())
        {
            int adj = iter.getAdjNode();
            double adjLat = g.getLatitude(adj);
            double adjLon = g.getLongitude(adj);
            iter.setDistance(distCalc.calcDist(lat, lon, adjLat, adjLon));
        }
    }

    @Test
    public void testBidirectional()
    {
        Graph graph = createGraph();
        initBiGraph(graph);

        // PrepareTowerNodesShortcutsTest.printEdges((LevelGraph) graph);
        Path p = prepareGraph(graph).createAlgo().calcPath(0, 4);
        // PrepareTowerNodesShortcutsTest.printEdges((LevelGraph) graph);
        assertEquals(p.toString(), Helper.createTList(0, 7, 6, 8, 3, 4), p.calcNodes());
        assertEquals(p.toString(), 335.77, p.getDistance(), 1e-2);

        p = prepareGraph(graph).createAlgo().calcPath(1, 2);
        // the other way around is even larger as 0-1 is already 11008.452
        assertEquals(p.toString(), Helper.createTList(1, 2), p.calcNodes());
        assertEquals(p.toString(), 10007.679, p.getDistance(), 1e-4);
    }

    // 1-2-3-4-5
    // |     / |
    // |    9  |
    // \   /   /
    //  8-7-6-/
    @Test
    public void testBidirectional2()
    {
        Graph graph = createGraph();
        initNodes(graph, 9);
        graph.edge(0, 1, 100, true);
        graph.edge(1, 2, 1, true);
        graph.edge(2, 3, 1, true);
        graph.edge(3, 4, 1, true);
        graph.edge(4, 5, 20, true);
        graph.edge(5, 6, 10, true);
        graph.edge(6, 7, 5, true);
        graph.edge(7, 0, 5, true);
        graph.edge(3, 8, 20, true);
        graph.edge(8, 6, 20, true);

        Path p = prepareGraph(graph).createAlgo().calcPath(0, 4);
        assertEquals(p.toString(), 40, p.getDistance(), 1e-4);
        assertEquals(p.toString(), 5, p.calcNodes().size());
        assertEquals(Helper.createTList(0, 7, 6, 5, 4), p.calcNodes());
    }

    @Test
    public void testRekeyBugOfIntBinHeap()
    {
        // using DijkstraSimple + IntBinHeap then rekey loops endlessly
        Path p = prepareGraph(getMatrixGraph()).createAlgo().calcPath(36, 91);
        assertEquals(12, p.calcNodes().size());

        TIntList list = p.calcNodes();
        if (!Helper.createTList(36, 46, 56, 66, 76, 86, 85, 84, 94, 93, 92, 91).equals(list)
                && !Helper.createTList(36, 46, 56, 66, 76, 86, 85, 84, 83, 82, 92, 91).equals(list))
        {
            assertTrue("wrong locations: " + list.toString(), false);
        }
        assertEquals(66f, p.getDistance(), 1e-3);
    }

	@Test
    public void testBug1()
    {
        Path p = prepareGraph(getMatrixGraph()).createAlgo().calcPath(34, 36);
        assertEquals(Helper.createTList(34, 35, 36), p.calcNodes());
        assertEquals(3, p.calcNodes().size());
        assertEquals(17, p.getDistance(), 1e-5);
    }

    @Test
    public void testCorrectWeight()
    {
        Path p = prepareGraph(getMatrixGraph()).createAlgo().calcPath(45, 72);
        assertEquals(Helper.createTList(45, 44, 54, 64, 74, 73, 72), p.calcNodes());
        assertEquals(38f, p.getDistance(), 1e-3);
    }

    @Test
    public void testCannotCalculateSP()
    {
        Graph graph = createGraph();
        initNodes(graph, 2);
        graph.edge(0, 1, 1, false);
        graph.edge(1, 2, 1, false);

        Path p = prepareGraph(graph).createAlgo().calcPath(0, 2);
        assertEquals(p.toString(), 3, p.calcNodes().size());
    }

    @Test
    public void testDirectedGraphBug1()
    {
        Graph graph = createGraph();
        initNodes(graph, 5);

        graph.edge(0, 1, 3, false);
        graph.edge(1, 2, 2.99, false);

        graph.edge(0, 3, 2, false);
        graph.edge(3, 4, 3, false);
        graph.edge(4, 2, 1, false);

        Path p = prepareGraph(graph).createAlgo().calcPath(0, 2);
        assertEquals(Helper.createTList(0, 1, 2), p.calcNodes());
        assertEquals(p.toString(), 5.99, p.getDistance(), 1e-4);
        assertEquals(p.toString(), 3, p.calcNodes().size());
    }

    @Test
    public void testDirectedGraphBug2()
    {
        Graph graph = createGraph();
        initNodes(graph, 5);

        graph.edge(0, 1, 1, false);
        graph.edge(1, 2, 1, false);
        graph.edge(2, 3, 1, false);

        graph.edge(3, 1, 4, true);

        Path p = prepareGraph(graph).createAlgo().calcPath(0, 3);
        assertEquals(Helper.createTList(0, 1, 2, 3), p.calcNodes());
    }

    // a-b-0-c-1
    // |   |  _/\
    // |  /  /  |
    // d-2--3-e-4
    @Test
    public void testWithCoordinates()
    {
        Graph graph = createGraph();
        graph.setNode(0, 0, 2);
        graph.setNode(1, 0, 3.5);
        graph.setNode(2, 1, 1);
        graph.setNode(3, 1.5, 2.5);
        graph.setNode(4, 0.5, 4.5);

        graph.edge(0, 1, 2, true).setWayGeometry(Helper.createPointList(0, 3));
        graph.edge(2, 3, 2, true);
        graph.edge(3, 4, 2, true).setWayGeometry(Helper.createPointList(1, 3.5));

        graph.edge(0, 2, 0.8, true).setWayGeometry(Helper.createPointList(0, 1.6, 0, 0, 1, 0));
        graph.edge(0, 2, 1.2, true);
        graph.edge(1, 3, 1.3, true);
        graph.edge(1, 4, 1, true);

        AlgorithmPreparation prepare = prepareGraph(graph);
        Path p = prepare.createAlgo().calcPath(4, 0);
        assertEquals(Helper.createTList(4, 1, 0), p.calcNodes());
        assertEquals(Helper.createPointList(0.5, 4.5, 0, 3.5, 0, 3, 0, 2), p.calcPoints());
        assertEquals(291110, p.calcPoints().calcDistance(new DistanceCalcEarth()), 1);

        // PrepareTowerNodesShortcutsTest.printEdges((LevelGraph) graph);
        p = prepare.createAlgo().calcPath(2, 1);
        // System.out.println(p.toDetailsString());
        assertEquals(Helper.createTList(2, 0, 1), p.calcNodes());
        assertEquals(Helper.createPointList(1, 1, 1, 0, 0, 0, 0, 1.6, 0, 2, 0, 3, 0, 3.5), p.calcPoints());
        assertEquals(611555, p.calcPoints().calcDistance(new DistanceCalcEarth()), 1);
    }

    @Test
    public void testViaEdges_BiGraph()
    {
        Graph graph = createGraph();
        initBiGraph(graph);

        // 0-7 to 4-3
        Path p = calcPathViaQuery(graph, 0.0009, 0, 0.001, 0.001105);
        assertEquals(p.toString(), Helper.createTList(10, 7, 6, 8, 3, 9), p.calcNodes());
        assertEquals(p.toString(), 324.11, p.getDistance(), 1e-2);

        // 0-1 to 2-3
        p = calcPathViaQuery(graph, 0.001, 0.0001, 0.010, 0.0011);
        assertEquals(p.toString(), Helper.createTList(10, 0, 7, 6, 8, 3, 9), p.calcNodes());
        assertEquals(p.toString(), 1335.42, p.getDistance(), .2);
    }

    @Test
    public void testViaEdges_FromEqualsTo()
    {
        Graph graph = createTestGraph();
        Path p = calcPath(graph, 0, 1, 0, 1);
        // or one node would be acceptable
        assertEquals(Helper.createTList(8, 9), p.calcNodes());
        assertEquals(p.toString(), 0, p.getDistance(), 1e-4);
    }

    @Test
    public void testViaEdges_WithCoordinates()
    {
        Graph graph = createTestGraph();
        Path p = calcPath(graph, 0, 1, 2, 3);
        assertEquals(Helper.createTList(9, 1, 2, 8), p.calcNodes());
        assertEquals(p.toString(), 2, p.getDistance(), 1e-4);
    }

    @Test
    public void testViaEdges_SpecialCases()
    {
        Graph graph = createGraph();
        // 0->1\
        // |    2
        // 4<-3/
        graph.edge(0, 1, 7, false);
        graph.edge(1, 2, 7, true);
        graph.edge(2, 3, 7, true);
        graph.edge(3, 4, 7, false);
        graph.edge(4, 0, 7, true);

        updateDistancesFor(graph, 4, 0, 0);
        updateDistancesFor(graph, 0, 0.00010, 0);
        updateDistancesFor(graph, 1, 0.00010, 0.0001);
        updateDistancesFor(graph, 2, 0.00005, 0.00015);
        updateDistancesFor(graph, 3, 0, 0.0001);

        // 0-1 to 3-4
        Path p = calcPathViaQuery(graph, 0.00010, 0.00001, 0, 0.00009);
        assertEquals(Helper.createTList(6, 1, 2, 3, 5), p.calcNodes());
        assertEquals(p.toString(), 26.81, p.getDistance(), .1);

        // overlapping edges: 2-3 and 3-2
        p = calcPathViaQuery(graph, 0.000049, 0.00015, 0.00001, 0.0001);
        assertEquals(Helper.createTList(5, 6), p.calcNodes());
        assertEquals(p.toString(), 7, p.getDistance(), .1);

        // 'from' and 'to' edge share one node '2': 1-2 to 3-2
        p = calcPathViaQuery(graph, 0.00009, 0.00011, 0.00001, 0.00011);
        assertEquals(p.toString(), Helper.createTList(6, 2, 5), p.calcNodes());
        assertEquals(p.toString(), 12.57, p.getDistance(), .1);
    }

    // Problem: for contraction hierarchy we cannot easily select egdes by nodes as some edges are skipped
    Path calcPathViaQuery( Graph graph, double fromLat, double fromLon, double toLat, double toLon )
    {
        LocationIndex index;
        if (graph instanceof LevelGraph)
            index = new LocationIndexTreeSC((LevelGraph) graph, new RAMDirectory());
        else
            index = new LocationIndexTree(graph, new RAMDirectory());

        index.prepareIndex();
        QueryResult from = index.findClosest(fromLat, fromLon, EdgeFilter.ALL_EDGES);
        QueryResult to = index.findClosest(toLat, toLon, EdgeFilter.ALL_EDGES);
        return prepareGraph(graph).createAlgo().calcPath(from, to);
    }

    Path calcPath( Graph graph, int fromNode1, int fromNode2, int toNode1, int toNode2 )
    {
        // lookup two edges: fromNode1-fromNode2 and toNode1-toNode2        
        QueryResult from = newQR(graph, fromNode1, fromNode2);
        QueryResult to = newQR(graph, toNode1, toNode2);
        return prepareGraph(graph).createAlgo().calcPath(from, to);
    }

    /**
     * Creates query result on edge (node1-node2) very close to node1.
     */
    QueryResult newQR( Graph graph, int node1, int node2 )
    {
        EdgeIteratorState edge = GHUtility.getEdge(graph, node1, node2);
        if (edge == null)
            throw new IllegalStateException("edge not found? " + node1 + "-" + node2);

        double lat = graph.getLatitude(edge.getBaseNode());
        double lon = graph.getLongitude(edge.getBaseNode());
        double latAdj = graph.getLatitude(edge.getAdjNode());
        double lonAdj = graph.getLongitude(edge.getAdjNode());
        // calculate query point near the base node but not directly on it!
        QueryResult res = new QueryResult(lat + (latAdj - lat) * .1, lon + (lonAdj - lon) * .1);
        res.setClosestNode(edge.getBaseNode());
        res.setClosestEdge(edge);
        res.setWayIndex(0);
        res.setSnappedPosition(QueryResult.Position.EDGE);
        res.calcSnappedPoint(distCalc);
        return res;
    }

    public Graph getMatrixGraph()
    {
        return getMatrixAlikeGraph();
    }

    public static Graph getMatrixAlikeGraph()
    {
        if (matrixGraph == null)
            matrixGraph = createMatrixAlikeGraph();
        return matrixGraph;
    }

    private static Graph createMatrixAlikeGraph()
    {
        int WIDTH = 10;
        int HEIGHT = 15;
        Graph tmp = new GraphBuilder(encodingManager).create();
        int[][] matrix = new int[WIDTH][HEIGHT];
        int counter = 0;
        Random rand = new Random(12);
        boolean print = false;
        for (int h = 0; h < HEIGHT; h++)
        {
            if (print)
            {
                for (int w = 0; w < WIDTH; w++)
                {
                    System.out.print(" |\t           ");
                }
                System.out.println();
            }

            for (int w = 0; w < WIDTH; w++)
            {
                matrix[w][h] = counter++;
                if (h > 0)
                {
                    float dist = 5 + Math.abs(rand.nextInt(5));
                    if (print)
                        System.out.print(" " + (int) dist + "\t           ");

                    tmp.edge(matrix[w][h], matrix[w][h - 1], dist, true);
                }
            }
            if (print)
            {
                System.out.println();
                if (h > 0)
                {
                    for (int w = 0; w < WIDTH; w++)
                    {
                        System.out.print(" |\t           ");
                    }
                    System.out.println();
                }
            }

            for (int w = 0; w < WIDTH; w++)
            {
                if (w > 0)
                {
                    float dist = 5 + Math.abs(rand.nextInt(5));
                    if (print)
                        System.out.print("-- " + (int) dist + "\t-- ");
                    tmp.edge(matrix[w][h], matrix[w - 1][h], dist, true);
                }
                if (print)
                    System.out.print("(" + matrix[w][h] + ")\t");
            }
            if (print)
                System.out.println();
        }

        return tmp;
    }
}
