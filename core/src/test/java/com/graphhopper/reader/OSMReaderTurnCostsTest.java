package com.graphhopper.reader;

import com.graphhopper.GraphHopper;
import com.graphhopper.routing.util.EncodingManager;
import com.graphhopper.routing.util.TurnCostEncoder;
import com.graphhopper.storage.*;
import com.graphhopper.util.EdgeExplorer;
import com.graphhopper.util.EdgeIterator;
import org.junit.Test;

import static org.junit.Assert.assertEquals;

/**
 * tests, if with {@link GraphStorageTurnCosts} everything stays the same, except that turn
 * relations will be imported
 * 
 * @author Karl Hübner
 * 
 */
public class OSMReaderTurnCostsTest extends OSMReaderTest
{

    private String file6 = "test-osm5.xml";
    private EncodingManager encodingManager = new EncodingManager("CAR,FOOT");;

    @Override
    GraphStorage buildGraph( String directory, EncodingManager encodingManager )
    {
        return new GraphStorageTurnCosts(new RAMDirectory(directory, false), encodingManager);
    }

    @Test
    public void testImportTurnRestrictions()
    {
        GraphHopper hopper = new GraphHopperTest(file6).setEncodingManager(encodingManager)
                .enableTurnRestrictions().
                importOrLoad();
        GraphTurnCosts g = (GraphTurnCosts) hopper.getGraph();

        assertEquals(9, g.getNodes());

        int n1 = AbstractGraphStorageTester.getIdOf(g, 51.0, 9.0);
        int n2 = AbstractGraphStorageTester.getIdOf(g, 51.2, 9.0);
        int n3 = AbstractGraphStorageTester.getIdOf(g, 51.2, 9.1);
        int n4 = AbstractGraphStorageTester.getIdOf(g, 51.2, 9.2);
        int n5 = AbstractGraphStorageTester.getIdOf(g, 51.0, 9.2);
        int n6 = AbstractGraphStorageTester.getIdOf(g, 51.1, 9.1);
        //int n7 = AbstractGraphTester.getIdOf(g, 51.4, 9.2);
        int n8 = AbstractGraphStorageTester.getIdOf(g, 51.4, 9.1);
        int n9 = AbstractGraphStorageTester.getIdOf(g, 51.4, 9.0);

        //node3 : restriction for turn (2,3)->(3,8) and (2,3)->(3,9), since only (2,3)->(3,4) is allowed 
        //+ everything allowed from other directions, except (4,3)->(3,8) since there is a 'no_right_turn'restriction
        assertEquals(TurnCostEncoder.restriction(), g.turnCosts(n3, e(g, n2, n3), e(g, n3, n9)), 0);
        assertEquals(TurnCostEncoder.restriction(), g.turnCosts(n3, e(g, n2, n3), e(g, n3, n8)), 0);
        assertEquals(TurnCostEncoder.restriction(), g.turnCosts(n3, e(g, n4, n3), e(g, n3, n8)), 0);
        assertEquals(TurnCostEncoder.noCosts(), g.turnCosts(n3, e(g, n2, n3), e(g, n3, n4)), 0);
        assertEquals(TurnCostEncoder.noCosts(), g.turnCosts(n3, e(g, n4, n3), e(g, n3, n2)), 0);
        assertEquals(TurnCostEncoder.noCosts(), g.turnCosts(n3, e(g, n8, n3), e(g, n3, n9)), 0);
        assertEquals(TurnCostEncoder.noCosts(), g.turnCosts(n3, e(g, n8, n3), e(g, n3, n2)), 0);

        //node5 : restriction for turn (4,5)->(5,1) since there is a 'right_turn_only' restriction 
        //+ everything allowed from other directions
        assertEquals(TurnCostEncoder.noCosts(), g.turnCosts(n5, e(g, n4, n5), e(g, n5, n6)), 0);
        assertEquals(TurnCostEncoder.restriction(), g.turnCosts(n5, e(g, n4, n5), e(g, n5, n1)), 0);
        assertEquals(TurnCostEncoder.noCosts(), g.turnCosts(n5, e(g, n1, n5), e(g, n5, n4)), 0);
        assertEquals(TurnCostEncoder.noCosts(), g.turnCosts(n5, e(g, n1, n5), e(g, n5, n6)), 0);
        assertEquals(TurnCostEncoder.noCosts(), g.turnCosts(n5, e(g, n6, n5), e(g, n5, n4)), 0);
        assertEquals(TurnCostEncoder.noCosts(), g.turnCosts(n5, e(g, n6, n5), e(g, n5, n1)), 0);

        //node1 : restriction for turn (6,1)->(1,5) since there is a 'no_u_turn' restriction 
        //+ everything allowed from other directions
        assertEquals(TurnCostEncoder.restriction(), g.turnCosts(n1, e(g, n6, n1), e(g, n1, n5)), 0);
        assertEquals(TurnCostEncoder.noCosts(), g.turnCosts(n1, e(g, n6, n1), e(g, n1, n2)), 0);
        assertEquals(TurnCostEncoder.noCosts(), g.turnCosts(n1, e(g, n5, n1), e(g, n1, n2)), 0);
        assertEquals(TurnCostEncoder.noCosts(), g.turnCosts(n1, e(g, n5, n1), e(g, n1, n6)), 0);
        assertEquals(TurnCostEncoder.noCosts(), g.turnCosts(n1, e(g, n2, n1), e(g, n1, n5)), 0);
        assertEquals(TurnCostEncoder.noCosts(), g.turnCosts(n1, e(g, n2, n1), e(g, n1, n6)), 0);
    }

    private int e( GraphTurnCosts graph, int nodeStart, int nodeEnd )
    {
        EdgeExplorer expl = graph.createEdgeExplorer();
        EdgeIterator edgeIterator = expl.setBaseNode(nodeStart);
        while ( edgeIterator.next() )
        {
            if ( edgeIterator.getAdjNode() == nodeEnd )
            {
                return edgeIterator.getEdge();
            }
        }
        return EdgeIterator.NO_EDGE;
    }
}
