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
package com.graphhopper.routing.edgebased;

import com.graphhopper.routing.Path;
import com.graphhopper.routing.util.FlagEncoder;
import com.graphhopper.routing.util.Weighting;
import com.graphhopper.storage.EdgeEntry;
import com.graphhopper.storage.Graph;
import com.graphhopper.util.EdgeExplorer;
import com.graphhopper.util.EdgeIterator;
import gnu.trove.map.TIntObjectMap;
import gnu.trove.map.hash.TIntObjectHashMap;

import java.util.PriorityQueue;

/**
 * An edge-based version of Dijkstras Algorithms. End link costs will be stored for each edge
 * instead of for each node. This is necessary when considering turn costs, but will be around three
 * times slower than classic Dijkstra.
 * 
 * @see http://www.easts.info/on-line/journal_06/1426.pdf
 * 
 *      TODO we better should reuse the code of Dijkstra instead instead of copying it. should be
 *      done later
 * 
 * @author Karl Hübner
 */
public class EdgeDijkstra extends AbstractEdgeBasedRoutingAlgorithm
{

    protected TIntObjectMap<EdgeEntry> map = new TIntObjectHashMap<EdgeEntry>();
    protected PriorityQueue<EdgeEntry> heap = new PriorityQueue<EdgeEntry>();
    protected boolean alreadyRun;
    protected int visitedNodes;
    private EdgeEntry currEdge;
    private int to;

    public EdgeDijkstra( Graph g, FlagEncoder encoder, Weighting weighting )
    {
        super(g, encoder, weighting);
    }

    @Override
    protected boolean finished() {
        return currEdge.endNode == to;
    }

    @Override
    public Path extractPath()
    {
        if ( currEdge == null || !finished() )
                    return createEmptyPath();
        return new Path(graph, flagEncoder).setEdgeEntry(currEdge).extract();
    }

    @Override
    public Path calcPath( int from, int to )
    {
        checkAlreadyRun();
        this.to = to;
        currEdge = createEdgeEntry(from, 0);
        return runAlgo();
    }

    private Path runAlgo()
    {
        EdgeExplorer explorer = outEdgeExplorer;
        while ( true )
        {
            visitedNodes++;
            if ( finished() )
                break;

            int neighborNode = currEdge.endNode;
            EdgeIterator iter = explorer.setBaseNode(neighborNode);
            while ( iter.next() )
            {
                if ( !accept(iter, currEdge) )
                    continue;

                //we need to distinguish between backward and forward direction when storing end weights
                int key = createIterKey(iter, false);

                int tmpNode = iter.getAdjNode();
                double tmpWeight = weighting.calcWeight(iter)
                        + currEdge.weight
                        + turnCostCalc.getTurnCosts(neighborNode, currEdge.edge, iter.getEdge());
                EdgeEntry nEdge = map.get(key);
                if ( nEdge == null )
                {
                    nEdge = new EdgeEntry(iter.getEdge(), tmpNode, tmpWeight);
                    nEdge.parent = currEdge;
                    map.put(key, nEdge);
                    heap.add(nEdge);
                } else if ( nEdge.weight > tmpWeight )
                {
                    heap.remove(nEdge);
                    nEdge.edge = iter.getEdge();
                    nEdge.weight = tmpWeight;
                    nEdge.parent = currEdge;
                    heap.add(nEdge);
                }
                updateShortest(nEdge, neighborNode);
            }

            if ( heap.isEmpty() )
                return null;
            currEdge = heap.poll();
            if ( currEdge == null )
                throw new AssertionError("cannot happen?");
        }
        return extractPath();
    }

    @Override
    public String getName()
    {
        return "edge-based dijkstra";
    }

    @Override
    public int getVisitedNodes()
    {
        return visitedNodes;
    }
}
