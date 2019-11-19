/* k Shortest Paths in O(E*log V + L*k*log k) time (L is path length)
   Implemented by Jon Graehl (jongraehl@earthling.net)
   Following David Eppstein's "Finding the k Shortest Paths" March 31, 1997 draft
   (http://www.ics.uci.edu/~eppstein/
    http://www.ics.uci.edu/~eppstein/pubs/p-kpath.ps)
   */

#include <sstream>

namespace graehl
{
    
#include "graph.h"

using namespace std;

struct pGraphArc {
  GraphArc *p;
  GraphArc * operator ->() const { return p; }
  operator GraphArc *() const { return p; }
};

int operator < (const pGraphArc l, const pGraphArc r) {
  return l->weight > r->weight;
}

// pGraphArc must be used rather than GraphArc * because in order to overload operator < "`operator <(const GraphArc *, const GraphArc *)' must have an argument of class or enumerated type"

struct GraphHeap {
  static List<GraphHeap *> usedBlocks;
  static GraphHeap *freeList;
  static const int newBlocksize;
  GraphHeap *left, *right;          // for balanced heap  
  int nDescend;
  GraphArc *arc;                // data at each vertex, or cross edge 
  pGraphArc *arcHeap;                // binary heap of sidetracks originating from a state
  int arcHeapSize;
  GraphHeap ()
  {
    arcHeap = NULL;
  }
  void *operator new(size_t)
  {
    GraphHeap *ret, *max;
    if (freeList) {
      ret = freeList;
      freeList = freeList->left;
      return ret;
    }
    freeList = (GraphHeap *)::operator new(newBlocksize * sizeof(GraphHeap));
    usedBlocks.push(freeList);
    freeList->left = NULL;
    max = freeList + newBlocksize - 1;
    for ( ret = freeList++; freeList < max ; ret = freeList++ )
      freeList->left = ret;
    return freeList--;
  }
  void operator delete(void *p) 
  {
    GraphHeap *e = (GraphHeap *)p;
    e->left = freeList;
    freeList = e;
  }
  static void freeAll()
  {
    while ( usedBlocks.notEmpty() ) {
      ::operator delete((void *)usedBlocks.top());
      usedBlocks.pop();
    }
    freeList = NULL;
  }
};

template<>
Node<GraphHeap *> *Node<GraphHeap *>::freeList = NULL;
template<>
const int Node<GraphHeap *>::newBlocksize = 64;

template<>
Node<GraphArc *> *Node<GraphArc *>::freeList = NULL;
template<>
const int Node<GraphArc *>::newBlocksize = 64;

template<>
Node<List<GraphArc *> > *Node<List<GraphArc *> >::freeList = NULL;
template<>
const int Node<List<GraphArc *> >::newBlocksize = 64;


List<GraphHeap *> GraphHeap::usedBlocks;
GraphHeap * GraphHeap::freeList = NULL;
const int GraphHeap::newBlocksize = 64;

int operator < (const GraphHeap &l, const GraphHeap &r) {
  return l.arc->weight > r.arc->weight;
}

struct EdgePath {
  GraphHeap *node;
  int heapPos;                        // -1 if arc is GraphHeap.arc
  EdgePath *last;
  float weight;
};

int operator < (const EdgePath &l, const EdgePath &r) {
  return l.weight > r.weight;
}

class Graehl
{
private:
    int source;
    int dest;
    bool ready;
    Graph graph;
    int nStates;
    List<List<GraphArc *> > *paths;
    ListIter<List<GraphArc *> > *insertHere;
    float *dist;
    Graph shortPathGraph;
    GraphState *shortPathTree;
    ListIter<GraphArc *> *path;
    List<List<GraphArc *> > graphPaths;
    Graph revPathTree;
    Graph sidetracks;
    GraphHeap **pathGraph;
    bool *visited;
    EdgePath *pathQueue;
    EdgePath *endQueue;
    EdgePath *retired;
    EdgePath *endRetired;
    EdgePath newPath;
    Node<List<GraphArc *> > *nextBest;
    Node<List<GraphArc *> > *prevBest;
    
public:
    
    Graehl (stringstream &graphstream, int s, int d)
      : source(s), dest(d), ready(false)
    {
        graphstream >> graph;
        nStates = graph.nStates;
        assert(nStates > 0 && graph.states);
        assert(source >= 0 && source < nStates);
        assert(dest >= 0 && dest < nStates);
        
        paths = new List<List<GraphArc *> >();
        insertHere = new ListIter<List<GraphArc *> >(*paths);
        List<GraphArc *> dummy;
        insertHere->insert(dummy);
        nextBest = paths->first();
        prevBest = NULL;
        
        dist = new float[nStates];
        shortPathGraph = shortestPathTree(graph, dest, dist);
        shortPathTree = shortPathGraph.states;
        
        path = NULL;
        revPathTree.states = NULL;
        pathGraph = NULL;
        sidetracks.states = NULL;
        visited = NULL;
        pathQueue = NULL;
        retired = NULL;
        if (shortPathTree[source].arcs.notEmpty() || source == dest)
        {
            path = new ListIter<GraphArc *>(insertHere->insert(List<GraphArc *>()));
            insertShortPath(source, dest, *path);
            
            GraphHeap::freeAll();
            graphPaths = List<List<GraphArc *> >();
            revPathTree = reverseGraph(shortPathGraph);
            pathGraph = new GraphHeap *[nStates];
            for (size_t i = 0; i < (size_t) nStates; i++)
                pathGraph[i] = NULL;
            sidetracks = sidetrackGraph(graph, shortPathGraph, dist);
            visited = new bool[nStates];
            for (int i = 0; i < nStates; i++)
                visited[i] = 0;
            depthFirstSearch(revPathTree, dest, visited, [=] (int state, int pred) { this->buildSidetracksHeap(state, pred); });
            if (pathGraph[source])
            {
                pathQueue = new EdgePath[4 * (MAXPATHS+1)];
                endQueue = pathQueue;
                retired = new EdgePath[MAXPATHS+1];
                endRetired = retired;
                newPath.weight = pathGraph[source]->arc->weight;
                newPath.heapPos = -1;
                newPath.node = pathGraph[source];
                newPath.last = NULL;
                heapAdd(pathQueue, endQueue++, newPath);
                
                ready = true;
                getNextPath();  // pull out the empty dummy path
            }
        }
    }
    
    ~Graehl ()
    {
        if (retired)
            delete [] retired;
        if (pathQueue)
            delete [] pathQueue;
        if (visited)
            delete [] visited;
        if (pathGraph)
            delete [] pathGraph;
        if (sidetracks.states)
            delete [] sidetracks.states;
        if (revPathTree.states)
            delete [] revPathTree.states;
        if (path)
            delete path;
        delete [] shortPathGraph.states;
        delete [] dist;
        delete insertHere;
        delete paths;
        delete [] graph.states;
    }

    List<GraphArc *> *getNextPath()
    {
        if (!ready)
            return NULL;
        
        if (prevBest != NULL)
        {
            if (prevBest->data != NULL)
                delete prevBest->data;
            delete prevBest;
        }
        
        if (heapSize(pathQueue, endQueue))
        {
            EdgePath *top = pathQueue;
            GraphArc *cutArc;
            List<GraphArc *> shortPath;
            if (top->heapPos == -1)
                cutArc = top->node->arc;
            else
                cutArc = top->node->arcHeap[top->heapPos];
            shortPath.push(cutArc);
            
            EdgePath *last;
            while ((last = top->last))
            {
                if (!((last->heapPos == -1 && (top->heapPos == 0 || top->node == last->node->left
                    || top->node == last->node->right)) || (last->heapPos >= 0 && top->heapPos != -1)))
                {
                    if (last->heapPos == -1)
                        cutArc = last->node->arc;
                    else
                        cutArc = last->node->arcHeap[last->heapPos];
                    shortPath.push(cutArc);
                }
                top = last;
            }
            
            ListIter<GraphArc *> fullPath(insertHere->insert(List<GraphArc *>()));
            int sourceState = source;
            for (ListIter<GraphArc *> cut(shortPath); cut; ++cut)
            {
                insertShortPath(sourceState, cut.data()->source, fullPath);
                sourceState = cut.data()->dest;
                fullPath.insert((GraphArc *)cut.data()->data);
            }
            insertShortPath(sourceState, dest, fullPath);
            *endRetired = pathQueue[0];
            newPath.last = endRetired++;
            heapPop(pathQueue, endQueue--);
            int lastHeapPos = newPath.last->heapPos;
            GraphArc *spawnVertex;
            GraphHeap *from = newPath.last->node;
            float lastWeight = newPath.last->weight;
            if (lastHeapPos == -1)
            {
                spawnVertex = from->arc;
                newPath.heapPos = -1;
                if (from->left)
                {
                    newPath.node = from->left;
                    newPath.weight = lastWeight + (newPath.node->arc->weight - spawnVertex->weight);
                    heapAdd(pathQueue, endQueue++, newPath);
                }
                if (from->right)
                {
                    newPath.node = from->right;
                    newPath.weight = lastWeight + (newPath.node->arc->weight - spawnVertex->weight);
                    heapAdd(pathQueue, endQueue++, newPath);
                }
                if (from->arcHeapSize)
                {
                    newPath.heapPos = 0;
                    newPath.node = from;
                    newPath.weight = lastWeight + (newPath.node->arcHeap[0]->weight - spawnVertex->weight);
                    heapAdd(pathQueue, endQueue++, newPath);
                }
            }
            else
            {
                spawnVertex = from->arcHeap[lastHeapPos];
                newPath.node = from;
                int iChild = 2 * lastHeapPos + 1;
                if (from->arcHeapSize > iChild)
                {
                    newPath.heapPos = iChild;
                    newPath.weight = lastWeight + (newPath.node->arcHeap[iChild]->weight - spawnVertex->weight);
                    heapAdd(pathQueue, endQueue++, newPath);
                    if (from->arcHeapSize > ++iChild)
                    {
                        newPath.heapPos = iChild;
                        newPath.weight = lastWeight + (newPath.node->arcHeap[iChild]->weight - spawnVertex->weight);
                        heapAdd(pathQueue, endQueue++, newPath);
                    }
                }
            }
            
            if (pathGraph[spawnVertex->dest])
            {
                newPath.heapPos = -1;
                newPath.node = pathGraph[spawnVertex->dest];
                newPath.heapPos = -1;
                newPath.weight = lastWeight + newPath.node->arc->weight;
                heapAdd(pathQueue, endQueue++, newPath);
            }
            
            List<GraphArc *> *ret = &nextBest->data;
            prevBest = nextBest;
            nextBest = nextBest->next;
            return ret;
        }
        else
        {
            return NULL;
        }
    }
    
private:
    
    void insertShortPath(int source, int dest, ListIter<GraphArc *> &path)
    {
        GraphArc *taken;
        for ( int iState = source ; iState != dest; iState = taken->dest ) {
            taken = &shortPathTree[iState].arcs.top();
            path.insert((GraphArc *)taken->data);
        }
    }
    
    // a sidetrack from a given state in an arc originating from any state along the shortest path to the destination, that is not in the shortest path tree.  Paths are uniquely determined by a sequence of sidetracks from the destination of the previous sidetrack, or the source state if there was no previous sidetrack (see Eppstein)
    Graph sidetrackGraph(Graph fullGraph, Graph shortGraph, float *dist)
    {
        //  subtracts shortGraph from fullGraph, arcs' data member points to arc in fullGraph
        assert(fullGraph.nStates == shortGraph.nStates);
        int nStates = fullGraph.nStates;
        GraphState *sub = new GraphState[nStates];
        for ( int i = 0 ; i < nStates ; ++i )
            if ( dist[i] != HUGE_VAL ) 
                for ( ListIter<GraphArc> l(fullGraph.states[i].arcs) ; l ; ++l ) {
                    assert(i == l.data().source);
                    int isShort = 0;
                    for ( ListIter<GraphArc> r(shortGraph.states[i].arcs) ; r ; ++r )
                        if ( r.data().data == &l.data() ) { // arcs in shortest path tree have data pointing to the arc corresponding to it in the original graph
                            isShort = 1;
                            break;
                        }
                                if ( !isShort )
                                    if ( dist[l.data().dest] != HUGE_VAL ) {
                                        GraphArc w = l.data();
                                        w.weight = w.weight - (dist[i] - dist[w.dest]);
                                        w.data = &l.data();
                                        sub[i].arcs.push(w);
                                    }
                }
                Graph ret;
                ret.nStates = fullGraph.nStates;
                ret.states = sub;
                return ret;
    }
        
    // see Eppstein's paper for explanation of this shared heap
    void buildSidetracksHeap(int state, int pred)
    {
        GraphHeap *prev;
        
        if ( pred == -1 )
            prev = NULL;
        else
            prev = pathGraph[pred];
        
        ListIter<GraphArc> s(sidetracks.states[state].arcs);
        if ( s ) {
            int heapSize = 0;
            GraphArc *min;
            min = &s.data();
            while ( ++s ) {
                if ( s.data().weight < min->weight )
                    min = &s.data();
                ++heapSize;
            }
                pathGraph[state] = new GraphHeap;
                pathGraph[state]->arc = min;
                pathGraph[state]->arcHeapSize = heapSize;
                if ( heapSize ) {
                    pGraphArc *heapStart = pathGraph[state]->arcHeap = new pGraphArc[heapSize];
                    pGraphArc *heapI = heapStart;
                    for ( ListIter<GraphArc> gArc(sidetracks.states[state].arcs) ; gArc ; ++gArc )
                        if ( &gArc.data() != min )
                            (heapI++)->p = &gArc.data();
                        assert(heapI == heapStart + heapSize);
                    heapBuild(heapStart, heapStart + heapSize);
                } else
                    pathGraph[state]->arcHeap = NULL;
                pathGraph[state] = newTreeHeapAdd(prev, pathGraph[state]);
        } else
            pathGraph[state] = prev;
    }
};

}

