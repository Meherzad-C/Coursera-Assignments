
/**
 * @file GraphSearchExercises.cpp
 * University of Illinois CS 400, MOOC 3, Week 3: Graph Search
 * Spring 2019
 *                        STUDENT STARTER FILE
 *
 * @author Eric Huber - University of Illinois staff
 *
**/

// Before beginning these exercises, you should read the instructions PDF,
// and look through the other code files in this directory for examples and
// hints. You only need to edit code in this file. There are comments here
// hinting at what you need to do, and "TODO" is written in various places
// where you need to edit some code in particular.

#include "GraphSearchCommon.h"

// =========================================================================
// EXERCISE 1: Adjacency List Utilities
//
// This exercise is based on the Week 3 lecture material.
//
// Our GridGraph class implements an "adjacency list" graph data structure,
// although it actually makes use of std::unordered_map and std::unordered_set
// to do this. You can read about GridGraph in the instructions PDF and by
// examining GridGraph.h. There are also various examples of GridGraph usage
// in the informal tests defined in main.cpp.
//
// Most of the GridGraph class is already implemented for you. For this
// exercise, you need to finish implementing two functions below:
//
// GridGraph::countEdges
// GridGraph::removePoint
//
// =========================================================================

// GridGraph::countEdges:
// This function should return the number of unique edges in the GridGraph.
// Remember that GridGraph doesn't explicitly store edge objects, but
// instead it stores point adjacency information. Since the graph edges are
// undirected, for any two vertices A and B, the edges (A,B) and (B,A) are the
// same edge. This means if we add up the sizes of the GridGraph::NeighborSet
// sets mapped in adjacencyMap, we'll be double-counting the number of edges.
// We can still use that to get the answer, or instead, the implementation of
// GridGraph::printDetails in GridGraph.h shows another method, by constructing
// a set of the unique edges only.
int GridGraph::countEdges() const {
    int numEdges = 0;

    // Loop through each vertex in the adjacencyMap
    for (const auto& entry : adjacencyMap) {
        // Add the number of neighbors (edges) for the current vertex
        numEdges += entry.second.size();
    }

    // Since each edge is counted twice, divide the result by 2
    numEdges /= 2;

    return numEdges;
}

// GridGraph::removePoint:
// This function takes a point (an IntPair) as input, and it should remove
// all references to the point from the data structure. That means it should
// remove the point's key from the adjacency map, and it should also remove
// other points' references to it in the map, if those other points had been
// connected to it by implicit edges. It shouldn't change anything else about
// the graph.
void GridGraph::removePoint(const IntPair& p1) {
    // If the point p1 is not in the GridGraph, do nothing and return
    if (!hasPoint(p1)) return;

    // Get the set of neighbors for the point p1
    const GridGraph::NeighborSet originalNeighbors = adjacencyMap.at(p1);

    // Remove p1 from the adjacency lists of all its neighbors
    for (const auto& neighbor : originalNeighbors) {
        // Remove p1 from each neighbor's neighbor set
        adjacencyMap[neighbor].erase(p1);
    }

    // Finally, erase p1 from the adjacency map
    adjacencyMap.erase(p1);
}

// =========================================================================
// EXERCISE 2: graphBFS
//
// This exercise is based on the Week 4 lecture material.
//
// The graphBFS function below largely implements the "breadth-first search"
// algorithm for the GridGraph class. You'll be asked to edit a few parts of
// the function to complete it. Those parts are marked with "TODO".
// (Please read the instructions PDF in case these hints are expanded upon
//  there.)
//
// Differences from the version of BFS discussed in lecture:
//
// - This implementation of BFS is geared toward finding the shortest path
//   from a start point to a goal point. So, it only explores within a single
//   connected component, and it may report that the goal is unreachable.
//   As soon as the goal point is found, our algorithm stops searching.
//
// - Our implementation uses sets of points to implicitly "label" points with
//   some status such as "visited", instead of assigning a label property to
//   a point itself. This lets us associate more than one kind of status with
//   any given point at the same time, by putting the point in several sets.
//   It's convenient to do this with STL containers like unordered set, since
//   we don't have to add some kind of status member variable to our own
//   classes like IntPair this way. It also means we don't have to initialize
//   a status for every vertex. In some graph search problems, such as later
//   in Exercise 3, the number of vertices is extremely large, so we'd rather
//   not have to initialize them all!
//
// - We use a map, "pred", to record the predecessor vertex of any newly-
//   discovered vertex during the search. This implicitly records what the
//   discovery edges are, as described in lecture. We can use that to rebuild
//   the path from start to goal at the end.
//
// - We do not assign status directly to edges. However, the vertex
//   predecessor information is basically recording the "discovery" edges.
//
// - We use a map, "dist", to record information about the shortest-path
//   distance to any given node that has been discovered so far. This is
//   similar to what Dijkstra's algorithm does, although with BFS graph search
//   when the edge lengths are all equal to 1, we know that as soon as we
//   discover a node, we have found the shortest path to it. We can still use
//   this to detect if we've taken more steps than expected and quit.
//
// - Redundantly, we have created a "dequeued set" that we use to help you
//   check for mistakes that could cause an infinite loop. This isn't normally
//   part of the BFS algorithm itself. In a way, this is mirroring what the
//   "visited set" is already supposed to accomplish.
//
// =========================================================================

// graphBFS:
// We use our GridGraph class where vertices are points (IntPair objects).
// The input consists of a start point, a goal point, and a GridGraph containing
// these points and the implicit edge information.
// We use BFS (breadth-first search) to find the shortest path from start to
// goal, if there exists any path at all. Then we return the path as a list of
// points. If there is no path, or if we take too many steps without success,
// then we return an empty list.
std::list<IntPair> graphBFS(const IntPair& start, const IntPair& goal, const GridGraph& graph) {

  // maxDist failsafe:
  constexpr int maxDist = 100;

  std::queue<IntPair> exploreQ;
  std::unordered_map<IntPair, IntPair> pred;
  std::unordered_map<IntPair, int> dist;
  std::unordered_set<IntPair> visitedSet;
  std::unordered_set<IntPair> dequeuedSet;

  if (!graph.hasPoint(start)) throw std::runtime_error("Starting point doesn't exist in graph");
  if (!graph.hasPoint(goal)) throw std::runtime_error("Goal point doesn't exist in graph");

  pred[start] = start;
  dist[start] = 0;
  visitedSet.insert(start);
  exploreQ.push(start);

  bool foundGoal = (start == goal);
  bool tooManySteps = false;

  while (!exploreQ.empty() && !foundGoal && !tooManySteps) {

    auto curPoint = exploreQ.front();
    exploreQ.pop();

    if (dequeuedSet.count(curPoint)) {
      std::cout << "graphBFS ERROR: Dequeued a vertex that had already been dequeued before." << std::endl;
      return std::list<IntPair>();
    } else {
      dequeuedSet.insert(curPoint);
    }

    // Get a copy of the set of neighbors we're going to loop over.
    GridGraph::NeighborSet neighbors = graph.adjacencyMap.at(curPoint);

    for (auto neighbor : neighbors) {

      bool neighborWasAlreadyVisited = visitedSet.count(neighbor) > 0;

      if (!neighborWasAlreadyVisited) {
        // Record the current point as the predecessor of the neighbor
        pred[neighbor] = curPoint;

        // Mark the neighbor as visited
        visitedSet.insert(neighbor);

        // Add the neighbor to the exploration queue
        exploreQ.push(neighbor);

        // Update the distance to this neighbor
        dist[neighbor] = dist[curPoint] + 1;

        if (dist[neighbor] > maxDist) {
          tooManySteps = true;
          break;
        }

        if (neighbor == goal) {
          foundGoal = true;
          break;
        }
      }
    }
  }

  if (tooManySteps) {
    std::cout << "graphBFS warning: Could not reach goal within the maximum allowed steps." << std::endl;
    return std::list<IntPair>();
  }

  if (!foundGoal) {
    std::cout << "graphBFS warning: Could not reach goal. (This may be expected if no path exists.)" << std::endl;
    return std::list<IntPair>();
  }

  // Reconstruct the path
  std::list<IntPair> path;
  auto cur = goal;
  path.push_front(cur);

  while (pred.count(cur) && pred[cur] != cur) {
    path.push_front(pred[cur]);
    cur = pred[cur];
  }

  return path;
}

// =========================================================================
// EXERCISE 3: puzzleBFS
//
// This time, we will use BFS to solve a graph modeling problem. This is
// where we model a realistic problem in terms of an imaginary graph, and
// then we can use graph search concepts to solve the modeled problem.
//
// (Please see the instructions PDF for a detailed introduction to this
//  problem, with illustrations.)
//
// The PuzzleState class represents one current state of the "8 puzzle",
// which is a puzzle played on a 3x3 grid containing 8 tiles, where a tile
// can slide into the blank space to shuffle the puzzle. Each state of the
// puzzle can be modeled as a vertex in an imaginary graph, where two states
// are virtually connected by an edge (adjacent) if a single tile move can
// transform the first state into the second state. We do not need a map
// structure for adjacencies since we can use the helper functions from the
// PuzzleState class to determine which states are adjacent at any given
// time. Therefore we also don't need an explicit graph class at all. It's
// important that we can use such an implicit graph representation, because
// the total number of vertices (states) and edges (moves) in the graph model
// for "8 puzzle" would be extremely large, and that would greatly impact the
// running time and memory usage. We don't need to examine every vertex or
// every edge in the graph model; we can just search from the start and quit
// after either finding the goal or giving up (in relatively few steps).
//
// This function is VERY similar to graphBFS, but now we are using the
// PuzzleState class to represent imaginary vertices instead of using IntPair
// to represent literal 2D points, and we do not use GridGraph. You should
// finish graphBFS first before trying this problem. The starter code shown
// below for puzzleBFS is so similar to graphBFS that the comments have mostly
// been removed.
//
// =========================================================================

// puzzleBFS:
// Given start and goal sates as PuzzleState objects, we perform BFS in the
// imaginary graph model implied by the start state, where the rest of the
// reachable vertices (states) and the edges leading to them (puzzle moves)
// can be figure out based on the tile sliding rules of the puzzle.
// If there exists any path from start to goal, then the puzzle can be solved,
// and we return the shortest path (which represents the solution steps).
// If there is no path to the goal, or if we take too many steps without
// success, then the puzzle cannot be solved, and we return an empty list.
std::list<PuzzleState> puzzleBFS(const PuzzleState& start, const PuzzleState& goal) {
  constexpr int maxDist = 35;  // max steps allowed for 8-puzzle
  std::queue<PuzzleState> exploreQ;
  std::unordered_map<PuzzleState, PuzzleState> pred;
  std::unordered_map<PuzzleState, int> dist;
  std::unordered_set<PuzzleState> visitedSet;
  std::unordered_set<PuzzleState> dequeuedSet;

  pred[start] = start;
  dist[start] = 0;
  visitedSet.insert(start);
  exploreQ.push(start);

  bool foundGoal = (start == goal);
  bool tooManySteps = false;

  // The main search loop --------------------------------------------------
  while (!exploreQ.empty() && !foundGoal && !tooManySteps) {
    auto curState = exploreQ.front();
    exploreQ.pop();

    bool curPointWasPreviouslyDequeued = dequeuedSet.count(curState);
    if (curPointWasPreviouslyDequeued) {
      std::cout << "puzzleBFS ERROR: Dequeued a vertex that had already been dequeued before." << std::endl
                << " If you're using visitedSet correctly, then no vertex should ever be added" << std::endl
                << " to the explore queue more than once. [Returning an empty path now.]" << std::endl << std::endl;
      return std::list<PuzzleState>();
    } else {
      dequeuedSet.insert(curState);
    }

    // =====================================================================
    // Get the neighbors (valid puzzle states from current state)
    auto neighbors = curState.getAdjacentStates();  // Use getAdjacentStates instead
    // =====================================================================

    for (auto neighbor : neighbors) {
      // ==================================================================
      // Check if the neighbor has already been visited.
      bool neighborWasAlreadyVisited = visitedSet.count(neighbor);
      // ==================================================================

      if (!neighborWasAlreadyVisited) {
        // ================================================================
        // Record the current state as the predecessor of the neighbor
        pred[neighbor] = curState;

        // Add neighbor to the visited set.
        visitedSet.insert(neighbor);

        // Push the neighbor state into the exploration queue.
        exploreQ.push(neighbor);
        // ================================================================

        // Update the distance for the neighbor
        dist[neighbor] = dist[curState] + 1;
        if (dist[neighbor] > maxDist) {
          tooManySteps = true;
          break;
        }

        // If the neighbor is the goal, set foundGoal flag to true
        if (neighbor == goal) {
          foundGoal = true;
          break;
        }

      } // end of handling the just-discovered neighbor

    } // end of for loop
  } // end of while loop

  // If we reached the maximum steps, return empty path
  if (tooManySteps) {
    std::cout << "puzzleBFS warning: Could not reach goal within the maximum allowed steps.\n (This may be expected if no path exists.)" << std::endl << std::endl;
    return std::list<PuzzleState>();
  }

  // If we never found the goal, return empty path
  if (!foundGoal) {
    std::cout << "puzzleBFS warning: Could not reach goal. (This may be expected\n if no path exists.)" << std::endl << std::endl;
    return std::list<PuzzleState>();
  }

  // Reconstruct the path from goal to start
  std::list<PuzzleState> path;
  auto cur = goal;
  path.push_front(cur);
  while (pred.count(cur) && pred[cur] != cur) {
    path.push_front(pred[cur]);
    cur = pred[cur];
  }

  return path;
}
