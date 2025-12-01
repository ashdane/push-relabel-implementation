import sys
import time
from collections import deque
from typing import List, Optional

# Equivalent to the C++ struct Edge
class Edge:
    """Represents an edge in the flow network."""
    def __init__(self, to_node: int, rev_index: int, flow: int, cap: int):
        self.to: int = to_node  # The node this edge goes to
        self.rev: int = rev_index  # Index of the reverse edge in adj[to]
        self.flow: int = flow    # Current flow on the edge
        self.cap: int = cap      # Capacity of the edge

# The PushRelabel class implements the algorithm
class PushRelabel:
    """Implements the highest label preflow push-relabel algorithm for max-flow."""
    def __init__(self, nodes: int):
        self.n: int = nodes
        # Adjacency list: adj[u] is a list of Edge objects from node u
        self.adj: List[List[Edge]] = [[] for _ in range(nodes)]
        # Preflow excess for each node
        self.excess: List[int] = [0] * nodes
        # Distance (height) label for each node
        self.dist: List[int] = [0] * nodes
        # count[i] stores the number of nodes v such that dist[v] == i (for gap heuristic)
        self.count: List[int] = [0] * (2 * nodes + 1)
        # active[v] is true if v has excess flow and is not the source/sink
        self.active: List[bool] = [False] * nodes
        # head[u] is the index of the 'current arc' (for arc-choosing heuristic)
        self.head: List[int] = [0] * nodes
        # Queue for active nodes
        self.Q: deque[int] = deque()

    def enqueue(self, v: int):
        """Adds a node to the active queue if it has positive excess and isn't already active."""
        if not self.active[v] and self.excess[v] > 0:
            self.active[v] = True
            self.Q.append(v)

    def push(self, u: int, e: Edge):
        """Pushes flow from node u to e.to across edge e."""
        amt: int = min(self.excess[u], e.cap - e.flow)
        
        if amt == 0:
            return

        # Forward edge flow update
        e.flow += amt
        # Reverse edge flow update (index is e.rev, which is stored in the edge)
        self.adj[e.to][e.rev].flow -= amt
        
        # Excess update
        self.excess[u] -= amt
        self.excess[e.to] += amt
        
        # Enqueue the receiving node if it gains excess
        self.enqueue(e.to)

    def gap(self, k: int):
        """Performs the gap heuristic when count[k] == 1."""
        for v in range(self.n):
            if self.dist[v] < k:
                continue
            
            self.count[self.dist[v]] -= 1
            # Relabel to n + 1 (effectively infinity)
            self.dist[v] = max(self.dist[v], self.n + 1)
            self.count[self.dist[v]] += 1
            self.enqueue(v)

    def relabel(self, u: int):
        """Increases the distance label of u."""
        self.count[self.dist[u]] -= 1
        new_dist: int = 2 * self.n  # Initialize to a large value
        
        # Find the minimum possible new distance label
        for edge in self.adj[u]:
            if edge.cap - edge.flow > 0:
                new_dist = min(new_dist, self.dist[edge.to] + 1)
        
        self.dist[u] = new_dist
        self.count[self.dist[u]] += 1
        self.enqueue(u)

    def discharge(self, u: int, s: int, t: int):
        """Discharges the excess flow from node u."""
        while self.excess[u] > 0:
            if self.head[u] < len(self.adj[u]):
                # Current arc heuristic
                edge: Edge = self.adj[u][self.head[u]]
                
                # Check for residual capacity and valid height for pushing
                if edge.cap - edge.flow > 0 and self.dist[u] == self.dist[edge.to] + 1:
                    self.push(u, edge)
                else:
                    self.head[u] += 1  # Advance current arc
            else:
                # No suitable arc found, need to relabel or apply gap heuristic
                
                # Check for gap heuristic condition (only one node at this height)
                if self.dist[u] < len(self.count) and self.count[self.dist[u]] == 1:
                    self.gap(self.dist[u])
                else:
                    self.relabel(u)
                    
                # After relabel/gap, reset current arc
                self.head[u] = 0

    def addEdge(self, u: int, v: int, cap: int):
        """Adds a forward edge and its reverse (residual) edge."""
        if u == v:
            return
        
        # Forward edge: u -> v
        # rev_index will be the index of the reverse edge in adj[v]
        forward_rev_index: int = len(self.adj[v])
        # Reverse edge: v -> u (capacity 0 for initial graph)
        reverse_rev_index: int = len(self.adj[u])
        
        self.adj[u].append(Edge(v, forward_rev_index, 0, cap))
        self.adj[v].append(Edge(u, reverse_rev_index, 0, 0)) # Residual edge

    def getMaxFlow(self, s: int, t: int) -> int:
        """Calculates the maximum flow from source s to sink t."""
        if s < 0 or t < 0 or s >= self.n or t >= self.n:
            return 0
        
        # Initialization
        self.dist[s] = self.n
        self.count[0] = self.n - 1 
        self.count[self.n] = 1     
        
        # Initial push from source s
        for edge in self.adj[s]:
            self.excess[s] += edge.cap
            amt = edge.cap
            
            if amt == 0:
                continue

            edge.flow += amt
            self.adj[edge.to][edge.rev].flow -= amt
            self.excess[s] -= amt
            self.excess[edge.to] += amt
            self.enqueue(edge.to)
            
        # Main loop: process active nodes
        while self.Q:
            u: int = self.Q.popleft()
            self.active[u] = False 
            
            if u == s or u == t:
                continue
            
            self.discharge(u, s, t)
            
        flow: int = 0
        for edge in self.adj[s]:
            flow += edge.flow
            
        return flow


def main():
    """
    Reads graph definition from a file provided as an argument 
    (or stdin if no argument is provided) and computes max-flow.
    """
    
    # Start timing
    start_time = time.time()

    solver: Optional[PushRelabel] = None
    s: int = -1  # Source node (0-indexed)
    t: int = -1  # Sink node (0-indexed)

    # Determine input source: file argument or stdin
    if len(sys.argv) > 1:
        try:
            # Open the file provided in the first argument
            input_source = open(sys.argv[1], 'r')
        except FileNotFoundError:
            sys.stderr.write(f"Error: File '{sys.argv[1]}' not found.\n")
            sys.exit(1)
    else:
        # Default to standard input
        input_source = sys.stdin

    # Read from the determined source
    with input_source:
        for line in input_source:
            line = line.strip()
            if not line:
                continue
            
            parts = line.split()
            if not parts:
                continue
                
            type_char = parts[0]
            
            if type_char == 'c':
                continue
            elif type_char == 'p':
                if len(parts) >= 3:
                    try:
                        num_nodes = int(parts[2])
                        solver = PushRelabel(num_nodes)
                    except ValueError:
                        sys.stderr.write("Error parsing 'p' line.\n")
                        return
            elif type_char == 'n':
                if len(parts) >= 3 and solver:
                    try:
                        node_id = int(parts[1])
                        node_type = parts[2]
                        if node_type == 's':
                            s = node_id - 1
                        elif node_type == 't':
                            t = node_id - 1
                    except ValueError:
                        sys.stderr.write("Error parsing 'n' line.\n")
                        return
            elif type_char == 'a':
                if len(parts) >= 4 and solver:
                    try:
                        u = int(parts[1])
                        v = int(parts[2])
                        cap = int(parts[3])
                        solver.addEdge(u - 1, v - 1, cap)
                    except ValueError:
                        sys.stderr.write("Error parsing 'a' line.\n")
                        return

    # Calculate and output the max flow
    if solver and s != -1 and t != -1:
        max_flow = solver.getMaxFlow(s, t)
        print(f"Max Flow: {max_flow}")
    elif solver:
        sys.stderr.write("Error: Source (s) or Sink (t) not defined in input.\n")
    else:
        sys.stderr.write("Error: Problem definition ('p' line) not found in input.\n")

    # End timing
    end_time = time.time()
    elapsed_time = end_time - start_time
    
    # Print time to stderr
    sys.stderr.write(f"Total time taken: {elapsed_time:.6f} seconds\n")

if __name__ == "__main__":
    main()
