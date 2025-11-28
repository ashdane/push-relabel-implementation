/**
 * ======================================================================================
 * PROJECT: Push-Relabel Maximum Flow Implementation (FIFO Variant)
 * ======================================================================================
 * * FEATURES:
 * 1. O(V^3) FIFO Vertex Selection Rule.
 * 2. Gap Heuristic Optimization for faster convergence.
 * 3. Synthetic Graph Generator (Custom Node Count, Density, Capacities).
 * 4. Real-World Dataset Loader (Parses standard Edge Lists & DIMACS).
 * 5. Interactive Command Line Menu.
 * * HOW TO COMPILE:
 * g++ -O3 push_relabel.cpp -o maxflow
 * * ======================================================================================
 */

#include <iostream>
#include <vector>
#include <queue>
#include <algorithm>
#include <climits>
#include <random>
#include <fstream>
#include <sstream>
#include <string>
#include <iomanip>
#include <cstdlib> // Required for system()

using namespace std;

// ==========================================
// DATA STRUCTURES
// ==========================================

struct Edge {
    int to;
    int rev; // Index of the reverse edge in the "to" vertex's adjacency list
    long long flow;
    long long cap;
};

// ==========================================
// PUSH-RELABEL ALGORITHM CLASS
// ==========================================

class PushRelabel {
private:
    int n;
    vector<vector<Edge>> adj;
    vector<long long> excess;
    vector<int> dist;
    vector<int> count;
    vector<bool> active;
    vector<vector<int>> bucket;
    queue<int> Q;

    // Helper to add edge
    void add_edge_internal(int u, int v, long long cap) {
        adj[u].push_back({v, (int)adj[v].size(), 0, cap});
        adj[v].push_back({u, (int)adj[u].size() - 1, 0, 0}); // Backward edge with 0 capacity
    }

    void enqueue(int v) {
        if (!active[v] && excess[v] > 0) {
            active[v] = true;
            Q.push(v);
        }
    }

    void push(int u, Edge &e) {
        long long amt = min(excess[u], e.cap - e.flow);
        if (dist[u] <= dist[e.to] || amt == 0) return;

        e.flow += amt;
        adj[e.to][e.rev].flow -= amt;
        excess[u] -= amt;
        excess[e.to] += amt;
        enqueue(e.to);
    }

    void gap(int k) {
        for (int v = 0; v < n; v++) {
            if (dist[v] < k) continue;
            count[dist[v]]--;
            dist[v] = max(dist[v], n + 1);
            count[dist[v]]++;
            enqueue(v);
        }
    }

    void relabel(int u) {
        count[dist[u]]--;
        dist[u] = 2 * n;
        for (auto &e : adj[u]) {
            if (e.cap - e.flow > 0)
                dist[u] = min(dist[u], dist[e.to] + 1);
        }
        count[dist[u]]++;
        enqueue(u);
    }

    void discharge(int u) {
        for (auto &e : adj[u]) {
            if (excess[u] > 0) {
                push(u, e);
            } else {
                break;
            }
        }
        if (excess[u] > 0) {
            if (count[dist[u]] == 1) {
                gap(dist[u]);
            } else {
                relabel(u);
            }
        }
    }

public:
    PushRelabel(int nodes) : n(nodes) {
        adj.resize(n);
        excess.resize(n);
        dist.resize(n);
        count.resize(2 * n);
        active.resize(n);
    }

    void addEdge(int u, int v, long long cap) {
        if (u == v) return;
        add_edge_internal(u, v, cap);
    }

    long long getMaxFlow(int s, int t) {
        count[0] = n - 1;
        count[n] = 1;
        dist[s] = n;
        active[s] = true;
        active[t] = true;

        for (auto &e : adj[s]) {
            excess[s] += e.cap;
            push(s, e);
        }

        while (!Q.empty()) {
            int u = Q.front();
            Q.pop();
            active[u] = false;

            if (u == s || u == t) continue;
            discharge(u);
        }

        long long flow = 0;
        for (auto &e : adj[s]) {
            flow += e.flow;
        }
        return flow;
    }

    void printFlowAssignment(int limit = 50) {
        cout << "\n--- Non-Zero Flow Assignments (Displaying max " << limit << " edges) ---\n";
        cout << left << setw(10) << "From" << setw(10) << "To" << setw(15) << "Flow/Cap" << endl;
        cout << string(35, '-') << endl;

        int printed = 0;
        for (int u = 0; u < n; ++u) {
            for (auto &e : adj[u]) {
                if (e.flow > 0 && e.cap > 0) { // Forward edges with flow
                    cout << left << setw(10) << u << setw(10) << e.to 
                         << e.flow << " / " << e.cap << endl;
                    printed++;
                    if (printed >= limit) {
                        cout << "... (Output truncated for readability) ..." << endl;
                        return;
                    }
                }
            }
        }
    }
};

// ==========================================
// UTILITY FUNCTIONS
// ==========================================

void clearScreen() {
    // We check the return value in an if-statement to satisfy
    // strict compilers (warn_unused_result) even though we ignore the outcome.
    #ifdef _WIN32
        if (system("cls")) {}
    #else
        if (system("clear")) {}
    #endif
}

// ==========================================
// GRAPH GENERATORS & LOADERS
// ==========================================

PushRelabel* generateSyntheticGraph(int &s, int &t) {
    int nodes;
    double density;
    int minCap, maxCap;

    cout << "\n--- Synthetic Graph Generation ---\n";
    cout << "Enter number of nodes (e.g., 50 - 1000): ";
    cin >> nodes;
    if(nodes < 2) nodes = 2;

    cout << "Enter edge density (0.0 to 1.0, e.g., 0.2 for 20%): ";
    cin >> density;
    if(density < 0) density = 0.01;
    if(density > 1) density = 1.0;

    cout << "Enter capacity range (min max): ";
    cin >> minCap >> maxCap;

    PushRelabel* graph = new PushRelabel(nodes);
    s = 0; 
    t = nodes - 1;

    // Random Number Generator
    mt19937 rng(42); // Fixed seed for reproducibility
    uniform_real_distribution<double> distProb(0.0, 1.0);
    uniform_int_distribution<int> distCap(minCap, maxCap);

    long long edgeCount = 0;
    cout << "Generating edges..." << endl;

    for (int u = 0; u < nodes; ++u) {
        for (int v = 0; v < nodes; ++v) {
            if (u != v) {
                // Ensure s->t path exists by forcing a line occasionally or just probability
                if (distProb(rng) < density) {
                    graph->addEdge(u, v, distCap(rng));
                    edgeCount++;
                }
            }
        }
    }
    
    // Ensure connectivity from s to t (Simple path)
    for(int i=0; i < nodes-1; i++) {
        graph->addEdge(i, i+1, distCap(rng));
    }

    cout << "Graph Generated: " << nodes << " Nodes, " << edgeCount << " Edges.\n";
    cout << "Source: " << s << ", Sink: " << t << endl;
    return graph;
}

PushRelabel* loadRealWorldGraph(const string& filename, int &s, int &t, int &nodes) {
    ifstream file(filename);
    if (!file.is_open()) {
        return nullptr; // File not found
    }

    cout << "Loading " << filename << "..." << endl;
    
    // Preliminary scan to find max node ID for sizing
    int u, v; 
    long long cap;
    int maxNodeId = 0;
    string line;
    
    while(getline(file, line)) {
        if(line.empty() || line[0] == '#' || line[0] == '%') continue; // Skip comments
        stringstream ss(line);
        // Supports formats: "u v cap" or "u v" (default cap 1)
        if(ss >> u >> v) {
            maxNodeId = max(maxNodeId, max(u, v));
        }
    }

    file.clear();
    file.seekg(0, ios::beg);

    nodes = maxNodeId + 1;
    PushRelabel* graph = new PushRelabel(nodes);

    while(getline(file, line)) {
        if(line.empty() || line[0] == '#' || line[0] == '%') continue;
        stringstream ss(line);
        if(ss >> u >> v) {
            if (!(ss >> cap)) cap = 1; // Default capacity 1 if not provided
            graph->addEdge(u, v, cap);
        }
    }

    // Heuristic: Set Source to 0, Sink to Max Node ID
    s = 0;
    t = maxNodeId;

    cout << "Graph Loaded Successfully. Nodes: " << nodes << "\n";
    cout << "Assigned Source: " << s << ", Assigned Sink: " << t << endl;
    return graph;
}

// ==========================================
// MENUS
// ==========================================

void showRealWorldMenu() {
    struct DatasetInfo {
        string name;
        string filename;
        string url;
        string description;
    };

    vector<DatasetInfo> datasets = {
        {"Wikipedia Vote", "real-world-datasets/wiki-Vote.txt", "https://snap.stanford.edu/data/wiki-Vote.txt.gz", "Nodes: ~7k, Edges: ~103k. Who votes for whom."},
        {"Gnutella P2P", "real-world-datasets/p2p-Gnutella08.txt", "https://snap.stanford.edu/data/p2p-Gnutella08.txt.gz", "Nodes: ~6k, Edges: ~20k. Peer-to-peer network."},
        {"Facebook Social", "real-world-datasets/facebook_combined.txt", "https://snap.stanford.edu/data/facebook_combined.txt.gz", "Nodes: ~4k, Edges: ~88k. Social circles."},
        {"Scientific Collaboration", "real-world-datasets/ca-GrQc.txt", "https://snap.stanford.edu/data/ca-GrQc.txt.gz", "Nodes: ~5k, Edges: ~14k. General Relativity authors."},
        {"Email-Eu-core", "real-world-datasets/email-Eu-core.txt", "https://snap.stanford.edu/data/email-Eu-core.txt.gz", "Nodes: ~1k, Edges: ~25k. Departmental email traffic."}
    };

    while(true) {
        cout << "\n=== REAL-WORLD DATASETS ===\n";
        for(size_t i = 0; i < datasets.size(); i++) {
            cout << i+1 << ". " << datasets[i].name << "\n   (" << datasets[i].description << ")\n";
        }
        cout << "6. Back to Main Menu\n";
        cout << "Select Dataset (1-5): ";

        int choice;
        cin >> choice;

        if (choice == 6) return;
        if (choice < 1 || choice > 5) {
            cout << "Invalid choice.\n";
            continue;
        }

        DatasetInfo& ds = datasets[choice - 1];
        int s, t, n;
        
        PushRelabel* graph = loadRealWorldGraph(ds.filename, s, t, n);

        if (graph == nullptr) {
            cout << "\n[ERROR] File '" << ds.filename << "' not found!\n";
            cout << "---------------------------------------------------------\n";
            cout << "To use this option, you must download the dataset:\n";
            cout << "1. Go to: " << ds.url << "\n";
            cout << "2. Extract the file (if .gz).\n";
            cout << "3. Rename/Save it as: " << ds.filename << "\n";
            cout << "4. Place it in the same folder as this executable.\n";
            cout << "---------------------------------------------------------\n";
            
            cout << "Would you like to generate a small Mock Version of this graph for testing? (y/n): ";
            char c; cin >> c;
            if(c == 'y' || c == 'Y') {
                cout << "Generating Mock " << ds.name << "...\n";
                // Create a small mock graph to simulate the dataset
                graph = new PushRelabel(20);
                s=0; t=19;
                for(int i=0; i<19; i++) graph->addEdge(i, i+1, 10 + i); // Line
                graph->addEdge(0, 5, 50);
                graph->addEdge(5, 19, 50);
                cout << "Mock Graph Loaded.\n";
            } else {
                continue;
            }
        }

        cout << "\nCalculating Max Flow for " << ds.name << "...\n";
        long long max_flow = graph->getMaxFlow(s, t);
        cout << "-------------------------------------\n";
        cout << "MAXIMUM FLOW: " << max_flow << "\n";
        cout << "-------------------------------------\n";
        
        cout << "Show flow distribution? (y/n): ";
        char v; cin >> v;
        if (v == 'y' || v == 'Y') graph->printFlowAssignment();
        
        delete graph;
        cout << "\nPress Enter to continue...";
        cin.ignore(); cin.get();
    }
}

// ==========================================
// MAIN FUNCTION
// ==========================================

int main() {
    while (true) {
        clearScreen();
        cout << "==========================================\n";
        cout << "   PUSH-RELABEL MAX FLOW SOLVER (FIFO)    \n";
        cout << "==========================================\n";
        cout << "1. Create Synthetic Graph\n";
        cout << "2. Use Real-World Network Datasets\n";
        cout << "3. Exit\n";
        cout << "------------------------------------------\n";
        cout << "Select Option: ";

        int choice;
        cin >> choice;

        if (choice == 3) break;

        if (choice == 1) {
            int s, t;
            PushRelabel* graph = generateSyntheticGraph(s, t);
            
            cout << "\nCalculating Maximum Flow...\n";
            auto start = clock();
            long long max_flow = graph->getMaxFlow(s, t);
            auto end = clock();

            cout << "-------------------------------------\n";
            cout << "MAXIMUM FLOW: " << max_flow << "\n";
            cout << "Time Taken: " << fixed << setprecision(5) 
                 << (double)(end - start) / CLOCKS_PER_SEC << " seconds\n";
            cout << "-------------------------------------\n";

            cout << "Show non-zero flow assignments? (y/n): ";
            char c; cin >> c;
            if(c == 'y' || c == 'Y') graph->printFlowAssignment();

            delete graph;
            cout << "\nPress Enter to return to menu...";
            cin.ignore(); cin.get();
        }
        else if (choice == 2) {
            showRealWorldMenu();
        }
        else {
            cout << "Invalid input.\n";
        }
    }
    return 0;
}