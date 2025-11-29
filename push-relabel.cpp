#include <iostream>
#include <vector>
#include <queue>
#include <algorithm>
#include <climits>
#include <string>
#include <sstream>

using namespace std;

struct Edge {
    int to;
    int rev;
    long long flow;
    long long cap;
};

class PushRelabel {
private:
    int n;
    vector<vector<Edge>> adj;
    vector<long long> excess;
    vector<int> dist;
    vector<int> count;
    vector<bool> active;
    vector<int> head; // current arc
    queue<int> Q;

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
        while (excess[u] > 0) {
            if (head[u] < adj[u].size()) {
                Edge &e = adj[u][head[u]]; // current arc
                if (e.cap - e.flow > 0 && dist[u] == dist[e.to] + 1) {
                    push(u, e);
                } else {
                    head[u]++; 
                }
            } else {
                if (count[dist[u]] == 1) {
                    gap(dist[u]);
                } else {
                    relabel(u);
                }
                head[u] = 0;
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
        head.resize(n, 0);
    }

    void addEdge(int u, int v, long long cap) {
        if (u == v) return;
        adj[u].push_back({v, (int)adj[v].size(), 0, cap});
        adj[v].push_back({u, (int)adj[u].size() - 1, 0, 0});
    }

    long long getMaxFlow(int s, int t) {
        if (s == -1 || t == -1) return 0;
        
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
};

int main() {
    ios_base::sync_with_stdio(false);
    cin.tie(NULL);

    PushRelabel* solver = nullptr;
    int s = -1, t = -1;
    char type;
    string line;

    while (getline(cin, line)) {
        if (line.empty()) continue;
        stringstream ss(line);
        ss >> type;

        if (type == 'c') {
            continue;
        }
        else if (type == 'p') {
            string fmt;
            int num_nodes, num_edges;
            ss >> fmt >> num_nodes >> num_edges;
            solver = new PushRelabel(num_nodes);
        }
        else if (type == 'n') {
            int id;
            char nodeType;
            ss >> id >> nodeType;
            if (nodeType == 's') s = id - 1;
            if (nodeType == 't') t = id - 1;
        }
        else if (type == 'a') {
            int u, v;
            long long cap;
            ss >> u >> v >> cap;
            if (solver) solver->addEdge(u - 1, v - 1, cap);
        }
    }

    if (solver && s != -1 && t != -1) {
        cout << solver->getMaxFlow(s, t) << endl;
    } else {
        if (solver) {
             cerr << "Error: Source (s) or Sink (t) not defined in input." << endl;
        }
    }

    delete solver;
    return 0;
}
