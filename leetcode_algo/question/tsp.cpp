#include <array>
#include <iostream>
#include <vector>

using namespace std;

namespace Solution {

using PII = pair<int, int>;
constexpr int INF = 1e5;
int min_distance = INF;

vector<vector<PII>> BuildGraph(const vector<vector<char>>& matrix) {
  int rows = matrix.size();
  int cols = matrix[0].size();

  // Up/Down/Left/Right
  array<int, 4> dr = {0, 0, -1, 1};
  array<int, 4> dc = {-1, 1, 0, 0};

  vector<vector<PII>> graph(rows * cols, vector<PII>());
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      if (matrix[r][c] == 'x') continue;

      for (int i = 0; i < 4; ++i) {
        int rr = r + dr[i];
        int cc = c + dc[i];
        if (!(rr >= 0 && rr < rows && cc >= 0 && cc < cols)) continue;
        if (matrix[rr][cc] == 'x') continue;

        // add edges
        int from = r * cols + c;
        int to = rr * cols + cc;
        graph[from].push_back({to, 1});
      }
    }
  }
  return graph;
}

vector<vector<int>> FloydWarshall(const vector<vector<PII>>& graph) {
  int n = graph.size();

  vector<vector<int>> dist(n, vector<int>(n, INF));
  for (int i = 0; i < n; ++i) {
    for (const auto& [to, dis] : graph[i]) {
      dist[i][to] = dis;
    }
  }
  for (int i = 0; i < n; ++i) dist[i][i] = 0;

  cout << "origin dist: " << endl;
  for (const auto& dis : dist) {
    for (const auto& d : dis) {
      if (d == Solution::INF)
        cout << "_"
             << " | ";
      else
        cout << d << " | ";
    }
    cout << endl;
  }
  cout << endl;

  // floyd warshall
  for (int k = 0; k < n; ++k) {
    for (int i = 0; i < n; ++i) {
      for (int j = 0; j < n; ++j) {
        if (dist[i][k] != INF && dist[k][j] != INF &&
            dist[i][k] + dist[k][j] < dist[i][j]) {
          dist[i][j] = dist[i][k] + dist[k][j];
        }
      }
    }
  }

  return dist;
}

// back trace
void BackTrace(const vector<int>& stations, const vector<vector<int>>& dist,
               const int start_point, vector<bool>& st, vector<int>& path) {
  if (path.size() == stations.size()) {
    cout << "path: " << start_point << " -> ";
    int sum = 0;
    for (int i = 0; i < path.size(); ++i) {
      if (i == 0)
        sum += dist[start_point][path[i]];
      else
        sum += dist[path[i - 1]][path[i]];
      cout << path[i] << " | ";
    }
    cout << endl;
    min_distance = min(min_distance, sum);
  }
  for (int i = 0; i < stations.size(); ++i) {
    if (st[i]) continue;
    path.push_back(stations[i]);
    st[i] = true;
    BackTrace(stations, dist, start_point, st, path);
    st[i] = false;
    path.pop_back();
  }
}

int Tsp(const vector<vector<char>>& matrix, const vector<vector<int>>& dist,
        const PII& start_pt) {
  int res = INF;
  int rows = matrix.size();
  int cols = matrix[0].size();

  int start_point = start_pt.first * cols + start_pt.second;

  // start_pt + station
  vector<int> stations;
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      if (matrix[r][c] == '1') {
        stations.push_back(r * cols + c);
      }
    }
  }
  cout << "start_point: " << start_point << endl;
  cout << "stations: " << endl;
  for (const auto& s : stations) cout << s << " | ";
  cout << endl;

  // back trace
  vector<bool> st(stations.size(), false);
  vector<int> path;
  BackTrace(stations, dist, start_point, st, path);

  return min_distance;
}

}  // namespace Solution

int main() {
  vector<vector<char>> matrix = {{'1', 'x', '1'}, {'.', 'x', '.'},{'.','.','1'}};
  cout << "matrix: " << endl;
  for (const auto& ma : matrix) {
    for (const auto& m : ma) {
      cout << m << " | ";
    }
    cout << endl;
  }
  cout << endl;

  /// Build graph
  auto graph = Solution::BuildGraph(matrix);
  cout << "graph: " << endl;
  for (int i = 0; i < graph.size(); ++i) {
    cout << "From: " << i << " "
         << "To: " << endl;
    for (const auto& g : graph[i]) {
      cout << g.first << "," << g.second << " | ";
    }
    cout << endl;
    cout << endl;
  }
  cout << endl;

  /// Folyd
  auto dist = Solution::FloydWarshall(graph);
  cout << "dist: " << endl;
  for (const auto& dis : dist) {
    for (const auto& d : dis) {
      if (d == Solution::INF)
        cout << "_"
             << " | ";
      else
        cout << d << " | ";
    }
    cout << endl;
  }
  cout << endl;

  /// Tsp
  auto res = Solution::Tsp(matrix, dist, {2, 0});
  cout << "Tsp: min distance is " << res << endl;

  return 0;
}