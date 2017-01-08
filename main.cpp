#include <tuple>
#include <iostream>
#include <string>
#include<fstream>
#include <vector>
#include <math.h>
#include <queue>
#include <algorithm>

using namespace std;

typedef pair<int,int> coord;
typedef vector<vector<int>> graph;
struct problem {
	int n;
	int nobs;
	double t;
	vector<coord> points;
	vector<coord> obstacle_vert;
};

problem pr;

struct edge {
	int p1;
	int p2;
	double dist;
	//integer when over number of points then is an obstacle_vert
	vector<int> shortest_path;
	bool straight_line;

	bool operator > (const edge& edg) const {
		return(dist>edg.dist);
	}
};

class CompareDist
{
public:
    bool operator()(pair<int,int> n1,pair<int,int> n2) {
        return n1.first>n2.first;
    }
};

void readFile(char filename[]) {
	cout << "Reading dataset: " << filename << endl;
	ifstream file(filename);
	int n, nobs,a,b;
	file >> n;
	file >> nobs;
	file >> a >> b;

	vector<coord> points(n);
	vector<coord> obstacle_vert(nobs);
	problem prbl;
	prbl.n = n;
	prbl.nobs = nobs;
	prbl.t = a/b;
	cout << "problem parameters " << prbl.n << " " << prbl.nobs << " "<< prbl.t << endl;
	int coord1,coord2;

	if (file.is_open()) {
		for(int i=0;i<n;++i){
			file >> coord1 >> coord2;
			points[i]=coord(coord1,coord2);
		}
		for(int i=0;i<nobs;++i){
			file >> coord1 >> coord2;
			obstacle_vert[i]=coord(coord1,coord2);
		}
	}
	prbl.points = points;
	prbl.obstacle_vert = obstacle_vert;
	file.close();
	pr = prbl;
}

double euclidean_distance(coord p1, coord p2) {
	return sqrt(pow(double(p1.first-p2.first),2)+pow(double(p1.second-p2.second),2));
}

coord getpoint(int i) {
	if(i<pr.n) {
		return pr.points[i];
	} else {
		return pr.obstacle_vert[i-pr.n];
	}
}

double dijkstra_shortest_path(graph& tspanner, int source, int target, vector<int>& path) {
	vector<double> distances(pr.n+pr.nobs,-1);
	vector<int> previous(pr.n+pr.nobs,-1);
	vector<bool> visited(pr.n+pr.nobs,false);
	priority_queue<pair<int,int>,vector<pair<int,int>>,CompareDist> pq;
	//priority queue has the pair: priority,point_number
	pq.push(make_pair(0,source));
	distances[source]=0;
	pair<int,int> prior_point;
	vector<int> neighbors;
	double alt_dist;
	while(!pq.empty()){
		prior_point = pq.top();
		pq.pop();
		if(prior_point.second == target) {
			int curr=target;
			while(curr!=source) {
				path.push_back(curr);
				curr=previous[curr];
			}
			path.push_back(curr);
			return distances[target];
		}
		neighbors = tspanner[prior_point.second];
		if(!visited[prior_point.second]) {
			visited[prior_point.second] = true;
			for(int i=0;i<neighbors.size();++i) {
				alt_dist = distances[prior_point.second]+euclidean_distance(getpoint(prior_point.second),getpoint(neighbors[i]));
				if(distances[neighbors[i]]==-1 || alt_dist<distances[neighbors[i]]) {
					distances[neighbors[i]] = alt_dist;
					previous[neighbors[i]] = prior_point.second;
					pq.push(make_pair(alt_dist,neighbors[i]));
				}
			}
		}
	}

	return distances[target];
}

void insert_edge(graph& tspanner, int p1, int p2) {
	tspanner[p1].push_back(p2);
	tspanner[p2].push_back(p1);
}

int orientation_3_points(coord p1,coord p2, coord p3) {
    int val = (p2.second - p1.second) * (p3.first - p2.first) -
              (p2.first - p1.first) * (p3.second - p2.second);

    if (val == 0) return 0;  // colinear points

    return (val > 0)? 1: 2; // clock or counterclock wise order
}

bool edges_intersect(coord p1, coord p2, coord v1, coord v2) {
	// Find the four orientations needed for general and
	// special cases
	int o1 = orientation_3_points(p1, p2, v1);
	int o2 = orientation_3_points(p1, p2, v2);
	int o3 = orientation_3_points(v1, v2, p1);
	int o4 = orientation_3_points(v1, v2, p2);

	// General case
	if (o1 != o2 && o3 != o4)
		return true;

	//This assumes general position of points

	return false;
}

bool edge_intersects_obstacle(int p1, int p2){
	if(edges_intersect(getpoint(p1),getpoint(p2),getpoint(pr.n+pr.nobs-1),getpoint(pr.n))) {
		return true;
	}
	for(int i=1;i<pr.nobs;++i) {
		if(edges_intersect(getpoint(p1),getpoint(p2),getpoint(pr.n+i-1),getpoint(pr.n+i))) {
			return true;
		}
	}
	return false;
}

vector<edge> find_all_edges() {
	vector<edge> all_edges;
	edge edg;
	for(int i=0;i<pr.n;++i){
		for(int j=i+1;j<pr.n;++j) {
			edg.p1 = i;
			edg.p2 = j;
			if(edge_intersects_obstacle(i, j)) {
				edg.straight_line = false;
				//TODO: visibility graph and finding path
				//If an edge intersects any other edge of the obstacle then do visibility graph and compute
				//distance using dijsktra in the visibility graph between the two points (including the two points
				//and the obstacle vertices, adding only the edges that do not cross the obstacle).
			}
			else {
				edg.dist = euclidean_distance(getpoint(i), getpoint(j));
				edg.straight_line = true;
				all_edges.push_back(edg);
			}
		}
	}

	sort(all_edges.begin(),all_edges.end(),greater<edge>());
	return all_edges;
}

void greedy_tspanner(graph& tspanner, vector<edge> edges){
	double dist;
	vector<int> shortest_path;
	vector<int> unused;
	for(int i=0;i<edges.size();++i) {
		dist = dijkstra_shortest_path(tspanner, edges[i].p1,edges[i].p2,unused);
		if(dist>pr.t*euclidean_distance(getpoint(edges[i].p1),getpoint(edges[i].p2))) {
			if(edges[i].straight_line) {
				insert_edge(tspanner, edges[i].p1, edges[i].p2);
			} else {
				shortest_path = edges[i].shortest_path;
				for(int j=1;j<shortest_path.size();++j) {
					insert_edge(tspanner,shortest_path[j-1],shortest_path[j]);
				}
			}
		}
	}
}

void print_vector(vector<int> vec) {
	cout<<"path"<<endl;
	for(int i=0;i<vec.size();++i) {
		cout<<vec[i]<<endl;
	}
}

int main() {
	char filename[] = "Data_examples/geometricspanners.txt";
	readFile(filename);
	graph tspanner(pr.n+pr.nobs, vector<int>(0));
	/*graph tspanner(4, vector<int>(0));
//	Example of dijkstra distance
	vector<coord> points = {make_pair(0,0),make_pair(0,1),make_pair(1,1),make_pair(1,2)};
	pr.points = points;
	pr.n=4;
    insert_edge(tspanner, 0, 1);
    insert_edge(tspanner,1, 2);
    insert_edge(tspanner,2,3);
    //insert_edge(tspanner,1,2);

    vector<int> path1;
    cout << dijkstra_shortest_path(tspanner, 1,0,path1) << endl;
    print_vector(path1);
    cout << "aaaaaaaa" << endl;
    vector<int> path2;
    cout << dijkstra_shortest_path(tspanner, 2,0,path2) << endl;
    print_vector(path2);
    cout << "aaaaaaaa" << endl;
    vector<int> path3;
    cout << dijkstra_shortest_path(tspanner, 0,3,path3) << endl;
    print_vector(path3);
    cout << "aaaaaaaa" << endl;
    vector<int> path4;
    cout << dijkstra_shortest_path(tspanner, 2,3,path4) << endl;
    print_vector(path4);
    cout << "aaaaaaaa" << endl;
    vector<int> path5;
    cout << dijkstra_shortest_path(tspanner, 1,3,path5) << endl;
    print_vector(path5);*/

	vector<edge> all_edges_sorted = find_all_edges();

	greedy_tspanner(tspanner,all_edges_sorted);


}
