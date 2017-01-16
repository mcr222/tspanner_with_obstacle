#include <tuple>
#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <math.h>
#include <queue>
#include <algorithm>
#include <set>
#include <limits>
#include <time.h>
#include <map>
#include "data.h"

using namespace std;

typedef pair<double,double> coord;
typedef vector<set<int>> graph;
struct problem {
	int n;
	int nobs;
	double t;
	vector<coord> points;
	vector<coord> obstacle_vert;
};

vector<coord> additional_points(2);

problem pr;

struct edge {
	int p1;
	int p2;
	double dist;
	//integer when over number of points then is an obstacle_vert
	vector<int> shortest_path;
	bool straight_line;

	bool operator < (const edge& edg) const {
		return(dist<edg.dist);
	}

	//tested
	bool operator == (const edge& edg) const {
			return((edg.p1==p1 && edg.p2==p2)||
					(edg.p1==p2 && edg.p2==p1));
	}
};

double euclidean_distance(coord p1, coord p2) {
	return sqrt(pow(double(p1.first-p2.first),2)+pow(double(p1.second-p2.second),2));
}

coord getpoint(int i) {
	if(i<pr.n) {
		return pr.points[i];
	} else if(i<(pr.n+pr.nobs)){
		return pr.obstacle_vert[i-pr.n];
	} else {
		return additional_points[i-pr.n-pr.nobs];
	}
}

struct event {
	int p;
	double angle;
	bool starting;
	edge segment;
	//this is for when points have the same angle
	int p_view;

	bool operator < (const event& evt) const {
		if(angle==evt.angle) {
			return (euclidean_distance(getpoint(p),getpoint(p_view))<euclidean_distance(getpoint(evt.p),getpoint(evt.p_view)));
		}
		return(angle<evt.angle);
	}
};

class CompareDist
{
public:
    bool operator()(pair<int,int> n1,pair<int,int> n2) {
        return n1.first>n2.first;
    }
};

//tested
void readFile(string filename) {
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
	prbl.t = a/(double)b;
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

//tested
double dijkstra_shortest_path(graph& tspanner, int source, int target, vector<int>& path) {
	vector<double> distances(pr.n+pr.nobs+2,-1);
	vector<int> previous(pr.n+pr.nobs+2,-1);
	vector<bool> visited(pr.n+pr.nobs+2,false);
	priority_queue<pair<int,int>,vector<pair<int,int>>,CompareDist> pq;
	//priority queue has the pair: priority,point_number
	pq.push(make_pair(0,source));
	distances[source]=0;
	pair<int,int> prior_point;
	set<int> neighbors;
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
			for(set<int>::iterator it=neighbors.begin();it!=neighbors.end();++it) {
				alt_dist = distances[prior_point.second]+euclidean_distance(getpoint(prior_point.second),getpoint(*it));
				if(distances[*it]==-1 || alt_dist<distances[*it]) {
					distances[*it] = alt_dist;
					previous[*it] = prior_point.second;
					pq.push(make_pair(alt_dist,*it));
				}
			}
		}
	}
	return distances[target];
}

//tested
void insert_edge(graph& tspanner, int p1, int p2) {
	tspanner[p1].insert(p2);
	tspanner[p2].insert(p1);
}

int orientation_3_points(coord p1,coord p2, coord p3) {
    int val = (p2.second - p1.second) * (p3.first - p2.first) -
              (p2.first - p1.first) * (p3.second - p2.second);

    if (val == 0) return 0;  // colinear points

    return (val > 0)? 1: 2; // clock or counterclock wise order
}

//tested
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

bool same_direction_vectors(coord v1, coord v2) {
	return (v1.second*v2.first==v1.first*v2.second && (v1.first*v2.first+v1.second*v2.second>0));
}
//tested
double compute_angle(coord p1, coord p, coord p2) {
	coord v1 = make_pair(p2.first-p.first,p2.second-p.second);
	coord v2 = make_pair(p1.first-p.first,p1.second-p.second);
	if(same_direction_vectors(v1,v2)) {
		return 0;
	}
	int dot = v1.first*v2.first + v1.second*v2.second;
	int det = v1.first*v2.second - v1.second*v2.first;
	return atan2(-det, -dot)+M_PI;
}

//tested
bool point_in_segment(coord inters, coord p1, coord p2) {
	coord v = make_pair(p1.first-p2.first,p1.second-p2.second);
	double t;
	if(v.first!=0) {
		t=(inters.first-p2.first)/v.first;
	} else {
		t=(inters.second-p2.second)/v.second;
	}
	//cout << "t: " << t << endl;
	return (0<=t && t<=1);
}

void insert_segment_events(vector<event>& events,int i, int p, int obst1, int obst2, coord infinity_point) {
	event evt1;
	event evt2;
	edge segment;
	segment.p1 = obst1;
	segment.p2 = obst2;
	segment.straight_line=true;
	evt1.segment = segment;
	evt2.segment = segment;
	evt1.p = obst1;
	evt2.p = obst2;
	evt1.angle = compute_angle(infinity_point,getpoint(p),getpoint(obst1));
	evt2.angle = compute_angle(infinity_point,getpoint(p),getpoint(obst2));
	evt1.p_view = p;
	evt2.p_view = p;
	if(edges_intersect(getpoint(p),infinity_point,getpoint(obst1),getpoint(obst2))
			&& evt1.angle!=0 &&
			evt2.angle!=0) {
		if(evt1.angle<evt2.angle) {
			evt1.starting=false;
			evt2.starting=true;
		} else if(evt1.angle>evt2.angle) {
			evt1.starting=true;
			evt2.starting=false;
		} else if (euclidean_distance(getpoint(evt1.p),getpoint(p))<euclidean_distance(getpoint(evt2.p),getpoint(p))){
			evt1.starting=true;
			evt2.starting=false;
		} else {
			evt1.starting=false;
			evt2.starting=true;
		}
	}
	else if(evt1.angle!=0 && evt2.angle!=0){
		if(evt1.angle<evt2.angle) {
			evt1.starting=true;
			evt2.starting=false;
		} else if(evt1.angle>evt2.angle){
			evt1.starting=false;
			evt2.starting=true;
		} else if (euclidean_distance(getpoint(evt1.p),getpoint(p))<euclidean_distance(getpoint(evt2.p),getpoint(p))){
			evt1.starting=true;
			evt2.starting=false;
		} else {
			evt1.starting=false;
			evt2.starting=true;
		}
	} else {
		if(evt1.angle==0) {
			if(evt2.angle<M_PI) {
				evt1.starting = true;
				evt2.starting = false;
			} else {
				evt1.starting = false;
				evt2.starting = true;
			}
		} else {
			if(evt1.angle<M_PI) {
				evt1.starting = false;
				evt2.starting = true;
			} else {
				evt1.starting = true;
				evt2.starting = false;
			}
		}
	}
	events[i] = evt1;
	events[i+pr.nobs] = evt2;
}

//tested
coord find_intersection_lines(coord p1, coord p2, coord v1, coord v2) {
	//RETURNS -nan if lines are parallel
	double n1,d,n2;
	n1=((p1.first*p2.second-p1.second*p2.first)*(v1.first-v2.first)-(p1.first-p2.first)*(v1.first*v2.second-v1.second*v2.first));
	n2=((p1.first*p2.second-p1.second*p2.first)*(v1.second-v2.second)-(p1.second-p2.second)*(v1.first*v2.second-v1.second*v2.first));
	d=((p1.first-p2.first)*(v1.second-v2.second)-(p1.second-p2.second)*(v1.first-v2.first));
	return make_pair(n1/d,n2/d);
}

//tested
bool positive_intersection(coord p, coord p1, coord inters) {
	coord v = make_pair(p1.first-p.first,p1.second-p.second);
	coord vinters = make_pair(inters.first-p.first,inters.second-p.second);
	if(false && p.first==1042 && p.second==1462) {
		cout << inters.first << " " << inters.second << endl;
		cout << p.first << " " << p.second << endl;
		cout << p1.first << " " << p1.second << endl;
		cout << v.first << " " << v.second << endl;
		cout << vinters.first << " " << vinters.second << endl;
		cout << v.first*vinters.first+v.second*vinters.second << endl;
	}

	return v.first*vinters.first+v.second*vinters.second>0;
}

//tested
bool point_inside_obstacle(coord p) {
	//cout << "point inside: " << p.first << " " << p.second << endl;
	//TODO: assumes general position of points returns false if point on edge
	//return false if point on edge because then (as points can't be inside obstacle)
	//then points will be in different "sides" of the obstacle thus not well separated probably
	//return true;
	coord infinity_point=make_pair(0,100000);
	int inters=0;
	coord intersection;
	if(edges_intersect(p,infinity_point,getpoint(pr.n+pr.nobs-1),getpoint(pr.n))) {
		if(!point_in_segment(p, getpoint(pr.n+pr.nobs-1),getpoint(pr.n))) {
			++inters;
		} else {
			return true;
		}
	}
	for(int i=1;i<pr.nobs;++i) {
		if(edges_intersect(p,infinity_point,getpoint(pr.n+i-1),getpoint(pr.n+i))) {
			if(!point_in_segment(p, getpoint(pr.n+pr.nobs-1),getpoint(pr.n))) {
				++inters;
			} else {
				return true;
			}
		}
	}
	if(inters%2==0) {
		return false;
	}
	return true;
}

struct status_segment {
	int p1;
	int p2;
	int p;

	//tested
	//OBS: if the touch (share one point p1 or p2), then they will be ordered randomly (but deterministically)
	//but because the shared point will be processed twice, then it will be at the bottom
	//for one of the segments processed
	bool operator < (const status_segment seg) const {
		//as they are status segments, they will overlap from the points p view
		//coord intersection = find_intersection(getpoint(p),infinity_point,getpoint(p1),getpoint(p2));
		int k=-20;
		if((seg.p1==p1 && seg.p2==p2)||
				(seg.p1==p2 && seg.p2==p1)) {
			//cout << "first false" << endl;
			return false;
		}

		coord inters;
		if(p==k) {
			cout << p1 << " " << p2 << " " << seg.p1 << " " << seg.p2 << endl;
		}
		if(seg.p1!= p1 && seg.p1!=p2) {
			if(edges_intersect(getpoint(p),getpoint(seg.p1),getpoint(p1),getpoint(p2))) {

				if(p==k) {
					cout << "1st true" << endl;
				}
				return true;
			}
			else {
				if(p==k) {
					cout << "inters1 "<< seg.p1 << endl;
				}
				inters = find_intersection_lines(getpoint(p),getpoint(seg.p1),getpoint(p1),getpoint(p2));
				if(p==k) {
					cout <<"inters "<< inters.first << " " << inters.second << endl;
					cout << "p "<< getpoint(p).first << " " << getpoint(p).second << endl;
					cout << "p2 "<< getpoint(p2).first << " " << getpoint(p2).second << endl;
					cout << "seg.p1 "<< getpoint(seg.p1).first << " " << getpoint(seg.p1).second << endl;
					cout << "seg.p2 "<< getpoint(seg.p2).first << " " << getpoint(seg.p2).second << endl;
					cout <<"positive int "<< positive_intersection(getpoint(p), getpoint(seg.p1), inters) << endl;
					cout << "point in seg"<< point_in_segment(inters,getpoint(p1),getpoint(p2)) << endl;
				}
				if(positive_intersection(getpoint(p), getpoint(seg.p1), inters)
						&& point_in_segment(inters,getpoint(p1),getpoint(p2))) {
					return false;
				}
			}
		}
		if(seg.p2!= p1 && seg.p2!=p2) {
			if(edges_intersect(getpoint(p),getpoint(seg.p2),getpoint(p1),getpoint(p2))) {

				if(p==k) {
					cout << "2nd true" << endl;
				}
				return true;
			}
			else {
				if(p==k) {
					cout << "inters2" << endl;
				}
				inters = find_intersection_lines(getpoint(p),getpoint(seg.p2),getpoint(p1),getpoint(p2));
				if(positive_intersection(getpoint(p), getpoint(seg.p2), inters) &&
						point_in_segment(inters,getpoint(p1),getpoint(p2))) {
					if(p==k) {
							cout << "inters2 false" << endl;
					}
					return false;
				}
			}

		}
		if(p1!= seg.p1 && p1!=seg.p2) {
			if(edges_intersect(getpoint(p),getpoint(p1),getpoint(seg.p1),getpoint(seg.p2))) {
				if(p==k) {
					cout << "1st false" << endl;
				}
				return false;
			}
			else {
				if(p==k) {
					cout << "inters3" << endl;
				}
				inters = find_intersection_lines(getpoint(p),getpoint(p1),getpoint(seg.p1),getpoint(seg.p2));
				if(positive_intersection(getpoint(p), getpoint(p1), inters) &&
						point_in_segment(inters,getpoint(seg.p1),getpoint(seg.p2))) {
					if(p==k) {
						cout << "inters3 true" << endl;
					}
					return true;
				}
			}

		}
		if(p2!= seg.p1 && p2!=seg.p2) {
			if(edges_intersect(getpoint(p),getpoint(p2),getpoint(seg.p1),getpoint(seg.p2))) {
				if(p==k) {
					cout << "2nd false" << endl;
				}
				return false;
			}
			else {
				if(p==k) {
					cout << "inters4" << endl;
				}
				inters = find_intersection_lines(getpoint(p),getpoint(p2),getpoint(seg.p1),getpoint(seg.p2));
				if(p==k) {
					cout <<"inters "<< inters.first << " " << inters.second << endl;
					cout << "p "<< getpoint(p).first << " " << getpoint(p).second << endl;
					cout << "p2 "<< getpoint(p2).first << " " << getpoint(p2).second << endl;
					cout << "seg.p1 "<< getpoint(seg.p1).first << " " << getpoint(seg.p1).second << endl;
					cout << "seg.p2 "<< getpoint(seg.p2).first << " " << getpoint(seg.p2).second << endl;
					cout <<"positive int "<< positive_intersection(getpoint(p), getpoint(p2), inters) << endl;
					cout << "point in seg"<< point_in_segment(inters,getpoint(seg.p1),getpoint(seg.p2)) << endl;

				}
				if(positive_intersection(getpoint(p), getpoint(p2), inters) &&
						point_in_segment(inters,getpoint(seg.p1),getpoint(seg.p2))) {
					if(p==k) {
						cout << "inters4 true" << endl;
					}
					return true;
				}
			}
		}


		//this is reached when segments share an edge and are ordered sideways
		//thus they do not have an order from the p point of view but must be
		//ordered anyway to be able to search in the set (binary search tree).
		//cout << p1 << " " << p2 << " " << seg.p1 << " " << seg.p2 << endl;
		string this_seg = to_string(p1)+to_string(p2);
		string other_seg = to_string(seg.p1)+to_string(seg.p2);
		if(this_seg<other_seg) {
			if(p==k) {
				cout << "last true" << endl;
			}
			return true;
		} else {
			if(p==k) {
				cout << "last false" << endl;
			}
			return false;
		}
	}

};

void initiate_status_segment(set<status_segment>& status, int p, int obs1, int obs2, coord infinity_point) {
	/*if(p==7) {
		cout << "initiating: " << p << " obs1: " << obs1 << " obs2: " << obs2 << endl;
		cout << "cond: " << edges_intersect(getpoint(p),infinity_point,getpoint(obs1),getpoint(obs2)) <<
				" " << (compute_angle(getpoint(obs1), getpoint(p), infinity_point)!=0.0) << " " <<
				(compute_angle(getpoint(obs2), getpoint(p), infinity_point)!=0.0) << endl;
	}*/
	if(edges_intersect(getpoint(p),infinity_point,getpoint(obs1),getpoint(obs2)) &&
		compute_angle(getpoint(obs1), getpoint(p), infinity_point)!=0 &&
		compute_angle(getpoint(obs2), getpoint(p), infinity_point)!=0 ) {
		status_segment edg;
		edg.p1 = obs1;
		edg.p2 = obs2;
		edg.p = p;
		status.insert(edg);
	}
}

void initiate_status(set<status_segment>& status, int p, coord infinity_point){
	if(p!=pr.n+pr.nobs-1 && p!=pr.n) {
		initiate_status_segment(status, p, pr.n+pr.nobs-1, pr.n, infinity_point);
	}
	for(int i=1;i<pr.nobs;++i) {
		if(p!=pr.n+i-1 && p!=pr.n+i) {
			initiate_status_segment(status, p, pr.n+i-1, pr.n+i, infinity_point);
		}
	}
}

void print_vector(vector<event> vec) {
	for(int i=0;i<vec.size();++i) {
		cout << " p "<<vec[i].p << " angle: " <<vec[i].angle<< " p1: " <<vec[i].segment.p1<< " p2: " <<vec[i].segment.p2
				<< " start: " <<vec[i].starting;
		cout << endl;
	}
	cout << endl;
}

void visibility_point(graph& visibility, int p) {
	int k=-50;
	if(p==k) {
		cout << "point: " << p << endl;
		cout << "point: " << getpoint(p).first << " "<< getpoint(p).second << endl;
	}
	vector<event> events(2*pr.nobs);
	coord infinity_point(0,100000);
	insert_segment_events(events,0, p, pr.n+pr.nobs-1, pr.n,infinity_point);
	for(int i=1;i<pr.nobs;++i) {
		insert_segment_events(events,i, p, pr.n+i-1, pr.n+i,infinity_point);
	}
	sort(events.begin(),events.end(),less<event>());
	if(p==k) {
		print_vector(events);
	}

	set<status_segment> status;
	initiate_status(status, p,infinity_point);
	if(p==k) {
		cout << "initial status: " << endl;
		for(set<status_segment>::iterator it = status.begin();it!=status.end();++it) {
			cout <<(*it).p1 << " " << (*it).p2 << endl;
		}
	}

	event evt;
	status_segment seg, closest_seg;
	edge closest_edge;
	set<int> visible_points;
	seg.p = p;
	for(int i=0;i<events.size();++i) {
		evt = events[i];
		seg.p1 = evt.segment.p1;
		seg.p2 = evt.segment.p2;
		if(p==k) {
			cout << "status "<< evt.p << endl;
		}
		if(evt.starting) {
			if(p==k) {
				cout << "inserting: " << seg.p1 << seg.p2 << endl;
			}
			status.insert(seg);
		} else {
			if(p==k) {
				cout << "erasing: " << seg.p1 << seg.p2 << endl;
			}

			status.erase(seg);
		}

		if(p==k) {
			for(set<status_segment>::iterator it = status.begin();it!=status.end();++it) {
				cout <<(*it).p1 << " " << (*it).p2 << endl;
			}
		}

		closest_seg = *(status.begin());
		closest_edge.p1 = closest_seg.p1;
		closest_edge.p2 = closest_seg.p2;

		if(status.size()==0 || evt.segment == closest_edge) {
			if(p==k) {
				cout << "visible: " << evt.p << endl;
			}
			visible_points.insert(evt.p);
		}
	}

	for(set<int>::iterator it=visible_points.begin();it!=visible_points.end();++it) {
		if(p!=*it) {
			insert_edge(visibility, p, *it);
		}
	}

}

void print_vector(vector<int> vec) {
	for(int i=0;i<vec.size();++i) {
		cout<<vec[i] << " ";
	}
	cout << endl;
}

void visibility_point_obstacle(graph& visibility, int p, int previous, int next) {
	int k = -20;
	if(p==k) {
		cout << "obstacle point: " << p << endl;
		cout << "obstacle point: " << getpoint(p).first << " " << getpoint(p).second << endl;
		cout << "previous: " << previous << " next: " << next << endl;
	}
	vector<event> events(2*pr.nobs);
	double max_angle =compute_angle(getpoint(previous),getpoint(p),getpoint(next));
	if(p==k) {
		cout << "max angle: " << max_angle << endl;
	}
	//as the points are ordered clockwise we will always go from previous to next and we will be
	//rotating on the outside of the polygon
	int mult=1000;
	coord infinity_point = make_pair(mult*getpoint(previous).first+(1-mult)*getpoint(p).first,mult*getpoint(previous).second+(1-mult)*getpoint(p).second);

	insert_segment_events(events,0, p, pr.n+pr.nobs-1, pr.n, infinity_point);
	for(int i=1;i<pr.nobs;++i) {
		insert_segment_events(events,i, p, pr.n+i-1, pr.n+i, infinity_point);
	}
	sort(events.begin(),events.end(),less<event>());
	if(p==k) {
		print_vector(events);
	}

	set<status_segment> status;
	initiate_status(status, p, infinity_point);
	if(p==k) {
		cout << "initial status: " << endl;
		for(set<status_segment>::iterator it = status.begin();it!=status.end();++it) {
			cout <<(*it).p1 << " " << (*it).p2 << endl;
		}
	}

	event evt;
	status_segment seg, closest_seg;
	edge closest_edge;
	set<int> visible_points;
	seg.p = p;
	for(int i=0;i<events.size();++i) {
		evt = events[i];
		if(evt.angle < max_angle && evt.p!=p){
			seg.p1 = evt.segment.p1;
			seg.p2 = evt.segment.p2;
			if(p==k) {
				cout << "status "<< evt.p << endl;
			}
			if(evt.starting) {
				if(p==k) {
					cout << "inserting: " << seg.p1 << seg.p2 << endl;
				}
				status.insert(seg);
			} else {
				if(p==k) {
					cout << "erasing: " << seg.p1 << seg.p2 << endl;
				}
				status.erase(seg);
			}
			if(p==k) {
				for(set<status_segment>::iterator it = status.begin();it!=status.end();++it) {
					cout <<(*it).p1 << " " << (*it).p2 << endl;
				}
			}

			closest_seg = *(status.begin());
			closest_edge.p1 = closest_seg.p1;
			closest_edge.p2 = closest_seg.p2;

			if(status.size()==0 || evt.segment == closest_edge) {
				if(p==k) {
					cout << "visible: " << evt.p << endl;
				}
				visible_points.insert(evt.p);
			}
		}
	}

	for(set<int>::iterator it=visible_points.begin();it!=visible_points.end();++it) {
		if(p!=*it) {
			insert_edge(visibility, p, *it);
		}
	}

}

bool first = true;
graph obstacle_visibility;

graph insert_obstacle_edges() {
	graph vis(pr.n+pr.nobs+2, set<int>());
	if(first) {
		graph visibility(pr.n+pr.nobs, set<int>());
		insert_edge(visibility,pr.n+pr.nobs-1,pr.n);
		for(int i=1;i<pr.nobs;++i) {
			insert_edge(visibility,pr.n+i-1,pr.n+i);
		}


		visibility_point_obstacle(visibility,pr.n,pr.n+pr.nobs-1,pr.n+1);
		for(int i=1;i<pr.nobs-1;++i) {
			visibility_point_obstacle(visibility,pr.n+i,pr.n+i-1,pr.n+i+1);
		}
		visibility_point_obstacle(visibility,pr.n+pr.nobs-1,pr.n+pr.nobs-2,pr.n);

		//cout << "visibility obstacle" << endl;
		/*for(int i = 0;i<visibility.size();++i) {
			cout << "row: " << i << endl;
			for(set<int>::iterator it=visibility[i].begin();it!=visibility[i].end();++it) {
				cout << *it << " ";
			}
			cout << endl;
		}*/
		first = false;
		obstacle_visibility = visibility;
	}
	if(!first){
		for(int i = 0;i<obstacle_visibility.size();++i) {
			for(set<int>::iterator it=obstacle_visibility[i].begin();it!=obstacle_visibility[i].end();++it) {
				vis[i].insert(*it);
			}
		}
	}
	return vis;
}

graph visibility_graph(int p1, int p2) {
	graph visibility;
	visibility = insert_obstacle_edges();
	visibility_point(visibility,p1);
	visibility_point(visibility,p2);
	return visibility;
}


vector<edge> find_all_edges() {
	vector<edge> all_edges;
	edge edg;
	graph vis_gr;
	double dist;
	for(int i=0;i<pr.n;++i){
		for(int j=i+1;j<pr.n;++j) {
			edg.p1 = i;
			edg.p2 = j;
			if(edge_intersects_obstacle(i, j)) {
				//cout << edg.p1 << " " << edg.p2 << endl;
				edg.straight_line = false;
				vis_gr = visibility_graph(i,j);
				/*for(int i = 0;i<vis_gr.size();++i) {
						cout << "row: " << i << endl;
						print_vector(vis_gr[i]);
				}*/
				vector<int> path;
				dist = dijkstra_shortest_path(vis_gr,i,j,path);
				//print_vector(path);
				//cout << "distance: " << dist << endl;
				edg.dist = dist;
				edg.shortest_path = path;
			}
			else {
				edg.dist = euclidean_distance(getpoint(i), getpoint(j));
				edg.straight_line = true;
			}
			all_edges.push_back(edg);
		}
	}
	sort(all_edges.begin(),all_edges.end(),less<edge>());
	return all_edges;
}

void greedy_tspanner(graph& tspanner, vector<edge> edges){
	double dist;
	vector<int> shortest_path;
	vector<int> unused;
	for(int i=0;i<edges.size();++i) {
//		if(i%100==0) {
//			cout << i << endl;
//		}
		dist = dijkstra_shortest_path(tspanner, edges[i].p1,edges[i].p2,unused);
		//cout << dist/euclidean_distance(getpoint(edges[i].p1),getpoint(edges[i].p2)) << endl;
		//dist>pr.t*euclidean_distance(getpoint(edges[i].p1),getpoint(edges[i].p2))
		if(dist == -1 || dist>pr.t*edges[i].dist) {
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

struct result {
	double dilation;
	int size;
	double weight;
	double execution_time;
};

void writeFile(string filename, graph& graph, result& res) {
	cout << "Writting output"<< endl;
    ifstream file(filename);
    string str;
    ofstream outputFile("output.txt");
    while (getline(file, str))
    {
        outputFile << str << endl;
    }

    for(int i=0;i<graph.size();++i) {
    	for(set<int>::iterator it=graph[i].begin();it!=graph[i].end();++it){
    		outputFile << " " << *it;
    	}
    	outputFile << endl;
    }

    outputFile << res.dilation << endl;
    outputFile << res.size << endl;
    outputFile << res.weight << endl;
    outputFile << res.execution_time << endl;


}

result computeResultParameters(graph& tspanner, vector<edge>& edges) {
	result res;

	double dist,dilation_max=0,dil;
	vector<int> unused;
	for(int i=0;i<edges.size();++i) {
		dist = dijkstra_shortest_path(tspanner, edges[i].p1,edges[i].p2,unused);

		dil=dist/edges[i].dist;
		cout << edges[i].p1 << " " << edges[i].p2 << endl;
		cout <<"p1 "<< getpoint(edges[i].p1).first << " " << getpoint(edges[i].p1).second << endl;
		cout <<"p2 " << getpoint(edges[i].p2).first<< " " << getpoint(edges[i].p2).second << endl;
		cout << dil<< endl;
		if(dil>dilation_max) {
			dilation_max=dil;
		}
		//cout << edges[i].p1 << " " << edges[i].p2 << " " << dil << endl;
	}
	res.dilation = dilation_max;

	int size=0;
	double weight=0;
	for(int i=0;i<tspanner.size();++i) {
		for(set<int>::iterator it=tspanner[i].begin();it!=tspanner[i].end();++it) {
			++size;
			weight += euclidean_distance(getpoint(*it),getpoint(i));
		}
	}

	res.size = size/(double)2;
	res.weight = weight/2;

	return res;
}

//
//
//
//
//

class Quadtree
{
	public:
        //4 children
        Quadtree* nw;
        Quadtree* ne;
        Quadtree* sw;
        Quadtree* se;

        vector<coord> boundary;
        vector<vector<coord>> containedPoints;
        double cellArea;


        Quadtree();
        Quadtree(vector<coord> boundary);
        Quadtree(vector<coord> boundary, vector<vector<coord>>containedPoints);

        ~Quadtree();

        void setNEChild(vector<coord> childBoundary, vector<vector<coord>> childPoints);

};

Quadtree::Quadtree()
{
    nw = NULL;
    ne = NULL;
    sw = NULL;
    se = NULL;
    boundary = std::vector<coord>();
    containedPoints = std::vector<vector<coord>>();
}

Quadtree::Quadtree(vector<coord> boundary)
{
	containedPoints = std::vector<vector<coord>>();
    nw = NULL;
    ne = NULL;
    sw = NULL;
    se = NULL;
    this->boundary = boundary;
}

Quadtree::Quadtree(vector<coord> boundary,vector<vector<coord>> containedPoints )
{
	this->containedPoints = containedPoints;
    nw = NULL;
    ne = NULL;
    sw = NULL;
    se = NULL;
    this->boundary = boundary;
}

Quadtree::~Quadtree()
{
    delete nw;
    delete sw;
    delete ne;
    delete se;
}

/*void Quadtree::setNEChild(vector<coord> childBoundary, vector<coord> childPoints){
	Quadtree child(childBoundary,childPoints);
	this->ne = temp;
}*/



vector<pair<Quadtree*,Quadtree*>> well_separated_pairs; // queue storing the well-separated pairs
vector<pair<Quadtree*,Quadtree*>> checkDuplicate; // vector storing cells for which wspd is already found

float s; // to calculate 1/epsilion well-seperated pair

double area(vector<coord> boundary){
	return euclidean_distance(boundary.at(0),boundary.at(1))*euclidean_distance(boundary.at(0),boundary.at(3));
}

coord getMidPoint(coord p1, coord p2) {
	return make_pair(((p1.first+p2.first)/2),(p1.second+p2.second)/2);
}

struct compare_x
{
    bool operator() (pair<double,double> &p1, pair<double,double> &p2)
    {
        return (p1.first < p2.first);
    }
};

struct compare_y
{
    inline bool operator() (coord &p1, coord &p2)
    {
        return (p1.second < p2.second);
    }
};

vector<coord> sort_x(vector<coord> points){
	sort(points.begin(), points.end(), compare_x());
	return points;
}

vector<coord> sort_y(vector<coord> points){
	sort(points.begin(), points.end(), compare_y());
	return points;
}

vector<coord> getHyperRectangle(vector<coord> points){

	//corner case for two axis parallel points
	if(points.size()==2){
		sort(points.begin(), points.end());
		if(points.at(0).first == points.at(1).first){
			// points are y parallel
			coord endpoint_lb = make_pair(points.at(0).first, points.at(0).second);
			coord endpoint_lt = make_pair(points.at(1).first, points.at(1).second);
			coord endpoint_rb = make_pair(points.at(0).first + 1, points.at(0).second);
			coord endpoint_rt = make_pair(points.at(1).first +1,points.at(0).second);
			// bounding box vertices stored
				vector<coord> hyper_rectangle;
				hyper_rectangle.push_back(endpoint_lb);
				hyper_rectangle.push_back(endpoint_lt);
				hyper_rectangle.push_back(endpoint_rt);
				hyper_rectangle.push_back(endpoint_rb);

				//point of intersection of diagonals
				coord intersection = make_pair((endpoint_lb.first + endpoint_rt.first)/2 ,(endpoint_lb.second + endpoint_rt.second)/2);
				hyper_rectangle.push_back(intersection);
			return hyper_rectangle;

		}
		if(points.at(0).second == points.at(1).second){
			// points are x parallel
			coord endpoint_lb = make_pair(points.at(0).first, points.at(0).second);
			coord endpoint_lt = make_pair(points.at(0).first , points.at(0).second+1);
			coord endpoint_rb = make_pair(points.at(1).first , points.at(1).second );
			coord endpoint_rt = make_pair(points.at(1).first,points.at(1).second +1);
			// bounding box vertices stored
				vector<coord> hyper_rectangle;
				hyper_rectangle.push_back(endpoint_lb);
				hyper_rectangle.push_back(endpoint_lt);
				hyper_rectangle.push_back(endpoint_rt);
				hyper_rectangle.push_back(endpoint_rb);

				//point of intersection of diagonals
				coord intersection = make_pair((endpoint_lb.first + endpoint_rt.first)/2 ,(endpoint_lb.second + endpoint_rt.second)/2);
				hyper_rectangle.push_back(intersection);
			return hyper_rectangle;

		}
	}


	// constructs hyper rectangle and returns points splitted in two vectors.
	vector<coord>  x_sorted_points = sort_x(points);
	vector<coord>  y_sorted_points = sort_y(points);
	// hyperrectangle's diagonal has endpoint_lb, endpoint_lt, endpoint_rb and endpoint_rt as its endpoints
	coord endpoint_lb = make_pair(x_sorted_points.front().first, y_sorted_points.front().second);
	coord endpoint_lt = make_pair(x_sorted_points.front().first, y_sorted_points.back().second);
	coord endpoint_rb = make_pair(x_sorted_points.back().first, y_sorted_points.front().second);
	coord endpoint_rt = make_pair(x_sorted_points.back().first, y_sorted_points.back().second);

	// bounding box vertices stored
	vector<coord> hyper_rectangle;
	hyper_rectangle.push_back(endpoint_lb);
	hyper_rectangle.push_back(endpoint_lt);
	hyper_rectangle.push_back(endpoint_rt);
	hyper_rectangle.push_back(endpoint_rb);

	//point of intersection of diagonals
	coord intersection = make_pair((endpoint_lb.first + endpoint_rt.first)/2 ,(endpoint_lb.second + endpoint_rt.second)/2);
	hyper_rectangle.push_back(intersection);

	return hyper_rectangle;
}


map<string, vector<coord>> split_points(vector<coord> points, vector<coord> boundary){
	map < string, vector<coord>> splits;
	vector<coord> split_ne;
	vector<coord> split_nw;
	vector<coord> split_se;
	vector<coord> split_sw;

	vector<coord> boundary_ne;
	vector<coord> boundary_nw;
	vector<coord> boundary_se;
	vector<coord> boundary_sw;

	coord lb;
	coord lt;
	coord rt;
	coord rb;
	coord intersection;

	if(points.size()<=1){
		splits["single"] = points;

	}

	else{
		// split points into 4 sets based on these 2 sides.
			double split_at_x;
			double split_at_y;


			split_at_x = boundary.at(4).first;
			split_at_y = boundary.at(4).second;

			/*split_at_x = (boundary.at(3).first - boundary.at(0).first )/2;
			split_at_y = (boundary.at(1).second - boundary.at(0).second)/2;

			cout<<"Split_x:"<< boundary.at(3).first<<"-"<< boundary.at(0).first<<"/2 ="<< split_at_x<<endl;
			cout<<"Split_y:"<< boundary.at(1).second <<"-"<< boundary.at(0).second<<"/2="<<split_at_y<<endl;*/
			//cout<<"("<<boundary.at(4).first<<","<<boundary.at(4).second<<")"<<"...intersection"<<endl;

			//if(split_at_x!=0 && split_at_y!=0){
		size_t i;
		for(i =0 ; i < points.size(); i++){
			//cout<<"("<<points.at(i).first<<","<<points.at(i).second<<")";
			if(points.at(i).first > split_at_x && points.at(i).second > split_at_y)
				split_ne.push_back(points.at(i));
			else if (points.at(i).first <= split_at_x && points.at(i).second > split_at_y)
				split_nw.push_back(points.at(i));
			else if (points.at(i).first <= split_at_x && points.at(i).second <= split_at_y)
				 split_sw.push_back(points.at(i));
			else
				split_se.push_back(points.at(i));
			}
		//cout<<"sw:"<<split_sw.size()<<endl;

		splits["nw"]= split_nw;
		splits["ne"]= split_ne;
		splits["sw"]= split_sw;
		splits["se"]= split_se;



		// ne boundary
		if(split_ne.size()>0){
				lb = boundary.at(4);
				lt = make_pair(boundary.at(4).first, boundary.at(1).second);
				rt = boundary.at(2);
				rb = make_pair(boundary.at(2).first,boundary.at(4).second);
				intersection = make_pair((lb.first + rt.first)/2 ,(lb.second + rt.second)/2);
				boundary_ne.push_back(lb);
				boundary_ne.push_back(lt);
				boundary_ne.push_back(rt);
				boundary_ne.push_back(rb);
				boundary_ne.push_back(intersection);

				splits["ne_boundary"] = boundary_ne;
				/*cout<<"NE_BOUNDARY"<<endl;
				for(size_t i =0;i<boundary_ne.size();i++){
						cout<<"("<<boundary_ne.at(i).first<<","<<boundary_ne.at(i).second<<"),";
					}
				cout<<endl;*/
			}

			// nw boundary
			if(split_nw.size() >0 ){
				lb = make_pair(boundary.at(0).first,boundary.at(4).second);
				lt = boundary.at(1);
				rt = make_pair(boundary.at(4).first, boundary.at(1).second);
				rb = boundary.at(4);
				intersection = make_pair((lb.first + rt.first)/2 ,(lb.second + rt.second)/2);
				boundary_nw.push_back(lb);
				boundary_nw.push_back(lt);
				boundary_nw.push_back(rt);
				boundary_nw.push_back(rb);
				boundary_nw.push_back(intersection);

				splits["nw_boundary"] = boundary_nw;
				/*cout<<"NW_BOUNDARY"<<endl;
				for(size_t i =0;i<boundary_ne.size();i++){
						cout<<"("<<boundary_nw.at(i).first<<","<<boundary_nw.at(i).second<<"),";
					}
				cout<<endl;*/
			}

			// sw boundary
			if(split_sw.size()>0){
				lb = boundary.at(0);
				lt = make_pair(boundary.at(0).first, boundary.at(4).second);
				rt = boundary.at(4);
				rb = make_pair(boundary.at(4).first,boundary.at(0).second);
				intersection = make_pair((lb.first + rt.first)/2 ,(lb.second + rt.second)/2);
				boundary_sw.push_back(lb);
				boundary_sw.push_back(lt);
				boundary_sw.push_back(rt);
				boundary_sw.push_back(rb);
				boundary_sw.push_back(intersection);

				splits["sw_boundary"] = boundary_sw;
				/*cout<<"SW_BOUNDARY"<<endl;
				for(size_t i =0;i<boundary_sw.size();i++){
						cout<<"("<<boundary_sw.at(i).first<<","<<boundary_sw.at(i).second<<"),";
					}
				cout<<endl;*/
			}

			// se boundary
			if(split_se.size() > 0){
				lb = make_pair(boundary.at(4).first,boundary.at(0).second);
				lt = boundary.at(4);
				rt = make_pair(boundary.at(3).first,boundary.at(4).second);
				rb = boundary.at(3);
				intersection = make_pair((lb.first + rt.first)/2 ,(lb.second + rt.second)/2);
				boundary_se.push_back(lb);
				boundary_se.push_back(lt);
				boundary_se.push_back(rt);
				boundary_se.push_back(rb);
				boundary_se.push_back(intersection);

				splits["se_boundary"] = boundary_se;
				/*cout<<"SE_BOUNDARY"<<endl;
				for(size_t i =0;i<boundary_se.size();i++){
						cout<<"("<<boundary_se.at(i).first<<","<<boundary_se.at(i).second<<"),";
					}
				cout<<endl;*/
			}
	//	}

	}

	return splits;
}




void constructQuadTree(Quadtree* quadtree,vector<coord> &points){

	/*cout<<"BOUNDARY"<<endl;
	for(size_t i =0;i<quadtree->boundary.size();i++){
			cout<<"("<<quadtree->boundary.at(i).first<<","<<quadtree->boundary.at(i).second<<"),";
		}
	cout<<endl;*/

	if(points.size()== 1){
		/*cout<<"HERE"<<endl;
		cout<<points.at(0).first<<","<<points.at(0).second<<endl;
		cout<<"***************";*/
		vector<vector<coord>> singlePointSet;
		singlePointSet.push_back(points);
		quadtree->containedPoints = singlePointSet;
		/*//boundary is rectangle of side zero i.e. point itself
		// inserting 5 coordinates for boundary vertices and intersection points
		vector<coord> bound;
		for(int i=0;i<5;i++){
			bound.push_back(points.at(0));
		}
		quadtree->boundary = bound;*/
	}
	else{
		// split_points constructs hyper rectangle and splits the points into two vectors
		map < string, vector<coord>> splits;
		splits = split_points(points, quadtree->boundary);
		vector<vector<coord>> pointSets;

		if(splits["ne"].size()!=0)
			pointSets.push_back(splits["ne"]);
		if(splits["nw"].size()!=0)
			pointSets.push_back(splits["nw"]);
		if(splits["se"].size()!=0)
			pointSets.push_back(splits["se"]);
		if(splits["sw"].size()!=0)
			pointSets.push_back(splits["sw"]);

		quadtree->containedPoints = pointSets ;

		if(splits["ne"].size()!=0 && splits.count("ne_boundary")==1){
			vector<coord> outerBoundary = getHyperRectangle(splits["ne"]);
			quadtree->ne=  new Quadtree(outerBoundary);
			//quadtree->ne->cellArea = area(splits["ne_boundary"]);
			quadtree->ne->cellArea = quadtree->cellArea /4;
			constructQuadTree(quadtree->ne, splits["ne"]);
		}
		if(splits["nw"].size()!=0 && splits.count("nw_boundary")==1){
			vector<coord> outerBoundary= getHyperRectangle(splits["nw"]);
			quadtree->nw=  new Quadtree(outerBoundary);
			//quadtree->nw->cellArea = area(splits["nw_boundary"]);
			quadtree->nw->cellArea = quadtree->cellArea /4;
			constructQuadTree(quadtree->nw, splits["nw"]);
		}
		if(splits["se"].size()!=0 && splits.count("se_boundary")==1){
			vector<coord> outerBoundary = getHyperRectangle(splits["se"]);
			quadtree->se= new Quadtree(outerBoundary);
			//quadtree->se->cellArea = area(splits["se_boundary"]);
			quadtree->se->cellArea = quadtree->cellArea /4;
			constructQuadTree(quadtree->se, splits["se"]);
		}
		if(splits["sw"].size()!=0 && splits.count("sw_boundary")==1){
			vector<coord> outerBoundary= getHyperRectangle(splits["sw"]);
			quadtree->sw= new Quadtree(outerBoundary);
			//quadtree->sw->cellArea = area(splits["sw_boundary"]);
			quadtree->sw->cellArea = quadtree->cellArea /4;
			constructQuadTree(quadtree->sw, splits["sw"]);
		}

	}

}

void printContainedPoints(Quadtree* quadTree){
	if(quadTree!=NULL){
			int total=0;
		for (size_t i=0;i<quadTree->containedPoints.size();i++){
					total += quadTree->containedPoints.at(i).size();
					cout<<"(";
					for(size_t j=0;j<quadTree->containedPoints.at(i).size();j++){
						cout<<"("<<quadTree->containedPoints.at(i).at(j).first<<","<<quadTree->containedPoints.at(i).at(j).second<<")";
					}
					cout<<"),";
				}
				cout<<endl;
				cout<<"Total Points in cell="<<total<<endl;
	}
}

void printQuadTree(Quadtree* quadTree){
	if(quadTree!=NULL){
		int total=0;
		for (size_t i=0;i<quadTree->containedPoints.size();i++){
			total += quadTree->containedPoints.at(i).size();
			cout<<"(";
			for(size_t j=0;j<quadTree->containedPoints.at(i).size();j++){
				cout<<"("<<quadTree->containedPoints.at(i).at(j).first<<","<<quadTree->containedPoints.at(i).at(j).second<<")";
			}
			cout<<"),";
		}
		cout<<endl;
		cout<<"Total Points in cell="<<total<<endl;
		cout<<"-----------------"<<endl;
		printQuadTree(quadTree->ne);
		printQuadTree(quadTree->nw);
		printQuadTree(quadTree->se);
		printQuadTree(quadTree->sw);
	}
}

vector<coord> getContainedPoints(Quadtree* quadTree){
	vector<coord> containedPoints;
	for (size_t i=0;i<quadTree->containedPoints.size();i++){
		for(size_t j=0;j<quadTree->containedPoints.at(i).size();j++){
			containedPoints.push_back(make_pair(quadTree->containedPoints.at(i).at(j).first,quadTree->containedPoints.at(i).at(j).second));
		}
	}
	return containedPoints;
}

vector<coord> getFarthestPoints(vector<coord> points){
	sort(points.begin(), points.end());
	vector<coord> farthestPoints;
	//cout<<"("<<points.front().first<<","<<points.front().second<<"),("<<points.back().first<<","<<points.back().second<<")"<<endl;
	farthestPoints.push_back(points.front());
	farthestPoints.push_back(points.back());
	return farthestPoints;
}

void getTotalPoints(Quadtree* quadTree){
	if(quadTree!=NULL){
		int total=0;
		for (size_t i=0;i<quadTree->containedPoints.size();i++){
			total += quadTree->containedPoints.at(i).size();
			cout<<"(";
			for(size_t j=0;j<quadTree->containedPoints.at(i).size();j++){
				cout<<"("<<quadTree->containedPoints.at(i).at(j).first<<","<<quadTree->containedPoints.at(i).at(j).second<<")";
			}
			cout<<"),";
		}
		cout<<"Total points:"<<total<<endl;
	}
	}

bool checkLeaves(Quadtree* cell1, Quadtree* cell2){
	if(cell1 == NULL) return false;
	//return checkLeaves(cell1->ne,cell2) && checkLeaves(cell1->nw,cell2) && checkLeaves(cell1->se,cell2) && checkLeaves(cell1->sw,cell2);
	return cell2 == cell1->ne || cell2 == cell1->nw || cell2 == cell1->se || cell2== cell1->sw ;
}

void constructWSPD(Quadtree* cell1, Quadtree* cell2, float s){
	/* boundary stored as..
	 * cell1.boundary.at(0) = lb
	 * cell1.boundary.at(1) = lt
	 * cell1.boundary.at(2) = rt
	 * cell1.boundary.at(3) = rb
	 * cell1.boundary.at(4) = intersection of cell diagonals
	 */
	if(cell1!=NULL && cell2!=NULL){


		/*double farthest1;
		double farthest2;*/

		vector<coord> cell1Points = getContainedPoints(cell1);
		vector<coord> cell2Points = getContainedPoints(cell2);

		/*farthest1 = getFarthestPoints(cell1Points);
		farthest2 = getFarthestPoints(cell2Points);*/

		double cellDiameter1 = euclidean_distance(cell1->boundary.at(0),cell1->boundary.at(2));
		double cellDiameter2 = euclidean_distance(cell2->boundary.at(0),cell2->boundary.at(2));

		double areaCell1 = euclidean_distance(cell1->boundary.at(0),cell1->boundary.at(1))*euclidean_distance(cell1->boundary.at(0),cell1->boundary.at(3));
		double areaCell2 = euclidean_distance(cell2->boundary.at(0),cell2->boundary.at(1))*euclidean_distance(cell2->boundary.at(0),cell2->boundary.at(3));

		double largerDiameter = cellDiameter1 >cellDiameter2 ? cellDiameter1 : cellDiameter2;

		//double dist = euclidean_distance(cell1->boundary.at(4), cell2->boundary.at(4));
		double dist;
		additional_points[0] = cell1->boundary.at(4);
		int i=pr.n+pr.nobs;
		additional_points[1] = cell2->boundary.at(4);
		int j=pr.n+pr.nobs+1;
		//cout << "cell boundary"<< endl;
		//cout << cell1->boundary.at(4).first << " " << cell1->boundary.at(4).second << endl;
		//cout << cell2->boundary.at(4).first << " " << cell2->boundary.at(4).second << endl;
		if(point_inside_obstacle(additional_points[0]) || point_inside_obstacle(additional_points[1])) {
			//pairs are not well separated when center point of any of the pairs is in the obstacle
			dist=0;
		}else if(edge_intersects_obstacle(i,j)) {
			graph vis_gr = visibility_graph(i,j);
			//result res;
			//donothing(vis_gr);
			//system( "python plot.py" );

			vector<int> path_unused;
			dist = dijkstra_shortest_path(vis_gr,i,j,path_unused);
		} else {
			dist = euclidean_distance(cell1->boundary.at(4), cell2->boundary.at(4));
		}


		// if cell2 is a leaf of cell1, return;
		if (cell1Points.size()==0 || cell2Points.size()==0 || checkLeaves(cell1,cell2)){
			return;
		}
		/*coord cen;
		cout<<"Diameters calculated"<<endl;
		cout<<cellDiameter1<<":cell1Diameter"<<endl;
		cout<<cellDiameter2<<":cell2Diameter"<<endl;
		cout<<largerDiameter<<":the larger one"<<endl;
		cen = cell1->boundary.at(4);
		cout<<"center1...("<<cen.first<<","<<cen.second<<")"<<endl;
		cen = cell2->boundary.at(4);
		cout<<"center2...("<<cen.first<<","<<cen.second<<")"<<endl;
		cout<<"distance........"<<dist<<endl;
		 cout<< "s * radius......"<< s * largerDiameter/2 <<endl;
		 cout<<"...cell1"<<endl;
		 printContainedPoints(cell1);
		 cout<<".....cell2"<<endl;
		 printContainedPoints(cell2);
		 cout<<"CHECK IF WS"<<endl;*/
		if(dist-cellDiameter1/2-cellDiameter2/2 >= s * (largerDiameter/2) ){

			 // If leaf cells, and equal, remove them
			 bool equalLeaves = false;
			 if(cell1->containedPoints.size() == 1 && cell2->containedPoints.size()==1){
				 if (cell1->containedPoints.at(0).size() == 1 && cell2->containedPoints.at(0).size()==1){
					 if( cell1->containedPoints.at(0).at(0).first == cell2->containedPoints.at(0).at(0).first && cell1->containedPoints.at(0).at(0).second == cell2->containedPoints.at(0).at(0).second){
						 equalLeaves = true;
					 }
				 }
			 }


			//The two cell's points are 1/s-separated
			//vector<coord> diff = instersection(cell1Points, cell2Points);
			// To remove such pairs.
			//if(diff.size() <=0 )
			 if(!equalLeaves)
				well_separated_pairs.push_back(make_pair(cell1,cell2));
			 return;
		}
	else{
		if((cell1->cellArea < cell2->cellArea)||(cell1->cellArea == cell2->cellArea && cell1Points.size()< cell2Points.size())){
		//if(areaCell1 < areaCell2){
				//Swap the cells
				Quadtree* temp;
				temp = cell1;
				cell1 = cell2;
				cell2 = temp;
				/*cout<<"SWAPPED"<<endl;
				printContainedPoints(cell1);
				printContainedPoints(cell2);
				cout<<"---------"<<endl;*/
			}

		if(!(cell1->ne == NULL && cell1->nw == NULL && cell1->se == NULL && cell1->sw == NULL)){
			if(cell1->ne !=NULL){
							if ( find(checkDuplicate.begin(), checkDuplicate.end(), make_pair(cell1->ne,cell2)) == checkDuplicate.end() && find(checkDuplicate.begin(), checkDuplicate.end(), make_pair(cell2,cell1->ne)) == checkDuplicate.end()){
								checkDuplicate.push_back(make_pair(cell1->ne,cell2));
								constructWSPD(cell1->ne,cell2,s);
							}

						}
						if(cell1->nw !=NULL){
							if ( find(checkDuplicate.begin(), checkDuplicate.end(), make_pair(cell1->nw,cell2)) == checkDuplicate.end() && find(checkDuplicate.begin(), checkDuplicate.end(), make_pair(cell2,cell1->nw)) == checkDuplicate.end()){
								checkDuplicate.push_back(make_pair(cell1->nw,cell2));
								constructWSPD(cell1->nw,cell2,s);
							}
						}
						if(cell1->se !=NULL){
							if ( find(checkDuplicate.begin(), checkDuplicate.end(), make_pair(cell1->se,cell2)) == checkDuplicate.end() && find(checkDuplicate.begin(), checkDuplicate.end(), make_pair(cell2,cell1->se)) == checkDuplicate.end()){
								checkDuplicate.push_back(make_pair(cell1->se,cell2));
								constructWSPD(cell1->se,cell2,s);
							}
						}
						if(cell1->sw !=NULL){
							if ( find(checkDuplicate.begin(), checkDuplicate.end(), make_pair(cell1->sw,cell2)) == checkDuplicate.end() && find(checkDuplicate.begin(), checkDuplicate.end(), make_pair(cell2,cell1->sw)) == checkDuplicate.end()){
								checkDuplicate.push_back(make_pair(cell1->sw,cell2));
								constructWSPD(cell1->sw,cell2,s);
							}
						}
					}
				}
			}
		}


void writeFile(char filename[], int depth) {
	/*
	 *
	 * DOES NOT WRITE TO FILE. will change after format decided upon
	 *
	 */
	cout << "Writing output"<< endl;
    ifstream file(filename);
    string str;
    ofstream outputFile("output_wspd.txt");
    /*while (getline(file, str))
    {
        outputFile << str << endl;
    }*/
    outputFile<< "Number of well-separated pairs created = "<<well_separated_pairs.size()<<endl;
    outputFile<<" Depth of quadtree = "<<depth;
    cout<< "Number of well-separated pairs created = "<<well_separated_pairs.size()<<endl;
    //outputFile << well_separated_pairs.size() <<endl;
    pair<Quadtree*,Quadtree*> ws_pairs;
    for(size_t i = 0; i<well_separated_pairs.size();i++){
    		ws_pairs = well_separated_pairs.at(i);
    		//well_separated_pairs.pop();
    		printContainedPoints(ws_pairs.first);
    		printContainedPoints(ws_pairs.second);
    		cout<<endl;
    	}
}


/*vector<coord> getAllPoints(vector<vector<coord>> points){
	vector<coord> allPoints;
	for(size_t i = 0;i< points.size();i++){
		for(size_t j=0;j<points.at(i).size();j++){
			allPoints.push_back(points.at(i).at(j));
		}
	}
	return allPoints;
}
*/
void build_tspanner(graph& tspanner) {

	for(size_t i=0;i<well_separated_pairs.size();i++){
		vector<coord> a = getContainedPoints(well_separated_pairs.at(i).first);
		vector<coord> b = getContainedPoints(well_separated_pairs.at(i).second);


		int p1 = find(pr.points.begin(),pr.points.end(),a.front())-pr.points.begin();
		int p2 = find(pr.points.begin(),pr.points.end(),b.front())-pr.points.begin();


		if(edge_intersects_obstacle(p1,p2)) {
			graph vis_gr = visibility_graph(p1,p2);
			vector<int> path;
			dijkstra_shortest_path(vis_gr,p1,p2,path);
			for(int j=1;j<path.size();++j) {
				insert_edge(tspanner,path[j-1],path[j]);
			}
		} else {
			insert_edge(tspanner, p1, p2);
		}

	}

}

void execute_greedy(string filename){
	graph tspanner(pr.n+pr.nobs, set<int>());

	clock_t tStart = clock();
	vector<edge> all_edges_sorted = find_all_edges();

	greedy_tspanner(tspanner,all_edges_sorted);
	double execution_timer = (clock()-tStart)/(double)(CLOCKS_PER_SEC/1000);

	result res = computeResultParameters(tspanner,all_edges_sorted);
	res.execution_time = execution_timer;
	cout << "result: " << res.dilation << " " << res.size << " " << res.weight << " " << res.execution_time << endl;

	writeFile(filename, tspanner, res);

	system( "python plot.py" );


	//writeFile(filename, obstacle_visibility, res);
	//system( "python plot_vis.py" );

}

void execute_WSPD(string filename){
	vector<coord> outerBoundary = getHyperRectangle(pr.points);
	Quadtree quadtree = Quadtree(outerBoundary);
	constructQuadTree(&quadtree,pr.points);

	//printQuadTree(&quadtree);

	s = (4*(pr.t +1))/( pr.t - 1);
	cout<<"s: "<<s <<endl;
	clock_t tStart = clock();
	constructWSPD(&quadtree,&quadtree,s);
	graph tspanner(pr.n+pr.nobs, set<int>());
	build_tspanner(tspanner);
	double execution_timer = (clock()-tStart)/(double)(CLOCKS_PER_SEC/1000);
	vector<edge> all_edges_sorted = find_all_edges();
	result res = computeResultParameters(tspanner,all_edges_sorted);
	res.execution_time = execution_timer;
	cout << "result: " << res.dilation << " " << res.size << " " << res.weight << " " << res.execution_time << endl;

	writeFile(filename, tspanner, res);
	system( "python plot.py" );
}

int main() {

	//datagene(50,10);

	string file="data";
	cout << "File name:" << endl;
	cin >> file;
	//char filename[] = "data.txt";

	string filename = "Data_examples/"+file+".txt";
	cout << filename << endl;
	readFile(filename);

//	cout << point_inside_obstacle(make_pair(2,3))<<endl;
//	cout << point_inside_obstacle(make_pair(3,5))<<endl;
//	cout << point_inside_obstacle(make_pair(9,9))<<endl;
//	cout << point_inside_obstacle(make_pair(0,5))<<endl;

	execute_greedy(filename);
	execute_WSPD(filename);


}
