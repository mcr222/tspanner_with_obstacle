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

struct node {
	vector<coord> value;
	node *left;
	node *right;
};

class btree
{
    public:
        btree();
        ~btree();

        void destroy_tree();
        node* getRoot();
        bool isLeaf(node);

    private:
        void destroy_tree(node *leaf);
        node *root;
};

btree::btree()
{
  root=NULL;
}

void btree::destroy_tree(node *leaf)
{
  if(leaf!=NULL)
  {
    destroy_tree(leaf->left);
    destroy_tree(leaf->right);
    delete leaf;
  }
}

node* btree::getRoot(){
	return root;
}

bool btree::isLeaf(node n){
	if(n.left == NULL && n.right == NULL)
		return true;
	else
		return false;
}


void btree::destroy_tree()
{
  destroy_tree(root);
}

problem pr;
double s;

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


struct compare_x
{
    bool operator() (pair<int,int> &p1, pair<int,int> &p2)
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

vector<vector<coord>> split_points(vector<coord> points){
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

	double side_1 = euclidean_distance(endpoint_lb, endpoint_lt);
	double side_2 = euclidean_distance(endpoint_lb, endpoint_rb);
	bool split_by_x = side_1 > side_2 ? false: true;

	vector<coord> split_1;
	vector<coord> split_2;
	vector<vector<coord>> splits;

	if(split_by_x){
		// split hyperrectangle along side_2
		int split_at = (endpoint_lb.first + endpoint_rb.first)/2;
		for(int i =0 ; i < points.size(); i++){
			if(points.at(i).first < split_at){
				split_1.push_back(points.at(i));
			}else{
				split_2.push_back(points.at(i));
			}
		}
		splits.push_back(split_1);
		splits.push_back(split_2);
	}else{
		// split hyperrectangle along side_2
		int split_at = (endpoint_lb.second + endpoint_lt.second)/2;
		for(int i =0 ; i < points.size(); i++){
			if(points.at(i).second < split_at){
				split_1.push_back(points.at(i));
			}else{
				split_2.push_back(points.at(i));
			}
		}
		splits.push_back(split_1);
		splits.push_back(split_2);
	}

	// hyper rectangle vertices pushed in too
	splits.push_back(hyper_rectangle);
	return splits;


}

node split_tree(vector<coord> points){
	node u;
	if(points.size()== 1){
		// node u stores the only point
		u.value = points;
	}else{
		// split_points constructs hyper rectangle and splits the points into two vectors
		vector<vector<coord>> splits = split_points(points);
		node v = split_tree(splits.at(0));
		node w = split_tree(splits.at(1));
		u.left= & v;
		u.right= & w;
		u.value = splits.at(2);  // vertices of hyper rectangle stored as the value of node
	}
	return u;
}

queue<pair<vector<coord>,vector<coord>>> pair_queue; // queue storing the well-separated pairs

double l_max(coord endpoint_lb, coord endpoint_lt, coord endpoint_rt, coord endpoint_rb){
	double side_1 = euclidean_distance(endpoint_lb, endpoint_lt);
	double side_2 = euclidean_distance(endpoint_lb, endpoint_rb);

	if (side_1 > side_2)
		return side_1;
	else
		return side_2;
}

void find_pairs(node* v, node* w){
	// push hyper rectangles of v and w in queue
	pair<vector<coord>,vector<coord>> pairs = make_pair(v->value, w->value);
	coord center1 = pairs.first.back();
	coord center2 = pairs.second.back();
	double diameter1 = euclidean_distance(pairs.first.at(0),pairs.first.at(2));
	double diameter2 = euclidean_distance(pairs.second.at(0),pairs.second.at(2));
	if(diameter1 == diameter2 && euclidean_distance(center1,center2)> s*(diameter1/2)){
		// both diameters equal and distance between centers is >sr TODO:obstacle handling
		//push pair in queue
		pair_queue.push(pairs);
	}else {
		if(l_max(pairs.first.at(0),pairs.first.at(1),pairs.first.at(2),pairs.first.at(3)) >= l_max(pairs.second.at(0),pairs.second.at(1),pairs.second.at(2),pairs.second.at(3)) ){
			// l_max(v) > l_max(w)
			if(v->left!=NULL && v->right!=NULL){
				find_pairs(v->left,w);
				find_pairs(v->right,w);
			}
		}else {
			if(w->left!=NULL && w->right!=NULL){
				find_pairs(v,w->left);
				find_pairs(v,w->right);
			}
		}
	}
}

void find_WSPD(node st, double s){
	// get all the internal nodes of split_tree beginning from the root
	find_pairs(st.left, st.right);
	// pair_queue has the well-separated pairs
}

int main() {
	char filename[] = "Data_examples/geometricspanners.txt";
	readFile(filename);
	graph tspanner(pr.n, vector<int>(0));

	node splitTree = split_tree(pr.points);

	s = 4* ((pr.t+1)/(pr.t -1));

	find_WSPD(splitTree, s );

	cout<< pair_queue.size();
	cout<< "end";
}
