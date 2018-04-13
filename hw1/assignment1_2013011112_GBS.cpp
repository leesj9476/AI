#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>
#include <queue>

#define 1D_DISTANCE(p1, p2)	(abs(p1.p.row - p2.p.row) + \
							 abs(p1.p.col - p2.p.col))

#define 2D_DISTANCE(p1, p2) (pow(abs(p1.p.row - p2.p.row), 2) + \
							 pow(abs(p1.p.col - p2.p.col), 2))

using namespace std;

// point info -> (row, col)
typedef struct Point {
	Point();
	Point(int, int);
	bool operator==(const Point &);
	bool operator!=(const Point &);

	int row;
	int col;
} Point;

// node info -> point info, child nodes, address of parent
typedef struct Node {
	Node(int, int, Node * = NULL, int = 0);

	Point p;
	vector<Node> child;
	Node *parent;

	int length;
	bool goal;
} Node;

// result info -> length, time
typedef struct Result {
	Result();
	Result(int, int);
	Result operator=(const Result &);

	int length;
	int time;
} Result;

// possible road direction at specific point
typedef struct Direction {
	Direction();

	bool up;
	bool right;
	bool down;
	bool left;
} Direction;

// Map enum data
typedef enum class Map {
	WALL = 1,
	ROAD = 2, 
	START = 3, 
	GOAL = 4,
	ROAD_G = 5
} Map;

// map checking enum data
typedef enum class CheckMap {
	UNCHECKED = 0,
	CHECKED = 1,
	START = 2,
	GOAL = 3
} CheckMap;

Point::Point()
	: row(-1), col(-1) {}

Point::Point(int row_, int col_)
	: row(row_), col(col_) {}

bool Point::operator==(const Point &p) {
	return (this->row == p.row && this->col == p.col);
}

bool Point::operator!=(const Point &p) {
	return (this->row != p.row || this->col != p.col);
}

Node::Node(int row_, int col_, Node *parent_, int length_)
	: p.row(row_), p.col(col_), length(length_), goal(false) {

	// for root node
	if (parent_ == NULL)
		parent = this;
	else
		parent = parent_;
}

Result::Result()
	: length(0), time(0) {}

Result::Result(int length_, int time_)
	: length(length_), time(time_) {}

Result Result::operator= (const Result &res) {
	length = res.length;
	time = res.time;

	return *this;
}

Direction::Direction()
	: up(false), right(false), down(false), left(false) {}

int MAP_SIZE_ROW = 0;
int MAP_SIZE_COL = 0;

Result calc(Map **, int, int, Point &, vector<Point> &);
bool makeReductionMapTree(Node &, Map **, int, int, Point &);

int main () {
	string input_filename = "input.txt";
	string output_filename = "output.txt";

	// open input.txt
	ifstream input_f(input_filename);
	if (!input_f.is_open()) {
		cerr << "input file is not exist" << endl;

		return -1;
	}

	// open output.txt
	ofstream output_f(output_filename);
	if (!output_f.is_open()) {
		cerr << "output file cannot be opened" << endl;
		input_f.close();

		return -1;
	}

	// read
	int row = 0;
	int col = 0;
	input_f >> row >> col;
	if (row <= 0 || row > 500 || col <= 0 || col > 500) {
		cerr << "row or col value error" << endl;
		return -1;
	}

	// set map size
	MAP_SIZE_ROW = row;
	MAP_SIZE_COL = col;

	// allocate map array data
	Map **map_info = NULL;
	map_info = new Map *[row];
	for (int i = 0; i < col; i++)
		map_info[i] = new Map[col];

	int map_1cell_data = 0;
	int row_i = 0;
	int col_j = 0;

	// get map data from input.txt and fill map array data
	Point start;
	vector<Point> goal;
	Result result;
	for (int i = 0; i < row * col; i++) {
		if (input_f.eof()) {
			cerr << "input file do not have sufficient map data" << endl;
			goto RELEASE_DATA;
		}

		input_f >> map_1cell_data;
		switch (map_1cell_data) {
			case 1:
				map_info[row_i][col_j] = Map::WALL;
				break;

			case 2:
				map_info[row_i][col_j] = Map::ROAD;
				break;

			// start point must exist only one
			case 3:
				map_info[row_i][col_j] = Map::START;
				if (start.row != -1 || start.col != -1) {
					cerr << "start point is duplicated" << endl;
					goto RELEASE_DATA;
				}

				start.row = row_i;
				start.col = col_j;
				break;

			// goal point can exist one or more
			case 4:
				map_info[row_i][col_j] = Map::GOAL;
				goal.emplace_back(row_i, col_j);
				break;

			default:
				cerr << "input file have unknown map data" << endl;
				goto RELEASE_DATA;
		}

		col_j++;
		if (col_j >= col) {
			row_i++;
			col_j = 0;
		}
	}

	// start num == 1
	// goal num >= 1
	if (start.row == -1 || start.col == -1 || goal.size() == 0) {
		cerr << "input file start or goal data error" << endl;
		goto RELEASE_DATA;
	}

	// map data must have row * col data
	if (!(row_i == row && col_j == 0)) {
		cerr << "input file do not have sufficient map data" << endl;
		goto RELEASE_DATA;
	}

	// calc best result
	result = calc(map_info, row, col, start, goal);

	// write
	for (int i = 0; i < row; i++) {
		for (int j = 0; j < col; j++) {
			output_f << static_cast<int>(map_info[i][j]) << " ";
		}

		output_f << endl;
	}

	output_f << "---" << endl;
	// best result
	if (result.length != -1 || result.time != -1) {
		output_f << "length=" << result.length << endl;
		output_f << "time=" << result.time << endl;
	}
	// no result
	else {
		output_f << "time=" << result.time << endl;
		output_f << "no result" << endl;
	}

RELEASE_DATA:
	input_f.close();
	output_f.close();

	for (int i = 0; i < row_i; i++)
		delete[] map_info[i];
	delete[] map_info;

	return 0;
}

Result calc(Map **map, int row, int col, Point &start, vector<Point> &goal) {
	// make map tree -> reduce 1way road to node including length
	bool reduction_map = makeReductionMapTree(root, map, row, col, start, goal);
	if (!reduction_map)
		return Result(-1, -1);

	// calc by greedy best-first search algo
	
}

bool makeReductionMapTree(Node &root, Map **map, int row, int col, Point &start) {
	queue<Node *> cross_roads;
	cross_roads.push(&root);

	while (!cross_roads.empty()) {
		Node *cur_node = cross_roads.front();
		cross_roads.pop();

		// go 1-line way -> length++
		


		// if find cross road, push that point to queue
	}
}
