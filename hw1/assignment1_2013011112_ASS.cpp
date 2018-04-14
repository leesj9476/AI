#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>
#include <queue>

// calc manhattan distance
#define DISTANCE(p1, p2)	(abs(p1.row - p2.row) + \
							 abs(p1.col - p2.col))

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

// node info -> point info, length data, child and parent node
typedef struct Node {
	Node(int, int, Node * = NULL, int = 0, int = 0);

	Point p;
	int length_from_start;
	int length_to_goal;

	vector<Node *> child;
	Node *parent;
} Node;

typedef struct Result {
	Result();
	Result(int, int);
	Result operator=(const Result &);

	int length;
	int time;
} Result;

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

Node::Node(int row_, int col_, Node *parent_, int length_from_start_, int length_to_goal_) {
	p.row = row_;
	p.col = col_;
	length_from_start = length_from_start_;
	length_to_goal = length_to_goal_;

	// for root node
	if (parent_ == NULL)
		parent = this;
	else
		parent = parent_;
}

// compare score -> smaller length, bigger score
struct Compare {
	bool operator() (const Node *n1, const Node *n2) {
		return n1->length_from_start + n1->length_to_goal > n2->length_from_start + n2->length_to_goal;
	}
};

// result info -> length, time
Result::Result()
	: length(0), time(0) {}

Result::Result(int length_, int time_)
	: length(length_), time(time_) {}

Result Result::operator= (const Result &res) {
	length = res.length;
	time = res.time;

	return *this;
}

int MAP_SIZE_ROW = 0;
int MAP_SIZE_COL = 0;

Result calc(Map **, int, int, Point &, vector<Point> &);
int findPossibleMoves(Map **, CheckMap **, int, int, Point &);
int shortestLength(int, int, vector<Point> &);

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
	for (int i = 0; i < row; i++)
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
	if (result.length != -1) {
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

// calc result road using greedy best-first search algorithm
Result calc(Map **map, int row, int col, Point &start, vector<Point> &goal) {
	Node root(start.row, start.col);
	CheckMap **search_map;
	Result res;
	vector<Node *> all_nodes;

	// make check map -> after searching, set to UNCHECKED
	search_map = new CheckMap *[row];
	for (int i = 0; i < row; i++) {
		search_map[i] = new CheckMap[col];
	
		for (int j = 0; j < col; j++)
			search_map[i][j] = CheckMap::UNCHECKED;
	}

	// set start and goal point
	search_map[start.row][start.col] = CheckMap::START;
	for (uint i = 0; i < goal.size(); i++)
		search_map[goal[i].row][goal[i].col] = CheckMap::GOAL;

	// search biggest score point first
	priority_queue<Node *, vector<Node *>, Compare> search_queue;
	search_queue.push(&root);
	Node *goal_node = NULL;

	// search continuously until finding result
	// search_queue is empty when there is no result
	while (!search_queue.empty()) {
		Node *cur_node = search_queue.top();
		search_queue.pop();
		all_nodes.push_back(cur_node);

		res.time++;

		Point cur_p(cur_node->p.row, cur_node->p.col);

		// check if goal node
		if (map[cur_p.row][cur_p.col] == Map::GOAL) {
			goal_node = cur_node;
			break;
		}

		// search possible way
		// bit flag -> 0x01(up), 0x02(right), 0x04(down), 0x08(left)
		int move_flag = findPossibleMoves(map, search_map, row, col, cur_p);

		// UP
		if (move_flag & 0x01) {
			Node *up_node = new Node(cur_p.row-1, cur_p.col, cur_node);
			up_node->length_from_start = up_node->parent->length_from_start + 1;
			up_node->length_to_goal = shortestLength(cur_p.row-1, cur_p.col, goal);
			search_queue.push(up_node);
		}

		// RIGHT
		if (move_flag & 0x02) {
			Node *right_node = new Node(cur_p.row, cur_p.col+1, cur_node);
			right_node->length_from_start = right_node->parent->length_from_start + 1;
			right_node->length_to_goal = shortestLength(cur_p.row, cur_p.col+1, goal);
			search_queue.push(right_node);
		}

		// DOWN
		if (move_flag & 0x04) {
			Node *down_node = new Node(cur_p.row+1, cur_p.col, cur_node);
			down_node->length_from_start = down_node->parent->length_from_start + 1;
			down_node->length_to_goal = shortestLength(cur_p.row+1, cur_p.col, goal);
			search_queue.push(down_node);
		}

		// LEFT
		if (move_flag & 0x08) {
			Node *left_node = new Node(cur_p.row, cur_p.col-1, cur_node);
			left_node->length_from_start = left_node->parent->length_from_start + 1;
			left_node->length_to_goal = shortestLength(cur_p.row, cur_p.col-1, goal);
			search_queue.push(left_node);
		}
	}

	// make result road to start point from goal
	if (goal_node) {
		Node *track_road = goal_node->parent;
		while (track_road->p != start) {
			map[track_road->p.row][track_road->p.col] = Map::ROAD_G;
			res.length++;

			track_road = track_road->parent;
		}
	}
	// no result
	else
		res.length = -1;

	// free datum
	for (int i = 0; i < row; i++)
		delete[] search_map[i];
	delete[] search_map;

	// all_nodes[0] -> root node
	for (uint i = 1; i < all_nodes.size(); i++)
		delete all_nodes[i];
	
	while (!search_queue.empty()) {
		Node *delete_node = search_queue.top();
		search_queue.pop();

		delete delete_node;
	}

	return res;
}

// find possible direction from current point
int findPossibleMoves(Map **map, CheckMap **search_map, int row, int col, Point &p) {
	int result = 0;

	// UP
	if (p.row > 0 && 
			(map[p.row-1][p.col] == Map::ROAD || map[p.row-1][p.col] == Map::GOAL) &&
			(search_map[p.row-1][p.col] == CheckMap::UNCHECKED || search_map[p.row-1][p.col] == CheckMap::GOAL)) {

		result |= 0x01;
		search_map[p.row-1][p.col] = CheckMap::CHECKED;
	}

	// RIGHT
	if (p.col < MAP_SIZE_COL - 1 && 
			(map[p.row][p.col+1] == Map::ROAD || map[p.row][p.col+1] == Map::GOAL) &&
			(search_map[p.row][p.col+1] == CheckMap::UNCHECKED || search_map[p.row][p.col+1] == CheckMap::GOAL)) {

		result |= 0x02;
		search_map[p.row][p.col+1] = CheckMap::CHECKED;
	}

	// DOWN
	if (p.row < MAP_SIZE_ROW - 1 && 
			(map[p.row+1][p.col] == Map::ROAD || map[p.row+1][p.col] == Map::GOAL) &&
			(search_map[p.row+1][p.col] == CheckMap::UNCHECKED || search_map[p.row+1][p.col] == CheckMap::GOAL)) {

		result |= 0x04;
		search_map[p.row+1][p.col] = CheckMap::CHECKED;
	}

	// LEFT
	if (p.col > 0 && 
			(map[p.row][p.col-1] == Map::ROAD || map[p.row][p.col-1] == Map::GOAL) &&
			(search_map[p.row][p.col-1] == CheckMap::UNCHECKED || search_map[p.row][p.col-1] == CheckMap::GOAL)) {

		result |= 0x08;
		search_map[p.row][p.col-1] = CheckMap::CHECKED;
	}

	return result;
}

// calc shortest length to several goals
int shortestLength(int row, int col, vector<Point> &goal) {
	int length = MAP_SIZE_ROW + MAP_SIZE_COL;

	Point cur_p(row, col);
	for (uint i = 0; i < goal.size(); i++) {
		int curLength = DISTANCE(cur_p, goal[i]);

		if (length > curLength)
			length = curLength;
	}

	return length;
}
