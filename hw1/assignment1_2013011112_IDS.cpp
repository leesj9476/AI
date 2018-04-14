#include <iostream>
#include <fstream>
#include <string>
#include <vector>

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
	Node(int, int, Node * = NULL);

	Point p;
	vector<Node> child;
	Node *parent;
} Node;

// result info -> length, time
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

Node::Node(int row_, int col_, Node *parent_) {
	p.row = row_;
	p.col = col_;

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

int MAP_SIZE_ROW = 0;
int MAP_SIZE_COL = 0;

Result calc(Map **, int, int, Point &, vector<Point> &);
bool findPossibleMoves(Map **, CheckMap**, Node *, vector<Node *> &);

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

Result calc(Map **map, int row, int col, Point &start, vector<Point> &goal) {
	Result res;

	// check the searched map info from older level -> ignore that space
	CheckMap **searched_map;
	searched_map = new CheckMap *[row];
	for (int i = 0; i < row; i++) {
		searched_map[i] = new CheckMap[col];

		for (int j = 0; j < col; j++)
			searched_map[i][j] = CheckMap::UNCHECKED;
	}

	// check start and goal space to searched map
	searched_map[start.row][start.col] = CheckMap::START;
	for (uint i = 0; i < goal.size(); i++)
		searched_map[goal[i].row][goal[i].col] = CheckMap::GOAL;

	// make tree
	Node root(start.row, start.col);
	vector<vector<Node *> > node_container;

	// initiate node pointer container
	vector<Node *> root_container;
	root_container.push_back(&root);
	node_container.push_back(root_container);

	// iterative deepening search(IDS) algorithm
	// skip level 0 -> start point
	int cur_level = 0;
	bool found_goal = false;
	Node *track_goal_road = NULL;

	// find road to goal
	while (node_container[cur_level].size() > 0) {
		vector<Node *> cur_level_leaf_nodes;

		for (int i = 0; i < node_container[cur_level].size(); i++) {	
			bool find_goal = findPossibleMoves(map, searched_map, node_container[cur_level][i], cur_level_leaf_nodes);
			res.time++;

			if (find_goal) {
				track_goal_road = node_container[cur_level][i];
				break;
			}
		}

		// find result
		if (track_goal_road != NULL)
			break;

		node_container.push_back(cur_level_leaf_nodes);
		cur_level++;
	}

	// set Map::ROAD_G
	if (track_goal_road != NULL) {
		// track_goal_road point parent of Goal Node
		while (track_goal_road->p != start) {
			map[track_goal_road->p.row][track_goal_road->p.col] = Map::ROAD_G;
			track_goal_road = track_goal_road->parent;

			res.length++;
		}
	}
	// no answer
	else {
		res.length = -1;
	}

	// free check map
	for (int i = 0; i < row; i++)
		delete[] searched_map[i];
	delete[] searched_map;
	
	return res;
}

bool findPossibleMoves(Map **map, CheckMap **searched_map, Node *cur_node, vector<Node *> &leaf_nodes) {
	int row = cur_node->p.row;
	int col = cur_node->p.col;

	// UP
	if (row > 0) {
		if (map[row-1][col] == Map::GOAL)
			return true;
		else if (map[row-1][col] == Map::ROAD && searched_map[row-1][col] == CheckMap::UNCHECKED) {
			cur_node->child.emplace_back(row - 1, col, cur_node);
			searched_map[row-1][col] = CheckMap::CHECKED;
		}
	}
	
	// RIGHT
	if (col < MAP_SIZE_COL-1) {
		if (map[row][col+1] == Map::GOAL)
			return true;
		else if (map[row][col+1] == Map::ROAD && searched_map[row][col+1] == CheckMap::UNCHECKED) {
			cur_node->child.emplace_back(row, col + 1, cur_node);
			searched_map[row][col+1] = CheckMap::CHECKED;
		}
	}

	// DOWN
	if (row < MAP_SIZE_ROW-1) {
		if (map[row+1][col] == Map::GOAL)
			return true;
		else if (map[row+1][col] == Map::ROAD && searched_map[row+1][col] == CheckMap::UNCHECKED) {
			cur_node->child.emplace_back(row + 1, col, cur_node);
			searched_map[row+1][col] = CheckMap::CHECKED;
		}
	}
	
	// LEFT
	if (col > 0) {
		if (map[row][col-1] == Map::GOAL)
			return true;
		else if (map[row][col-1] == Map::ROAD && searched_map[row][col-1] == CheckMap::UNCHECKED) {
			cur_node->child.emplace_back(row, col - 1, cur_node);
			searched_map[row][col-1] = CheckMap::CHECKED;
		}
	}

	// add child nodes to vector<Node *> leaf_nodes
	for (int i = 0; i < cur_node->child.size(); i++) {
		leaf_nodes.push_back(&cur_node->child[i]);
	}

	return false;
}
