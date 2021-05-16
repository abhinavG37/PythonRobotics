//A* Grid Planning C++ Variant 
//Uses Sciplot plotting as a substitute for matplotlib
//Eigen used for Linear algebra 
//vector<float/int> used for grid formation and update
//map<int, Node*> used for maintaining traversed nodes
/*Major Chunk working now------------VISUALIZATION PENDING*/


#include <iostream>
#include <cmath>
#include <utility>
#include <vector>
#include <map>
#include <string>
#include<opencv2/core.hpp>
#include<opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace std;

class A_Star_planner {
public:
  A_Star_planner(int rows_, int cols_, float resolution_, vector<float> ox_,
                 vector<float> oy_, float rr_) {
    vector<vector<float>> grid_(rows_, vector<float>(cols_, 0));
    grid = grid_;
    resolution = resolution_;
    rr = rr_;
    ox = std::move(ox_);
    oy = std::move(oy_);
    this->min_x = 0.0, this->min_y = 0.0;
    this->max_x = 0.0, this->max_y = 0.0;
    this->x_width = 0.0;
    this->y_width = 0.0;
    this->motion = this->get_motion_model();

    this->obmap = this->calc_obmap(ox, oy);
  } // CONSTRUCTOR FOR CLASS A*_Planner

  ////////////////////////CLASS NODE//////////////////////////////////////////////////
  class Node {
  public:
    Node(float x, float y, float cost, int parent_index,
         Node *p_node_ = nullptr) {
      this->x = x;
      this->y = y;
      this->cost = cost;
      this->parent_index = parent_index;
      parent_node = p_node_;
    } // DEFAULT CONSTRUCTOR TO INITIALIZE NODE BASED ON PARENT NODE

    [[nodiscard]] string str_maker() const {
      return to_string(this->x) + "," + to_string(this->y) + "," +
             to_string(this->cost) + "," + to_string(this->parent_index);
    }
    // Data members of subclass Node
    float x, y, cost;
    float parent_index;
    [[maybe_unused]] Node *parent_node;
  };
  //////////////////////////////////////////END CLASS NODE//////////////////////////////////

  vector<vector<float>> planner(float sx, float sy, float gx, float gy) {
    /*  A star path search

      input:
          sx: start x position [m]
          sy: start y position [m]
          gx: goal x position [m]
          gy: goal y position [m]

      output:
          rx: x position list of the final path
          ry: y position list of the final path
  */

    Node *startNode = new Node(this->calc_xy_index(sx, this->min_x),
                               this->calc_xy_index(sy, this->min_y), 0.0, -1);
    Node *goalNode = new Node(this->calc_xy_index(gx, this->min_x),
                              this->calc_xy_index(gy, this->min_y), 0.0, -1);
    // pointer to an object created
    printf("START----------NODE_X: %f\tNODE_Y:%f\tNODE_COST:%f\tPARENT_INDEX:%f", startNode->x, startNode->y, startNode->cost, startNode->parent_index);
    printf("GOAL----------NODE_X: %f\tNODE_Y:%f\tNODE_COST:%f\tPARENT_INDEX:%f", goalNode->x, goalNode->y, goalNode->cost, goalNode->parent_index);

    map<int, Node *> open_set;
    map<int, Node *> closed_set;
    map<int, Node *>::iterator it;

    // {  TAKES A pointer to NODE object}     //calc_grid_index(Node* node)
    open_set.insert(pair<int, Node *>(this->calc_grid_index(startNode), startNode));
    int count = 0;
    int c_id = 0; // grid index of current node used as key in open_set

    while (true) {
      count += 1;
      printf("%d\n", count);
      if (open_set.empty()) {
        printf("OPEN SET EMPTY");
        break;
      }

      // LINE 81: Needs a lambda function equivalent ---C++ functor---- to capture max cost Node in open_set


      auto temp_min_cost = INFINITY;
      // BLOCK FUNCTIONING NOT PROPER
      for (it = open_set.begin(); it != open_set.end(); it++) {

        if ((it->second->cost) + this->calc_heuristic(goalNode, it->second)<=temp_min_cost) {
          temp_min_cost = (it->second->cost) + this->calc_heuristic(goalNode, it->second);
          c_id = it->first;
        }
      } // find key corresponding to a temp_min_cost
      Node *current = open_set.at(c_id); // ERROR

      cv::rectangle(bg,
                    cv::Point(startNode->x * this->resolution + 1,startNode->y * this->resolution + 1),
                    cv::Point((startNode->x + 1) * this->resolution,(startNode->y + 1) * this->resolution),
                    cv::Scalar(255, 0, 0), -1); // Create RED SQUARE ON
                                                //  START NODE
      cv::rectangle(bg,
                    cv::Point(goalNode->x * this->resolution + 1,
                              goalNode->y * this->resolution + 1),
                    cv::Point((goalNode->x + 1) * this->resolution,
                              (goalNode->y + 1) * this->resolution),
                    cv::Scalar(0, 0, 255), -1);
      imshow("astar", bg);

      if (current->x == goalNode->x && current->y == goalNode->y) {
        printf("-------------Goal Found-------------");
        goalNode->parent_index = current->parent_index;
        goalNode->cost = current->cost;
        break;
      }

      open_set.erase(c_id); // Deletes pair at key value c_id
      closed_set.insert(
          pair<int, Node *>(c_id, current)); // add traversed node to closed set

      /////////continue here from line 110
      for (auto &i : motion) {
        Node *node = new Node(current->x + i[0], current->y + i[1],
                              current->cost + i[2], c_id);
        int n_id = this->calc_grid_index(node);

        // Verification of new node
        // Node not out of bounds or colliding with obstacle
        if (!this->verify_node(node)) {
          continue;
        }

        // Node exists on already traversed node then do nothing
        if (closed_set.count(n_id) > 0) {
          continue;
        }

        if (open_set.count(n_id) == 0) {
          open_set.insert(pair<int, Node *>(n_id, node));
        } else {
          if (open_set.at(n_id)->cost > node->cost) {
            open_set.insert(pair<int, Node *>(n_id, node));
          }
        }
        cv::rectangle(bg,
                      cv::Point(node->x * this->resolution + 1,
                                node->y * this->resolution + 1),
                      cv::Point((node->x + 1) * this->resolution,
                                (node->y + 1) * this->resolution),
                      cv::Scalar(0, 255, 0));

        cv::imshow("astar", bg);
      }
    }
    return this->calc_final_path(goalNode, closed_set);
  }

  vector<vector<int>> calc_obmap(vector<float> ox, vector<float> oy) {

    this->min_x = round(*std::min_element(ox.begin(), ox.end()));
    this->max_x = round(*std::max_element(ox.begin(), ox.end()));
    this->min_y = round(*std::min_element(oy.begin(), oy.end()));
    this->max_y = round(*std::max_element(oy.begin(), oy.end()));

    printf("min_x: %f", this->min_x);
    printf("min_y: %f", this->min_y);
    printf("max_x: %f", this->max_x);
    printf("max_y: %f", this->max_y);

    this->x_width = round((this->max_x - this->min_x) / this->resolution);
    this->y_width = round((this->max_y - this->min_y) / this->resolution);
    printf("x_width: %f", this->x_width);
    printf("y_width: %f", this->y_width);

//    this->bg = cv::Mat(this->resolution * this->x_width,this->resolution * this->y_width, CV_8UC3, cv::Scalar(255, 255, 255));
    this->bg = cv::Mat(this->resolution * this->x_width,this->resolution * this->y_width, CV_8UC3, cv::Scalar(255, 255, 255));

    vector<vector<int>> temp_obmap((int)this->y_width,
                                   vector<int>((int)this->x_width, 0));
    for (int i = 0; i < this->x_width; i++) {
      int x = this->calc_grid_pos(i, this->min_x);
      for (int j = 0; j < this->y_width; j++) {
        int y = this->calc_grid_pos(j, this->min_y);
        for (unsigned int k = 0; k < ox.size(); k++) {
          float d = sqrt(pow((ox[k] - x), 2) + pow((oy[k] - y), 2));
          if (d <= this->rr) {
            temp_obmap[i][j] = 1;
            cv::rectangle(
                this->bg,
                cv::Point(i * this->resolution + 1, j * this->resolution + 1),
                cv::Point((i + 1) * this->resolution,
                          (j + 1) * this->resolution),
                cv::Scalar(0, 0, 0), 1);
            cv::imshow("astar", this->bg);
            break;
          }
        }
      }
    }
    obmap = temp_obmap;

    return obmap;
  }
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  vector<vector<float>> calc_final_path(Node *goal, map<int, Node *> closed_set) {
    vector<float> rx;
    vector<float> ry;
    vector<float>::iterator it1;
    vector<float>::iterator it2;
    rx.push_back(this->calc_grid_pos(goal->x, this->min_x));
    ry.push_back(this->calc_grid_pos(goal->y, this->min_y));
    int parent_index = goal->parent_index;
    while (parent_index != -1) {
      Node *n = closed_set.at(parent_index);
      rx.push_back(this->calc_grid_pos(n->x, this->min_x));
      ry.push_back(this->calc_grid_pos(n->y, this->min_y));
      parent_index = n->parent_index;
    }

    for(it1 = rx.begin(), it2 = ry.begin(); it1 != rx.end(), it2 != ry.end(); it1++, it2++){
      cv::circle(this->bg, cv::Point(*it1, *it2),this->rr, cv::Scalar(100,100,100), cv::FILLED);
      imshow("astar", bg);
      int k = cv::waitKey(5);
      if ((k & 0xFF) == 27) {
        cv::destroyAllWindows();
      }
    }



    return {rx, ry};
  }
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
   [[nodiscard]] inline  float calc_grid_pos(int index,float min_pos) const{
       return  index * this->resolution+min_pos;
   }
      ///////////////////////////////////////////////////////////////////////////////////////////////////// 
    [[nodiscard]] inline int calc_xy_index( float position, float min_pos) const{
        return round((position - min_pos) / this->resolution);
    }
   /////////////////////////////////////////////////////////////////////////////////////////////////////
    inline int calc_grid_index(Node* node) const{
        return ((node->y-this->min_y)*this->x_width)+(node->x-this->min_x); //number of rows*columns+ number of extra columns to that row from left
        /*  000000000000000000
         *  111100000000000000        Row -> 2:  4 columns
         *  111111111111111111        Row->  1: 18 columns
         *  111111111111111111        Row->  0: 18 columns
         *  min-y = 0 min-x = 0 xwidth  = 18 y width = 4
         *  Returns: 2-0*18 + 4-0 = 40 -> Grid index
         */
    }//calculates delX+delY between min pos of grid and current pos of node
   ///////////////////////////////////////////////////////////////////////////////////////////////////// 

    static float calc_heuristic(Node* n1, Node* n2){
        float w = 1.0;
        float dist = w*sqrt(pow((n1->x-n2->x),2) + pow( (n1->y-n2->y),2));
        return dist;    
   }
   ///////////////////////////////////////////////////////////////////////////////////////////////////// 
    bool verify_node(Node* node){
        float px  = this->calc_grid_pos(node->x, this->min_x);
        float py = this->calc_grid_pos(node->y, this->min_y);
         if (px < this->min_x || py < this->min_y || px >= this->max_x || py >= max_y){
            return false;
        } //Out of bounds grid check
       
       if (this->obmap[node->x][node->y] == true){
           return false;
        } //if node resides on obstacle map--> collision==TRUE

        return true;
    }
///////////////////////////////////////////////////////////////////////////////////////////
    static vector<vector<float>> get_motion_model(){
                                                               
         vector<vector<float>> motion = {//{dx   dy    cost}
                                                                {1.0, 0.0, 1.0},
                                                                {0.0, 1.0, 1.0},
                                                                {-1.0, 0.0, 1.0},
                                                                {0, -1.0, 1.0},
                                                                {-1.0, -1.0, float(sqrt(2))},
                                                                {-1.0, 1.0, float(sqrt(2))},
                                                                {1.0, -1.0, float(sqrt(2))},
                                                                {1.0, 1.0,  float(sqrt(2))}
                                                                };

    return motion;
}
    
/////////////DATA MEMBERS OF PARENT CLASS///////////////////////////////////////ap
    vector<vector<float>> grid;
    vector<vector<int>> obmap;
    float resolution, rr, min_x, min_y, max_x, max_y, x_width, y_width;
    vector<vector<float>> motion;
    vector<float> ox, oy;
    cv::Mat bg;
//////////////////END OF A_STAR_PLANNER CLASS/////////////////
};


////////////////////////////////MAIN//////////////////////////////////////////////////////////////
int main(int argc, char* argv[]){
   printf("Name = %s \n\t\t______START__________\n\n", argv[0]);

    ////////////////VARIABLE DECLARATION//////////////////////////
    
    //Start and goal pos declaration
    float sx = 10.0;               // [m]
    float sy = 10.0;                 // [m]
    float gx = 500.0;                 // [m]
    float gy = 500.0;                 // [m]
    //Grid properties
    int grid_rows = 100;
    int grid_cols  = 100;
    float grid_resolution = 10.0;        //[m]
    //Robot Properties
    float robot_radius     = 5.0;   //[m]

    //Obstacle list Initialization
    vector<float> ox_;
    vector<float> oy_;
    // add edges
    for(float i=-100.0; i<600.0; i++){
        ox_.push_back(i);
        oy_.push_back(-100.0);
    }
    for(float i=-100.0; i<600.0; i++){
        ox_.push_back(600.0);
        oy_.push_back(i);
    }
    for(float i=-100.0; i<610.0; i++){
        ox_.push_back(i);
        oy_.push_back(600.0);
    }
    for(float i=-100.0; i<610.0; i++){
        ox_.push_back(-100.0);
        oy_.push_back(i);
    }
    for(float i=-100.0; i<400; i++){
        ox_.push_back(200.0);
        oy_.push_back(i);
    }
    for(float i=0; i<400; i++){
        ox_.push_back(400.0);
        oy_.push_back(600.0 - i);
    }


//  //Start and goal pos declaration
//  float sx = 10.0;               // [m]
//  float sy = 10.0;                 // [m]
//  float gx = 50.0;                 // [m]
//  float gy = 50.0;                 // [m]
//  //Grid properties
//  int grid_rows = 10;
//  int grid_cols  = 10;
//  float grid_resolution = 2.0;        //[m]
//  //Robot Properties
//  float robot_radius     = 1.0;   //[m]
//
//  //Obstacle list Initialization
//  vector<float> ox_;
//  vector<float> oy_;
//  // add edges
//  for(float i=-10.0; i<60.0; i++){
//    ox_.push_back(i);
//    oy_.push_back(-10.0);
//  }
//  for(float i=-10.0; i<60.0; i++){
//    ox_.push_back(60.0);
//    oy_.push_back(i);
//  }
//  for(float i=-10.0; i<61.0; i++){
//    ox_.push_back(i);
//    oy_.push_back(60.0);
//  }
//  for(float i=-10.0; i<61.0; i++){
//    ox_.push_back(-10.0);
//    oy_.push_back(i);
//  }
//  for(float i=-10.0; i<40; i++){
//    ox_.push_back(20.0);
//    oy_.push_back(i);
//  }
//  for(float i=0; i<40; i++){
//    ox_.push_back(40.0);
//    oy_.push_back(60.0 - i);
//  }
//


  //Calling constructor for A_Star_planner Class
    cv::namedWindow("astar", cv::WINDOW_AUTOSIZE);
    A_Star_planner planner_obj(grid_rows, grid_cols, grid_resolution, ox_, oy_, robot_radius);
    //Calling planner function
    vector<vector<float>> rxry = planner_obj.planner(sx, sy, gx,gy);
    cout<<"CODE FINISHED"<<endl;

}