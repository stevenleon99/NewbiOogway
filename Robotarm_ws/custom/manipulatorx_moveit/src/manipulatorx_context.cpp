#include "manipulatorx_context.h"
#include <moveit/planning_scene/planning_scene.h>
#include <random>
#include <math.h>
#include <algorithm>

ASBRContext::ASBRContext( const moveit::core::RobotModelConstPtr& robotmodel,
			  const std::string& name, 
			  const std::string& group ):
  planning_interface::PlanningContext( name, group ),
  robotmodel( robotmodel ){}

ASBRContext::~ASBRContext(){}

bool ASBRContext::state_collides( const vertex& q ) const {

  // create a robot state
  moveit::core::RobotState robotstate( robotmodel );
  robotstate.setJointGroupPositions( "open_manipulator_x", q );

  if( getPlanningScene()->isStateColliding( robotstate, "open_manipulator_x", false ) )
    { return true; }
  else
    { return false; }
  
}

ASBRContext::vertex ASBRContext::interpolate( const ASBRContext::vertex& qA, 
					      const ASBRContext::vertex& qB, 
					      double t ){

  ASBRContext::vertex qt( qA.size(), 0.0 );
  for( std::size_t i=0; i<qt.size(); i++ )
    { qt[i] = ( 1.0 - t )*qA[i] + t*qB[i]; }
  return qt;

}


// TODO
ASBRContext::index ASBRContext::select_config_from_tree( const std::vector<ASBRContext::weight>& w ){
  
  ASBRContext::index i;

  // TODO
  // Use the weights to return the index of a random configuration in the tree
  std::cout << "[select_config_from_tree]: " << std::endl;
  // if the weight list is empty
  if (w.empty()){
    std::cout << "[select_config_from_tree]: w is empty" << std::endl;
    return -1;
  }

  i = 0;
  ASBRContext::weight maxWeight = w[0];

  for (auto j=1; j<w.size(); j++){
    if (w[j]>maxWeight){
      maxWeight = w[j];
      i = j;
    }
  }

  return i;
}


// TODO
ASBRContext::vertex ASBRContext::sample_nearby( const ASBRContext::vertex& q ){
  ASBRContext::vertex q_rand(q.size());

  // TODO
  // Generate a random configuration near q
  std::cout << "[sample_nearby]: " << std::endl;
  double range = 0.3;

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> dist(-range, range);

  for (auto i=0; i<q.size(); i++){
    q_rand[i] = q[i] + dist(gen);
  }

  return q_rand;
}

// TODO
bool ASBRContext::is_local_path_collision_free( const ASBRContext::vertex& q,
						const ASBRContext::vertex& q_rand ){

  std::cout << "[is_local_path_collision_free]: " << std::endl;            
  // TODO find if the straightline path between q_near and q_rand is collision free
  double step = 0.01;
  int stepN = 1/0.01;
  ASBRContext::vertex qt;

  for (int i = 0; i<=stepN; i++){
    qt = this->interpolate(q, q_rand, step*i);
    // state_collides return true if collision
    if (this->state_collides(qt)) {return false;}
  }

  return true;
}


// TODO
ASBRContext::path ASBRContext::search_path( const std::vector<vertex>& V,
 					      const std::vector<index> parent,
 					      const index& idx_init,
					      const index& idx_goal ){
  ASBRContext::path P;
  
  // TODO Once q_goal has been added to the tree, find the path (sequence of configurations) between
  // q_init and q_goal (hint: this is easier by using recursion).
  index cur = idx_goal;
  P.push_back(V[cur]);
  while (cur != idx_init){
    cur = parent[cur];
    P.push_back(V[cur]);
  }

  return P;
}

ASBRContext::index ASBRContext::find_nearest_configuration( const std::vector<ASBRContext::vertex>& V,
							    const ASBRContext::vertex& q ){
  
  std::cout << "[find_nearest_configuration]: " << std::endl; 
  int imin = 0;
  double minDist = std::numeric_limits<double>::max();
  // TODO find the nearest configuration in the tree
  for (auto i=0; i<V.size(); i++){
    double distance = this->twoVertexDist(V[i], q);
    if (distance < minDist){
      minDist = distance;
      imin = i;
    }
  }
  return imin;
}

// TODO
ASBRContext::path ASBRContext::est( const ASBRContext::vertex& q_init,
				    const ASBRContext::vertex& q_goal ){
  
  ASBRContext::path p_init;
  ASBRContext::path p_goal;
  // TODO implement EST algorithm and return the path (an ordered sequence of configurations).
  // initialize two trees
  std::vector<ASBRContext::vertex> tr_init = {q_init};
  std::vector<ASBRContext::vertex> tr_goal = {q_goal};
  // initialize two trees weight
  std::vector<ASBRContext::weight> weight_list_init = {1.0};
  std::vector<ASBRContext::weight> weight_list_goal = {1.0};
  // initialize parent index list
  std::vector<ASBRContext::index> index_parent_tr_init = {0};
  std::vector<ASBRContext::index> index_parent_tr_goal = {0};

  // start loop
  while(1){
    // check if trees can be connected
    auto idx_nearest = find_nearest_configuration(tr_goal, tr_init.back());
    if (is_local_path_collision_free(tr_goal[idx_nearest], tr_init.back())) { 
      std::cout << "[est]: solution found!" << std::endl;
      std::cout << "[est]: idx_nearest: " << idx_nearest << std::endl;
      for (auto i : index_parent_tr_init){
        std::cout << i << " ";
      }
      std::cout << std::endl;
      p_init = this->search_path(tr_init, index_parent_tr_init, 0, tr_init.size()-1); // need to reverse
      std::reverse(p_init.begin(), p_init.end());
      p_goal = this->search_path(tr_goal, index_parent_tr_goal, 0, idx_nearest); 
      p_init.insert(p_init.end(), p_goal.begin(), p_goal.end()); // add tr_goal to tr_init

      break;
    };

    // add new branch to tr_init
    while (1) {
      auto idx_init = select_config_from_tree(weight_list_init);
      auto vertex_nearby = sample_nearby(tr_init[idx_init]);
      if (is_local_path_collision_free(vertex_nearby, tr_init[idx_init])){
        std::cout << "idx_init: " << idx_init << " value is: " << weight_list_init[idx_init] << std::endl;
        tr_init.push_back(vertex_nearby); // add new branch
        weight_list_init[idx_init] = 1 / (1 + 1/(weight_list_init[idx_init])); //update weight
        weight_list_init.push_back(0.5); //add weight
        index_parent_tr_init.push_back(idx_init);
        break;
      }
    }

    // add new branch to tr_goal
    while (1) {
      auto idx_goal = select_config_from_tree(weight_list_goal);
      auto vertex_nearby = sample_nearby(tr_goal[idx_goal]);
      if (is_local_path_collision_free(vertex_nearby, tr_goal[idx_goal])){
        std::cout << "idx_goal: " << idx_goal << " value is: " << weight_list_init[idx_goal] << std::endl;
        tr_goal.push_back(vertex_nearby); // add new branch
        weight_list_goal[idx_goal] = 1 / (1 + 1/(weight_list_goal[idx_goal])); //update weight
        weight_list_goal.push_back(0.5); //add weight
        index_parent_tr_goal.push_back(idx_goal);
        break;
      }
    }
  }

  std::cout << "size of index_parent_tr_init: " << index_parent_tr_init.size() << std::endl;
  std::cout << "size of index_parent_tr_goal: " << index_parent_tr_goal.size() << std::endl;
  return p_init;
}

bool ASBRContext::solve( planning_interface::MotionPlanResponse &res ){

  // Create a new empty trajectory
  res.trajectory_.reset(new robot_trajectory::RobotTrajectory(robotmodel, 
  							      getGroupName()));
  res.trajectory_->clear();

  // copy the initial/final joints configurations to vectors qfin and qini
  // This is mainly for convenience.
  std::vector<double> qstart, qfinal;

  for( size_t i=0; i<robotmodel->getVariableCount(); i++ ){
    qfinal.push_back(request_.goal_constraints[0].joint_constraints[i].position);
    qstart.push_back(request_.start_state.joint_state.position[i]);
  }

  // start the timer
  rclcpp::Clock clock;
  rclcpp::Time t1 = clock.now();
  path P = est( qstart, qfinal );
  rclcpp::Time t2 = clock.now();
  std::cout << "Your path has length " << P.size() << std::endl;
  // end the timer

  // The rest is to fill in the animation.
  moveit::core::RobotState robotstate( robotmodel );
  robotstate.setJointGroupPositions( "open_manipulator_x", qstart );
  res.trajectory_->addSuffixWayPoint( robotstate, 0.001 );

  for( std::size_t i=1; i<P.size(); i++ ){
    for( double t=0.0; t<=1.0; t+=0.01 ){
      vertex q = interpolate( P[i-1], P[i], t );
      robotstate.setJointGroupPositions( "open_manipulator_x", q );
      res.trajectory_->addSuffixWayPoint( robotstate, 0.001 );
    }
  }

  //
  rclcpp::Duration planning_time = t2-t1;
  res.planning_time_ = planning_time.seconds();
  res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;

  return true;
  
}

bool ASBRContext::solve( planning_interface::MotionPlanDetailedResponse& )
{ return true; }

void ASBRContext::clear(){}

bool ASBRContext::terminate(){return true;}
