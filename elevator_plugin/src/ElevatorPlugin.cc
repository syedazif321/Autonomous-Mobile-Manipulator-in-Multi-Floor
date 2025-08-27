#include <functional>
#include <thread>
#include <mutex>
#include <list>
#include <cmath>

#include <ignition/common/Profiler.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Console.hh>
#include <gazebo/common/PID.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo_ros/node.hpp>

#include "elevator_plugin/ElevatorPlugin.hh"

using namespace gazebo;

//////////////////////////////////////////////////
// Internal implementation
class ElevatorPlugin::ElevatorPluginPrivate
{
public:
  // State machine base
  class State { public: virtual ~State()=default; virtual void Start(){}; virtual bool Update(){return true;} bool started{false}; };
  class CloseState; class OpenState; class MoveState; class WaitState;

  // Controllers
  class DoorController;
  class LiftController;

  physics::ModelPtr model;
  sdf::ElementPtr sdf;
  physics::JointPtr doorJoint;
  physics::JointPtr liftJoint;

  DoorController *doorController{nullptr};
  LiftController *liftController{nullptr};

  std::list<State*> states;
  std::mutex stateMutex;
  common::Time doorWaitTime{5,0};

  transport::NodePtr node;
  transport::SubscriberPtr elevatorSub;
  event::ConnectionPtr updateConnection;

  ~ElevatorPluginPrivate();
};

//////////////////////////////////////////////////
// Controllers
class ElevatorPlugin::ElevatorPluginPrivate::DoorController
{
public:
  enum Target { CLOSE, OPEN };
  enum State { MOVING, STATIONARY };

  DoorController(physics::JointPtr _joint) : doorJoint(_joint), state(STATIONARY), target(CLOSE)
  { this->doorPID.Init(2,0,1.0); }

  void SetTarget(Target _t){ this->target=_t; }
  Target GetTarget() const { return target; }
  State GetState() const { return state; }
  void Reset(){ prevSimTime=common::Time::Zero; }

  bool Update(const common::UpdateInfo &_info)
  {
    if(prevSimTime==common::Time::Zero){ prevSimTime=_info.simTime; return false; }
    double targetPos = (target==OPEN)?1.0:0.0;
    double err = doorJoint->Position()-targetPos;
    double force = doorPID.Update(err,_info.simTime-prevSimTime);
    prevSimTime=_info.simTime;
    doorJoint->SetForce(0,force);
    if(std::abs(err)<0.05){ state=STATIONARY; return true; }
    state=MOVING; return false;
  }
private:
  physics::JointPtr doorJoint;
  State state; Target target;
  common::Time prevSimTime{0};
  gazebo::common::PID doorPID;
};

class ElevatorPlugin::ElevatorPluginPrivate::LiftController
{
public:
  enum State { MOVING, STATIONARY };

  LiftController(physics::JointPtr _joint,float h):state(STATIONARY),floor(0),floorHeight(h),liftJoint(_joint)
  { liftPID.Init(100000,0,100000.0); }

  void Reset(){ prevSimTime=common::Time::Zero; }
  void SetFloor(int f){ floor=f; }
  int GetFloor() const { return floor; }
  State GetState() const { return state; }

  bool Update(const common::UpdateInfo &_info)
  {
    if(prevSimTime==common::Time::Zero){ prevSimTime=_info.simTime; return false; }
    double err=liftJoint->Position()-(floor*floorHeight);
    double force=liftPID.Update(err,_info.simTime-prevSimTime);
    prevSimTime=_info.simTime;
    liftJoint->SetForce(0,force);
    if(std::abs(err)<0.15){ state=STATIONARY; return true; }
    state=MOVING; return false;
  }
private:
  State state; int floor; float floorHeight; physics::JointPtr liftJoint;
  common::Time prevSimTime{0}; gazebo::common::PID liftPID;
};

//////////////////////////////////////////////////
// States
class ElevatorPlugin::ElevatorPluginPrivate::CloseState : public State
{
public: CloseState(DoorController *_c):ctrl(_c){} 
  void Start() override{ ctrl->SetTarget(DoorController::CLOSE); started=true; }
  bool Update() override{ if(!started){Start();return false;} return ctrl->GetTarget()==DoorController::CLOSE && ctrl->GetState()==DoorController::STATIONARY; }
private: DoorController *ctrl;
};

class ElevatorPlugin::ElevatorPluginPrivate::OpenState : public State
{
public: OpenState(DoorController *_c):ctrl(_c){}
  void Start() override{ ctrl->SetTarget(DoorController::OPEN); started=true; }
  bool Update() override{ if(!started){Start();return false;} return ctrl->GetTarget()==DoorController::OPEN && ctrl->GetState()==DoorController::STATIONARY; }
private: DoorController *ctrl;
};

class ElevatorPlugin::ElevatorPluginPrivate::MoveState : public State
{
public: MoveState(int f,LiftController *_c):floor(f),ctrl(_c){}
  void Start() override{ ctrl->SetFloor(floor); started=true; }
  bool Update() override{ if(!started){Start();return false;} return ctrl->GetState()==LiftController::STATIONARY; }
private:int floor; LiftController *ctrl;
};

class ElevatorPlugin::ElevatorPluginPrivate::WaitState : public State
{
public: WaitState(const common::Time &_t):waitTime(_t){}
  void Start() override{ start=common::Time::GetWallTime(); started=true; }
  bool Update() override{ if(!started){Start();return false;} return (common::Time::GetWallTime()-start)>=waitTime; }
private: common::Time waitTime; common::Time start{0};
};

//////////////////////////////////////////////////
ElevatorPlugin::ElevatorPluginPrivate::~ElevatorPluginPrivate()
{
  delete doorController; delete liftController;
  for(auto s:states) delete s; states.clear();
}

//////////////////////////////////////////////////
// Plugin main class
GZ_REGISTER_MODEL_PLUGIN(ElevatorPlugin)

ElevatorPlugin::ElevatorPlugin() : dataPtr(new ElevatorPluginPrivate) {}
ElevatorPlugin::~ElevatorPlugin(){ this->dataPtr->updateConnection.reset(); }

void ElevatorPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->dataPtr->model=_model; this->dataPtr->sdf=_sdf;
  float floorHeight=3.0; if(_sdf->HasElement("floor_height")) floorHeight=_sdf->Get<float>("floor_height");

  std::string liftJointName=_sdf->Get<std::string>("lift_joint");
  std::string doorJointName=_sdf->Get<std::string>("door_joint");

  this->dataPtr->liftJoint=_model->GetJoint(liftJointName);
  this->dataPtr->doorJoint=_model->GetJoint(doorJointName);
  if(!this->dataPtr->liftJoint||!this->dataPtr->doorJoint){ gzerr<<"[ElevatorPlugin] joints missing\n"; return; }

  this->dataPtr->doorController=new ElevatorPluginPrivate::DoorController(this->dataPtr->doorJoint);
  this->dataPtr->liftController=new ElevatorPluginPrivate::LiftController(this->dataPtr->liftJoint,floorHeight);

  this->dataPtr->updateConnection=event::Events::ConnectWorldUpdateBegin(std::bind(&ElevatorPlugin::Update,this,std::placeholders::_1));

  this->dataPtr->node=transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init(_model->GetWorld()->Name());
  this->dataPtr->elevatorSub=this->dataPtr->node->Subscribe("~/elevator",&ElevatorPlugin::OnElevator,this);


  this->ros_node_ = gazebo_ros::Node::Get(_sdf);

  this->srv_ = this->ros_node_->create_service<std_srvs::srv::SetBool>(
    "/elevator_cmd",
    std::bind(&ElevatorPlugin::RosServiceCb,this,std::placeholders::_1,std::placeholders::_2));

  RCLCPP_INFO(this->ros_node_->get_logger(), "ElevatorPlugin ROS2 service /elevator_cmd ready");
}

void ElevatorPlugin::RosServiceCb(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,std::shared_ptr<std_srvs::srv::SetBool::Response> res)
{
  int target=current_floor_; if(req->data) target+=1; else target=std::max(0,current_floor_-1);
  MoveToFloor(target); current_floor_=target;
  res->success=true; res->message="Moving to floor "+std::to_string(target);
}

void ElevatorPlugin::OnElevator(ConstGzStringPtr &_msg)
{
  try{ int f=std::stoi(_msg->data()); MoveToFloor(f); current_floor_=f;}
  catch(...){ gzerr<<"Invalid elevator command: "<<_msg->data()<<"\n"; }
}

void ElevatorPlugin::MoveToFloor(const int f)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->stateMutex);
  if(!this->dataPtr->states.empty()) return;
  this->dataPtr->states.push_back(new ElevatorPluginPrivate::CloseState(this->dataPtr->doorController));
  this->dataPtr->states.push_back(new ElevatorPluginPrivate::MoveState(f,this->dataPtr->liftController));
  this->dataPtr->states.push_back(new ElevatorPluginPrivate::OpenState(this->dataPtr->doorController));
  this->dataPtr->states.push_back(new ElevatorPluginPrivate::WaitState(this->dataPtr->doorWaitTime));
  this->dataPtr->states.push_back(new ElevatorPluginPrivate::CloseState(this->dataPtr->doorController));
}

void ElevatorPlugin::Update(const common::UpdateInfo &_info)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->stateMutex);
  if(!this->dataPtr->states.empty())
  {
    if(this->dataPtr->states.front()->Update()){ delete this->dataPtr->states.front(); this->dataPtr->states.pop_front(); }
  }
  this->dataPtr->doorController->Update(_info);
  this->dataPtr->liftController->Update(_info);
}

void ElevatorPlugin::Reset()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->stateMutex);
  for(auto s:this->dataPtr->states) delete s;
  this->dataPtr->states.clear();
  this->dataPtr->doorController->Reset();
  this->dataPtr->liftController->Reset();
}
