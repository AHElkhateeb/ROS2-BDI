// header file for Communications MA (Multi-Agent) ACL Communicator node
#include "ros2_bdi_core/acl_communicator.hpp"
// Inner logic + ROS PARAMS & FIXED GLOBAL VALUES for ROS2 core nodes
#include "ros2_bdi_core/params/core_common_params.hpp"
// Inner logic + ROS2 PARAMS & FIXED GLOBAL VALUES for Communications (Multi-Agent) Request Handler node
#include "ros2_bdi_core/params/acl_communicator_params.hpp"
// Inner logic + ROS2 PARAMS & FIXED GLOBAL VALUES for Scheduler node (for desire set topic)
#include "ros2_bdi_core/params/scheduler_params.hpp"
// Inner logic + ROS2 PARAMS & FIXED GLOBAL VALUES for Belief Manager node (for belief set topic)
#include "ros2_bdi_core/params/belief_manager_params.hpp"

#include "ros2_bdi_utils/BDIFilter.hpp"

using std::string;
using std::vector;
using std::set;
using std::mutex;
using std::shared_ptr;
using std::chrono::milliseconds;
using std::bind;
using std::placeholders::_1;
using std::placeholders::_2;

using ros2_bdi_interfaces::msg::Belief;
using ros2_bdi_interfaces::msg::BeliefSet;
using ros2_bdi_interfaces::msg::Desire;
using ros2_bdi_interfaces::msg::DesireSet;
using ros2_bdi_interfaces::msg::AclMsg;
using ros2_bdi_interfaces::srv::AclSrv;

using BDIManaged::ManagedBelief;
using BDIManaged::ManagedDesire;


ACLCommunicator::ACLCommunicator()
  : rclcpp::Node(ACL_COMMUNICATOR_NODE_NAME)
{
  this->declare_parameter(PARAM_AGENT_ID, "agent0");
  this->declare_parameter(PARAM_AGENT_GROUP_ID, "agent0_group");
  this->declare_parameter(PARAM_DEBUG, true);
}

/*
  Init to call at the start, after construction method, to get the node actually started
*/
void ACLCommunicator::init()
{ 
  // agent's namespace
  agent_id_ = this->get_parameter(PARAM_AGENT_ID).as_string();

  rclcpp::QoS qos_keep_all = rclcpp::QoS(10);
  qos_keep_all.keep_all();

  // to make the belief/desire set subscription callbacks to run on different threads of execution wrt srv callbacks
  callback_group_upd_subscribers_ = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::Reentrant);
  auto sub_opt = rclcpp::SubscriptionOptions();
  sub_opt.callback_group = callback_group_upd_subscribers_;

  //register to belief set updates to have the mirroring of the last published version of it
  belief_set_subscriber_ = this->create_subscription<BeliefSet>(
              BELIEF_SET_TOPIC, qos_keep_all,
              bind(&ACLCommunicator::updatedBeliefSet, this, _1), sub_opt);
  
  //register to desire set updates to have the mirroring of the last published version of it
  desire_set_subscriber_ = this->create_subscription<DesireSet>(
              DESIRE_SET_TOPIC, qos_keep_all,
              bind(&ACLCommunicator::updatedDesireSet, this, _1), sub_opt);

  // to make the msg receivals callbacks run on different threads
  callback_group_msg_receivals_ = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::Reentrant);
  auto incoming_msg_opt = rclcpp::SubscriptionOptions();
  incoming_msg_opt.callback_group = callback_group_msg_receivals_;

  // init server for handling msg requests from other agents
  messaging_server_ = this->create_service<AclSrv>(ACL_SRV, 
      bind(&ACLCommunicator::handleMsgReceived, this, _1, _2), rmw_qos_profile_services_default, callback_group_msg_receivals_);
  
  // init server for handling incoming acl msgs from other agents
  incoming_messages_sub_ = this->create_subscription<AclMsg>(ACL_MSG_TOPIC, qos_keep_all,
      bind(&ACLCommunicator::handleIncomingMsg, this, _1), incoming_msg_opt);

  // to remove conversation clients from the conv_clients_ set and make callbacks run on same thread of execution wrt srv callbacks
  callback_group_del_conv_clients_ = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
  auto del_conv_clients_opt = rclcpp::SubscriptionOptions();
  del_conv_clients_opt.callback_group = callback_group_del_conv_clients_;

  del_conv_clients_subscriber_ = this->create_subscription<std_msgs::msg::String>(
              DEL_CONV_TOPIC, qos_keep_all,
              bind(&ACLCommunicator::deleteConversationclients, this, _1), del_conv_clients_opt);

  // add belief publisher -> to publish on the topic and alter the belief set when the request can go through
  add_belief_publisher_ = this->create_publisher<Belief>(ADD_BELIEF_TOPIC, 10);
  // del belief publisher -> to publish on the topic and alter the belief set when the request can go through
  del_belief_publisher_ = this->create_publisher<Belief>(DEL_BELIEF_TOPIC, 10);

  // init two locks for waiting belief upd
  belief_set_upd_locks_ = vector<mutex>(2);
  // init two counter for waiting belief upd
  belief_waiting_for_counter_ = vector<int>(2);
  // init two empty managed beliefs where to store the two you're waiting for an update
  belief_waiting_for_ = vector<ManagedBelief>(2);

  // add desire publisher -> to publish on the topic and alter the desire set when the request can go through
  add_desire_publisher_ = this->create_publisher<Desire>(ADD_DESIRE_TOPIC, 10);
  // del desire publisher -> to publish on the topic and alter the desire set when the request can go through
  del_desire_publisher_ = this->create_publisher<Desire>(DEL_DESIRE_TOPIC, 10);

  // init two locks for waiting desire upd
  desire_set_upd_locks_ = vector<mutex>(2);
  // init two counter for waiting desire upd
  desire_waiting_for_counter_ = vector<int>(2);
  // init two empty managed desires where to store the two you're waiting for an update
  desire_waiting_for_ = vector<ManagedDesire>(2);

  RCLCPP_INFO(this->get_logger(), "Multi-Agent ACL Communicator node initialized\n");
}

/*
  @updIndex to be used to know which lock has to be checked among the two in desire_set_upd_locks_
  and which waiting counter has to be incremented and/or checked

  @countCheck in order to check if desire is present (ADD op) or not present (DEL op)
*/
void ACLCommunicator::checkDesireSetWaitingUpd(const int& updIndex, const int& countCheck)
{
  if(updIndex > desire_set_upd_locks_.size() || countCheck < 0)//invalid params
    return;

    
  if(desire_set_upd_locks_[updIndex].try_lock())//try lock 
  {
    // if acquired, release immediately (no waiting for any belief upd operation)
    desire_set_upd_locks_[updIndex].unlock();
  }
  else
  {
    //waiting for a belief upd operation
    if(desire_set_.count(desire_waiting_for_[updIndex]) == countCheck || desire_waiting_for_counter_[updIndex] == MAX_WAIT_UPD)
      desire_set_upd_locks_[updIndex].unlock();// acquired by add_desire/del_desire srv, release it so it can proceed if alteration done or waited too much already
    else
      desire_waiting_for_counter_[updIndex]++;
  }
}

/*
    The desire set has been updated
*/
void ACLCommunicator::updatedDesireSet(const DesireSet::SharedPtr msg)
{
    desire_set_ = BDIFilter::extractMGDesires(msg->value);

            
    //check for waiting belief set alteration
    checkDesireSetWaitingUpd(ADD_I, 1);//check belief set for addition
    checkDesireSetWaitingUpd(DEL_I, 0);//check belief set for deletion
}

/*
  @updIndex to be used to know which lock has to be checked among the two in belief_set_upd_locks_
  and which waiting counter has to be incremented and/or checked

  @countCheck in order to check if belief is present (ADD op) or not present (DEL op)
*/
void ACLCommunicator::checkBeliefSetWaitingUpd(const int& updIndex, const int& countCheck)
{
  if(updIndex > belief_set_upd_locks_.size() || countCheck < 0)//invalid params
    return;

    
  if(belief_set_upd_locks_[updIndex].try_lock())//try lock 
  {
    // if acquired, release immediately (no waiting for any belief upd operation)
    belief_set_upd_locks_[updIndex].unlock();
  }
  else
  {
    //waiting for a belief upd operation
    if(belief_set_.count(belief_waiting_for_[updIndex]) == countCheck || belief_waiting_for_counter_[updIndex] == MAX_WAIT_UPD)
      belief_set_upd_locks_[updIndex].unlock();// acquired by add_belief/del_belief srv, release it so it can proceed if alteration done or waited too much already
    else
      belief_waiting_for_counter_[updIndex]++;
  }
}

/*
    The belief set has been updated
*/
void ACLCommunicator::updatedBeliefSet(const BeliefSet::SharedPtr msg)
{
    belief_set_ = BDIFilter::extractMGBeliefs(msg->value);
    
    //check for waiting belief set alteration
    checkBeliefSetWaitingUpd(ADD_I, 1);//check belief set for addition
    checkBeliefSetWaitingUpd(DEL_I, 0);//check belief set for deletion
}

/*
    Deletes ConvID from set
*/
void ACLCommunicator::deleteConversationclients(const std_msgs::msg::String::SharedPtr msg)
{
  //Mutex with addition of conversations
  conv_clients_upd_lock_.lock();
  conversations.erase(msg->data);
  conv_clients_upd_lock_.unlock();
}

/*  
    ACL message service handler        
*/
void ACLCommunicator::handleMsgReceived(const AclSrv::Request::SharedPtr request,
    const AclSrv::Response::SharedPtr response)
{
  ACLMessage msg_received = ACLMessage(request->msg);

  conv_clients_upd_lock_.lock();
  
  if(conversations.find(msg_received.getConversationId()) == conversations.end()) //NotFound
    {
      //add to list of conversations and dispatch a new ConversationClient Node
      conversations[ msg_received.getConversationId() ] = std::make_shared<ContractNetInitiator>(&desire_set_, &belief_set_);
      conv_clients_upd_lock_.unlock();
      RCLCPP_INFO(this->get_logger(), "ConvID: " + msg_received.getConversationId() + " added and will be dispatched");
      conversations[ msg_received.getConversationId() ]->receiveMsg(msg_received);
    }
  else //Found
    {
      conv_clients_upd_lock_.unlock();
      RCLCPP_INFO(this->get_logger(), "ConvID: " + msg_received.getConversationId() + " found and will be dispatched");
      //dispatch to the found ConversationClient Node;
      conversations[ msg_received.getConversationId() ]->receiveMsg(msg_received);
    }

  //replies to request with true for message being received
  response->received = true;
}

void ACLCommunicator::handleIncomingMsg(const ros2_bdi_interfaces::msg::AclMsg::SharedPtr msg)
{
  ACLMessage msg_received = ACLMessage(*msg);

  conv_clients_upd_lock_.lock();
  
  if(conversations.find(msg_received.getConversationId()) == conversations.end()) //NotFound
    {
      //add to list of conversations and dispatch a new ConversationClient Node
      conversations[ msg_received.getConversationId() ] = std::make_shared<ContractNetInitiator>(&desire_set_, &belief_set_);
      conv_clients_upd_lock_.unlock();
      RCLCPP_INFO(this->get_logger(), "ConvID: " + msg_received.getConversationId() + " added and will be dispatched");
      conversations[ msg_received.getConversationId() ]->receiveMsg(msg_received);
    }
  else //Found
    {
      conv_clients_upd_lock_.unlock();
      RCLCPP_INFO(this->get_logger(), "ConvID: " + msg_received.getConversationId() + " found and will be dispatched");
      //dispatch to the found ConversationClient Node;
      conversations[ msg_received.getConversationId() ]->receiveMsg(msg_received);
    }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ACLCommunicator>();
  //bool psys2_booted = node->wait_psys2_boot(std::chrono::seconds(8));//Wait max 8 seconds for plansys2 to boot
  bool psys2_booted = true;
  
  if(psys2_booted)
  {
    node->init();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
  }
  else
  {
    std::cerr << "PlanSys2 failed to boot: node will not spin and process will terminate" << std::endl;
  }
  rclcpp::shutdown();

  return 0;
}
