// header file for Communications ACL Communicator node
#include "ros2_bdi_core/acl_communicator.hpp"
// Inner logic + ROS PARAMS & FIXED GLOBAL VALUES for ROS2 core nodes
#include "ros2_bdi_core/params/core_common_params.hpp"
// Inner logic + ROS2 PARAMS & FIXED GLOBAL VALUES for Communications ACL Communicator node
#include "ros2_bdi_core/params/acl_communicator_params.hpp"
// Inner logic + ROS2 PARAMS & FIXED GLOBAL VALUES for Scheduler node (for desire set topic)
#include "ros2_bdi_core/params/scheduler_params.hpp"
// Inner logic + ROS2 PARAMS & FIXED GLOBAL VALUES for Belief Manager node (for belief set topic)
#include "ros2_bdi_core/params/belief_manager_params.hpp"

#include "ros2_bdi_utils/BDIFilter.hpp"

using std::string;
using std::vector;
using std::set;
using std::map;
using std::mutex;
using std::shared_ptr;
using std::chrono::milliseconds;
using std::bind;
using std::placeholders::_1;
using std::placeholders::_2;

using ros2_bdi_interfaces::msg::LifecycleStatus;
using ros2_bdi_interfaces::msg::Belief;
using ros2_bdi_interfaces::msg::BeliefSet;
using ros2_bdi_interfaces::msg::Desire;
using ros2_bdi_interfaces::msg::DesireSet;
using ros2_bdi_interfaces::srv::IsAcceptedOperation;
using ros2_bdi_interfaces::srv::CheckBelief;
using ros2_bdi_interfaces::srv::UpdBeliefSet;
using ros2_bdi_interfaces::srv::CheckDesire;
using ros2_bdi_interfaces::srv::UpdDesireSet;
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
  this->declare_parameter(PARAM_BELIEF_CHECK, vector<string>());
  this->declare_parameter(PARAM_BELIEF_WRITE, vector<string>());
  this->declare_parameter(PARAM_DESIRE_CHECK, vector<string>());
  this->declare_parameter(PARAM_DESIRE_WRITE, vector<string>());
  this->declare_parameter(PARAM_DESIRE_MAX_PRIORITIES, vector<double>());
  
  this->declare_parameter(PARAM_PLANNING_MODE, PLANNING_MODE_OFFLINE);

  sel_planning_mode_ = this->get_parameter(PARAM_PLANNING_MODE).as_string() == PLANNING_MODE_OFFLINE? OFFLINE : ONLINE;
  this->undeclare_parameter(PARAM_PLANNING_MODE);
}

/*
  Init to call at the start, after construction method, to get the node actually started
*/
void ACLCommunicator::init()
{ 
  // agent's namespace
  agent_id_ = this->get_parameter(PARAM_AGENT_ID).as_string();

  // init server for handling is accepted group queries
  accepted_server_ = this->create_service<IsAcceptedOperation>(IS_ACCEPTED_OP_SRV, 
      bind(&ACLCommunicator::handleIsAcceptedGroup, this, _1, _2));

  rclcpp::QoS qos_reliable = rclcpp::QoS(10);
  qos_reliable.reliable();

  //lifecycle status init
  auto lifecycle_status = LifecycleStatus{};
  lifecycle_status_ = map<string, uint8_t>();
  lifecycle_status_[BELIEF_MANAGER_NODE_NAME] = lifecycle_status.BOOTING;
  lifecycle_status_[SCHEDULER_NODE_NAME] = lifecycle_status.UNKNOWN;
  lifecycle_status_[PLAN_DIRECTOR_NODE_NAME] = lifecycle_status.UNKNOWN;
  lifecycle_status_[PSYS_MONITOR_NODE_NAME] = lifecycle_status.UNKNOWN;
  lifecycle_status_[EVENT_LISTENER_NODE_NAME] = lifecycle_status.UNKNOWN;
  lifecycle_status_[ACL_COMMUNICATOR_NODE_NAME] = lifecycle_status.UNKNOWN;

  // init step_counter
  step_counter_ = 0;

  //Lifecycle status publisher
  lifecycle_status_publisher_ = this->create_publisher<LifecycleStatus>(LIFECYCLE_STATUS_TOPIC, 10);

  //Lifecycle status subscriber
  lifecycle_status_subscriber_ = this->create_subscription<LifecycleStatus>(
              LIFECYCLE_STATUS_TOPIC, qos_reliable,
              bind(&ACLCommunicator::callbackLifecycleStatus, this, _1));

  // to make the belief/desire set subscription callbacks to run on different threads of execution wrt srv callbacks
  callback_group_upd_subscribers_ = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::Reentrant);
  auto sub_opt = rclcpp::SubscriptionOptions();
  sub_opt.callback_group = callback_group_upd_subscribers_;

  //register to belief set updates to have the mirroring of the last published version of it
  belief_set_subscriber_ = this->create_subscription<BeliefSet>(
              BELIEF_SET_TOPIC, qos_reliable,
              bind(&ACLCommunicator::updatedBeliefSet, this, _1), sub_opt);
  
  //register to desire set updates to have the mirroring of the last published version of it
  desire_set_subscriber_ = this->create_subscription<DesireSet>(
              DESIRE_SET_TOPIC, qos_reliable,
              bind(&ACLCommunicator::updatedDesireSet, this, _1), sub_opt);

  // init server for handling check belief requests from other agents
  chk_belief_server_ = this->create_service<CheckBelief>(CK_BELIEF_SRV, 
      bind(&ACLCommunicator::handleCheckBeliefRequest, this, _1, _2));
  
  // init server for handling add belief requests from other agents
  add_belief_server_ = this->create_service<UpdBeliefSet>(ADD_BELIEF_SRV, 
      bind(&ACLCommunicator::handleAddBeliefRequest, this, _1, _2));
    
  // init server for handling del belief requests from other agents
  del_belief_server_ = this->create_service<UpdBeliefSet>(DEL_BELIEF_SRV, 
      bind(&ACLCommunicator::handleDelBeliefRequest, this, _1, _2));
  
  // add belief publisher -> to publish on the topic and alter the belief set when the request can go through
  add_belief_publisher_ = this->create_publisher<Belief>(ADD_BELIEF_TOPIC, 10);
  // del belief publisher -> to publish on the topic and alter the belief set when the request can go through
  del_belief_publisher_ = this->create_publisher<Belief>(DEL_BELIEF_TOPIC, 10);

  // to make the msg receivals callbacks run on different threads
  callback_group_msg_receivals_ = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::Reentrant);
  auto incoming_msg_opt = rclcpp::SubscriptionOptions();
  incoming_msg_opt.callback_group = callback_group_msg_receivals_;

  // init server for handling msg requests from other agents
  /*
  messaging_server_ = this->create_service<AclSrv>(ACL_SRV, 
      bind(&ACLCommunicator::handleMsgReceived, this, _1, _2), rmw_qos_profile_services_default, callback_group_msg_receivals_);
  */

  // init server for handling incoming acl msgs from other agents
  incoming_messages_sub_ = this->create_subscription<AclMsg>(ACL_MSG_TOPIC, qos_reliable,
      bind(&ACLCommunicator::handleIncomingMsg, this, _1), incoming_msg_opt);

  // to remove conversation clients from the conv_clients_ set and make callbacks run on same thread of execution wrt srv callbacks
  callback_group_del_conv_clients_ = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
  auto del_conv_clients_opt = rclcpp::SubscriptionOptions();
  del_conv_clients_opt.callback_group = callback_group_del_conv_clients_;

  del_conv_clients_subscriber_ = this->create_subscription<std_msgs::msg::String>(
              DEL_CONV_TOPIC, qos_reliable,
              bind(&ACLCommunicator::deleteConversationclients, this, _1), del_conv_clients_opt);

  // init two locks for waiting belief upd
  belief_set_upd_locks_ = vector<mutex>(2);
  // init two counter for waiting belief upd
  belief_waiting_for_counter_ = vector<int>(2);
  // init two empty managed beliefs where to store the two you're waiting for an update
  belief_waiting_for_ = vector<ManagedBelief>(2);

  // init server for handling check desire requests from other agents
  chk_desire_server_ = this->create_service<CheckDesire>(CK_DESIRE_SRV, 
      bind(&ACLCommunicator::handleCheckDesireRequest, this, _1, _2));

  // init server for handling add belief requests from other agents
  add_desire_server_ = this->create_service<UpdDesireSet>(ADD_DESIRE_SRV, 
      bind(&ACLCommunicator::handleAddDesireRequest, this, _1, _2));
    
    // init server for handling del belief requests from other agents
  del_desire_server_ = this->create_service<UpdDesireSet>(DEL_DESIRE_SRV, 
      bind(&ACLCommunicator::handleDelDesireRequest, this, _1, _2));

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

  string acceptingBeliefsMsg = "accepting beliefs alteration from: ";
  vector<string> acceptingBeliefsGroups = this->get_parameter(PARAM_BELIEF_WRITE).as_string_array();
  if(acceptingBeliefsGroups.size() == 0)
    acceptingBeliefsMsg += "no group";
  else
    for(int i = 0; i<acceptingBeliefsGroups.size(); i++)
      acceptingBeliefsMsg += acceptingBeliefsGroups[i]  + (((i+1)==acceptingBeliefsGroups.size()) ? "" : ", ");

  string acceptingDesiresMsg = "accepting desires alteration from: ";
  vector<string> acceptingDesiresGroups = this->get_parameter(PARAM_DESIRE_WRITE).as_string_array();
  vector<double> acceptingDesiresPrioritiesGroups = this->get_parameter(PARAM_DESIRE_MAX_PRIORITIES).as_double_array();
  if(acceptingDesiresGroups.size() == 0)
    acceptingDesiresMsg += "no group";
  else
    for(int i = 0; i<acceptingDesiresGroups.size(); i++)
    {
      acceptingDesiresMsg += acceptingDesiresGroups[i];
      
      acceptingDesiresMsg += (acceptingDesiresPrioritiesGroups.size() > i)? 
        "(max_pr = " + std::to_string(acceptingDesiresPrioritiesGroups[i]) + ")" : 
        "(max_pr = 0.01)"; 
        
      acceptingDesiresMsg +=  (((i+1)==acceptingDesiresGroups.size()) ? "" : ", ");
    }

  RCLCPP_INFO(this->get_logger(), "ACL Communicator node initialized:\n" + 
      acceptingBeliefsMsg + ";\n" + acceptingDesiresMsg);
  
  
  do_work_timer_ = this->create_wall_timer(
                    milliseconds(2000),
                      [&](){
                        if(step_counter_ % 4 == 0)
                          lifecycle_status_publisher_->publish(getLifecycleStatus());
                        
                        step_counter_++;
                      }
                    ); 
}

/*Build updated LifecycleStatus msg*/
LifecycleStatus ACLCommunicator::getLifecycleStatus()
{
    LifecycleStatus lifecycle_status = LifecycleStatus{};
    lifecycle_status.node_name = ACL_COMMUNICATOR_NODE_NAME;
    lifecycle_status.status = lifecycle_status.RUNNING;
    return lifecycle_status;
}


/*
  Return true if the request agent's group name is among the accepted ones wrt.
  either belief or desire modify acceptance 
*/
bool ACLCommunicator::isAcceptableRequest(const string& requestingAgentGroup, 
  const RequestObjType& requestObjType, const RequestObjOp& requestObjOp)
{
  vector<string> acceptedGroups;
  switch(requestObjType)
  {
    case BELIEF:
      switch(requestObjOp)
      {
        case CHECK:
          acceptedGroups = this->get_parameter(PARAM_BELIEF_CHECK).as_string_array();
          break;
        case WRITE:
          acceptedGroups = this->get_parameter(PARAM_BELIEF_WRITE).as_string_array();
          break;
      }
        
      break;

      case DESIRE:
        switch(requestObjOp)
        {
        case CHECK:
          acceptedGroups = this->get_parameter(PARAM_DESIRE_CHECK).as_string_array();
          break;
        case WRITE:
          acceptedGroups = this->get_parameter(PARAM_DESIRE_WRITE).as_string_array();
          break;
      }
      break;
  }

  for(string accepted : acceptedGroups)
    if(accepted == requestingAgentGroup)
      return true;// found among accepted ones

  return false;// not found among accepted ones
}

/*  
    Accepted group service handler
*/
void ACLCommunicator::handleIsAcceptedGroup(const IsAcceptedOperation::Request::SharedPtr request,
    const IsAcceptedOperation::Response::SharedPtr response)
{
  bool validType = request->type == request->BELIEF_TYPE || request->type == request->DESIRE_TYPE;
  bool validOp = request->operation == request->CHECK || request->operation == request->WRITE;

  RequestObjType objType = (request->type==request->DESIRE_TYPE)? DESIRE : BELIEF;
  RequestObjOp objOp = (request->operation==request->CHECK)? CHECK : WRITE;
  if(!validType || !validOp || !isAcceptableRequest(request->agent_group, objType, objOp))
      response->accepted = false;

  else
  {
      response->accepted = true;
      if(objType == DESIRE && objOp == WRITE)
        response->desire_max_priority = getMaxAcceptedPriority(request->agent_group);
      else
        response->desire_max_priority = -1.0f;
  }

}

/*
  Return value of the max accepted priority for add desire requests coming from a requesting agent group
  Return negative value if not present
*/
float ACLCommunicator::getMaxAcceptedPriority(const string& requestingAgentGroup)
{
  int indexAccepted = -1;
  float maxAcceptedPriority = -1.0f;//init to negative value

  // find index of accepted group in accept_desires_from array string paramters
  vector<string> acceptedGroups = this->get_parameter(PARAM_DESIRE_WRITE).as_string_array();
  for(int i = 0; indexAccepted == -1 && i <  acceptedGroups.size(); i++)
    if(acceptedGroups[i] == requestingAgentGroup)
      indexAccepted = i;

  if(indexAccepted >= 0)//if found
  {
    // retrieve correspondent priority from accept_desires_max_priorities double array
    vector<double> acceptedPriorities = this->get_parameter(PARAM_DESIRE_MAX_PRIORITIES).as_double_array();
    
    if(indexAccepted < acceptedPriorities.size())
      maxAcceptedPriority = std::max(0.000f, std::min(1.0f, (float)acceptedPriorities[indexAccepted]));//priority has to be between 0.001 and 1
    
    else
      maxAcceptedPriority = 0.000f; //bad formatted accept_desires_max_priorities array, but do not refuse just put 0.000 as priority of the desire
  }

  return maxAcceptedPriority;
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
    process_desire_set_upd_lock_.lock();
    {
      desire_set_ = BDIFilter::extractMGDesires(msg->value);

              
      //check for waiting belief set alteration
      checkDesireSetWaitingUpd(ADD_I, 1);//check belief set for addition
      checkDesireSetWaitingUpd(DEL_I, 0);//check belief set for deletion
    }
    process_desire_set_upd_lock_.unlock();
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
    process_belief_set_upd_lock_.lock();
    {
      belief_set_ = BDIFilter::extractMGBeliefs(msg->value);

      //check for waiting belief set alteration
      checkBeliefSetWaitingUpd(ADD_I, 1);//check belief set for addition
      checkBeliefSetWaitingUpd(DEL_I, 0);//check belief set for deletion
    }
    process_belief_set_upd_lock_.unlock();
}

/*  
    Read Belief Request service handler        
*/
void ACLCommunicator::handleCheckBeliefRequest(const CheckBelief::Request::SharedPtr request,
    const CheckBelief::Response::SharedPtr response)
{
  //see if the requesting agent belongs to a group which is entitled to this kind of requests
  if(!isAcceptableRequest(request->agent_group, BELIEF, CHECK))
    response->accepted = false;

  else
  {
    response->accepted = true;
    response->found = belief_set_.count(ManagedBelief{request->belief}) == 1;
  }
}


/*  
    Add Belief Request service handler

*/
void ACLCommunicator::handleAddBeliefRequest(const UpdBeliefSet::Request::SharedPtr request,
    const UpdBeliefSet::Response::SharedPtr response)
{
  //see if the requesting agent belongs to a group which is entitled to this kind of requests
  if(!isAcceptableRequest(request->agent_group, BELIEF, WRITE))
    response->accepted = false;

  else
  {
    response->accepted = true;
    add_belief_publisher_->publish(request->belief);
    
    belief_set_upd_locks_[ADD_I].lock();
      belief_waiting_for_[ADD_I] = ManagedBelief{request->belief};
      belief_waiting_for_counter_[ADD_I] = 0;
    belief_set_upd_locks_[ADD_I].lock();//stuck until belief_set upd unlock it
    belief_set_upd_locks_[ADD_I].unlock();//release it

    response->updated = belief_set_.count(ManagedBelief{request->belief}) == 1;

  }
}

/*  
    Del Belief Request service handler        
*/
void ACLCommunicator::handleDelBeliefRequest(const UpdBeliefSet::Request::SharedPtr request,
    const UpdBeliefSet::Response::SharedPtr response)
{
  //see if the requesting agent belongs to a group which is entitled to this kind of requests
  if(!isAcceptableRequest(request->agent_group, BELIEF, WRITE))
    response->accepted = false;

  else
  {
    response->accepted = true;
    del_belief_publisher_->publish(request->belief);
    
    belief_set_upd_locks_[DEL_I].lock();
      belief_waiting_for_[DEL_I] = ManagedBelief{request->belief};
      belief_waiting_for_counter_[DEL_I] = 0;
    belief_set_upd_locks_[DEL_I].lock();//stuck until belief_set upd unlock it
    belief_set_upd_locks_[DEL_I].unlock();//release it

    response->updated = belief_set_.count(ManagedBelief{request->belief}) == 0;
  }
}

/*  
    Read Desire Request service handler        
*/
void ACLCommunicator::handleCheckDesireRequest(const CheckDesire::Request::SharedPtr request,
    const CheckDesire::Response::SharedPtr response)
{
  //see if the requesting agent belongs to a group which is entitled to this kind of requests
  if(!isAcceptableRequest(request->agent_group, DESIRE, CHECK))
    response->accepted = false;

  else
  {
    response->accepted = true;
    response->found = desire_set_.count(ManagedDesire{request->desire}) == 1;
  }
}

/*  
    Add Desire Request service handler        
*/
void ACLCommunicator::handleAddDesireRequest(const UpdDesireSet::Request::SharedPtr request,
    const UpdDesireSet::Response::SharedPtr response)
{
  //see if the requesting agent belongs to a group which is entitled to this kind of requests
  if(!isAcceptableRequest(request->agent_group, DESIRE, WRITE))
    response->accepted = false;

  else
  {
    float maxAcceptedPriority = getMaxAcceptedPriority(request->agent_group);
    
    if(maxAcceptedPriority >= 0)
    {
      response->accepted = true;
      // set at most the desire priority to the fixed upper threshold
      request->desire.priority = std::max(0.000f, std::min(request->desire.priority, maxAcceptedPriority)); 
      add_desire_publisher_->publish(request->desire);
              
      desire_set_upd_locks_[ADD_I].lock();
        desire_waiting_for_[ADD_I] = ManagedDesire{request->desire};
        desire_waiting_for_counter_[ADD_I] = 0;
      desire_set_upd_locks_[ADD_I].lock();//stuck until desire_set upd unlock it
      desire_set_upd_locks_[ADD_I].unlock();//release it

      response->updated = desire_set_.count(ManagedDesire{request->desire}) == 1 || ManagedDesire{request->desire}.isFulfilled(belief_set_);
    }
    else
      response->accepted = false;// max priority for given agent's requesting group is negative -> not accepted
    
  }
}

/*  
      Del Desire Request service handler       
*/
void ACLCommunicator::handleDelDesireRequest(const UpdDesireSet::Request::SharedPtr request,
    const UpdDesireSet::Response::SharedPtr response)
{
  //see if the requesting agent belongs to a group which is entitled to this kind of requests
  if(!isAcceptableRequest(request->agent_group, DESIRE, WRITE))
    response->accepted = false;

  else
  {
    response->accepted = true;
    del_desire_publisher_->publish(request->desire);
    desire_set_upd_locks_[DEL_I].lock();
        desire_waiting_for_[DEL_I] = ManagedDesire{request->desire};
        desire_waiting_for_counter_[DEL_I] = 0;
    desire_set_upd_locks_[DEL_I].lock();//stuck until desire_set upd unlock it
    desire_set_upd_locks_[DEL_I].unlock();//release it

    response->updated = desire_set_.count(ManagedDesire{request->desire}) == 0;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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
/*
void ACLCommunicator::handleMsgReceived(const AclSrv::Request::SharedPtr request,
    const AclSrv::Response::SharedPtr response)
{
  ACLMessage msg_received = ACLMessage(request->msg);

  conv_clients_upd_lock_.lock();
  
  if(conversations.find(msg_received.getConversationId()) == conversations.end()) //NotFound
    {
      if(msg_received.getConversationId() == "") { msg_received.setConversationId( agent_id_+CURRENT_TIME_MILLIS ); }
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
*/

/*    ACL message handler    */

void ACLCommunicator::handleIncomingMsg(const ros2_bdi_interfaces::msg::AclMsg::SharedPtr msg)
{
  ACLMessage msg_received = ACLMessage(*msg);

  conv_clients_upd_lock_.lock();
  
  if(conversations.find(msg_received.getConversationId()) == conversations.end()) //NotFound
    {
      RCLCPP_INFO(this->get_logger(), "Message received with ConvID: " + msg_received.getConversationId() + " which is part of a new conversation");
      if(msg_received.getConversationId() == "") { msg_received.setConversationId( agent_id_+CURRENT_TIME_MILLIS ); }

      //add to list of conversations and dispatch a new ConversationClient Node
      /*TO-DO move messsage template definitions outside of the callback function*/
      MessageTemplate msg_outgoing, msg_cnet_init, msg_cnet_respond;
      
      msg_outgoing.matchSender(agent_id_);
      msg_cnet_init.matchProtocol(AclMsg::FIPA_CONTRACT_NET);
      msg_cnet_init.matchSender(agent_id_);
      msg_cnet_respond.matchProtocol(AclMsg::FIPA_CONTRACT_NET);

      if(msg_cnet_init.isMatch(msg_received))
      {
        conversations[ msg_received.getConversationId() ] = std::make_shared<OrganizerBehaviour>(&desire_set_, &belief_set_, agent_id_, msg_received.getConversationId());
        conv_clients_upd_lock_.unlock();
      }
      else if (msg_cnet_respond.isMatch(msg_received))
      {
        conversations[ msg_received.getConversationId() ] = std::make_shared<TransporterBehaviour>(&desire_set_, &belief_set_, agent_id_, msg_received.getConversationId());
        conv_clients_upd_lock_.unlock();
      }
      else if (msg_outgoing.isMatch(msg_received))
      {
        conversations[ msg_received.getConversationId() ] = std::make_shared<OutgoingMessage>(&desire_set_, &belief_set_, agent_id_, msg_received.getConversationId());
        conv_clients_upd_lock_.unlock();
      }
      else
      {
        conversations[ msg_received.getConversationId() ] = std::make_shared<ConversationsClient>(&desire_set_, &belief_set_, agent_id_, msg_received.getConversationId());
        conv_clients_upd_lock_.unlock();
      }
      
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

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ACLCommunicator>();
  node->init();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  /*
  bool psys2_booted = node->wait_psys2_boot(std::chrono::seconds(8));//Wait max 8 seconds for plansys2 to boot
  
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
  */
  rclcpp::shutdown();

  return 0;
}
