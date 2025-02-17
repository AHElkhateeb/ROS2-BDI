#ifndef ACL_COMMUNICATOR_H_
#define ACL_COMMUNICATOR_H_

#include <mutex>
#include <vector>
#include <map>
#include <thread>

#include "ros2_bdi_interfaces/msg/lifecycle_status.hpp"
#include "ros2_bdi_interfaces/msg/belief.hpp"
#include "ros2_bdi_interfaces/msg/belief_set.hpp"
#include "ros2_bdi_interfaces/msg/desire.hpp"
#include "ros2_bdi_interfaces/msg/desire_set.hpp"
#include "ros2_bdi_interfaces/srv/is_accepted_operation.hpp"
#include "ros2_bdi_interfaces/srv/check_belief.hpp"
#include "ros2_bdi_interfaces/srv/upd_belief_set.hpp"
#include "ros2_bdi_interfaces/srv/check_desire.hpp"
#include "ros2_bdi_interfaces/srv/upd_desire_set.hpp"
#include "ros2_bdi_interfaces/msg/acl_msg.hpp"
#include "ros2_bdi_interfaces/srv/acl_srv.hpp"

#include "ros2_bdi_utils/ManagedBelief.hpp"
#include "ros2_bdi_utils/ManagedDesire.hpp"

#include "ros2_bdi_core/params/core_common_params.hpp"
#include "ros2_bdi_core/params/acl_communicator_params.hpp"
#include "ros2_bdi_core/support/planning_mode.hpp"
#include "ros2_bdi_core/support/plansys_monitor_client.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "ros2_bdi_behaviours/conversations_client.hpp"
#include "ros2_bdi_behaviours/ContractNetResponder.hpp"
#include "ros2_bdi_behaviours/ContractNetInitiator.hpp"
#include "ros2_bdi_behaviours/OutgoingMessage.hpp"
#include "ros2_bdi_behaviours/OrganizerBehaviour.hpp"
#include "ros2_bdi_behaviours/TransporterBehaviour.hpp"

#include "ros2_bdi_behaviours/ACLMessage.hpp"
#include "ros2_bdi_behaviours/MessageTemplate.hpp"

typedef enum {BELIEF, DESIRE} RequestObjType;  
typedef enum {CHECK, WRITE} RequestObjOp;  

class ACLCommunicator : public rclcpp::Node
{
public:
  ACLCommunicator();

    /*
        Init to call at the start, after construction method, to get the node actually started
    */
    void init();

    /*
        Wait for PlanSys2 to boot at best for max_wait
    */
    bool wait_psys2_boot(const std::chrono::seconds max_wait = std::chrono::seconds(16))
    {
        psys_monitor_client_ = std::make_shared<PlanSysMonitorClient>(ACL_COMMUNICATOR_NODE_NAME + std::string("_psys2caller_"), sel_planning_mode_);
        return psys_monitor_client_->areAllPsysNodeActive(max_wait);
    }
  

private:

    /*Build updated ros2_bdi_interfaces::msg::LifecycleStatus msg*/
    ros2_bdi_interfaces::msg::LifecycleStatus getLifecycleStatus();

    /*
        Received notification about ROS2-BDI Lifecycle status
    */
    void callbackLifecycleStatus(const ros2_bdi_interfaces::msg::LifecycleStatus::SharedPtr msg)
    {
        if(lifecycle_status_.find(msg->node_name) != lifecycle_status_.end())//key in map, record upd value
            lifecycle_status_[msg->node_name] = msg->status;
    }

    /*
      Return true if the request agent's group name is among the accepted ones wrt.
      either belief or desire modify acceptance 
    */
    bool isAcceptableRequest(const std::string& requestingAgentGroup, 
      const RequestObjType& requestObjType, const RequestObjOp& requestObjOp);

    /*  
        Accepted group service handler
    */
    void handleIsAcceptedGroup(const ros2_bdi_interfaces::srv::IsAcceptedOperation::Request::SharedPtr request,
        const ros2_bdi_interfaces::srv::IsAcceptedOperation::Response::SharedPtr response);

    /*
      Return value of the max accepted priority for add desire requests coming from a requesting agent group
      Return negative value if not present
    */
    float getMaxAcceptedPriority(const std::string& requestingAgentGroup);

    /*
      @updIndex to be used to know which lock has to be checked among the two in desire_set_upd_locks_
      and which waiting counter has to be incremented and/or checked

      @countCheck in order to check if desire is present (ADD op) or not present (DEL op)
    */
    void checkDesireSetWaitingUpd(const int& updIndex, const int& countCheck);

    /*
        The desire set has been updated
    */
    void updatedDesireSet(const ros2_bdi_interfaces::msg::DesireSet::SharedPtr msg);

    /*
      @updIndex to be used to know which lock has to be checked among the two in belief_set_upd_locks_
      and which waiting counter has to be incremented and/or checked

      @countCheck in order to check if belief is present (ADD op) or not present (DEL op)
    */
    void checkBeliefSetWaitingUpd(const int& updIndex, const int& countCheck);
    
    /*
        The belief set has been updated
    */
    void updatedBeliefSet(const ros2_bdi_interfaces::msg::BeliefSet::SharedPtr msg);

    

    /*  
        Read Belief Request service handler        
    */
    void handleCheckBeliefRequest(const ros2_bdi_interfaces::srv::CheckBelief::Request::SharedPtr request,
        const ros2_bdi_interfaces::srv::CheckBelief::Response::SharedPtr response);


    /*  
        Add Belief Request service handler
    
    */
    void handleAddBeliefRequest(const ros2_bdi_interfaces::srv::UpdBeliefSet::Request::SharedPtr request,
        const ros2_bdi_interfaces::srv::UpdBeliefSet::Response::SharedPtr response);

    /*  
        Del Belief Request service handler        
    */
    void handleDelBeliefRequest(const ros2_bdi_interfaces::srv::UpdBeliefSet::Request::SharedPtr request,
        const ros2_bdi_interfaces::srv::UpdBeliefSet::Response::SharedPtr response);

    /*  
        Read Desire Request service handler        
    */
    void handleCheckDesireRequest(const ros2_bdi_interfaces::srv::CheckDesire::Request::SharedPtr request,
        const ros2_bdi_interfaces::srv::CheckDesire::Response::SharedPtr response);

    /*  
        Add Desire Request service handler        
    */
    void handleAddDesireRequest(const ros2_bdi_interfaces::srv::UpdDesireSet::Request::SharedPtr request,
        const ros2_bdi_interfaces::srv::UpdDesireSet::Response::SharedPtr response);

    /*  
         Del Desire Request service handler       
    */
    void handleDelDesireRequest(const ros2_bdi_interfaces::srv::UpdDesireSet::Request::SharedPtr request,
        const ros2_bdi_interfaces::srv::UpdDesireSet::Response::SharedPtr response);

    
    /*  
        ACL incoming message handler        
    */
    void handleIncomingMsg(const ros2_bdi_interfaces::msg::AclMsg::SharedPtr msg);

    /*  
        ACL message service handler        
    */
    void handleMsgReceived(const ros2_bdi_interfaces::srv::AclSrv::Request::SharedPtr request,
        const ros2_bdi_interfaces::srv::AclSrv::Response::SharedPtr response);

    
    // agent id that defines the namespace in which the node operates
    std::string agent_id_;
    // step counter
    uint64_t step_counter_;

    // timer to trigger callback to perform main loop of work regularly
    rclcpp::TimerBase::SharedPtr do_work_timer_;

    // Selected planning mode
    PlanningMode sel_planning_mode_;

    // handle accepted group queries by other agents
    rclcpp::Service<ros2_bdi_interfaces::srv::IsAcceptedOperation>::SharedPtr accepted_server_;

    // mirroring of the current state of the belief set
    std::set<BDIManaged::ManagedBelief> belief_set_;
    // belief set update subscription
    rclcpp::Subscription<ros2_bdi_interfaces::msg::BeliefSet>::SharedPtr belief_set_subscriber_;
    
    rclcpp::callback_group::CallbackGroup::SharedPtr callback_group_upd_subscribers_;
    rclcpp::callback_group::CallbackGroup::SharedPtr callback_group_msg_receivals_;
    rclcpp::callback_group::CallbackGroup::SharedPtr callback_group_del_conv_clients_;

    // mirroring of the current state of the desire set
    std::set<BDIManaged::ManagedDesire> desire_set_;
    // desire set update subscription
    rclcpp::Subscription<ros2_bdi_interfaces::msg::DesireSet>::SharedPtr desire_set_subscriber_;

    // handle check belief requests from other agents
    rclcpp::Service<ros2_bdi_interfaces::srv::CheckBelief>::SharedPtr chk_belief_server_;
    // handle add belief requests from other agents
    rclcpp::Service<ros2_bdi_interfaces::srv::UpdBeliefSet>::SharedPtr add_belief_server_;
    // handle del belief requests from other agents
    rclcpp::Service<ros2_bdi_interfaces::srv::UpdBeliefSet>::SharedPtr del_belief_server_;
    
    // lock to put waiting for next belief set addition/deletion
    std::mutex process_belief_set_upd_lock_;
    std::vector<std::mutex> belief_set_upd_locks_;
    // managed belief you're waiting for (to be added or deleted)
    std::vector<BDIManaged::ManagedBelief> belief_waiting_for_;
    // belief operation in waiting and not performed in last belief set upd counter
    std::vector<int> belief_waiting_for_counter_;
    
    //add_belief publisher
    rclcpp::Publisher<ros2_bdi_interfaces::msg::Belief>::SharedPtr add_belief_publisher_;
    //del_belief publisher
    rclcpp::Publisher<ros2_bdi_interfaces::msg::Belief>::SharedPtr del_belief_publisher_;

    // handle check desire requests from other agents
    rclcpp::Service<ros2_bdi_interfaces::srv::CheckDesire>::SharedPtr chk_desire_server_;
    // handle add desire requests from other agents
    rclcpp::Service<ros2_bdi_interfaces::srv::UpdDesireSet>::SharedPtr add_desire_server_;
    // handle del desire requests from other agents
    rclcpp::Service<ros2_bdi_interfaces::srv::UpdDesireSet>::SharedPtr del_desire_server_;
    
    // lock to put waiting for next desire set addition/deletion
    std::mutex process_desire_set_upd_lock_;
    std::vector<std::mutex> desire_set_upd_locks_;
    // managed desire you're waiting for (to be added or deleted)
    std::vector<BDIManaged::ManagedDesire> desire_waiting_for_;
    // desire operation in waiting and not performed in last desire set upd counter
    std::vector<int> desire_waiting_for_counter_;
    
    //add_desire publisher
    rclcpp::Publisher<ros2_bdi_interfaces::msg::Desire>::SharedPtr add_desire_publisher_;
    //del_desire publisher
    rclcpp::Publisher<ros2_bdi_interfaces::msg::Desire>::SharedPtr del_desire_publisher_;

    // current known status of the system nodes
    std::map<std::string, uint8_t> lifecycle_status_;
    // Publish updated lifecycle status
    rclcpp::Publisher<ros2_bdi_interfaces::msg::LifecycleStatus>::SharedPtr lifecycle_status_publisher_;
    // Sub to updated lifecycle status
    rclcpp::Subscription<ros2_bdi_interfaces::msg::LifecycleStatus>::SharedPtr lifecycle_status_subscriber_;

    // PlanSys2 Monitor Client supporting nodes & clients for calling the {psys2_node}/get_state services
    std::shared_ptr<PlanSysMonitorClient> psys_monitor_client_;

    // handle acl msgs requests from other agents
    rclcpp::Service<ros2_bdi_interfaces::srv::AclSrv>::SharedPtr messaging_server_;

    // handle incoming acl msgs
    rclcpp::Subscription<ros2_bdi_interfaces::msg::AclMsg>::SharedPtr incoming_messages_sub_;
    

    //map of conversations
    std::map<std::string, std::shared_ptr<ACLConversations::ConversationsClient>> conversations;

    //
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr del_conv_clients_subscriber_;

    //
    void deleteConversationclients(const std_msgs::msg::String::SharedPtr msg);

    // lock to prevent addition/deletion of convID simultaneously
    std::mutex conv_clients_upd_lock_;

};

#endif // ACL_COMMUNICATOR_H_