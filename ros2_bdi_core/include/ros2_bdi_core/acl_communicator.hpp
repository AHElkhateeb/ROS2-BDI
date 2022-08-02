#ifndef ACL_COMMUNICATOR_H_
#define ACL_COMMUNICATOR_H_

#include <mutex>
#include <vector>
#include <thread>

#include "ros2_bdi_interfaces/msg/belief.hpp"
#include "ros2_bdi_interfaces/msg/belief_set.hpp"
#include "ros2_bdi_interfaces/msg/desire.hpp"
#include "ros2_bdi_interfaces/msg/desire_set.hpp"
#include "ros2_bdi_interfaces/srv/upd_belief_set.hpp"
#include "ros2_bdi_interfaces/srv/upd_desire_set.hpp"
#include "ros2_bdi_interfaces/srv/acl_srv.hpp"

#include "ros2_bdi_utils/ManagedBelief.hpp"
#include "ros2_bdi_utils/ManagedDesire.hpp"

#include "ros2_bdi_core/params/acl_communicator_params.hpp"
#include "ros2_bdi_core/support/plansys2_monitor_client.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "ros2_bdi_behaviours/conversations_client.hpp"
#include "ros2_bdi_behaviours/ContractNetResponder.hpp"
#include "ros2_bdi_behaviours/ContractNetInitiator.hpp"

#include "ros2_bdi_behaviours/ACLMessage.hpp"

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
        psys2_monitor_client_ = std::make_shared<PlanSys2MonitorClient>(ACL_COMMUNICATOR_NODE_NAME + std::string("_psys2caller_"));
        return psys2_monitor_client_->areAllPsysNodeActive(max_wait);
    }
  

private:

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
        ACL message service handler        
    */
    void handleMsgReceived(const ros2_bdi_interfaces::srv::AclSrv::Request::SharedPtr request,
        const ros2_bdi_interfaces::srv::AclSrv::Response::SharedPtr response);
    
    // agent id that defines the namespace in which the node operates
    std::string agent_id_;

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

    // lock to put waiting for next belief set addition/deletion
    std::vector<std::mutex> belief_set_upd_locks_;
    // managed belief you're waiting for (to be added or deleted)
    std::vector<BDIManaged::ManagedBelief> belief_waiting_for_;
    // belief operation in waiting and not performed in last belief set upd counter
    std::vector<int> belief_waiting_for_counter_;
    
    //add_belief publisher
    rclcpp::Publisher<ros2_bdi_interfaces::msg::Belief>::SharedPtr add_belief_publisher_;
    //del_belief publisher
    rclcpp::Publisher<ros2_bdi_interfaces::msg::Belief>::SharedPtr del_belief_publisher_;
    
    // lock to put waiting for next desire set addition/deletion
    std::vector<std::mutex> desire_set_upd_locks_;
    // managed desire you're waiting for (to be added or deleted)
    std::vector<BDIManaged::ManagedDesire> desire_waiting_for_;
    // desire operation in waiting and not performed in last desire set upd counter
    std::vector<int> desire_waiting_for_counter_;
    
    //add_desire publisher
    rclcpp::Publisher<ros2_bdi_interfaces::msg::Desire>::SharedPtr add_desire_publisher_;
    //del_desire publisher
    rclcpp::Publisher<ros2_bdi_interfaces::msg::Desire>::SharedPtr del_desire_publisher_;

    // PlanSys2 Monitor Client supporting nodes & clients for calling the {psys2_node}/get_state services
    std::shared_ptr<PlanSys2MonitorClient> psys2_monitor_client_;

    // handle acl msgs from other agents
    rclcpp::Service<ros2_bdi_interfaces::srv::AclSrv>::SharedPtr messaging_server_;

    //map of conversations
    std::map<std::string, std::shared_ptr<ACLConversations::ConversationsClient>> conversations;

    //
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr del_conv_clients_subscriber_;

    /*
        The desire set has been updated
    */
    void deleteConversationclients(const std_msgs::msg::String::SharedPtr msg);

    // lock to prevent addition/deletion of convID simultaneously
    std::mutex conv_clients_upd_lock_;

};

#endif // ACL_COMMUNICATOR_H_